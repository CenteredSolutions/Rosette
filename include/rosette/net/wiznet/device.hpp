#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <type_traits>
#include <variant>

#include "rosette/platform/host_runtime.hpp"  // rosette_platform_get_ms for the bounded ARP probe

#ifndef ROSETTE_NO_FREERTOS
extern "C" {
#include "FreeRTOS.h"
#include "task.h"
}
#endif

namespace rosette::net::wiznet {

struct ChipBase {
    using address_type = uint32_t;
    using socket_type = uint8_t;

    struct Header {
        std::array<uint8_t, 8> bytes{};
        std::size_t length = 0u;

        const uint8_t* data() const noexcept { return bytes.data(); }
    };
};

enum class Interrupt : uint8_t {
    Receive = 0x04u,
};

enum class DeviceMode : uint8_t {
    uninitialized = 0u,
    macraw,
    socket,
};

template<typename T>
struct is_chip_type : std::is_base_of<ChipBase, T> {};

template<typename Chip, typename Bus>
class WiznetDevice {
    static_assert(is_chip_type<Chip>::value, "Chip must derive from ChipBase");

public:
    using chip_type = Chip;
    using bus_type = Bus;
    using address_type = typename Chip::address_type;
    using socket_type = typename Chip::socket_type;
    using header_type = typename Chip::Header;
    using mode_type = DeviceMode;

    struct InvalidChipVersion {
        uint8_t actual = 0u;
        uint8_t expected = 0u;
    };

    struct Ipv4Config {
        std::array<uint8_t, 4> ip{};
        std::array<uint8_t, 4> subnet{};
        std::array<uint8_t, 4> gateway{};
    };

    explicit WiznetDevice(Bus& bus) noexcept
        : bus_(bus) {}

    bool verify() noexcept
    {
        const auto lock = bus_.acquire();
        return read_u8(Chip::common_version()) == Chip::chip_version();
    }

    void hardware_reset() noexcept
    {
        const auto lock = bus_.acquire();
        // How a part is reset (and, on the W61xx/W63xx, unlocked for network
        // config) is chip-specific; the trait issues the register writes.
        Chip::reset([this](address_type addr, uint8_t value) { write_u8(addr, value); });
    }

    void set_mac_address(const std::array<uint8_t, 6>& mac) noexcept
    {
        const auto lock = bus_.acquire();
        write(Chip::common_mac_address(), mac.data(), mac.size());
    }

    std::array<uint8_t, 6> mac_address() noexcept
    {
        std::array<uint8_t, 6> mac{};
        const auto lock = bus_.acquire();
        read(Chip::common_mac_address(), mac.data(), mac.size());
        return mac;
    }

    std::variant<std::monostate, InvalidChipVersion>
    init_macraw(const std::array<uint8_t, 6>& mac) noexcept
    {
        const auto lock = bus_.acquire();
        hardware_reset();

        const uint8_t version = read_u8(Chip::common_version());
        if (version != Chip::chip_version()) {
            return InvalidChipVersion{version, Chip::chip_version()};
        }

        // MACRAW gives the whole buffer memory to socket 0; the other sockets
        // are unused in this mode. Socket 0's buffer spans the region from
        // offset 0.
        write_u8(Chip::socket_tx_buffer_size(Chip::macraw_socket()), Chip::socket_tx_buffer_kib());
        write_u8(Chip::socket_rx_buffer_size(Chip::macraw_socket()), Chip::socket_rx_buffer_kib());
        buffers_[Chip::macraw_socket()] = SocketBuffer{
            0u, static_cast<uint16_t>(Chip::socket_tx_buffer_kib() * 1024u),
            0u, static_cast<uint16_t>(Chip::socket_rx_buffer_kib() * 1024u)};

        set_mac_address(mac);
        write_u8(Chip::common_socket_interrupt_mask(),
                 static_cast<uint8_t>(1u << Chip::macraw_socket()));
        write_u8(Chip::socket_interrupt_mask(Chip::macraw_socket()),
                 Chip::socket_interrupt_receive());
        write_u8(Chip::socket_mode(Chip::macraw_socket()), Chip::macraw_mode());
        issue_socket_command(Chip::macraw_socket(), Chip::command_open());
        mode_ = DeviceMode::macraw;

        return std::monostate{};
    }

    std::variant<std::monostate, InvalidChipVersion>
    init_socket_mode(const std::array<uint8_t, 6>& mac) noexcept
    {
        const auto lock = bus_.acquire();
        hardware_reset();

        const uint8_t version = read_u8(Chip::common_version());
        if (version != Chip::chip_version()) {
            return InvalidChipVersion{version, Chip::chip_version()};
        }

        // Split the buffer memory evenly across the sockets. Each socket's
        // buffer offset is the running sum of the sizes actually assigned, so
        // the layout the driver caches always matches the layout it wrote.
        const uint8_t tx_kib = Chip::socket_tx_buffer_kib() / Chip::socket_count();
        const uint8_t rx_kib = Chip::socket_rx_buffer_kib() / Chip::socket_count();
        const uint16_t tx_size = static_cast<uint16_t>(tx_kib * 1024u);
        const uint16_t rx_size = static_cast<uint16_t>(rx_kib * 1024u);
        uint16_t tx_offset = 0u;
        uint16_t rx_offset = 0u;
        for (socket_type socket = 0; socket < Chip::socket_count(); ++socket) {
            write_u8(Chip::socket_tx_buffer_size(socket), tx_kib);
            write_u8(Chip::socket_rx_buffer_size(socket), rx_kib);
            buffers_[socket] = SocketBuffer{tx_offset, tx_size, rx_offset, rx_size};
            tx_offset = static_cast<uint16_t>(tx_offset + tx_size);
            rx_offset = static_cast<uint16_t>(rx_offset + rx_size);
            write_u8(Chip::socket_interrupt_mask(socket), 0u);
            clear_socket_interrupt(socket, 0xFFu);
            issue_socket_command(socket, Chip::command_close());
            send_pending_[socket] = false;
        }

        set_mac_address(mac);
        write_u8(Chip::common_socket_interrupt_mask(), 0u);
        clear_common_interrupt(0xFFu);
        mode_ = DeviceMode::socket;
        return std::monostate{};
    }

    mode_type mode() const noexcept
    {
        return mode_;
    }

#ifndef ROSETTE_NO_FREERTOS
    void register_socket_rx_task(socket_type socket,
                                 TaskHandle_t task = xTaskGetCurrentTaskHandle()) noexcept
    {
        if (socket >= Chip::socket_count()) {
            return;
        }

        taskENTER_CRITICAL();
        socket_rx_tasks_[socket] = task;
        taskEXIT_CRITICAL();
    }

    void unregister_socket_rx_task(socket_type socket,
                                   TaskHandle_t task = nullptr) noexcept
    {
        if (socket >= Chip::socket_count()) {
            return;
        }

        if (task == nullptr) {
            task = xTaskGetCurrentTaskHandle();
        }

        taskENTER_CRITICAL();
        if ((task == nullptr) || (socket_rx_tasks_[socket] == task)) {
            socket_rx_tasks_[socket] = nullptr;
        }
        taskEXIT_CRITICAL();
    }

    void register_interrupt_dispatch_task(
        TaskHandle_t task = xTaskGetCurrentTaskHandle()) noexcept
    {
        taskENTER_CRITICAL();
        interrupt_dispatch_task_ = task;
        taskEXIT_CRITICAL();
    }

    void unregister_interrupt_dispatch_task(TaskHandle_t task = nullptr) noexcept
    {
        if (task == nullptr) {
            task = xTaskGetCurrentTaskHandle();
        }

        taskENTER_CRITICAL();
        if ((task == nullptr) || (interrupt_dispatch_task_ == task)) {
            interrupt_dispatch_task_ = nullptr;
        }
        taskEXIT_CRITICAL();
    }

    void handle_interrupt_from_isr() noexcept
    {
        BaseType_t should_yield = pdFALSE;

        if ((mode_ == DeviceMode::socket) && (interrupt_dispatch_task_ != nullptr)) {
            vTaskNotifyGiveFromISR(interrupt_dispatch_task_, &should_yield);
        }

        if ((mode_ == DeviceMode::macraw) &&
            (Chip::macraw_socket() < Chip::socket_count()) &&
            (socket_rx_tasks_[Chip::macraw_socket()] != nullptr)
        ) {
            TaskHandle_t macraw_task = socket_rx_tasks_[Chip::macraw_socket()];
            vTaskNotifyGiveFromISR(macraw_task, &should_yield);
        }

        portYIELD_FROM_ISR(should_yield);
    }

    void service_socket_interrupts() noexcept
    {
        if (mode_ != DeviceMode::socket) {
            return;
        }

        std::array<TaskHandle_t, Chip::socket_count()> tasks{};
        uint8_t interrupt_mask = 0u;
        uint8_t common_ack = 0u;

        {
            const auto lock = bus_.acquire();
            interrupt_mask = read_u8(Chip::common_interrupt());

            if ((interrupt_mask & Chip::common_ip_conflict_mask()) != 0u) {
                ip_conflict_pending_ = true;
                common_ack = static_cast<uint8_t>(
                    common_ack | Chip::common_ip_conflict_mask());
            }

            for (socket_type socket = 0; socket < Chip::socket_count(); ++socket) {
                const uint8_t socket_bit = static_cast<uint8_t>(1u << socket);
                if ((interrupt_mask & socket_bit) == 0u) {
                    continue;
                }

                const uint8_t socket_interrupt = read_u8(Chip::socket_interrupt(socket));
                const uint8_t receive_interrupt =
                    static_cast<uint8_t>(socket_interrupt & Chip::socket_interrupt_receive());

                if (receive_interrupt != 0u) {
                    clear_socket_interrupt(socket, receive_interrupt);
                    tasks[socket] = socket_rx_tasks_[socket];
                }

                common_ack = static_cast<uint8_t>(common_ack | socket_bit);
            }

            if (common_ack != 0u) {
                clear_common_interrupt(common_ack);
            }
        }

        for (socket_type socket = 0; socket < Chip::socket_count(); ++socket) {
            if (tasks[socket] != nullptr) {
                xTaskNotifyGive(tasks[socket]);
            }
        }
    }
#endif

    bool is_link_up() noexcept
    {
        const auto lock = bus_.acquire();
        return (read_u8(Chip::common_link_status()) & Chip::link_up_mask()) != 0u;
    }

    /**
     * @brief Reads the common interrupt register once and latches the
     * IP-conflict bit if the chip has flagged an ARP clash for our own SIPR.
     *
     * Costs one register read; call it from a context that already runs at a
     * low cadence (the superloop `WiznetLink::poll()` or a DHCP-renewal tick).
     * On the FreeRTOS build the interrupt-dispatch task
     * (`service_socket_interrupts`) latches the same bit, so either path feeds
     * `take_ip_conflict()`.
     *
     * @return `true` when a conflict was newly latched by this call.
     */
    bool poll_common_interrupt() noexcept
    {
        if (Chip::common_ip_conflict_mask() == 0u) {
            return false;
        }
        const auto lock = bus_.acquire();
        const uint8_t ir = read_u8(Chip::common_interrupt());
        const uint8_t bit = static_cast<uint8_t>(ir & Chip::common_ip_conflict_mask());
        if (bit != 0u) {
            clear_common_interrupt(bit);
            ip_conflict_pending_ = true;
        }
        return bit != 0u;
    }

    /**
     * @brief Consumes the latched IP-conflict flag.
     *
     * @return `true` once per detected conflict; `false` afterwards until the
     * next one.
     */
    bool take_ip_conflict() noexcept
    {
        const auto lock = bus_.acquire();
        const bool was = ip_conflict_pending_;
        ip_conflict_pending_ = false;
        return was;
    }

    void configure_ipv4(const std::array<uint8_t, 4>& ip,
                        const std::array<uint8_t, 4>& subnet,
                        const std::array<uint8_t, 4>& gateway) noexcept
    {
        const auto lock = bus_.acquire();
        write(Chip::common_gateway(), gateway.data(), gateway.size());
        write(Chip::common_subnet(), subnet.data(), subnet.size());
        write(Chip::common_ip_address(), ip.data(), ip.size());
    }

    Ipv4Config ipv4_config() noexcept
    {
        Ipv4Config config{};
        const auto lock = bus_.acquire();
        read(Chip::common_gateway(), config.gateway.data(), config.gateway.size());
        read(Chip::common_subnet(), config.subnet.data(), config.subnet.size());
        read(Chip::common_ip_address(), config.ip.data(), config.ip.size());
        return config;
    }

    std::size_t read_frame(uint8_t* frame, std::size_t capacity) noexcept
    {
        const auto lock = bus_.acquire();
        const socket_type socket = Chip::macraw_socket();
        const uint16_t available = socket_rx_received_size(socket);
        if ((frame == nullptr) || (capacity == 0u) ||
            (available < Chip::macraw_rx_header_bytes())) {
            return 0u;
        }

        uint8_t header[Chip::macraw_rx_header_bytes()] = {0};
        uint16_t pointer = socket_rx_read_pointer(socket);
        pointer = ring_read(socket,
                            pointer,
                            header,
                            static_cast<uint16_t>(sizeof(header)));

        const uint16_t total_len =
            static_cast<uint16_t>((static_cast<uint16_t>(header[0]) << 8) | header[1]);
        if (total_len < Chip::macraw_rx_header_bytes()) {
            set_socket_rx_read_pointer(socket, pointer);
            issue_socket_command(socket, Chip::command_receive_done());
            return 0u;
        }

        const uint16_t frame_len =
            static_cast<uint16_t>(total_len - Chip::macraw_rx_header_bytes());
        if ((frame_len == 0u) || (frame_len > capacity)) {
            pointer = ring_skip(pointer, frame_len);
            set_socket_rx_read_pointer(socket, pointer);
            issue_socket_command(socket, Chip::command_receive_done());
            return 0u;
        }

        pointer = ring_read(socket, pointer, frame, frame_len);
        set_socket_rx_read_pointer(socket, pointer);
        issue_socket_command(socket, Chip::command_receive_done());
        return frame_len;
    }

    bool write_frame(const uint8_t* frame, std::size_t length) noexcept
    {
        const auto lock = bus_.acquire();
        const socket_type socket = Chip::macraw_socket();
        if ((frame == nullptr) || (length == 0u) || (length > Chip::max_frame_bytes())) {
            return false;
        }

        const uint16_t required = static_cast<uint16_t>(length);
        while (socket_tx_free_size(socket) < required) {
        }

        // Finish the previous frame before reusing the TX ring; usually a no-op
        // by now. Its SENDOK/timeout result is not propagated — FreeRTOS+TCP
        // treats the output return as advisory and retransmits at L4.
        wait_pending_send(socket);

        uint16_t pointer = socket_tx_write_pointer(socket);
        pointer = ring_write(socket, pointer, frame, required);
        set_socket_tx_write_pointer(socket, pointer);
        issue_socket_command(socket, Chip::command_send());
        send_pending_[socket] = true;
        return true;
    }

    bool open_udp_socket(socket_type socket, uint16_t local_port) noexcept
    {
        const auto lock = bus_.acquire();
        close_socket(socket);
        write_u8(Chip::socket_mode(socket), Chip::mode_udp());
        write_u16(Chip::socket_source_port(socket), local_port);
        write_u8(Chip::socket_interrupt_mask(socket), Chip::socket_interrupt_receive());
        write_u8(Chip::common_socket_interrupt_mask(),
                 static_cast<uint8_t>(read_u8(Chip::common_socket_interrupt_mask()) |
                                      static_cast<uint8_t>(1u << socket)));
        issue_socket_command(socket, Chip::command_open());
        return read_u8(Chip::socket_status(socket)) == Chip::socket_status_udp();
    }

    bool open_udp_multicast_socket(socket_type socket,
                                   uint16_t local_port,
                                   const std::array<uint8_t, 4>& group,
                                   uint16_t group_port) noexcept
    {
        const auto lock = bus_.acquire();
        close_socket(socket);

        const std::array<uint8_t, 6> group_mac = multicast_mac_from_ipv4(group);
        write(Chip::socket_dest_hardware(socket), group_mac.data(), group_mac.size());
        write(Chip::socket_dest_ip(socket), group.data(), group.size());
        write_u16(Chip::socket_dest_port(socket), group_port);
        write_u8(Chip::socket_mode(socket),
                 static_cast<uint8_t>(Chip::mode_udp() |
                                      Chip::mode_multicast() |
                                      Chip::mode_igmp_v2()));
        write_u16(Chip::socket_source_port(socket), local_port);
        write_u8(Chip::socket_interrupt_mask(socket), Chip::socket_interrupt_receive());
        write_u8(Chip::common_socket_interrupt_mask(),
                 static_cast<uint8_t>(read_u8(Chip::common_socket_interrupt_mask()) |
                                      static_cast<uint8_t>(1u << socket)));
        issue_socket_command(socket, Chip::command_open());
        return read_u8(Chip::socket_status(socket)) == Chip::socket_status_udp();
    }

    void close_socket(socket_type socket) noexcept
    {
        const auto lock = bus_.acquire();
        // Let any outstanding SEND finish and clear send_pending_ — CLOSE wipes
        // Sn_IR, so a stale flag would make the next send on a reopened socket
        // spin forever waiting for a SENDOK that can never arrive.
        wait_pending_send(socket);
        write_u8(Chip::socket_interrupt_mask(socket), 0u);
        write_u8(Chip::common_socket_interrupt_mask(),
                 static_cast<uint8_t>(read_u8(Chip::common_socket_interrupt_mask()) &
                                      ~static_cast<uint8_t>(1u << socket)));
        clear_socket_interrupt(socket, 0xFFu);
        issue_socket_command(socket, Chip::command_close());
    }

    bool configure_udp_peer(socket_type socket,
                            const std::array<uint8_t, 4>& address,
                            uint16_t port) noexcept
    {
        const auto lock = bus_.acquire();
        if (is_multicast_ipv4(address)) {
            const auto group_mac = multicast_mac_from_ipv4(address);
            write(Chip::socket_dest_hardware(socket), group_mac.data(), group_mac.size());
        }
        write(Chip::socket_dest_ip(socket), address.data(), address.size());
        write_u16(Chip::socket_dest_port(socket), port);
        return true;
    }

    std::ptrdiff_t udp_send_to(socket_type socket,
                               const uint8_t* data,
                               std::size_t length,
                               const std::array<uint8_t, 4>& address,
                               uint16_t port) noexcept
    {
        const auto lock = bus_.acquire();
        if ((data == nullptr) || (length == 0u) ||
            (length > static_cast<std::size_t>(socket_tx_capacity(socket)))) {
            std::printf("Invalid parameters for udp_send_to: data=%p length=%zu socket_tx_capacity=%u\n",
                        static_cast<const void*>(data), length, socket_tx_capacity(socket));
            return -1;
        }

        if (!configure_udp_peer(socket, address, port)) {
            std::printf("Failed to configure UDP peer for socket %d\n", socket);
            return -1;
        }

        const uint16_t required = static_cast<uint16_t>(length);
        while (socket_tx_free_size(socket) < required) {
        }

        // Finish the previous datagram before reusing the TX ring so the chip
        // does not fuse the two. Usually already done by now.
        wait_pending_send(socket);

        uint16_t pointer = socket_tx_write_pointer(socket);
        pointer = ring_write(socket, pointer, data, required);
        set_socket_tx_write_pointer(socket, pointer);
        issue_socket_command(socket, Chip::command_send());
        send_pending_[socket] = true;

        return static_cast<std::ptrdiff_t>(length);
    }

    std::ptrdiff_t udp_receive_from(socket_type socket,
                                    uint8_t* data,
                                    std::size_t capacity,
                                    std::array<uint8_t, 4>* address,
                                    uint16_t* port) noexcept
    {
        struct udp4_header {
            uint8_t src_ip[4];
            uint8_t src_port[2];
            uint8_t length[2];
        };

        const auto lock = bus_.acquire();
        const uint16_t available = socket_rx_received_size(socket);
        if ((data == nullptr) || (capacity == 0u) ||
            (available < Chip::udp_rx_header_bytes())) {
            return 0;
        }

        udp4_header header;
        uint16_t pointer = socket_rx_read_pointer(socket);
        pointer = ring_read(socket,
                            pointer,
                            reinterpret_cast<uint8_t*>(&header),
                            static_cast<uint16_t>(sizeof(header)));

        const uint16_t payload_len =
            static_cast<uint16_t>((static_cast<uint16_t>(header.length[0]) << 8) | header.length[1]);

        if (address != nullptr) {
            std::memcpy(address->data(), header.src_ip, address->size());
        }
        if (port != nullptr) {
            *port = static_cast<uint16_t>((static_cast<uint16_t>(header.src_port[0]) << 8) | header.src_port[1]);
        }

        if ((payload_len == 0u) || (payload_len > capacity)) {
            pointer = ring_skip(pointer, payload_len);
            set_socket_rx_read_pointer(socket, pointer);
            issue_socket_command(socket, Chip::command_receive_done());
            ack_socket_rx_interrupt(socket);
            return -1;
        }

        pointer = ring_read(socket, pointer, data, payload_len);
        set_socket_rx_read_pointer(socket, pointer);
        issue_socket_command(socket, Chip::command_receive_done());
        ack_socket_rx_interrupt(socket);
        return payload_len;
    }

    uint16_t socket_rx_available(socket_type socket) noexcept
    {
        const auto lock = bus_.acquire();
        return socket_rx_received_size(socket);
    }

    /// Socket RX / TX buffer capacity in bytes, as configured by the last
    /// init_*(). Zero for a socket that has not been set up.
    uint16_t socket_rx_capacity(socket_type socket) const noexcept { return buffers_[socket].rx_size; }
    uint16_t socket_tx_capacity(socket_type socket) const noexcept { return buffers_[socket].tx_size; }

    /// Bus address of socket's RX / TX buffer base. Valid after an init_*() call.
    address_type rx_buffer_address(socket_type socket) const noexcept { return rx_buffer_addr(socket, 0u); }
    address_type tx_buffer_address(socket_type socket) const noexcept { return tx_buffer_addr(socket, 0u); }

private:
    static bool is_multicast_ipv4(const std::array<uint8_t, 4>& address) noexcept
    {
        return (address[0] >= 224u) && (address[0] <= 239u);
    }

    static std::array<uint8_t, 6>
    multicast_mac_from_ipv4(const std::array<uint8_t, 4>& address) noexcept
    {
        return {
            0x01u,
            0x00u,
            0x5eu,
            static_cast<uint8_t>(address[1] & 0x7fu),
            address[2],
            address[3],
        };
    }

    void write(address_type address, const uint8_t* data, std::size_t length) noexcept
    {
        const auto prefix = Chip::template make_header<false>(address);
        bus_.write(prefix.data(), prefix.length, data, length);
    }

    void read(address_type address, uint8_t* data, std::size_t length) noexcept
    {
        const auto prefix = Chip::template make_header<true>(address);
        bus_.read(prefix.data(), prefix.length, data, length);
    }

    void write_u8(address_type address, uint8_t value) noexcept
    {
        write(address, &value, 1u);
    }

    uint8_t read_u8(address_type address) noexcept
    {
        uint8_t value = 0u;
        read(address, &value, 1u);
        return value;
    }

    void write_u16(address_type address, uint16_t value) noexcept
    {
        const uint8_t raw[2] = {
            static_cast<uint8_t>(value >> 8),
            static_cast<uint8_t>(value),
        };
        write(address, raw, sizeof(raw));
    }

    uint16_t read_u16(address_type address) noexcept
    {
        uint8_t raw[2] = {0u, 0u};
        read(address, raw, sizeof(raw));
        return static_cast<uint16_t>((static_cast<uint16_t>(raw[0]) << 8) | raw[1]);
    }

    uint16_t read_u16_stable(address_type address) noexcept
    {
        uint16_t first = read_u16(address);
        uint16_t second = 0u;

        while (first != second) {
            second = first;
            first = read_u16(address);
        } 

        return second;
    }

    void issue_socket_command(socket_type socket, uint8_t command) noexcept
    {
        write_u8(Chip::socket_command(socket), command);
        while (read_u8(Chip::socket_command(socket)) != 0u) {
        }
    }

    /**
     * @brief Completes the previous SEND on this socket, if one is outstanding.
     *
     * Datasheet: "HOST must execute next SEND Command after Sn_IR[SENDOK] is set
     * to 1." The command register clears as soon as SEND is *accepted*, well
     * before the datagram is on the wire, and the chip transmits everything
     * between Sn_TX_RD and Sn_TX_WR — so issuing the next SEND before SENDOK
     * races the write pointer and the chip fuses both payloads into one datagram
     * (or drops the second).
     *
     * Rather than block at the tail of every send, `udp_send_to` / `write_frame`
     * just set `send_pending_[socket]` and call this at the head of the next one.
     * By then the SPI traffic since the SEND has almost always covered the
     * transmit time, so the common case is a single Sn_IR read that already
     * shows SENDOK. The `send_pending_` guard also means a context that never
     * sent (or whose SENDOK another path already consumed) does not spin.
     *
     * SENDOK is cleared here unconditionally (harmless when already clear).
     * `Sn_IR.TIMEOUT` (ARP/retransmit failure) and a closed socket end the wait
     * so a dead peer cannot hang the caller.
     */
    void wait_pending_send(socket_type socket) noexcept
    {
        while (send_pending_[socket]) {
            const uint8_t irq = read_u8(Chip::socket_interrupt(socket));
            if ((irq & (Chip::socket_interrupt_send_ok() |
                        Chip::socket_interrupt_timeout())) != 0u) {
                break;
            }
            if (read_u8(Chip::socket_status(socket)) == Chip::socket_status_closed()) {
                break;
            }
        }
        send_pending_[socket] = false;
        clear_socket_interrupt(socket, Chip::socket_interrupt_send_ok());
    }

    /**
     * @brief Clears the socket's RX interrupt so the chip INT line deasserts.
     *
     * On the no-RTOS build this lets the next datagram raise a fresh edge for
     * the polling executor's `WFE` wake. Under FreeRTOS the interrupt-dispatch
     * task owns `Sn_IR` servicing and its per-socket notification, so this is a
     * no-op there.
     */
    void ack_socket_rx_interrupt(socket_type socket) noexcept
    {
#ifdef ROSETTE_NO_FREERTOS
        clear_socket_interrupt(socket, Chip::socket_interrupt_receive());
#else
        (void)socket;
#endif
    }

    // Sn_IR / IR are write-1-to-clear on the W5100S, but a separate write-only
    // *CLR register on the W61xx/W63xx. `Chip::socket_interrupt_clear()` /
    // `Chip::common_interrupt_clear()` name whichever register takes the ack.
    void clear_socket_interrupt(socket_type socket, uint8_t bits) noexcept
    {
        write_u8(Chip::socket_interrupt_clear(socket), bits);
    }

    void clear_common_interrupt(uint8_t bits) noexcept
    {
        write_u8(Chip::common_interrupt_clear(), bits);
    }

    uint16_t socket_rx_received_size(socket_type socket) noexcept
    {
        return read_u16_stable(Chip::socket_rx_received_size(socket));
    }

    uint16_t socket_tx_free_size(socket_type socket) noexcept
    {
        return read_u16_stable(Chip::socket_tx_free_size(socket));
    }

    uint16_t socket_rx_read_pointer(socket_type socket) noexcept
    {
        return read_u16(Chip::socket_rx_read_pointer(socket));
    }

    uint16_t socket_tx_write_pointer(socket_type socket) noexcept
    {
        return read_u16(Chip::socket_tx_write_pointer(socket));
    }

    void set_socket_rx_read_pointer(socket_type socket, uint16_t pointer) noexcept
    {
        write_u16(Chip::socket_rx_read_pointer(socket), pointer);
    }

    void set_socket_tx_write_pointer(socket_type socket, uint16_t pointer) noexcept
    {
        write_u16(Chip::socket_tx_write_pointer(socket), pointer);
    }

    static address_type ring_skip(address_type pointer, uint16_t length) noexcept
    {
        return static_cast<address_type>(pointer + length);
    }

    // Bus address of a byte offset within a socket's RX / TX buffer.
    //
    // Two hardware layouts: parts with one shared buffer region carve it into
    // per-socket spans (`buffers_[socket].rx_offset` is where this socket's span
    // starts, filled by init_*()); parts with `Chip::per_socket_buffer_blocks()`
    // give each socket its own address block starting at 0.
    address_type rx_buffer_addr(socket_type socket, uint16_t offset) const noexcept
    {
        if constexpr (Chip::per_socket_buffer_blocks()) {
            return Chip::rx_buffer_block_addr(socket, offset);
        } else {
            return static_cast<address_type>(Chip::rx_buffer_base() + buffers_[socket].rx_offset + offset);
        }
    }

    address_type tx_buffer_addr(socket_type socket, uint16_t offset) const noexcept
    {
        if constexpr (Chip::per_socket_buffer_blocks()) {
            return Chip::tx_buffer_block_addr(socket, offset);
        } else {
            return static_cast<address_type>(Chip::tx_buffer_base() + buffers_[socket].tx_offset + offset);
        }
    }

    // Reads `length` bytes from the socket's RX ring starting at `pointer`. On
    // parts whose buffer hardware wraps on its own (`Chip::auto_wrap()`) the
    // pointer is handed over as-is; otherwise the read is split at the
    // buffer-size boundary here.
    address_type ring_read(socket_type socket,
                       address_type pointer,
                       uint8_t* data,
                       uint16_t length) noexcept
    {
        const uint16_t buf_size = socket_rx_capacity(socket);
        if ((buf_size == 0u) || (data == nullptr) || (length == 0u)) {
            return pointer;
        }

        if (Chip::auto_wrap()) {
            read(rx_buffer_addr(socket, static_cast<uint16_t>(pointer)), data, length);
            return ring_skip(pointer, length);
        }

        const uint16_t offset = static_cast<uint16_t>(pointer % buf_size);
        const uint16_t head = static_cast<uint16_t>(buf_size - offset);
        if (length > head) {
            read(rx_buffer_addr(socket, offset), data, head);
            read(rx_buffer_addr(socket, 0u), data + head, static_cast<uint16_t>(length - head));
        } else {
            read(rx_buffer_addr(socket, offset), data, length);
        }

        return ring_skip(pointer, length);
    }

    // Mirror of `ring_read` for the TX ring.
    address_type ring_write(socket_type socket,
                        address_type pointer,
                        const uint8_t* data,
                        uint16_t length) noexcept
    {
        if (Chip::auto_wrap()) {
            write(tx_buffer_addr(socket, static_cast<uint16_t>(pointer)), data, length);
            return ring_skip(pointer, length);
        }

        const uint16_t buffer_size = socket_tx_capacity(socket);
        const uint16_t addr = static_cast<uint16_t>(pointer % buffer_size);
        if (addr + length <= buffer_size) {
            write(tx_buffer_addr(socket, addr), data, length);
        } else {
            const uint16_t n = static_cast<uint16_t>(buffer_size - addr);
            write(tx_buffer_addr(socket, addr), data, n);
            write(tx_buffer_addr(socket, 0u), data + n, static_cast<uint16_t>(length - n));
        }

        return ring_skip(pointer, length);
    }

    Bus& bus_;
#ifndef ROSETTE_NO_FREERTOS
    TaskHandle_t interrupt_dispatch_task_ = nullptr;
    std::array<TaskHandle_t, Chip::socket_count()> socket_rx_tasks_{};
#endif
    mode_type mode_ = DeviceMode::uninitialized;

    // Per-socket buffer layout, configured once by the last init_*() and never
    // changed by the chip on its own — so it is cached here rather than re-read
    // over SPI on every packet. `*_offset` is the byte offset of this socket's
    // buffer within the chip's shared buffer region (used only when
    // `!Chip::per_socket_buffer_blocks()`); `*_size` is its capacity in bytes.
    struct SocketBuffer {
        uint16_t tx_offset = 0u;
        uint16_t tx_size = 0u;
        uint16_t rx_offset = 0u;
        uint16_t rx_size = 0u;
    };
    std::array<SocketBuffer, Chip::socket_count()> buffers_{};

    // A SEND was issued on this socket and its Sn_IR.SENDOK has not been
    // drained yet; see wait_pending_send().
    std::array<bool, Chip::socket_count()> send_pending_{};

    // The chip has flagged an IPv4 address conflict (its own SIPR seen in an
    // ARP from another MAC). Latched under the bus lock by poll_common_interrupt()
    // or service_socket_interrupts(); consumed by take_ip_conflict().
    bool ip_conflict_pending_ = false;
};

}  // namespace rosette::wiznet
