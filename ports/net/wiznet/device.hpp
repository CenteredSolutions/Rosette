#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <type_traits>
#include <variant>

#include "rosette/platform/platform.hpp"

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

/**
 * @brief One end of a hardware-socket UDP exchange: address family, address, port.
 *
 * The chip reports the sender of each received datagram in this form and
 * `WiznetDevice::udp_send_to()` takes the destination in it. `address` holds
 * the IPv4 address in bytes `[0..3]` when `ipv6` is false, all sixteen bytes
 * otherwise, in network byte order.
 */
struct UdpEndpoint {
    bool ipv6 = false;
    std::array<uint8_t, 16> address{};
    uint16_t port = 0u;

    /// @brief The IPv4 address in bytes `[0..3]`; meaningful when `!ipv6`.
    std::array<uint8_t, 4> ipv4() const noexcept
    {
        return {address[0], address[1], address[2], address[3]};
    }
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

    /// Receive buffer size of each socket, in KiB, in socket-number order.
    using RxLayout = std::array<uint8_t, Chip::socket_count()>;

    explicit WiznetDevice(Bus& bus) noexcept
        : bus_(bus) {}

    /**
     * @brief True when the chip can split its receive memory as `kib` says.
     *
     * Each socket's Sn_RXBUF_SIZE takes 0 or a power of two KiB no larger
     * than the chip's whole receive memory, and the sizes together must fit
     * in it; the chip lays the buffers out back to back in socket order.
     *
     * @param[in] kib Receive buffer KiB per socket.
     * @return `true` for a layout set_rx_layout() accepts.
     *
     * Verification:
     * - Correct means: every value is 0 or a power of two at most
     *   Chip::socket_rx_buffer_kib(), and the sum is at most that total.
     * - Covered by: TC-NET-WIZ-LAYOUT-001.
     *
     * References:
     * - W5100S datasheet §Sn_RXBUF_SIZE; W6100 / W6300 datasheets §Sn_RX_BSR
     */
    static constexpr bool is_valid_rx_layout(const RxLayout& kib) noexcept
    {
        uint32_t total = 0u;
        for (uint8_t size : kib) {
            if (size > Chip::socket_rx_buffer_kib() || (size & (size - 1u)) != 0u) {
                return false;
            }

            total += size;
        }

        return total <= Chip::socket_rx_buffer_kib();
    }

    /**
     * @brief Sets how the next init_socket_mode() splits receive memory across sockets.
     *
     * Datagrams wait in a socket's receive buffer until the host reads them,
     * and the chip drops any that no longer fit, so a socket that receives
     * bursts (RTPS user traffic: every fragment of a large sample arrives
     * back to back) needs a buffer several datagrams deep, while one that
     * carries a DHCP or SNTP exchange needs one datagram. The layout starts
     * as Chip::default_rx_layout_kib(); transmit memory stays split evenly,
     * since a socket holds one outgoing datagram at a time (udp_send_to()
     * waits for the previous SEND to complete). WiznetSocketProvider places
     * each socket on the smallest buffer that meets its
     * Socket::set_receive_buffer_size() request.
     *
     * @param[in] kib Receive buffer KiB per socket, in socket order.
     * @return `false`, leaving the layout unchanged, when is_valid_rx_layout()
     *   rejects `kib`.
     *
     * Verification:
     * - Correct means: after init_socket_mode(), socket_rx_capacity(s) is
     *   kib[s] KiB and the buffers are contiguous in socket order.
     * - Covered by: TC-NET-WIZ-LAYOUT-001.
     *
     * References:
     * - W5100S datasheet §Sn_RXBUF_SIZE; W6100 / W6300 datasheets §Sn_RX_BSR
     */
    bool set_rx_layout(const RxLayout& kib) noexcept
    {
        if (!is_valid_rx_layout(kib)) {
            return false;
        }

        rx_layout_kib_ = kib;
        return true;
    }

    /// @brief The receive layout the next init_socket_mode() applies.
    const RxLayout& rx_layout() const noexcept { return rx_layout_kib_; }

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

        // Transmit memory is split evenly; receive memory follows
        // rx_layout_kib_ (set_rx_layout()). Each socket's buffer offset is the
        // running sum of the sizes actually assigned, so the layout the driver
        // caches always matches the layout it wrote.
        const uint8_t tx_kib = Chip::socket_tx_buffer_kib() / Chip::socket_count();
        const uint16_t tx_size = static_cast<uint16_t>(tx_kib * 1024u);
        uint16_t tx_offset = 0u;
        uint16_t rx_offset = 0u;
        for (socket_type socket = 0; socket < Chip::socket_count(); ++socket) {
            const uint8_t rx_kib = rx_layout_kib_[socket];
            const uint16_t rx_size = static_cast<uint16_t>(rx_kib * 1024u);
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

    /**
     * @brief Programs the IPv4 address, subnet mask, and gateway (SIPR/SUBR/GAR).
     *
     * The values are also kept for ipv4_route(), and each call counts as a
     * new configuration, so the ARP cache built on the old one is discarded.
     *
     * @param[in] ip      Own address; all zero withdraws it.
     * @param[in] subnet  Subnet mask; all zero puts every destination on-link.
     * @param[in] gateway Default router; all zero when none.
     */
    void configure_ipv4(const std::array<uint8_t, 4>& ip,
                        const std::array<uint8_t, 4>& subnet,
                        const std::array<uint8_t, 4>& gateway) noexcept
    {
        const auto lock = bus_.acquire();
        write(Chip::common_gateway(), gateway.data(), gateway.size());
        write(Chip::common_subnet(), subnet.data(), subnet.size());
        write(Chip::common_ip_address(), ip.data(), ip.size());
        ipv4_ = Ipv4Config{ip, subnet, gateway};
        ipv4_generation_.store(ipv4_generation_.load(std::memory_order_relaxed) + 1u,
                               std::memory_order_release);
    }

    /**
     * @brief The count of configure_ipv4() calls, read without the bus lock.
     *
     * The ARP resolver checks it on every send and receive, and copies the
     * configuration with ipv4_route() only when it differs from the count it
     * last saw. A send therefore never waits on the bus lock, held for a
     * whole datagram transfer by another socket, just to learn that the
     * address is unchanged.
     *
     * @return The same count ipv4_route() returns.
     */
    uint32_t ipv4_generation() const noexcept {
        return ipv4_generation_.load(std::memory_order_acquire);
    }

    /**
     * @brief The IPv4 configuration last programmed by configure_ipv4(), without a bus transfer.
     *
     * @param[out] config Address, subnet mask, and gateway.
     * @return A count of configure_ipv4() calls; a different value than last
     *   time means the configuration changed.
     */
    uint32_t ipv4_route(Ipv4Config& config) noexcept {
        const auto lock = bus_.acquire();
        config = ipv4_;
        return ipv4_generation_.load(std::memory_order_relaxed);
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

    /**
     * @brief Programs the IPv6 link-local address (LLAR).
     *
     * Once written, the chip sources link-scope traffic from this address and
     * answers neighbor solicitations and echo requests for it. An all-zero
     * address withdraws it. Parts without `Chip::has_hw_ipv6()` do not provide
     * this member.
     *
     * @param[in] address fe80::/64 address, network byte order.
     */
    void configure_ipv6_link_local(const std::array<uint8_t, 16>& address) noexcept
    {
        const auto lock = bus_.acquire();
        write(Chip::common_ip6_link_local(), address.data(), address.size());
    }

    /**
     * @brief Reads the IPv6 link-local address (LLAR) back from the chip.
     *
     * @return The address the chip holds; all zero when none is programmed.
     */
    std::array<uint8_t, 16> ipv6_link_local() noexcept {
        std::array<uint8_t, 16> address{};
        const auto lock = bus_.acquire();
        read(Chip::common_ip6_link_local(), address.data(), address.size());
        return address;
    }

    /**
     * @brief Programs the IPv6 global address, on-link prefix mask, and default router.
     *
     * The chip delivers a destination that matches `address` under
     * `prefix_mask` directly on the link and sends every other global
     * destination to `gateway`. All-zero arguments withdraw the configuration.
     *
     * @param[in] address     Global unicast address (GUAR).
     * @param[in] prefix_mask On-link prefix as a mask, e.g. ffff:ffff:ffff:ffff:: (SUB6R).
     * @param[in] gateway     Default router, usually its link-local address (GA6R); zero when unknown.
     */
    void configure_ipv6_global(const std::array<uint8_t, 16>& address,
                               const std::array<uint8_t, 16>& prefix_mask,
                               const std::array<uint8_t, 16>& gateway) noexcept
    {
        const auto lock = bus_.acquire();
        write(Chip::common_ip6_gateway(), gateway.data(), gateway.size());
        write(Chip::common_ip6_prefix_mask(), prefix_mask.data(), prefix_mask.size());
        write(Chip::common_ip6_global(), address.data(), address.size());
    }

    /**
     * @brief Starts a socket-less ICMPv6 exchange (SLCR) without waiting for it.
     *
     * The W6100 and W6300 run neighbor solicitation (duplicate address
     * detection) and router solicitation as single register commands. This
     * writes the target to SLDIP6R, sets the retransmission timeout, clears any
     * stale completion bits, and issues the command; poll_socketless_status()
     * collects the outcome.
     *
     * The chip runs one socket-less command at a time, and ARP probes for the
     * ARP cache (start_arp_probe()) share it. A probe that has finished is
     * collected for poll_arp_probe() and the command is issued. A probe still
     * in flight is never waited for under the bus lock: the call returns
     * `false` and reserves the engine, so no new probe starts before the
     * caller's retry on a later poll. The reservation lapses after
     * kHostReserveMs, so a caller that gives up does not block ARP for good.
     *
     * @param[in] command    One `Chip::socketless_command_*()` value.
     * @param[in] target     Address the command concerns (the tentative address
     *   for NS); ignored by RS.
     * @param[in] timeout_ms Wait for an answer before SLIR.TOUT, 1..6553 ms.
     * @return `true` when the command was issued and the caller owns the
     *   engine until it completes; `false` when an ARP probe still holds the
     *   engine, in which case nothing was written and the caller retries.
     *
     * Verification:
     * - Correct means: the command is issued with a clear SLIR; an ARP probe
     *   in flight is neither overwritten nor waited for; no probe starts
     *   between a refused call and its retry; a finished probe still reports
     *   its own outcome.
     * - Covered by: tests/host/wiznet_slaac_provider_test.cpp;
     *   TC-NET-WIZ-ARP-015 (sharing with ARP probes).
     *
     * References:
     * - W6100 datasheet §SLCR, §SLIR, §SLRTR
     * - WIZnet io6Library wizchip_conf.c wizchip_dad(), wizchip_slaac()
     */
    bool start_socketless_command(uint8_t command, const std::array<uint8_t, 16>& target,
                                  uint16_t timeout_ms) noexcept {
        const auto lock = bus_.acquire();
        if (socketless_owner_ == SocketlessOwner::arp_probe) {
            collect_arp_probe_locked();
        }

        if (socketless_owner_ == SocketlessOwner::arp_probe) {
            host_reserved_until_ms_ = rosette::platform::monotonic_ms() + kHostReserveMs;
            host_reserved_ = true;
            return false;
        }

        host_reserved_ = false;
        host_status_latched_ = 0u;
        clear_socketless_status_locked(0xFFu);
        write_u16(Chip::common_socketless_retry_time(), static_cast<uint16_t>(timeout_ms * 10u));
        write_u8(Chip::common_socketless_retry_count(), 0u);
        write(Chip::common_socketless_peer_ip6(), target.data(), target.size());
        write_u8(Chip::common_socketless_command(), command);
        socketless_owner_ = SocketlessOwner::host;
        return true;
    }

    /**
     * @brief Reads and clears the socket-less status bits (SLIR) of a start_socketless_command().
     *
     * Bits that belong to an ARP probe are never returned: while a probe owns
     * the engine this returns only what was latched for the caller's own
     * command, and poll_arp_probe() collects the probe.
     *
     * @return The SLIR bits that were set: `*_neighbor_solicit` when a
     *   neighbor answered, `*_timeout` when none did (the command is over
     *   either way); 0 while a command is still in progress.
     */
    uint8_t poll_socketless_status() noexcept
    {
        const auto lock = bus_.acquire();
        uint8_t status = host_status_latched_;
        host_status_latched_ = 0u;
        if (socketless_owner_ == SocketlessOwner::arp_probe) {
            return status;
        }

        const uint8_t chip_status = read_u8(Chip::common_socketless_status());
        if (chip_status != 0u) {
            clear_socketless_status_locked(chip_status);
        }

        status = static_cast<uint8_t>(status | chip_status);
        if ((status & Chip::socketless_status_done_mask()) != 0u) {
            socketless_owner_ = SocketlessOwner::none;
        }

        return status;
    }

    /// @brief Outcome of a socket-less ARP probe; see poll_arp_probe().
    enum class ArpProbe : uint8_t {
        idle,        ///< No probe was started, or its outcome was already taken.
        pending,     ///< The chip is still waiting for a reply.
        answered,    ///< The target replied; the MAC is written.
        unanswered,  ///< No reply within the timeout and retries.
    };

    /**
     * @brief Starts a socket-less ARP request (SLCR ARP) for an IPv4 next hop.
     *
     * Backs the ARP cache (WiznetArpResolver): the chip broadcasts an ARP
     * request for `target` from its own SIPR/SHAR, without a socket, and
     * latches the reply's sender MAC in SLPHAR (W5100S) or SLDHAR (W6x00).
     * The probe does not start while another socket-less command owns the
     * engine (SLAAC's NS or RS on the W6x00) or has reserved it after a
     * refused start_socketless_command(); a completed one is latched for its
     * owner's poll_socketless_status() first.
     *
     * @param[in] target     Next-hop IPv4 address, network byte order.
     * @param[in] timeout_ms Wait per request, 1..6553 ms (SLRTR).
     * @param[in] retries    Requests after the first (SLRCR).
     * @return `true` when the request was issued; `false` when the engine is
     *   busy or reserved, or an earlier probe's outcome has not been collected.
     *
     * Verification:
     * - Correct means: SLPIPR/SLDIP4R holds `target` when SLCR is written,
     *   and no SLAAC command is overwritten.
     * - Covered by: TC-NET-WIZ-ARP-010, TC-NET-WIZ-ARP-015.
     *
     * References:
     * - W5100S datasheet §SLCR, §SLRTR, §SLRCR, §SLPIPR, §SLPHAR, §SLIR
     * - W6100 datasheet §SLCR (ARP4), §SLDIP4R, §SLDHAR; io6Library wizchip_arp()
     */
    bool start_arp_probe(const std::array<uint8_t, 4>& target, uint16_t timeout_ms,
                         uint8_t retries) noexcept {
        const auto lock = bus_.acquire();
        if (arp_outcome_ != ArpProbe::idle || socketless_owner_ == SocketlessOwner::arp_probe) {
            return false;
        }

        const bool reserved =
            host_reserved_
            && static_cast<int32_t>(rosette::platform::monotonic_ms() - host_reserved_until_ms_)
                   < 0;
        if (reserved) {
            return false;
        }

        if (socketless_owner_ == SocketlessOwner::host) {
            const uint8_t status = read_u8(Chip::common_socketless_status());
            if ((status & Chip::socketless_status_done_mask()) == 0u) {
                return false;
            }

            clear_socketless_status_locked(status);
            host_status_latched_ = static_cast<uint8_t>(host_status_latched_ | status);
        }

        clear_socketless_status_locked(Chip::socketless_status_done_mask());
        write_u16(Chip::common_socketless_retry_time(), static_cast<uint16_t>(timeout_ms * 10u));
        write_u8(Chip::common_socketless_retry_count(), retries);
        write(Chip::common_socketless_peer_ip4(), target.data(), target.size());
        write_u8(Chip::common_socketless_command(), Chip::socketless_command_arp4());
        socketless_owner_ = SocketlessOwner::arp_probe;

        // A deadline past the chip's own (timeout x attempts) ends a probe the
        // chip never completes, so the engine cannot stay claimed for ever.
        const uint32_t chip_ms = static_cast<uint32_t>(timeout_ms) * (retries + 1u);
        arp_deadline_ms_ = rosette::platform::monotonic_ms() + 2u * chip_ms + kArpDeadlineSlackMs;
        return true;
    }

    /**
     * @brief Collects the outcome of start_arp_probe().
     *
     * One SLIR read while the probe is pending; each outcome is returned
     * exactly once.
     *
     * @param[out] mac The replying host's MAC, written on `answered`.
     * @return See ArpProbe.
     */
    ArpProbe poll_arp_probe(std::array<uint8_t, 6>& mac) noexcept {
        const auto lock = bus_.acquire();
        if (arp_outcome_ == ArpProbe::idle && socketless_owner_ == SocketlessOwner::arp_probe) {
            collect_arp_probe_locked();
        }

        const ArpProbe outcome = arp_outcome_;
        if (outcome == ArpProbe::idle && socketless_owner_ == SocketlessOwner::arp_probe) {
            return ArpProbe::pending;
        }

        if (outcome == ArpProbe::answered) {
            mac = arp_mac_;
        }

        arp_outcome_ = ArpProbe::idle;
        return outcome;
    }

    /**
     * @brief Sets which ICMPv6 messages bypass the chip's handler (ICMP6BLKR).
     *
     * A blocked message type is delivered to an open IPRAW6 socket instead of
     * being processed by the chip, and dropped when none is open.
     *
     * @param[in] bits `Chip::icmp6_block_*()` values.
     */
    void set_icmp6_block(uint8_t bits) noexcept
    {
        const auto lock = bus_.acquire();
        write_u8(Chip::common_icmp6_block(), bits);
    }

    /**
     * @brief Opens a hardware socket in IPRAW6 mode for one IPv6 next header.
     *
     * The socket receives IPv6 payloads carrying `next_header` that the chip
     * does not consume itself, e.g. ICMPv6 (58) messages blocked with
     * set_icmp6_block(). Read them with ipraw_receive_from().
     *
     * @param[in] socket      Hardware socket index.
     * @param[in] next_header IPv6 next-header value (Sn_PNR).
     * @return `true` when the chip reports SOCK_IPRAW6.
     *
     * References: W6100 datasheet §Sn_MR IPRAW6, §Sn_PNR, §ICMP6BLKR.
     */
    bool open_ipraw6_socket(socket_type socket, uint8_t next_header) noexcept
    {
        const auto lock = bus_.acquire();
        close_socket(socket);
        set_socket_mode2(socket, 0u);
        write_u8(Chip::socket_mode(socket), Chip::mode_ipraw6());
        write_u8(Chip::socket_next_header(socket), next_header);
        issue_socket_command(socket, Chip::command_open());
        return read_u8(Chip::socket_status(socket)) == Chip::socket_status_ipraw6();
    }

    /**
     * @brief Reads one datagram from an IPRAW6 socket.
     *
     * @param[in]  socket   Socket opened by open_ipraw6_socket().
     * @param[out] data     IPv6 payload (e.g. the ICMPv6 message).
     * @param[in]  capacity Size of `data`.
     * @param[out] source   Sender address; may be null.
     * @return As udp_receive_from().
     */
    std::ptrdiff_t ipraw_receive_from(socket_type socket, uint8_t* data, std::size_t capacity,
                                      UdpEndpoint* source) noexcept
    {
        return receive_datagram(socket, data, capacity, source, [](auto&& read_next, UdpEndpoint& from) {
            return Chip::read_ipraw_rx_header(read_next, from);
        });
    }

    /**
     * @brief Length of the MACRAW frame at the head of the receive ring, without consuming it.
     *
     * The MACRAW socket stores each received frame behind a length header
     * (`Chip::macraw_rx_header_bytes()` octets, big-endian, counting itself),
     * and the received-size register counts every frame queued. A network
     * interface sizes its receive buffer from this call, so it decodes the
     * head frame's header rather than returning the queue total.
     *
     * A header shorter than itself, or longer than Sn_RX_RSR, means the ring
     * is not at a frame boundary (the chip counts a frame in Sn_RX_RSR only
     * once all of it is stored). Everything queued is then dropped by moving
     * Sn_RX_RD to Sn_RX_WR, as udp_receive_from() does, and counted in
     * rx_resyncs(). A well-formed frame longer than `Chip::max_frame_bytes()`
     * is skipped whole. Either way the caller is never asked to allocate for
     * it.
     *
     * @return Frame length in octets, without the length header, at most
     *   `Chip::max_frame_bytes()`; 0 when no complete header is queued or the
     *   head record was discarded.
     *
     * Verification:
     * - Correct means: with frames of lengths a and b queued, the call returns
     *   a, read_frame() then returns a, the call returns b, and the ring is
     *   unchanged by the call itself; after a header longer than the received
     *   size the ring is empty and the next frame reads intact.
     * - Covered by: tests/host/wiznet_macraw_device_test.cpp
     *   (TC-NET-WIZ-MACRAW-003, TC-NET-WIZ-MACRAW-004, TC-NET-WIZ-MACRAW-006).
     *
     * References:
     * - W5100S datasheet §Sn_RX_RSR, §Sn_RX_RD, §Sn_RX_WR, and the MACRAW receive data format
     * - W6100 / W6300 datasheets §Sn_RX_RSR, §Sn_RX_RD, §Sn_RX_WR, and the MACRAW PACKET INFO
     */
    std::size_t peek_frame_size() noexcept {
        const auto lock = bus_.acquire();
        const socket_type socket = Chip::macraw_socket();
        const uint16_t    available = socket_rx_received_size(socket);
        if (available < Chip::macraw_rx_header_bytes()) {
            return 0u;
        }

        uint8_t header[Chip::macraw_rx_header_bytes()] = {0};
        const uint16_t start = socket_rx_read_pointer(socket);
        const uint16_t after_header = static_cast<uint16_t>(
            ring_read(socket, start, header, static_cast<uint16_t>(sizeof(header))));
        const uint16_t total_len =
            static_cast<uint16_t>((static_cast<uint16_t>(header[0]) << 8) | header[1]);
        if ((total_len < Chip::macraw_rx_header_bytes()) || (total_len > available)) {
            resync_rx_ring_locked(socket);
            return 0u;
        }

        const uint16_t frame_len =
            static_cast<uint16_t>(total_len - Chip::macraw_rx_header_bytes());
        if ((frame_len > 0u) && (frame_len <= Chip::max_frame_bytes())) {
            return frame_len;
        }

        set_socket_rx_read_pointer(socket,
                                   static_cast<uint16_t>(ring_skip(after_header, frame_len)));
        issue_socket_command(socket, Chip::command_receive_done());
        return 0u;
    }

    /**
     * @brief Copies the MACRAW frame at the head of the receive ring into @p frame and frees its
     * space.
     *
     * The network interface calls this after peek_frame_size() has sized the
     * buffer. The RECV interrupt is cleared before the received size is read,
     * so a frame that lands afterwards raises a fresh INT edge. A frame larger
     * than @p capacity is skipped whole; a length header shorter than itself
     * or longer than Sn_RX_RSR drops everything queued, as peek_frame_size()
     * describes.
     *
     * @param[out] frame    Destination for the frame, without the length header.
     * @param[in]  capacity Size of @p frame in octets.
     * @return Frame length in octets; 0 when nothing is queued, or when the head
     *   record was skipped or the ring was resynchronised.
     *
     * Verification:
     * - Correct means: queued frames come back in order and intact, each call
     *   consumes exactly one record, and a misaligned ring is emptied.
     * - Covered by: tests/host/wiznet_macraw_device_test.cpp
     *   (TC-NET-WIZ-MACRAW-003, TC-NET-WIZ-MACRAW-006).
     *
     * References:
     * - W5100S datasheet §Sn_RX_RSR, §Sn_RX_RD, §Sn_RX_WR, and the MACRAW receive data format
     * - W6100 / W6300 datasheets §Sn_RX_RSR, §Sn_RX_RD, §Sn_RX_WR, and the MACRAW PACKET INFO
     */
    std::size_t read_frame(uint8_t* frame, std::size_t capacity) noexcept
    {
        const auto lock = bus_.acquire();
        const socket_type socket = Chip::macraw_socket();
        // Cleared before the size is read, so a frame landing after the read
        // raises a fresh INT edge for the caller's next wait.
        clear_socket_interrupt(socket, Chip::socket_interrupt_receive());
        const uint16_t available = socket_rx_received_size(socket);
        if ((frame == nullptr) || (capacity == 0u) ||
            (available < Chip::macraw_rx_header_bytes())) {
            return 0u;
        }

        uint8_t header[Chip::macraw_rx_header_bytes()] = {0};
        uint16_t pointer = socket_rx_read_pointer(socket);
        pointer = static_cast<uint16_t>(
            ring_read(socket, pointer, header, static_cast<uint16_t>(sizeof(header))));

        const uint16_t total_len =
            static_cast<uint16_t>((static_cast<uint16_t>(header[0]) << 8) | header[1]);
        // Off a frame boundary: see peek_frame_size().
        if ((total_len < Chip::macraw_rx_header_bytes()) || (total_len > available)) {
            resync_rx_ring_locked(socket);
            return 0u;
        }

        const uint16_t frame_len =
            static_cast<uint16_t>(total_len - Chip::macraw_rx_header_bytes());
        if ((frame_len == 0u) || (frame_len > capacity)) {
            pointer = static_cast<uint16_t>(ring_skip(pointer, frame_len));
            set_socket_rx_read_pointer(socket, pointer);
            issue_socket_command(socket, Chip::command_receive_done());
            return 0u;
        }

        pointer = static_cast<uint16_t>(ring_read(socket, pointer, frame, frame_len));
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
        pointer = static_cast<uint16_t>(ring_write(socket, pointer, frame, required));
        set_socket_tx_write_pointer(socket, pointer);
        issue_socket_command(socket, Chip::command_send());
        send_pending_[socket] = true;
        return true;
    }

    /**
     * @brief Opens a hardware socket in UDP mode on `local_port`.
     *
     * An IPv4 socket uses the chip's UDP4 mode; an IPv6 socket (parts with
     * `Chip::has_hw_ipv6()`) uses UDP6 mode, which sends and receives IPv6
     * only, from the link-local or global address the chip holds (LLAR / GUAR,
     * see configure_ipv6()). The chip picks the source address for each
     * datagram from the destination's scope.
     *
     * With `manual_dest_mac` on a part where the destination MAC is chosen
     * per socket (`Chip::dest_mac_per_socket()`, the W6x00), Sn_MR2.DHAM_MANUAL
     * is set before OPEN, so the chip sends every datagram to Sn_DHAR without
     * ARP, and every udp_send_to() on the socket must then pass its
     * destination's MAC. On the W5100S the MAC is chosen per send (SEND_MAC)
     * and the flag has no effect at open.
     *
     * @param[in] socket          Hardware socket index, below `Chip::socket_count()`.
     * @param[in] local_port      Source port, host byte order, nonzero.
     * @param[in] ipv6            `true` for an IPv6 socket.
     * @param[in] manual_dest_mac `true` when an ARP cache supplies each IPv4
     *   send's MAC; ignored for IPv6.
     * @return `true` when the chip reports the socket open in UDP mode;
     *   `false` on failure or when IPv6 is requested of a part without it.
     *
     * Verification:
     * - Correct means: Sn_SR reads SOCK_UDP after OPEN, and the socket then
     *   exchanges datagrams of the requested family only; on the W6x00,
     *   Sn_MR2 holds DHAM_MANUAL exactly when `manual_dest_mac` asked for it.
     * - Covered by: tests/host/wiznet_udp_socket_test.cpp (register sequence),
     *   TC-NET-WIZ-ARP-014 (Sn_MR2); examples/wiznet_udp_echo on the
     *   W6100-EVB-Pico (on the wire).
     *
     * References:
     * - W5100S datasheet §Sn_MR, §Sn_CR OPEN
     * - W6100 datasheet §Sn_MR (UDP4 0x02, UDP6 0x0A), §Sn_MR2 (DHAM), §Sn_DHAR
     */
    bool open_udp_socket(socket_type socket, uint16_t local_port, bool ipv6 = false,
                         bool manual_dest_mac = false) noexcept {
        if constexpr (!Chip::has_hw_ipv6()) {
            if (ipv6) return false;
        }
        const auto lock = bus_.acquire();
        close_socket(socket);
        // Reset Sn_DIPR so the chip's ARP-skip optimization can't fire on the
        // first send after a reopen. Per the WIZnet SEND command: if the
        // newly-written destination IP equals what Sn_DIPR already holds, the
        // chip skips ARP and reuses whatever MAC it resolved for that IP last
        // time (Sn_DHAR is just the cache the chip reuses; it plays no part
        // in the skip decision itself, and CLOSE/OPEN do not clear Sn_DIPR).
        // That is wrong the moment the peer's MAC has changed while its IP
        // stayed the same — e.g. the Agent host rebooted with a new virtual
        // NIC — since nothing about the reopened socket looks different to
        // the comparator. Zeroing Sn_DIPR here guarantees the real Agent
        // address, written moments later via configure_udp_peer(), reads as
        // a changed destination and forces a fresh ARP request.
        static constexpr std::array<uint8_t, 4> kNoDestinationAddress{};
        write(Chip::socket_dest_ip(socket), kNoDestinationAddress.data(),
              kNoDestinationAddress.size());
        uint8_t mode = Chip::mode_udp();
        if constexpr (Chip::has_hw_ipv6()) {
            // The same comparator guards neighbor discovery for Sn_DIP6R.
            static constexpr std::array<uint8_t, 16> kNoDestinationAddress6{};
            write(Chip::socket_dest_ip6(socket), kNoDestinationAddress6.data(),
                  kNoDestinationAddress6.size());
            if (ipv6) mode = Chip::mode_udp6();
        }

        set_socket_mode2(socket, manual_dest_mac && !ipv6 ? mode2_dest_mac_manual() : 0u);
        write_u8(Chip::socket_mode(socket), mode);
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
        set_socket_mode2(socket, 0u);
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

    /**
     * @brief Writes a UDP socket's destination registers.
     *
     * IPv4 goes to Sn_DIPR (plus the group MAC in Sn_DHAR for a multicast
     * group, which SEND_MAC uses); IPv6 goes to Sn_DIP6R.
     *
     * @param[in] socket Hardware socket index.
     * @param[in] peer   Destination; an IPv6 peer requires `Chip::has_hw_ipv6()`.
     * @return `false` when the peer's family is not supported by the part.
     */
    bool configure_udp_peer(socket_type socket, const UdpEndpoint& peer) noexcept
    {
        const auto lock = bus_.acquire();
        if (peer.ipv6) {
            if constexpr (Chip::has_hw_ipv6()) {
                write(Chip::socket_dest_ip6(socket), peer.address.data(), peer.address.size());
                write_u16(Chip::socket_dest_port(socket), peer.port);
                return true;
            }
            return false;
        }
        const std::array<uint8_t, 4> address = peer.ipv4();
        if (is_multicast_ipv4(address)) {
            const auto group_mac = multicast_mac_from_ipv4(address);
            write(Chip::socket_dest_hardware(socket), group_mac.data(), group_mac.size());
        }
        write(Chip::socket_dest_ip(socket), address.data(), address.size());
        write_u16(Chip::socket_dest_port(socket), peer.port);
        return true;
    }

    /**
     * @brief Sends one datagram from a UDP socket.
     *
     * Waits out the previous SEND on this socket (wait_pending_send()), writes
     * the destination, copies the payload into the TX ring, and issues the SEND
     * command that matches the destination: SEND for IPv4 unicast, SEND_MAC for
     * an IPv4 multicast group on parts that need it, SEND6 for IPv6. Completion
     * is collected by the next send on the socket.
     *
     * With `dest_mac` (from the ARP cache, WiznetArpResolver), an IPv4
     * datagram goes to that MAC with no ARP at all: it is written to Sn_DHAR
     * and sent with SEND_MAC on the W5100S, or with SEND on a W6x00 socket
     * opened with `manual_dest_mac`. SENDOK then follows in wire time, and the
     * Sn_IR.TIMEOUT of a failed ARP cannot occur.
     *
     * @param[in] socket   Hardware socket opened by open_udp_socket().
     * @param[in] data     Payload, `length` bytes, copied before return.
     * @param[in] length   Payload size; at most the socket's TX buffer.
     * @param[in] peer     Destination address, family, and port.
     * @param[in] dest_mac Next hop's MAC for an IPv4 `peer`, or null to let
     *   the chip ARP; required on a socket opened with `manual_dest_mac`.
     * @return `length` when the SEND was issued; -1 on bad arguments or a
     *   destination family the part does not support.
     *
     * Verification:
     * - Correct means: every call is exactly one datagram on the wire, to
     *   `peer`, from the family's own source address, and to `dest_mac` when
     *   one is given.
     * - Covered by: TC-NET-WIZ-ARP-010, TC-NET-WIZ-ARP-014;
     *   examples/rtps_chatter (IPv4), examples/wiznet_udp_echo (IPv4 and IPv6)
     *   on hardware.
     *
     * References:
     * - W5100S datasheet §Sn_CR SEND / SEND_MAC, §Sn_DHAR
     * - W6100 datasheet §Sn_CR SEND6 (0xA0), §Sn_DIP6R, §Sn_MR2 (DHAM), §Sn_DHAR
     */
    std::ptrdiff_t udp_send_to(socket_type socket, const uint8_t* data, std::size_t length,
                               const UdpEndpoint& peer,
                               const std::array<uint8_t, 6>* dest_mac = nullptr) noexcept {
        const auto lock = bus_.acquire();
        if ((data == nullptr) || (length == 0u) ||
            (length > static_cast<std::size_t>(socket_tx_capacity(socket)))) {
            std::printf("Invalid parameters for udp_send_to: data=%p length=%zu socket_tx_capacity=%u\n",
                        static_cast<const void*>(data), length, socket_tx_capacity(socket));
            return -1;
        }

        // Finish the previous datagram before touching Sn_DIPR/Sn_DPORT or the
        // TX ring: the chip reads both while a SEND is in progress, so either
        // write would redirect it or fuse the two payloads. Usually done by now.
        wait_pending_send(socket);

        if (!configure_udp_peer(socket, peer)) {
            return -1;
        }

        const bool to_given_mac = dest_mac != nullptr && !peer.ipv6;
        if (to_given_mac) {
            write(Chip::socket_dest_hardware(socket), dest_mac->data(), dest_mac->size());
        }

        const uint16_t required = static_cast<uint16_t>(length);
        while (socket_tx_free_size(socket) < required) {
        }

        uint16_t pointer = socket_tx_write_pointer(socket);
        pointer = static_cast<uint16_t>(ring_write(socket, pointer, data, required));
        set_socket_tx_write_pointer(socket, pointer);
        // A multicast destination has no ARP answer; SEND_MAC uses the group
        // MAC configure_udp_peer() wrote to Sn_DHAR instead.
        uint8_t command = Chip::command_send();
        if (peer.ipv6) {
            if constexpr (Chip::has_hw_ipv6()) {
                command = Chip::command_send6();
            }
        } else if ((to_given_mac || is_multicast_ipv4(peer.ipv4()))
                   && Chip::command_send_mac() != 0u) {
            command = Chip::command_send_mac();
        }
        issue_socket_command(socket, command);
        send_pending_[socket] = true;

        return static_cast<std::ptrdiff_t>(length);
    }

    /**
     * @brief Reads one received datagram from a UDP socket's RX ring.
     *
     * The chip stores each datagram as a packet-info header followed by the
     * payload. The header layout is chip-specific, so `Chip::read_udp_rx_header()`
     * decodes it: the W5100S writes `{ip[4], port[2], length[2]}`; the W6100 and
     * W6300 write `{info|length[2], ip[4 or 16], port[2]}`, where the info bits
     * say whether the sender is IPv4 or IPv6. The payload is then copied out, the
     * read pointer advanced past the whole datagram, and RECV issued so the chip
     * frees the space.
     *
     * The chip counts a datagram in Sn_RX_RSR only once all of it is stored, so
     * the header and payload of the datagram at the read pointer always fit in
     * the received size. A header that claims more is not at a datagram
     * boundary (a corrupted read, or the ring misaligned some other way), and
     * every later header would be payload octets. The ring is then put back on
     * a boundary by moving Sn_RX_RD to Sn_RX_WR, which drops whatever is
     * queued, and the event is counted in rx_resyncs().
     *
     * @param[in]  socket   Hardware socket opened by open_udp_socket().
     * @param[out] data     Payload destination, `capacity` bytes.
     * @param[in]  capacity Size of `data`.
     * @param[out] source   Sender address and port; may be null.
     * @return Payload length (>0); 0 when no datagram is waiting; -1 when the
     *   datagram was larger than `capacity` or empty, in which case it has been
     *   discarded, or when its header did not fit the received size, in which
     *   case everything queued has been discarded.
     *
     * Verification:
     * - Correct means: the payload, sender address, family, and port equal the
     *   datagram the peer sent, and the ring is left at the next datagram; after
     *   a header longer than the received size, the next call returns 0 and the
     *   next datagram the chip stores reads intact.
     * - Covered by: tests/host/wiznet_udp_socket_test.cpp (W5100S and W6100 /
     *   W6300 layouts, IPv4 and IPv6 senders, oversize discard,
     *   TC-NET-WIZ-RX-004 resync).
     *
     * References:
     * - W5100S datasheet v1.2.8 §5 (UDP receive, packet-info header),
     *   §3.2.18 Sn_RX_RSR (= Sn_RX_WR - Sn_RX_RD), §3.2.20 Sn_RX_WR
     * - W6100 datasheet §Sn_RX_RSR, §Sn_RX_WR
     * - WIZnet io6Library Ethernet/socket.c recvfrom() (W6100 PACK_INFO layout)
     */
    std::ptrdiff_t udp_receive_from(socket_type socket,
                                    uint8_t* data,
                                    std::size_t capacity,
                                    UdpEndpoint* source) noexcept
    {
        return receive_datagram(socket, data, capacity, source, [](auto&& read_next, UdpEndpoint& from) {
            return Chip::read_udp_rx_header(read_next, from);
        });
    }

    /// @brief UDP/MACRAW SENDs that ended in Sn_IR.TIMEOUT (ARP failure) since start-up.
    uint32_t send_timeouts() const noexcept { return send_timeouts_; }

    /// @brief Receives that found a record header larger than Sn_RX_RSR and moved Sn_RX_RD to
    /// Sn_RX_WR, since start-up.
    uint32_t rx_resyncs() const noexcept { return rx_resyncs_; }

    /**
     * @brief Clears the RECV interrupt of every socket the chip has flagged, so INT deasserts.
     *
     * INT stays low while any socket's interrupt is pending, and a falling
     * edge is what wakes a sleeping thread; a datagram left unread on one
     * socket would otherwise keep every other socket's datagram from raising
     * one. Clearing loses nothing: whether a socket holds data is its
     * received-size register, which each owner reads after clearing.
     *
     * @return Bit `n` set when socket `n` was flagged.
     *
     * References: W5100S datasheet §IR, §Sn_IR, §INT pin; W6100 datasheet §SIR, §Sn_IRCLR.
     */
    uint8_t clear_rx_interrupts() noexcept {
        const auto lock = bus_.acquire();
        const uint8_t flagged =
            static_cast<uint8_t>(read_u8(Chip::common_socket_interrupt()) & kAllSocketsMask);
        for (socket_type socket = 0; socket < Chip::socket_count(); ++socket) {
            if ((flagged & (1u << socket)) != 0u) {
                clear_socket_interrupt(socket, Chip::socket_interrupt_receive());
            }
        }

        return flagged;
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

    /// @brief True for an IPv4 multicast group (224.0.0.0/4).
    static bool is_multicast_ipv4(const std::array<uint8_t, 4>& address) noexcept
    {
        return (address[0] >= 224u) && (address[0] <= 239u);
    }

    /// @brief The Ethernet group address of an IPv4 multicast group (RFC 1112 §6.4).
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

private:
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

    // One datagram out of a socket's RX ring: `read_header` decodes the
    // chip's packet-info header (UDP or IPRAW layout), the record is checked
    // against the received size (udp_receive_from() says why), the payload is
    // copied or skipped when it does not fit, and RECV frees the space.
    template<typename ReadHeader>
    std::ptrdiff_t receive_datagram(socket_type socket, uint8_t* data, std::size_t capacity,
                                    UdpEndpoint* source, ReadHeader&& read_header) noexcept
    {
        const auto lock = bus_.acquire();
        const uint16_t available = socket_rx_received_size(socket);
        if ((data == nullptr) || (capacity == 0u) ||
            (available < Chip::udp_rx_header_bytes())) {
            return 0;
        }

        const uint16_t start = socket_rx_read_pointer(socket);
        uint16_t       pointer = start;
        UdpEndpoint sender{};
        const uint16_t payload_len = read_header(
            [&](uint8_t* out, uint16_t n) {
                pointer = static_cast<uint16_t>(ring_read(socket, pointer, out, n));
            },
            sender);
        const uint16_t header_len = static_cast<uint16_t>(pointer - start);
        if (static_cast<uint32_t>(header_len) + payload_len > available) {
            resync_rx_ring_locked(socket);
            return -1;
        }

        if (source != nullptr) {
            *source = sender;
        }

        if ((payload_len == 0u) || (payload_len > capacity)) {
            pointer = static_cast<uint16_t>(ring_skip(pointer, payload_len));
            set_socket_rx_read_pointer(socket, pointer);
            issue_socket_command(socket, Chip::command_receive_done());
            ack_socket_rx_interrupt(socket);
            return -1;
        }

        pointer = static_cast<uint16_t>(ring_read(socket, pointer, data, payload_len));
        set_socket_rx_read_pointer(socket, pointer);
        issue_socket_command(socket, Chip::command_receive_done());
        ack_socket_rx_interrupt(socket);
        return payload_len;
    }

    /**
     * @brief Puts a socket's RX ring back on a record boundary by dropping everything queued.
     *
     * Called when the header at Sn_RX_RD claims a record larger than Sn_RX_RSR.
     * Sn_RX_WR is always where the chip will store its next record, so setting
     * Sn_RX_RD to it and issuing RECV leaves the ring empty and aligned. A
     * record that lands between the read of Sn_RX_WR and the write of Sn_RX_RD
     * starts at the value written, so it is kept and reads intact. The caller
     * holds the bus lock.
     *
     * @param[in] socket Socket whose ring is misaligned.
     *
     * Verification:
     * - Correct means: afterwards Sn_RX_RSR reads 0 and the next record the
     *   chip stores is returned intact; rx_resyncs() has grown by one.
     * - Covered by: tests/host/wiznet_udp_socket_test.cpp (TC-NET-WIZ-RX-004),
     *   tests/host/wiznet_macraw_device_test.cpp (TC-NET-WIZ-MACRAW-006).
     *
     * References:
     * - W5100S datasheet v1.2.8 §3.2.16 Sn_RX_RD, §3.2.18 Sn_RX_RSR, §3.2.20 Sn_RX_WR
     * - W6100 datasheet §Sn_RX_RD, §Sn_RX_WR
     */
    void resync_rx_ring_locked(socket_type socket) noexcept {
        set_socket_rx_read_pointer(socket, socket_rx_write_pointer(socket));
        issue_socket_command(socket, Chip::command_receive_done());
        ack_socket_rx_interrupt(socket);
        ++rx_resyncs_;
    }

    // SLIRCLR (SLIR itself on the W5100S) is write-1-to-clear; the caller holds the bus lock.
    void clear_socketless_status_locked(uint8_t bits) noexcept
    {
        write_u8(Chip::common_socketless_status_clear(), bits);
    }

    /**
     * @brief Moves a finished ARP probe's result from SLIR into `arp_outcome_`.
     *
     * Reads SLIR once and never waits, since the caller holds the bus lock
     * that every socket needs. ARP's reply bit means answered; the timeout
     * bit, or the driver's own deadline passing, means unanswered, and the
     * engine is then free. With neither, the probe is still in flight and
     * nothing changes.
     */
    void collect_arp_probe_locked() noexcept {
        const uint8_t status = read_u8(Chip::common_socketless_status());
        const bool answered = (status & Chip::socketless_status_arp4()) != 0u;
        const bool timed_out = (status & Chip::socketless_status_timeout()) != 0u;
        const bool abandoned =
            static_cast<int32_t>(rosette::platform::monotonic_ms() - arp_deadline_ms_) >= 0;
        if (answered) {
            read(Chip::common_socketless_peer_hardware(), arp_mac_.data(), arp_mac_.size());
            arp_outcome_ = ArpProbe::answered;
        } else if (timed_out || abandoned) {
            arp_outcome_ = ArpProbe::unanswered;
        } else {
            return;
        }

        clear_socketless_status_locked(
            static_cast<uint8_t>(status & Chip::socketless_status_done_mask()));
        socketless_owner_ = SocketlessOwner::none;
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
     * The SEND ends with SENDOK, or with `Sn_IR.TIMEOUT` when ARP for the
     * destination fails (Sn_RTR x Sn_RCR, about 1.8 s at reset values); a
     * closed socket also ends the wait, so a dead peer cannot hang the caller.
     * Both bits are cleared here, as WIZnet's ioLibrary sendto() does. A
     * TIMEOUT left set would end every later wait at once, and each following
     * datagram would then be fused into the one still in flight.
     *
     * Verification:
     * - Correct means: on the wire, every udp_send_to() is exactly one
     *   datagram to its own destination, including after an ARP timeout.
     * - Covered by: examples/rtps_chatter against a peer that advertises an
     *   unreachable locator (send_timeouts() > 0 with no fused datagrams).
     *
     * References:
     * - W5100S datasheet §Sn_IR (SENDOK, TIMEOUT), §Sn_CR SEND; WIZnet
     *   ioLibrary socket.c sendto()
     */
    void wait_pending_send(socket_type socket) noexcept
    {
        while (send_pending_[socket]) {
            const uint8_t irq = read_u8(Chip::socket_interrupt(socket));
            if ((irq & Chip::socket_interrupt_send_ok()) != 0u) break;
            if ((irq & Chip::socket_interrupt_timeout()) != 0u) {
                ++send_timeouts_;
                break;
            }
            if (read_u8(Chip::socket_status(socket)) == Chip::socket_status_closed()) {
                break;
            }
        }
        send_pending_[socket] = false;
        clear_socket_interrupt(socket, static_cast<uint8_t>(Chip::socket_interrupt_send_ok() |
                                                            Chip::socket_interrupt_timeout()));
    }

    /**
     * @brief Clears the socket's RX interrupt once its datagram is read.
     *
     * A socket that no thread waits on would otherwise hold INT low, and no
     * other socket's datagram could raise an edge.
     */
    void ack_socket_rx_interrupt(socket_type socket) noexcept {
        clear_socket_interrupt(socket, Chip::socket_interrupt_receive());
    }

    // Sn_IR / IR are write-1-to-clear on the W5100S, but a separate write-only
    // *CLR register on the W61xx/W63xx. `Chip::socket_interrupt_clear()` /
    // `Chip::common_interrupt_clear()` name whichever register takes the ack.
    void clear_socket_interrupt(socket_type socket, uint8_t bits) noexcept
    {
        write_u8(Chip::socket_interrupt_clear(socket), bits);
    }

    // Sn_MR2 exists only where the destination MAC is chosen per socket
    // (W6x00). Every open writes it, since it keeps its value across CLOSE.
    void set_socket_mode2(socket_type socket, uint8_t bits) noexcept {
        if constexpr (Chip::dest_mac_per_socket()) {
            write_u8(Chip::socket_mode2(socket), bits);
        }
    }

    static constexpr uint8_t mode2_dest_mac_manual() noexcept {
        if constexpr (Chip::dest_mac_per_socket()) {
            return Chip::mode2_dest_mac_manual();
        } else {
            return 0u;
        }
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

    // Sn_RX_WR moves as the chip stores records, so it is read like Sn_RX_RSR.
    uint16_t socket_rx_write_pointer(socket_type socket) noexcept {
        return read_u16_stable(Chip::socket_rx_write_pointer(socket));
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

    static constexpr uint8_t kAllSocketsMask =
        static_cast<uint8_t>((1u << Chip::socket_count()) - 1u);

    Bus& bus_;
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

    // Receive KiB per socket for the next init_socket_mode(); see set_rx_layout().
    RxLayout rx_layout_kib_ = Chip::default_rx_layout_kib();

    // A SEND was issued on this socket and its Sn_IR.SENDOK has not been
    // drained yet; see wait_pending_send().
    std::array<bool, Chip::socket_count()> send_pending_{};
    uint32_t send_timeouts_ = 0u;
    uint32_t                               rx_resyncs_ = 0u;

    // The chip has flagged an IPv4 address conflict (its own SIPR seen in an
    // ARP from another MAC). Latched under the bus lock by poll_common_interrupt();
    // consumed by take_ip_conflict().
    bool ip_conflict_pending_ = false;

    // The last configure_ipv4() values and a count of the calls, so the ARP
    // resolver computes next hops without reading SIPR/SUBR/GAR over SPI.
    Ipv4Config ipv4_{};
    std::atomic<uint32_t> ipv4_generation_{0u};

    // Who the single socket-less engine (SLCR/SLIR) is running a command for.
    // `host` is start_socketless_command() (SLAAC); `arp_probe` is
    // start_arp_probe(). A completion read by the other user is latched in
    // `host_status_latched_` or `arp_outcome_` for its owner.
    enum class SocketlessOwner : uint8_t { none, host, arp_probe };
    SocketlessOwner socketless_owner_ = SocketlessOwner::none;
    uint8_t host_status_latched_ = 0u;
    ArpProbe arp_outcome_ = ArpProbe::idle;
    std::array<uint8_t, 6> arp_mac_{};
    uint32_t arp_deadline_ms_ = 0u;
    static constexpr uint32_t kArpDeadlineSlackMs = 50u;

    // A start_socketless_command() refused because a probe was in flight
    // reserves the engine until this deadline, so its retry is not beaten to
    // the engine by the next probe. Long enough to cover a few board polls.
    bool host_reserved_ = false;
    uint32_t host_reserved_until_ms_ = 0u;
    static constexpr uint32_t kHostReserveMs = 1000u;
};

}  // namespace rosette::wiznet
