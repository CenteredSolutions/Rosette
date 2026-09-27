#pragma once

/**
 * @file tests/host/fake_wiznet_chip.hpp
 * @brief A register-file model of a WIZnet chip, behind the `Bus` interface.
 *
 * `FakeWiznetChip<Chip>` decodes each SPI frame the driver sends (the address
 * phase `Chip::make_header()` builds), stores register and buffer bytes, and
 * acts on the writes that have side effects on a real part: socket commands
 * (OPEN in UDP or IPRAW6 mode, CLOSE, SEND, SEND6, SEND_MAC, RECV), the interrupt-clear registers,
 * and the socket-less command register of the W6100/W6300. Received-size
 * reads are computed from what the test delivered, so `WiznetDevice` runs its
 * real receive and send paths, ring arithmetic included, against it.
 *
 * Tests inject datagrams with `deliver()` (the exact octets the chip would
 * store in the socket's RX ring, packet-info header first), inspect datagrams
 * the driver sent in `sent`, and script socket-less outcomes with
 * `socketless_reply`.
 *
 * Only the behaviour the driver depends on is modelled; the model's buffer
 * layout matches `WiznetDevice::init_socket_mode()` (2 KiB per socket).
 */

#include <cstddef>
#include <cstdint>
#include <map>
#include <vector>

#include "rosette/net/wiznet/w5100s.hpp"
#include "rosette/net/wiznet/w6x00.hpp"

namespace fake {

namespace wiz = rosette::net::wiznet;

/// @brief One datagram the driver asked the chip to send.
struct SentDatagram {
    uint8_t socket = 0u;
    uint8_t command = 0u;
    std::vector<uint8_t> dest_ip;    ///< Sn_DIPR (4) at SEND time.
    std::vector<uint8_t> dest_ip6;   ///< Sn_DIP6R (16) at SEND time; empty on parts without IPv6.
    uint16_t dest_port = 0u;
    std::vector<uint8_t> payload;
};

/// @brief One socket-less command the driver issued (W6100/W6300 SLCR).
struct SocketlessCommand {
    uint8_t command = 0u;
    std::vector<uint8_t> target;     ///< SLDIP6R at issue time.
    uint16_t retry_time = 0u;        ///< SLRTR at issue time, 100 us units.
};

template<typename Chip>
class FakeWiznetChip {
public:
    using address_type = typename Chip::address_type;
    static constexpr uint16_t kSocketBufferBytes = 2048u;

    std::map<address_type, uint8_t> mem;
    std::vector<SentDatagram> sent;
    std::vector<SocketlessCommand> socketless;
    /// SLIR bits raised by the next socket-less command.
    uint8_t socketless_reply = 0u;

    FakeWiznetChip() { mem[Chip::common_version()] = Chip::chip_version(); }

    struct Lock { ~Lock() {} };
    Lock acquire() noexcept { return {}; }

    void write(const uint8_t* prefix, std::size_t, const uint8_t* data, std::size_t len)
    {
        const address_type base = decode(prefix);
        if (len == 1u && on_write_u8(base, data[0])) return;
        for (std::size_t i = 0; i < len; ++i) mem[step(base, i)] = data[i];
    }

    void read(const uint8_t* prefix, std::size_t, uint8_t* data, std::size_t len)
    {
        const address_type base = decode(prefix);
        for (uint8_t s = 0; s < Chip::socket_count(); ++s) {
            if (len == 2u && base == Chip::socket_rx_received_size(s)) {
                const uint16_t n = static_cast<uint16_t>(rx_written_[s] - get_u16(Chip::socket_rx_read_pointer(s)));
                data[0] = static_cast<uint8_t>(n >> 8);
                data[1] = static_cast<uint8_t>(n);
                return;
            }
            if (len == 2u && base == Chip::socket_tx_free_size(s)) {
                data[0] = static_cast<uint8_t>(kSocketBufferBytes >> 8);
                data[1] = static_cast<uint8_t>(kSocketBufferBytes);
                return;
            }
        }
        for (std::size_t i = 0; i < len; ++i) {
            const auto it = mem.find(step(base, i));
            data[i] = it == mem.end() ? 0u : it->second;
        }
    }

    /// @brief Appends `bytes` to socket `s`'s RX ring, as the chip does on receipt.
    void deliver(uint8_t s, const std::vector<uint8_t>& bytes)
    {
        for (uint8_t b : bytes) {
            mem[rx_byte_address(s, rx_written_[s])] = b;
            rx_written_[s] = static_cast<uint16_t>(rx_written_[s] + 1u);
        }
        mem[Chip::socket_interrupt(s)] |= Chip::socket_interrupt_receive();
    }

    /// @brief Starts socket `s`'s RX ring at `pointer` (to exercise wrap-around).
    void set_rx_pointer(uint8_t s, uint16_t pointer)
    {
        rx_written_[s] = pointer;
        set_u16(Chip::socket_rx_read_pointer(s), pointer);
    }

    uint8_t u8(address_type a) const
    {
        const auto it = mem.find(a);
        return it == mem.end() ? 0u : it->second;
    }

    std::vector<uint8_t> bytes(address_type a, std::size_t n) const
    {
        std::vector<uint8_t> out(n);
        for (std::size_t i = 0; i < n; ++i) out[i] = u8(step(a, i));
        return out;
    }

    uint16_t get_u16(address_type a) const
    {
        return static_cast<uint16_t>((u8(a) << 8) | u8(step(a, 1u)));
    }

private:
    static constexpr bool kPacked = Chip::per_socket_buffer_blocks();   // W6x00 (offset << 8) | block

    static address_type decode(const uint8_t* p)
    {
        if constexpr (std::is_same_v<Chip, wiz::W6300>) {
            return (static_cast<address_type>(p[1]) << 16) | (static_cast<address_type>(p[2]) << 8) |
                   (p[0] & 0x1Fu);
        } else if constexpr (kPacked) {
            return (static_cast<address_type>(p[0]) << 16) | (static_cast<address_type>(p[1]) << 8) |
                   (p[2] >> 3);
        } else {
            return (static_cast<address_type>(p[1]) << 8) | p[2];
        }
    }

    // The address of the i-th byte of a burst. Buffer blocks of the W6x00 wrap
    // at the socket buffer size, as the hardware does (Chip::auto_wrap()).
    static address_type step(address_type base, std::size_t i)
    {
        if constexpr (kPacked) {
            const uint8_t block = static_cast<uint8_t>(base & 0xFFu);
            uint32_t offset = static_cast<uint32_t>(base >> 8) + static_cast<uint32_t>(i);
            if (block != 0u && (block % 4u) != 1u) offset %= kSocketBufferBytes;
            return (static_cast<address_type>(offset) << 8) | block;
        } else {
            return static_cast<address_type>(base + i);
        }
    }

    static address_type rx_byte_address(uint8_t s, uint16_t pointer)
    {
        if constexpr (kPacked) {
            return Chip::rx_buffer_block_addr(s, static_cast<uint16_t>(pointer % kSocketBufferBytes));
        } else {
            return static_cast<address_type>(Chip::rx_buffer_base() + s * kSocketBufferBytes +
                                             pointer % kSocketBufferBytes);
        }
    }

    static address_type tx_byte_address(uint8_t s, uint16_t pointer)
    {
        if constexpr (kPacked) {
            return Chip::tx_buffer_block_addr(s, static_cast<uint16_t>(pointer % kSocketBufferBytes));
        } else {
            return static_cast<address_type>(Chip::tx_buffer_base() + s * kSocketBufferBytes +
                                             pointer % kSocketBufferBytes);
        }
    }

    void set_u16(address_type a, uint16_t v)
    {
        mem[a] = static_cast<uint8_t>(v >> 8);
        mem[step(a, 1u)] = static_cast<uint8_t>(v);
    }

    // Side-effecting single-byte writes; returns true when the byte is not stored.
    bool on_write_u8(address_type a, uint8_t v)
    {
        for (uint8_t s = 0; s < Chip::socket_count(); ++s) {
            if (a == Chip::socket_command(s)) {
                on_socket_command(s, v);
                return true;
            }
            if constexpr (kPacked) {
                if (a == Chip::socket_interrupt_clear(s)) {
                    mem[Chip::socket_interrupt(s)] &= static_cast<uint8_t>(~v);
                    return true;
                }
            } else if (a == Chip::socket_interrupt(s)) {    // W5100S: write-1-to-clear in place
                mem[a] &= static_cast<uint8_t>(~v);
                return true;
            }
        }
        if constexpr (Chip::has_hw_ipv6()) {
            if (a == Chip::common_socketless_status_clear()) {
                mem[Chip::common_socketless_status()] &= static_cast<uint8_t>(~v);
                return true;
            }
            if (a == Chip::common_socketless_command()) {
                socketless.push_back({v, bytes(Chip::common_socketless_peer_ip6(), 16u),
                                      get_u16(Chip::common_socketless_retry_time())});
                mem[Chip::common_socketless_status()] |= socketless_reply;
                return true;
            }
        }
        return false;
    }

    void on_socket_command(uint8_t s, uint8_t command)
    {
        if (command == Chip::command_open()) {
            mem[Chip::socket_status(s)] = Chip::socket_status_udp();
            if constexpr (Chip::has_hw_ipv6()) {
                if (u8(Chip::socket_mode(s)) == Chip::mode_ipraw6()) {
                    mem[Chip::socket_status(s)] = Chip::socket_status_ipraw6();
                }
            }
        } else if (command == Chip::command_close()) {
            mem[Chip::socket_status(s)] = Chip::socket_status_closed();
        } else if (command == Chip::command_receive_done()) {
            // The driver has already advanced Sn_RX_RD; the received size follows it.
        } else if (is_send(command)) {
            SentDatagram d{};
            d.socket = s;
            d.command = command;
            d.dest_ip = bytes(Chip::socket_dest_ip(s), 4u);
            if constexpr (Chip::has_hw_ipv6()) d.dest_ip6 = bytes(Chip::socket_dest_ip6(s), 16u);
            d.dest_port = get_u16(Chip::socket_dest_port(s));
            const uint16_t end = get_u16(Chip::socket_tx_write_pointer(s));
            for (uint16_t p = tx_read_[s]; p != end; ++p) d.payload.push_back(u8(tx_byte_address(s, p)));
            tx_read_[s] = end;
            sent.push_back(d);
            mem[Chip::socket_interrupt(s)] |= Chip::socket_interrupt_send_ok();
        }
    }

    static bool is_send(uint8_t command)
    {
        if (command == Chip::command_send()) return true;
        if (Chip::command_send_mac() != 0u && command == Chip::command_send_mac()) return true;
        if constexpr (Chip::has_hw_ipv6()) {
            if (command == Chip::command_send6()) return true;
        }
        return false;
    }

    uint16_t rx_written_[Chip::socket_count()] = {};
    uint16_t tx_read_[Chip::socket_count()] = {};
};

}  // namespace fake
