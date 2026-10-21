#pragma once

/**
 * @file tests/host/fake_wiznet_chip.hpp
 * @brief A register-file model of a WIZnet chip, behind the `Bus` interface.
 *
 * `FakeWiznetChip<Chip>` decodes each SPI frame the driver sends (the address
 * phase `Chip::make_header()` builds), stores register and buffer bytes, and
 * acts on the writes that have side effects on a real part: socket commands
 * (OPEN in UDP or IPRAW6 mode, CLOSE, SEND, SEND6, SEND_MAC, RECV), the interrupt-clear registers,
 * and the socket-less command register (ARP on every part; NS and RS on the
 * W6100/W6300). Received-size
 * reads are computed from what the test delivered, so `WiznetDevice` runs its
 * real receive and send paths, ring arithmetic included, against it.
 *
 * Tests inject datagrams with `deliver()` (the exact octets the chip would
 * store in the socket's RX ring, packet-info header first), inspect datagrams
 * the driver sent in `sent`, and script socket-less outcomes with
 * `socketless_reply` (NS, RS) and `arp_hosts` (ARP: the hosts that answer,
 * after `arp_reply_reads` reads of SLIR).
 *
 * Only the behaviour the driver depends on is modelled; the model's buffer
 * layout matches `WiznetDevice::init_socket_mode()` (2 KiB per socket).
 */

#include <array>
#include <cstddef>
#include <cstdint>
#include <map>
#include <vector>

#include "ports/net/wiznet/w5100s.hpp"
#include "ports/net/wiznet/w6x00.hpp"

namespace fake {

namespace wiz = rosette::net::wiznet;

/// @brief One datagram the driver asked the chip to send.
struct SentDatagram {
    uint8_t socket = 0u;
    uint8_t command = 0u;
    std::vector<uint8_t> dest_ip;    ///< Sn_DIPR (4) at SEND time.
    std::vector<uint8_t> dest_ip6;   ///< Sn_DIP6R (16) at SEND time; empty on parts without IPv6.
    uint16_t dest_port = 0u;
    std::vector<uint8_t> dest_mac;  ///< Sn_DHAR (6) at SEND time.
    std::vector<uint8_t> payload;
};

/// @brief One socket-less command the driver issued (SLCR).
struct SocketlessCommand {
    uint8_t command = 0u;
    std::vector<uint8_t>
        target;  ///< SLDIP4R / SLPIPR (4) for ARP, else SLDIP6R (16), at issue time.
    uint16_t retry_time = 0u;        ///< SLRTR at issue time, 100 us units.
};

template<typename Chip>
class FakeWiznetChip {
public:
    using address_type = typename Chip::address_type;
    /// Buffer size of a socket whose Sn_RXBUF_SIZE / Sn_TXBUF_SIZE was never
    /// written: 2 KiB, the reset value on the W5100S, W6100 and W6300.
    static constexpr uint8_t kResetBufferKib = 2u;

    std::map<address_type, uint8_t> mem;
    std::vector<SentDatagram> sent;
    std::vector<SocketlessCommand> socketless;
    /// SLIR bits raised by the next socket-less command other than ARP.
    uint8_t socketless_reply = 0u;
    /// Hosts that answer an ARP request, by IPv4 address; others time out.
    std::map<std::array<uint8_t, 4>, std::array<uint8_t, 6>> arp_hosts;
    /// SLIR reads an ARP request stays in progress before its outcome is raised.
    unsigned arp_reply_reads = 0u;

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
        if (len == 1u && base == Chip::common_socketless_status() && arp_pending_) {
            if (arp_reads_left_ == 0u) {
                finish_arp();
            } else {
                --arp_reads_left_;
            }
        }

        for (uint8_t s = 0; s < Chip::socket_count(); ++s) {
            if (len == 2u && base == Chip::socket_rx_received_size(s)) {
                const uint16_t n = static_cast<uint16_t>(rx_written_[s] - get_u16(Chip::socket_rx_read_pointer(s)));
                data[0] = static_cast<uint8_t>(n >> 8);
                data[1] = static_cast<uint8_t>(n);
                return;
            }
            if (len == 2u && base == Chip::socket_tx_free_size(s)) {
                const uint16_t free = tx_bytes(s);
                data[0] = static_cast<uint8_t>(free >> 8);
                data[1] = static_cast<uint8_t>(free);
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
            return static_cast<address_type>((static_cast<address_type>(p[1]) << 8) | p[2]);
        }
    }

    /// Socket `s`'s buffer size in bytes, from the Sn_RXBUF_SIZE / Sn_TXBUF_SIZE
    /// register the driver wrote, as the chip sizes it.
    uint16_t buffer_bytes(address_type size_register) const
    {
        const auto it = mem.find(size_register);
        const uint8_t kib = it == mem.end() ? kResetBufferKib : it->second;
        return static_cast<uint16_t>(kib * 1024u);
    }

    uint16_t rx_bytes(uint8_t s) const { return buffer_bytes(Chip::socket_rx_buffer_size(s)); }
    uint16_t tx_bytes(uint8_t s) const { return buffer_bytes(Chip::socket_tx_buffer_size(s)); }

    /// Byte offset of socket `s`'s buffer in the W5100S's shared region: the
    /// chip lays the buffers out back to back in socket order.
    uint16_t linear_offset(uint8_t s, bool rx) const
    {
        uint16_t offset = 0u;
        for (uint8_t i = 0; i < s; ++i) {
            offset = static_cast<uint16_t>(offset + (rx ? rx_bytes(i) : tx_bytes(i)));
        }

        return offset;
    }

    // The address of the i-th byte of a burst. Buffer blocks of the W6x00
    // (block select 4s+2 for TX, 4s+3 for RX) wrap at that socket's buffer
    // size, as the hardware does (Chip::auto_wrap()).
    address_type step(address_type base, std::size_t i) const
    {
        if constexpr (kPacked) {
            const uint8_t block = static_cast<uint8_t>(base & 0xFFu);
            uint32_t offset = static_cast<uint32_t>(base >> 8) + static_cast<uint32_t>(i);
            if (block != 0u && (block % 4u) != 1u) {
                const uint8_t s = static_cast<uint8_t>(block / 4u);
                offset %= (block % 4u) == 3u ? rx_bytes(s) : tx_bytes(s);
            }

            return (static_cast<address_type>(offset) << 8) | block;
        } else {
            return static_cast<address_type>(base + i);
        }
    }

    address_type rx_byte_address(uint8_t s, uint16_t pointer) const
    {
        if constexpr (kPacked) {
            return Chip::rx_buffer_block_addr(s, static_cast<uint16_t>(pointer % rx_bytes(s)));
        } else {
            return static_cast<address_type>(Chip::rx_buffer_base() + linear_offset(s, true)
                                             + pointer % rx_bytes(s));
        }
    }

    address_type tx_byte_address(uint8_t s, uint16_t pointer) const
    {
        if constexpr (kPacked) {
            return Chip::tx_buffer_block_addr(s, static_cast<uint16_t>(pointer % tx_bytes(s)));
        } else {
            return static_cast<address_type>(Chip::tx_buffer_base() + linear_offset(s, false)
                                             + pointer % tx_bytes(s));
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

        if (a == Chip::common_socketless_status_clear()) {
            mem[Chip::common_socketless_status()] &= static_cast<uint8_t>(~v);
            return true;
        }

        if (a == Chip::common_socketless_command()) {
            on_socketless_command(v);
            return true;
        }

        return false;
    }

    void on_socketless_command(uint8_t command) {
        const uint16_t retry_time = get_u16(Chip::common_socketless_retry_time());
        if (command == Chip::socketless_command_arp4()) {
            socketless.push_back(
                {command, bytes(Chip::common_socketless_peer_ip4(), 4u), retry_time});
            arp_pending_ = true;
            arp_reads_left_ = arp_reply_reads;
            if (arp_reads_left_ == 0u) {
                finish_arp();
            }

            return;
        }

        if constexpr (Chip::has_hw_ipv6()) {
            socketless.push_back(
                {command, bytes(Chip::common_socketless_peer_ip6(), 16u), retry_time});
            mem[Chip::common_socketless_status()] |= socketless_reply;
        }
    }

    // Raises the outstanding ARP request's outcome: the reply bit and the
    // host's MAC in the peer hardware register, or the timeout bit.
    void finish_arp() {
        arp_pending_ = false;
        const auto ip = bytes(Chip::common_socketless_peer_ip4(), 4u);
        const auto it = arp_hosts.find({ip[0], ip[1], ip[2], ip[3]});
        if (it == arp_hosts.end()) {
            mem[Chip::common_socketless_status()] |= Chip::socketless_status_timeout();
            return;
        }

        for (std::size_t i = 0; i < 6u; ++i) {
            mem[step(Chip::common_socketless_peer_hardware(), i)] = it->second[i];
        }

        mem[Chip::common_socketless_status()] |= Chip::socketless_status_arp4();
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
            d.dest_mac = bytes(Chip::socket_dest_hardware(s), 6u);
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

    bool arp_pending_ = false;
    unsigned arp_reads_left_ = 0u;
    uint16_t rx_written_[Chip::socket_count()] = {};
    uint16_t tx_read_[Chip::socket_count()] = {};
};

}  // namespace fake
