/**
 * @file net/wiznet/w5500.hpp
 * @brief Register traits for the WIZnet W5500 (SPI, variable-length data mode).
 *
 * The W5500 combines two layouts the other traits already describe:
 *
 *   - its SPI frame and memory map follow the W6x00: a 16-bit offset, then a
 *     control byte `(BSB << 3) | (R/W << 2) | OM` selecting one of 32 blocks
 *     (common = 0, socket N registers = 1+4N, socket N TX buffer = 2+4N,
 *     socket N RX buffer = 3+4N), and each socket's buffer is its own block
 *     whose offset the chip wraps at the buffer size;
 *   - its register offsets within each block, socket modes, commands, and
 *     the UDP packet-info header follow the W5100S, including SEND_MAC.
 *
 * Addresses are packed as in w6x00.hpp, `(offset << 8) | block`.
 *
 * The W5500 has 8 sockets sharing 16 KiB of TX and 16 KiB of RX memory, no
 * IPv6, and no socket-less commands (SLCR): it cannot ARP without a socket,
 * so `has_socketless_arp()` is false and the board runs no ARP cache. A
 * unicast SEND then ARPs inline whenever a socket's destination changes, and
 * a multicast SEND uses SEND_MAC with the group MAC, so it never ARPs.
 *
 * Sources: W5500 datasheet v1.1.0 §2.2 (SPI frame), §3 (memory map), §4.1
 * (common registers), §4.2 (socket registers), §5 (socket buffers);
 * WIZnet ioLibrary_Driver Ethernet/W5500/w5500.{c,h}.
 */

#pragma once

#include <array>
#include <cstdint>

#include "device.hpp"

namespace rosette::net::wiznet {

struct W5500 final : ChipBase {
    using address_type = uint32_t;  // packed (offset << 8) | block
    using socket_type = uint8_t;
    using Header = ChipBase::Header;

    static constexpr socket_type socket_count() { return 8u; }
    static constexpr socket_type macraw_socket() { return 0u; }
    static constexpr bool auto_wrap() { return true; }

    // ── address packing ─────────────────────────────────────────────────────
    static constexpr uint8_t common_block() { return 0x00u; }

    /// @brief Socket `s`'s block of kind `kind`: 1 registers, 2 TX buffer, 3 RX buffer (§3).
    static constexpr uint8_t socket_block(socket_type s, uint8_t kind) {
        return static_cast<uint8_t>(kind + 4u * s);
    }

    static constexpr uint8_t socket_reg_block(socket_type s) { return socket_block(s, 1u); }
    static constexpr uint8_t socket_tx_block(socket_type s) { return socket_block(s, 2u); }
    static constexpr uint8_t socket_rx_block(socket_type s) { return socket_block(s, 3u); }

    static constexpr address_type reg(uint16_t offset, uint8_t block) {
        return (static_cast<address_type>(offset) << 8) | block;
    }

    /// @brief A common register at `offset` (§4.1).
    static constexpr address_type common(uint16_t offset) { return reg(offset, common_block()); }

    /// @brief Socket `s`'s register at `offset`, the datasheet's `Sn_` registers (§4.2).
    static constexpr address_type sn(uint16_t offset, socket_type s) {
        return reg(offset, socket_reg_block(s));
    }

    // ── common registers (§4.1) ─────────────────────────────────────────────
    static constexpr uint8_t chip_version() { return 0x04u; }                         // VERSIONR
    static constexpr auto common_mode() { return common(0x0000u); }                   // MR
    static constexpr auto common_gateway() { return common(0x0001u); }                // GAR
    static constexpr auto common_subnet() { return common(0x0005u); }                 // SUBR
    static constexpr auto common_mac_address() { return common(0x0009u); }            // SHAR
    static constexpr auto common_ip_address() { return common(0x000Fu); }             // SIPR
    static constexpr auto common_interrupt() { return common(0x0015u); }              // IR
    static constexpr auto common_interrupt_clear() { return common_interrupt(); }     // IR, W1C
    static constexpr auto common_socket_interrupt() { return common(0x0017u); }       // SIR
    static constexpr auto common_socket_interrupt_mask() { return common(0x0018u); }  // SIMR
    static constexpr auto common_link_status() { return common(0x002Eu); }            // PHYCFGR
    static constexpr auto common_version() { return common(0x0039u); }                // VERSIONR

    // SIR bit n is set while socket n has an interrupt pending; Sn_IR clears it. IR bit 7
    // (CONFLICT) is set when an ARP reply carries our own SIPR from another MAC.
    static constexpr uint8_t link_up_mask() { return 0x01u; }             // PHYCFGR.LNK
    static constexpr uint8_t common_ip_conflict_mask() { return 0x80u; }  // IR.CONFLICT

    // ── capabilities ────────────────────────────────────────────────────────
    static constexpr bool has_hw_ipv6() { return false; }

    /// No SLCR: the chip cannot ARP without a socket, so no ARP cache runs on it.
    static constexpr bool has_socketless_arp() { return false; }

    /// The destination MAC is chosen per SEND (SEND_MAC), not per socket.
    static constexpr bool dest_mac_per_socket() { return false; }

    // ── socket registers (§4.2) ─────────────────────────────────────────────
    // Sn_MR, CR, IR, SR, PORT, DHAR, DIPR, DPORT, RXBUF_SIZE, TXBUF_SIZE, TX_FSR, TX_WR,
    // RX_RSR, RX_RD, IMR; Sn_IR is write-1-to-clear in place.
    static constexpr auto socket_mode(socket_type s) { return sn(0x0000u, s); }
    static constexpr auto socket_command(socket_type s) { return sn(0x0001u, s); }
    static constexpr auto socket_interrupt(socket_type s) { return sn(0x0002u, s); }
    static constexpr auto socket_interrupt_clear(socket_type s) { return sn(0x0002u, s); }
    static constexpr auto socket_status(socket_type s) { return sn(0x0003u, s); }
    static constexpr auto socket_source_port(socket_type s) { return sn(0x0004u, s); }
    static constexpr auto socket_dest_hardware(socket_type s) { return sn(0x0006u, s); }
    static constexpr auto socket_dest_ip(socket_type s) { return sn(0x000Cu, s); }
    static constexpr auto socket_dest_port(socket_type s) { return sn(0x0010u, s); }
    static constexpr auto socket_rx_buffer_size(socket_type s) { return sn(0x001Eu, s); }
    static constexpr auto socket_tx_buffer_size(socket_type s) { return sn(0x001Fu, s); }
    static constexpr auto socket_tx_free_size(socket_type s) { return sn(0x0020u, s); }
    static constexpr auto socket_tx_write_pointer(socket_type s) { return sn(0x0024u, s); }
    static constexpr auto socket_rx_received_size(socket_type s) { return sn(0x0026u, s); }
    static constexpr auto socket_rx_read_pointer(socket_type s) { return sn(0x0028u, s); }
    static constexpr auto socket_rx_write_pointer(socket_type s) { return sn(0x002Au, s); }
    static constexpr auto socket_interrupt_mask(socket_type s) { return sn(0x002Cu, s); }

    // ── socket buffers (§5) ─────────────────────────────────────────────────
    // Each socket's TX / RX buffer is its own block; the offset within it is
    // the raw pointer, and the chip wraps at the buffer size (auto_wrap).
    static constexpr bool per_socket_buffer_blocks() { return true; }

    static constexpr address_type tx_buffer_block_addr(socket_type s, uint16_t offset) {
        return reg(offset, socket_tx_block(s));
    }

    static constexpr address_type rx_buffer_block_addr(socket_type s, uint16_t offset) {
        return reg(offset, socket_rx_block(s));
    }

    static constexpr uint8_t socket_tx_buffer_kib() { return 16u; }
    static constexpr uint8_t socket_rx_buffer_kib() { return 16u; }

    /// Receive buffer KiB per socket in socket mode (WiznetDevice::set_rx_layout()):
    /// half the memory for one bulk socket, 2 KiB for a medium one, 1 KiB for the rest.
    static constexpr std::array<uint8_t, 8> default_rx_layout_kib() {
        return {8u, 2u, 1u, 1u, 1u, 1u, 1u, 1u};
    }

    // ── socket mode / command / interrupt / status ──────────────────────────
    // Sn_MR.MC (bit 5) selects the IGMP version: 0 = IGMPv2, 1 = IGMPv1.
    static constexpr uint8_t mode_udp() { return 0x02u; }                // Sn_MR.P
    static constexpr uint8_t mode_multicast() { return 0x80u; }          // Sn_MR.MULTI (UDP)
    static constexpr uint8_t mode_igmp_v2() { return 0x00u; }            // Sn_MR.MC
    static constexpr uint8_t mode_macraw() { return 0x04u; }             // Sn_MR.P
    static constexpr uint8_t mode_mac_filter_enable() { return 0x80u; }  // Sn_MR.MFEN (MACRAW)

    static constexpr uint8_t macraw_mode() {
        return static_cast<uint8_t>(mode_macraw() | mode_mac_filter_enable());
    }

    // SEND_MAC (UDP) transmits to Sn_DHAR without ARP (§4.2 Sn_CR).
    static constexpr uint8_t command_open() { return 0x01u; }
    static constexpr uint8_t command_send() { return 0x20u; }
    static constexpr uint8_t command_send_mac() { return 0x21u; }
    static constexpr uint8_t command_close() { return 0x10u; }
    static constexpr uint8_t command_receive_done() { return 0x40u; }

    static constexpr uint8_t socket_interrupt_send_ok() { return 0x10u; }
    static constexpr uint8_t socket_interrupt_timeout() { return 0x08u; }
    static constexpr uint8_t socket_interrupt_receive() { return 0x04u; }

    static constexpr uint8_t socket_status_closed() { return 0x00u; }
    static constexpr uint8_t socket_status_udp() { return 0x22u; }

    /// Software reset: MR.RST, which clears itself when the reset is done.
    /// The caller's next register read (the version check) covers the settle time.
    template <typename WriteU8>
    static void reset(WriteU8&& write_u8) {
        write_u8(common_mode(), 0x80u);
    }

    static constexpr uint16_t macraw_rx_header_bytes() { return 2u; }
    static constexpr uint16_t udp_rx_header_bytes() { return 8u; }

    /**
     * @brief Decodes the packet-info header the chip stores before each UDP datagram.
     *
     * Layout (§5, UDP receive; the W5100S layout): sender IPv4 address (4
     * octets), sender port (2), payload length (2), all big-endian.
     *
     * @param[in]  read_next `void(uint8_t* out, uint16_t n)`: reads the next `n`
     *   octets of the RX ring.
     * @param[out] source    Sender address and port; always IPv4.
     * @return Payload length in octets.
     */
    template <typename ReadNext>
    static uint16_t read_udp_rx_header(ReadNext&& read_next, UdpEndpoint& source) {
        uint8_t header[8] = {0u};
        read_next(header, 8u);
        source.ipv6 = false;
        source.address = {header[0], header[1], header[2], header[3]};
        source.port = static_cast<uint16_t>((header[4] << 8) | header[5]);
        return static_cast<uint16_t>((header[6] << 8) | header[7]);
    }

    static constexpr uint16_t max_frame_bytes() { return 1536u; }

    /// Frame (§2.2): `{ addr[15:8], addr[7:0], (BSB << 3) | (RWB << 2) | OM }`,
    /// then data; RWB = 1 writes, OM = 00 is variable-length data mode.
    template <bool IsRead>
    static constexpr Header make_header(address_type addr) {
        const uint8_t block = static_cast<uint8_t>(addr & 0xFFu);
        const uint8_t rwb = IsRead ? 0x00u : 0x04u;
        return Header{
            .bytes =
                {
                    static_cast<uint8_t>(addr >> 16),
                    static_cast<uint8_t>(addr >> 8),
                    static_cast<uint8_t>((block << 3) | rwb),
                },
            .length = 3u,
        };
    }
};

}  // namespace rosette::net::wiznet
