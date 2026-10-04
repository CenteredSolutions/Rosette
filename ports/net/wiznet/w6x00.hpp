/**
 * @file net/wiznet/w6x00.hpp
 * @brief Register traits for the WIZnet W6100 and W6300 (1-bit SPI).
 *
 * The W6100 and W6300 share a register map, socket model (8 sockets, per-socket
 * TX/RX buffer blocks, hardware address auto-wrap) and bring-up sequence. They
 * differ only in:
 *
 *   - the SPI frame byte layout (see `make_header`),
 *   - the chip-version byte, and
 *   - the size of the socket buffer memory (16 KiB each on the W6100,
 *     32 KiB each on the W6300).
 *
 * `W6x00` carries everything common; `W6100` / `W6300` add the three differences.
 * Both are driven over an ordinary 4-wire SPI: on the W6300 this is QSPI in
 * single-lane mode, which is electrically identical. 4-bit QSPI is a separate
 * bus implementation (see `ports/chip/rp2/rp2_qspi_bus.hpp`) and does not change
 * these traits.
 *
 * Addresses are packed into a 24-bit value: `(offset << 8) | block`, where
 * `block` is the raw block-select number (common = 0, socket N registers =
 * 1+4N, socket N TX buffer = 2+4N, socket N RX buffer = 3+4N). `make_header`
 * unpacks that into the wire frame.
 *
 * Sources: WIZnet io(6)Library `Ethernet/W6100/w6100.{c,h}` and
 * `Ethernet/W6300/w6300.{c,h}` (WIZCHIP_READ / WIZCHIP_WRITE and the register
 * `#define`s), cross-checked against the W6100 / W6300 datasheets.
 */

#pragma once

#include <array>
#include <cstdint>

#include "device.hpp"

namespace rosette::net::wiznet {

struct W6x00 : ChipBase {
    using address_type = uint32_t;  // packed (offset << 8) | block
    using socket_type = uint8_t;
    using Header = ChipBase::Header;

    static constexpr socket_type socket_count() { return 8u; }
    static constexpr socket_type macraw_socket() { return 0u; }
    static constexpr bool auto_wrap() { return true; }

    // ── address packing ─────────────────────────────────────────────────────
    static constexpr uint8_t common_block() { return 0x00u; }
    static constexpr uint8_t socket_reg_block(socket_type s) { return static_cast<uint8_t>(1u + 4u * s); }
    static constexpr uint8_t socket_tx_block(socket_type s) { return static_cast<uint8_t>(2u + 4u * s); }
    static constexpr uint8_t socket_rx_block(socket_type s) { return static_cast<uint8_t>(3u + 4u * s); }

    static constexpr address_type reg(uint16_t offset, uint8_t block)
    {
        return (static_cast<address_type>(offset) << 8) | block;
    }

    // ── common registers ────────────────────────────────────────────────────
    static constexpr address_type common_version() { return reg(0x0000u, common_block()); }  // CIDR
    static constexpr address_type common_mac_address() { return reg(0x4120u, common_block()); }
    static constexpr address_type common_gateway() { return reg(0x4130u, common_block()); }
    static constexpr address_type common_subnet() { return reg(0x4134u, common_block()); }
    static constexpr address_type common_ip_address() { return reg(0x4138u, common_block()); }
    static constexpr address_type common_interrupt() { return reg(0x2100u, common_block()); }        // IR   (read)
    static constexpr address_type common_interrupt_clear() { return reg(0x2108u, common_block()); }  // IRCLR (write)
    static constexpr address_type common_socket_interrupt_mask() { return reg(0x2114u, common_block()); }  // SIMR
    static constexpr address_type common_link_status() { return reg(0x3000u, common_block()); }      // PHYSR
    static constexpr uint8_t link_up_mask() { return 0x01u; }                                        // PHYSR_LNK_UP

    // IR (0x2100) bit 2, IR_IPCONF: set when an ARP reply carries our own SIPR.
    // Same register `common_interrupt()` names; acked through IRCLR (0x2108).
    static constexpr uint8_t common_ip_conflict_mask() { return 0x04u; }  // IR_IPCONF

    // SIR (0x2101): bit n set while socket n has an interrupt pending.
    static constexpr address_type common_socket_interrupt() { return reg(0x2101u, common_block()); }

    // ── IPv6 (hardware dual stack) ──────────────────────────────────────────
    // The chip answers neighbor solicitations and echo requests for the
    // addresses in LLAR / GUAR on its own; the host programs the addresses and
    // runs the socket-less commands (SLCR) that probe for duplicates (NS) and
    // solicit a router advertisement (RS). Router advertisements are read by
    // the host from an IPRAW6 socket: the chip's own RA prefix latch
    // (PLR/PFR/VLTR/PLTR/PAR) decodes the prefix option at the wrong offset
    // unless an 8-octet option precedes it, so it is not used.
    static constexpr bool has_hw_ipv6() { return true; }

    static constexpr address_type common_ip6_link_local() { return reg(0x4140u, common_block()); }  // LLAR
    static constexpr address_type common_ip6_global() { return reg(0x4150u, common_block()); }      // GUAR
    static constexpr address_type common_ip6_prefix_mask() { return reg(0x4160u, common_block()); } // SUB6R
    static constexpr address_type common_ip6_gateway() { return reg(0x4170u, common_block()); }     // GA6R

    static constexpr address_type common_socketless_status() { return reg(0x2102u, common_block()); }        // SLIR
    static constexpr address_type common_socketless_status_clear() { return reg(0x2128u, common_block()); }  // SLIRCLR
    static constexpr address_type common_socketless_command() { return reg(0x2130u, common_block()); }       // SLCR
    static constexpr address_type common_socketless_peer_ip6() { return reg(0x4180u, common_block()); }      // SLDIP6R
    static constexpr address_type common_socketless_retry_time() { return reg(0x4208u, common_block()); }    // SLRTR, 100 us units
    static constexpr address_type common_socketless_retry_count() { return reg(0x420Cu, common_block()); }   // SLRCR

    // ICMPv6 messages set here bypass the chip's handler and reach an IPRAW6 socket.
    static constexpr address_type common_icmp6_block() { return reg(0x41F0u, common_block()); }  // ICMP6BLKR
    static constexpr uint8_t icmp6_block_router_advert() { return 0x04u; }                       // ICMP6BLKR_RA

    static constexpr uint8_t socketless_command_neighbor_solicit() { return 0x04u; }  // SLCR_NS (DAD probe)
    static constexpr uint8_t socketless_command_router_solicit() { return 0x02u; }    // SLCR_RS
    static constexpr uint8_t socketless_status_timeout() { return 0x80u; }            // SLIR_TOUT
    static constexpr uint8_t socketless_status_neighbor_solicit() { return 0x04u; }   // SLIR_NS: a neighbor answered

    // ── IPv4 ARP without a socket, and sends without ARP ────────────────────
    // WiznetDevice::start_arp_probe() resolves a next hop's MAC for the ARP
    // cache (arp_cache.hpp), and a socket opened with Sn_MR2.DHAM_MANUAL then
    // sends every datagram to the Sn_DHAR the cache supplies, without ARP
    // (WiznetDevice::open_udp_socket()). The W6x00 has no SEND_MAC.

    /// SLDIP4R: the IPv4 address an SLCR ARP4 request asks for.
    static constexpr address_type common_socketless_peer_ip4() {
        return reg(0x418Cu, common_block());
    }

    /// SLDHAR: the MAC of the host that answered.
    static constexpr address_type common_socketless_peer_hardware() {
        return reg(0x4190u, common_block());
    }

    /// SLCR_ARP4 and SLIR_ARP4 (the host answered).
    static constexpr uint8_t socketless_command_arp4() { return 0x40u; }

    static constexpr uint8_t socketless_status_arp4() { return 0x40u; }

    /// SLIR bits that end a socket-less command: all but SLIR_RA, which
    /// reports a received router advertisement rather than a completion.
    static constexpr uint8_t socketless_status_done_mask() { return 0xFEu; }

    /// Sn_MR2, written before OPEN; DHAM_MANUAL makes SEND use Sn_DHAR.
    static constexpr address_type socket_mode2(socket_type s) {
        return reg(0x0144u, socket_reg_block(s));
    }

    static constexpr uint8_t mode2_dest_mac_manual() { return 0x02u; }

    /// The destination MAC is chosen per socket (Sn_MR2.DHAM), not per SEND.
    static constexpr bool dest_mac_per_socket() { return true; }

    // ── socket registers ────────────────────────────────────────────────────
    static constexpr address_type socket_mode(socket_type s) { return reg(0x0000u, socket_reg_block(s)); }
    static constexpr address_type socket_command(socket_type s) { return reg(0x0010u, socket_reg_block(s)); }
    static constexpr address_type socket_interrupt(socket_type s) { return reg(0x0020u, socket_reg_block(s)); }        // Sn_IR    (read)
    static constexpr address_type socket_interrupt_clear(socket_type s) { return reg(0x0028u, socket_reg_block(s)); }  // Sn_IRCLR (write)
    static constexpr address_type socket_interrupt_mask(socket_type s) { return reg(0x0024u, socket_reg_block(s)); }
    static constexpr address_type socket_status(socket_type s) { return reg(0x0030u, socket_reg_block(s)); }
    static constexpr address_type socket_source_port(socket_type s) { return reg(0x0114u, socket_reg_block(s)); }
    static constexpr address_type socket_dest_hardware(socket_type s) { return reg(0x0118u, socket_reg_block(s)); }
    static constexpr address_type socket_dest_ip(socket_type s) { return reg(0x0120u, socket_reg_block(s)); }
    static constexpr address_type socket_dest_ip6(socket_type s) { return reg(0x0130u, socket_reg_block(s)); }        // Sn_DIP6R
    static constexpr address_type socket_dest_port(socket_type s) { return reg(0x0140u, socket_reg_block(s)); }
    static constexpr address_type socket_next_header(socket_type s) { return reg(0x0100u, socket_reg_block(s)); }     // Sn_PNR (IPRAW)
    static constexpr address_type socket_tx_buffer_size(socket_type s) { return reg(0x0200u, socket_reg_block(s)); }   // Sn_TX_BSR
    static constexpr address_type socket_tx_free_size(socket_type s) { return reg(0x0204u, socket_reg_block(s)); }     // Sn_TX_FSR
    static constexpr address_type socket_tx_write_pointer(socket_type s) { return reg(0x020Cu, socket_reg_block(s)); } // Sn_TX_WR
    static constexpr address_type socket_rx_buffer_size(socket_type s) { return reg(0x0220u, socket_reg_block(s)); }   // Sn_RX_BSR
    static constexpr address_type socket_rx_received_size(socket_type s) { return reg(0x0224u, socket_reg_block(s)); } // Sn_RX_RSR
    static constexpr address_type socket_rx_read_pointer(socket_type s) { return reg(0x0228u, socket_reg_block(s)); }  // Sn_RX_RD

    // Each socket's TX / RX buffer is its own address block; the offset within
    // it is the raw pointer and the chip wraps at the buffer size (auto_wrap).
    static constexpr bool per_socket_buffer_blocks() { return true; }

    static constexpr address_type tx_buffer_block_addr(socket_type s, uint16_t offset)
    {
        return reg(offset, socket_tx_block(s));
    }

    static constexpr address_type rx_buffer_block_addr(socket_type s, uint16_t offset)
    {
        return reg(offset, socket_rx_block(s));
    }

    // ── socket mode / command / interrupt / status ──────────────────────────
    static constexpr uint8_t mode_udp() { return 0x02u; }         // Sn_MR_UDP4
    static constexpr uint8_t mode_udp6() { return 0x0Au; }        // Sn_MR_UDP6
    static constexpr uint8_t mode_ipraw6() { return 0x0Bu; }      // Sn_MR_IPRAW6
    static constexpr uint8_t mode_multicast() { return 0x80u; }   // Sn_MR_MULTI
    // Sn_MR_MC (bit 5) selects the IGMP version: 0 = IGMPv2 (the default we
    // want), 1 = IGMPv1. Nothing to OR in for v2.
    static constexpr uint8_t mode_igmp_v2() { return 0x00u; }
    static constexpr uint8_t mode_macraw() { return 0x07u; }              // Sn_MR_MACRAW
    static constexpr uint8_t mode_mac_filter_enable() { return 0x80u; }   // Sn_MR_MF
    static constexpr uint8_t macraw_mode()
    {
        return static_cast<uint8_t>(mode_macraw() | mode_mac_filter_enable());
    }

    static constexpr uint8_t command_open() { return 0x01u; }
    static constexpr uint8_t command_send() { return 0x20u; }
    static constexpr uint8_t command_send6() { return 0xA0u; }  // Sn_CR_SEND6: destination in Sn_DIP6R

    /// No SEND_MAC (0 = send with SEND); see dest_mac_per_socket().
    static constexpr uint8_t command_send_mac() { return 0x00u; }
    static constexpr uint8_t command_close() { return 0x10u; }
    static constexpr uint8_t command_receive_done() { return 0x40u; }  // Sn_CR_RECV

    static constexpr uint8_t socket_interrupt_send_ok() { return 0x10u; }
    static constexpr uint8_t socket_interrupt_timeout() { return 0x08u; }
    static constexpr uint8_t socket_interrupt_receive() { return 0x04u; }

    static constexpr uint8_t socket_status_closed() { return 0x00u; }  // SOCK_CLOSED
    static constexpr uint8_t socket_status_udp() { return 0x22u; }     // SOCK_UDP
    static constexpr uint8_t socket_status_ipraw6() { return 0x33u; }  // SOCK_IPRAW6

    static constexpr uint16_t macraw_rx_header_bytes() { return 2u; }
    /// Smallest UDP packet-info header: info/length, an IPv4 address, a port.
    static constexpr uint16_t udp_rx_header_bytes() { return 8u; }

    /**
     * @brief Decodes the packet-info header the chip stores before each UDP datagram.
     *
     * Layout (io6Library socket.c recvfrom()): two octets whose top five bits
     * are packet info and low eleven bits the payload length, then the sender's
     * address (4 octets, or 16 when info bit 7, PACK_IPv6, is set), then the
     * sender's port, big-endian.
     *
     * @param[in]  read_next `void(uint8_t* out, uint16_t n)`: reads the next `n`
     *   octets of the RX ring.
     * @param[out] source    Sender family, address, and port.
     * @return Payload length in octets.
     */
    template<typename ReadNext>
    static uint16_t read_udp_rx_header(ReadNext&& read_next, UdpEndpoint& source)
    {
        uint8_t info[2] = {0u, 0u};
        read_next(info, 2u);
        source.ipv6 = (info[0] & 0x80u) != 0u;
        source.address = {};
        read_next(source.address.data(), source.ipv6 ? 16u : 4u);
        uint8_t port[2] = {0u, 0u};
        read_next(port, 2u);
        source.port = static_cast<uint16_t>((port[0] << 8) | port[1]);
        return static_cast<uint16_t>(((info[0] & 0x07u) << 8) | info[1]);
    }

    /**
     * @brief Decodes the packet-info header before each IPRAW6 datagram.
     *
     * The UDP layout without the port: info/length (2 octets), then the
     * sender's IPv6 address (16).
     *
     * @param[in]  read_next `void(uint8_t* out, uint16_t n)`.
     * @param[out] source    Sender address; `port` is 0.
     * @return Payload length in octets (the IPv6 payload, e.g. the ICMPv6 message).
     */
    template<typename ReadNext>
    static uint16_t read_ipraw_rx_header(ReadNext&& read_next, UdpEndpoint& source)
    {
        uint8_t info[2] = {0u, 0u};
        read_next(info, 2u);
        source.ipv6 = (info[0] & 0x80u) != 0u;
        source.address = {};
        read_next(source.address.data(), source.ipv6 ? 16u : 4u);
        source.port = 0u;
        return static_cast<uint16_t>(((info[0] & 0x07u) << 8) | info[1]);
    }
    static constexpr uint16_t max_frame_bytes() { return 1536u; }

    /**
     * @brief Unlock + soft reset.
     *
     * The W61xx/W63xx gate the system- and network-config registers behind lock
     * registers. Unlock the chip, pulse `SYCR0.RST`, then unlock the network
     * registers so `SHAR` / `SIPR` / `GAR` / `SUBR` are writable for the rest of
     * bring-up (they stay unlocked — nothing here re-locks them).
     *
     * The caller issues a register read immediately after (the chip-version
     * check), which covers the post-reset settle time.
     *
     * @p write_u8 is `void(address_type, uint8_t)`.
     */
    template<typename WriteU8>
    static void reset(WriteU8&& write_u8)
    {
        write_u8(reg(0x41F4u, common_block()), 0xCEu);  // CHPLCKR <- CHIP unlock
        write_u8(reg(0x2004u, common_block()), 0x00u);  // SYCR0   <- SYCR0_RST
        write_u8(reg(0x41F5u, common_block()), 0x3Au);  // NETLCKR <- NET unlock
    }
};

/// WIZnet W6100 — 16 KiB TX / 16 KiB RX socket memory.
struct W6100 final : W6x00 {
    static constexpr uint8_t chip_version() { return 0x61u; }  // CIDR high byte (0x6100)
    static constexpr uint8_t socket_tx_buffer_kib() { return 16u; }
    static constexpr uint8_t socket_rx_buffer_kib() { return 16u; }

    /// Receive buffer KiB per socket in socket mode (WiznetDevice::set_rx_layout()):
    /// half the memory for one bulk socket, 2 KiB for a medium one, 1 KiB for the rest.
    static constexpr std::array<uint8_t, 8> default_rx_layout_kib()
    {
        return {8u, 2u, 1u, 1u, 1u, 1u, 1u, 1u};
    }

    /// Frame: `{ addr[15:8], addr[7:0], (BSB << 3) | R/W }`, then data.
    /// `_W6100_SPI_WRITE_ == 0x01 << 2`; operation mode (VDM) is 0.
    template<bool IsRead>
    static constexpr Header make_header(address_type addr)
    {
        const uint8_t block = static_cast<uint8_t>(addr & 0xFFu);
        const uint8_t rwb = IsRead ? 0x00u : 0x04u;
        return Header{
            .bytes = {
                static_cast<uint8_t>(addr >> 16),
                static_cast<uint8_t>(addr >> 8),
                static_cast<uint8_t>((block << 3) | rwb),
            },
            .length = 3u,
        };
    }
};

/// WIZnet W6300 — 32 KiB TX / 32 KiB RX socket memory.
struct W6300 final : W6x00 {
    static constexpr uint8_t chip_version() { return 0x63u; }  // CIDR high byte (0x6300)
    static constexpr uint8_t socket_tx_buffer_kib() { return 32u; }
    static constexpr uint8_t socket_rx_buffer_kib() { return 32u; }

    /// Receive buffer KiB per socket in socket mode (WiznetDevice::set_rx_layout()):
    /// half the memory for one bulk socket, 4 KiB for a medium one, 2 KiB for the rest.
    static constexpr std::array<uint8_t, 8> default_rx_layout_kib()
    {
        return {16u, 4u, 2u, 2u, 2u, 2u, 2u, 2u};
    }

    /// Frame: `{ OP-code, addr[15:8], addr[7:0] }`, then data — OP-code first.
    /// OP-code = `QMODE[7:6] | R/W[5] | BSB[4:0]`; `_W6300_SPI_WRITE_ == 0x01 << 5`,
    /// QMODE = 0 for single-lane (1-bit) SPI.
    template<bool IsRead>
    static constexpr Header make_header(address_type addr)
    {
        const uint8_t block = static_cast<uint8_t>(addr & 0xFFu);
        const uint8_t rwb = IsRead ? 0x00u : 0x20u;
        return Header{
            .bytes = {
                static_cast<uint8_t>(block | rwb),
                static_cast<uint8_t>(addr >> 16),
                static_cast<uint8_t>(addr >> 8),
            },
            .length = 3u,
        };
    }
};

}  // namespace rosette::net::wiznet
