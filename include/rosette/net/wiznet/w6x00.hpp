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
 * bus implementation (see `platform/bus_pico_qspi.hpp`) and does not change
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

#include "wiznet/device.hpp"

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
    static constexpr address_type socket_dest_port(socket_type s) { return reg(0x0140u, socket_reg_block(s)); }
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
    static constexpr uint8_t command_close() { return 0x10u; }
    static constexpr uint8_t command_receive_done() { return 0x40u; }  // Sn_CR_RECV

    static constexpr uint8_t socket_interrupt_send_ok() { return 0x10u; }
    static constexpr uint8_t socket_interrupt_timeout() { return 0x08u; }
    static constexpr uint8_t socket_interrupt_receive() { return 0x04u; }

    static constexpr uint8_t socket_status_closed() { return 0x00u; }  // SOCK_CLOSED
    static constexpr uint8_t socket_status_udp() { return 0x22u; }     // SOCK_UDP

    static constexpr uint16_t macraw_rx_header_bytes() { return 2u; }
    static constexpr uint16_t udp_rx_header_bytes() { return 8u; }
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
