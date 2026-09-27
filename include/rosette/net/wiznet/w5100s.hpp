#pragma once

#include <array>
#include <cstdint>

#include "wiznet/device.hpp"

namespace rosette::net::wiznet {

struct W5100S final : ChipBase {
    using address_type = uint16_t;
    using socket_type = uint8_t;
    using Header = ChipBase::Header;

    static constexpr socket_type socket_count() { return 4u; }
    static constexpr socket_type macraw_socket() { return 0u; }

    static constexpr uint8_t chip_version() { return 0x51u; }
    static constexpr bool auto_wrap() { return false; }

    static constexpr address_type common_mode() { return 0x0000u; }
    static constexpr address_type common_gateway() { return 0x0001u; }
    static constexpr address_type common_subnet() { return 0x0005u; }
    static constexpr address_type common_mac_address() { return 0x0009u; }
    static constexpr address_type common_ip_address() { return 0x000Fu; }
    static constexpr address_type common_interrupt() { return 0x0015u; }
    static constexpr address_type common_socket_interrupt_mask() { return 0x0016u; }
    static constexpr address_type common_link_status() { return 0x003Cu; }
    static constexpr address_type common_version() { return 0x0080u; }

    static constexpr uint8_t link_up_mask() { return 0x01u; }

    // IR bit 7 (IR_CONFLICT): set when an ARP reply carries our own SIPR from a
    // different MAC. Lives in the same IR register `common_interrupt()` names,
    // cleared the same write-1 way.
    static constexpr uint8_t common_ip_conflict_mask() { return 0x80u; }

    // The W5100S has no IPv6; `Chip::has_hw_ipv6()` gates every IPv6 path.
    static constexpr bool has_hw_ipv6() { return false; }

    static constexpr address_type socket_register_base(socket_type socket)
    {
        return static_cast<address_type>(0x0400u + (static_cast<address_type>(socket) * 0x0100u));
    }

    static constexpr address_type socket_mode(socket_type socket) { return socket_register_base(socket) + 0x0000u; }
    static constexpr address_type socket_command(socket_type socket) { return socket_register_base(socket) + 0x0001u; }
    static constexpr address_type socket_interrupt(socket_type socket) { return socket_register_base(socket) + 0x0002u; }
    // Sn_IR / IR are write-1-to-clear in place on the W5100S (no separate *CLR).
    static constexpr address_type socket_interrupt_clear(socket_type socket) { return socket_interrupt(socket); }
    static constexpr address_type common_interrupt_clear() { return common_interrupt(); }
    static constexpr address_type socket_status(socket_type socket) { return socket_register_base(socket) + 0x0003u; }
    static constexpr address_type socket_source_port(socket_type socket) { return socket_register_base(socket) + 0x0004u; }
    static constexpr address_type socket_dest_hardware(socket_type socket) { return socket_register_base(socket) + 0x0006u; }
    static constexpr address_type socket_dest_ip(socket_type socket) { return socket_register_base(socket) + 0x000Cu; }
    static constexpr address_type socket_dest_port(socket_type socket) { return socket_register_base(socket) + 0x0010u; }
    static constexpr address_type socket_rx_buffer_size(socket_type socket) { return socket_register_base(socket) + 0x001Eu; }
    static constexpr address_type socket_tx_buffer_size(socket_type socket) { return socket_register_base(socket) + 0x001Fu; }
    static constexpr address_type socket_tx_free_size(socket_type socket) { return socket_register_base(socket) + 0x0020u; }
    static constexpr address_type socket_tx_write_pointer(socket_type socket) { return socket_register_base(socket) + 0x0024u; }
    static constexpr address_type socket_rx_received_size(socket_type socket) { return socket_register_base(socket) + 0x0026u; }
    static constexpr address_type socket_rx_read_pointer(socket_type socket) { return socket_register_base(socket) + 0x0028u; }
    static constexpr address_type socket_interrupt_mask(socket_type socket) { return socket_register_base(socket) + 0x002Cu; }

    // One shared 8 KiB TX region and one shared 8 KiB RX region; init_*()
    // carves each into per-socket spans starting at these bases.
    static constexpr bool per_socket_buffer_blocks() { return false; }
    static constexpr address_type tx_buffer_base() { return 0x4000u; }
    static constexpr address_type rx_buffer_base() { return 0x6000u; }
    static constexpr uint16_t bytes_per_kib() { return 1024u; }

    static constexpr uint8_t socket_tx_buffer_kib() { return 8u; }
    static constexpr uint8_t socket_rx_buffer_kib() { return 8u; }

    static constexpr uint8_t socket_interrupt_send_ok() { return 0x10u; }
    static constexpr uint8_t socket_interrupt_timeout() { return 0x08u; }
    static constexpr uint8_t socket_interrupt_receive() { return 0x04u; }

    static constexpr uint8_t command_open() { return 0x01u; }
    static constexpr uint8_t command_send() { return 0x20u; }
    /// SEND_MAC (UDP): transmit to Sn_DHAR without ARP. Needed for multicast
    /// from a unicast socket, where ARP for the group address never resolves
    /// (W5100S datasheet §Sn_CR).
    static constexpr uint8_t command_send_mac() { return 0x21u; }
    static constexpr uint8_t command_close() { return 0x10u; }
    static constexpr uint8_t command_receive_done() { return 0x40u; }

    static constexpr uint8_t mode_udp() { return 0x02u; }
    static constexpr uint8_t mode_multicast() { return 0x80u; }
    static constexpr uint8_t mode_igmp_v2() { return 0x20u; }
    static constexpr uint8_t mode_macraw() { return 0x04u; }
    static constexpr uint8_t mode_mac_filter_enable() { return 0x40u; }
    static constexpr uint8_t macraw_mode()
    {
        return static_cast<uint8_t>(mode_macraw() | mode_mac_filter_enable());
    }

    static constexpr uint8_t socket_status_closed() { return 0x00u; }
    static constexpr uint8_t socket_status_udp() { return 0x22u; }

    /// Software reset: pulse MR.RST. `write_u8` is `void(address_type, uint8_t)`.
    template<typename WriteU8>
    static void reset(WriteU8&& write_u8)
    {
        write_u8(common_mode(), 0x80u);
    }

    static constexpr uint16_t macraw_rx_header_bytes() { return 2u; }
    static constexpr uint16_t udp_rx_header_bytes() { return 8u; }

    /**
     * @brief Decodes the packet-info header the chip stores before each UDP datagram.
     *
     * Layout (W5100S datasheet §5, UDP receive): sender IPv4 address (4
     * octets), sender port (2), payload length (2), all big-endian.
     *
     * @param[in]  read_next `void(uint8_t* out, uint16_t n)`: reads the next `n`
     *   octets of the RX ring.
     * @param[out] source    Sender address and port; always IPv4.
     * @return Payload length in octets.
     */
    template<typename ReadNext>
    static uint16_t read_udp_rx_header(ReadNext&& read_next, UdpEndpoint& source)
    {
        uint8_t header[8] = {0u};
        read_next(header, 8u);
        source.ipv6 = false;
        source.address = {header[0], header[1], header[2], header[3]};
        source.port = static_cast<uint16_t>((header[4] << 8) | header[5]);
        return static_cast<uint16_t>((header[6] << 8) | header[7]);
    }
    static constexpr uint16_t max_frame_bytes() { return 1536u; }

    template<bool IsRead>
    static constexpr Header make_header(address_type address)
    {
        return Header{
            .bytes = IsRead ? make_read_header(address)
                            : make_write_header(address),
            .length = 3u,
        };
    }

private:
    static constexpr std::array<uint8_t, 8> make_read_header(address_type address)
    {
        return {
            0x0Fu,
            static_cast<uint8_t>(address >> 8),
            static_cast<uint8_t>(address),
        };
    }

    static constexpr std::array<uint8_t, 8> make_write_header(address_type address)
    {
        return {
            0xF0u,
            static_cast<uint8_t>(address >> 8),
            static_cast<uint8_t>(address),
        };
    }
};

}  // namespace rosette::wiznet
