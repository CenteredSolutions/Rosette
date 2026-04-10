/**
 * @file rtps/transport.cpp
 * @brief WiznetUdpTransport implementation — §9.3 (OMG formal/2022-04-01).
 *
 * Implements UDP send/receive for the WizNET W5500 via its ioLibrary
 * socket API (socket.h / wizchip_conf.h from the WizNET SDK).
 *
 * All socket calls are non-blocking (SOCK_IO_NONBLOCK).  If the W5500
 * TX buffer is full, send_unicast() returns false.  The caller (typically
 * StatefulWriter::tick() or SpdpWriter::announce()) should retry on the
 * next tick.
 *
 * §9.3.1, p. 183: "The RTPS Protocol uses UDP/IPv4 as its default
 *  transport."
 */

#include "transport.hpp"

#include "participant.hpp"

// WizNET ioLibrary headers — provided by the BSP / SDK.
// Guard with RTPS_NO_WIZNET for unit-test builds that compile without
// the WizNET SDK.
#ifndef RTPS_NO_WIZNET
#  include "socket.h"      // socket(), bind(), sendto(), recvfrom(), etc.
#  include "wiznet_api.h"
#endif

#include <string.h>

namespace rtps {

#ifndef RTPS_NO_WIZNET

// ============================================================
// Helpers — extract IPv4 address from a Locator.
//
// §9.4.1.11 — IPv4 is in the last 4 bytes of the 16-byte address.
// ============================================================

static inline uint32_t locator_ipv4(const Locator& l) noexcept {
    // Bytes 12–15, MSB-first.
    return (static_cast<uint32_t>(l.address[12]) << 24u) |
           (static_cast<uint32_t>(l.address[13]) << 16u) |
           (static_cast<uint32_t>(l.address[14]) <<  8u) |
           (static_cast<uint32_t>(l.address[15]));
}

// ============================================================
// WiznetUdpTransport::open()
//
// §9.3.1.2, p. 186: join the SPDP multicast group so that
// incoming SPDP announcements are received.
//
// The W5500 ioLibrary socket() call opens a UDP socket bound to the
// specified port.  Multicast join is performed via setsockopt
// equivalent (W5500 uses the IGMP MCAST GROUP register).
// ============================================================

bool WiznetUdpTransport::open() noexcept {
    meta_sock_ = wiznet_acquire_socket();
    user_sock_ = wiznet_acquire_socket();

    wiznet_spi_acquire(UINT32_MAX);
    // Open metatraffic socket and join multicast.
    if (socket(meta_sock_, Sn_MR_UDP,
               meta_port_, SF_MULTI_ENABLE) != RTPS_WIZNET_META_SOCKET) {
        wiznet_spi_release();
        return false;
    }
    meta_open_ = true;

    // Open user-data socket.
    if (socket(user_sock_, Sn_MR_UDP,
               user_port_, 0) != RTPS_WIZNET_USER_SOCKET) {
        wiznet_spi_release();
        return false;
    }
    user_open_ = true;
    wiznet_spi_release();
    return true;
}

// ============================================================
// WiznetUdpTransport::close()
// ============================================================

void WiznetUdpTransport::close() noexcept {
    wiznet_spi_acquire(UINT32_MAX);
    if (meta_open_) { wiznet_close(RTPS_WIZNET_META_SOCKET); meta_open_ = false; }
    if (user_open_) { wiznet_close(RTPS_WIZNET_USER_SOCKET); user_open_ = false; }
    wiznet_spi_release();
}

// ============================================================
// WiznetUdpTransport::poll_meta() / poll_user()
//
// Drain one datagram from the W5500 RX buffer.
//
// W5500 recvfrom() fills: dst buffer, addr[4], port[2].
// Returns bytes received; negative on error.
// ============================================================

uint32_t WiznetUdpTransport::poll_meta(Octet*    buf,
                                       uint32_t  buf_size,
                                       uint32_t& src_addr,
                                       uint16_t& src_port) noexcept
{
    if (!meta_open_) return 0;
    uint8_t addr[4] = {0};
    int32_t n = wiznet_recv(RTPS_WIZNET_META_SOCKET,
                            buf,
                            static_cast<uint16_t>(buf_size),
                            addr,
                            &src_port);
    if (n <= 0) return 0;
    src_addr = (static_cast<uint32_t>(addr[0]) << 24u) |
               (static_cast<uint32_t>(addr[1]) << 16u) |
               (static_cast<uint32_t>(addr[2]) <<  8u) |
               (static_cast<uint32_t>(addr[3]));
    return static_cast<uint32_t>(n);
}

uint32_t WiznetUdpTransport::poll_user(Octet*    buf,
                                       uint32_t  buf_size,
                                       uint32_t& src_addr,
                                       uint16_t& src_port) noexcept
{
    if (!user_open_) return 0;
    uint8_t addr[4] = {0};
    int32_t n = wiznet_recv(RTPS_WIZNET_USER_SOCKET,
                            buf,
                            static_cast<uint16_t>(buf_size),
                            addr,
                            &src_port);
    if (n <= 0) return 0;
    src_addr = (static_cast<uint32_t>(addr[0]) << 24u) |
               (static_cast<uint32_t>(addr[1]) << 16u) |
               (static_cast<uint32_t>(addr[2]) <<  8u) |
               (static_cast<uint32_t>(addr[3]));
    return static_cast<uint32_t>(n);
}

// ============================================================
// WiznetUdpTransport::send_unicast()
//
// §9.3.1.1, p. 184: send via UDP unicast.
//
// The W5500 sendto() call writes one datagram to the specified
// destination IPv4 address and port.  Returns the number of bytes
// actually queued; a short write returns false.
// ============================================================

bool WiznetUdpTransport::send_unicast(const BufRef&  buf,
                                      const Locator& dest) noexcept
{
    if (!buf.valid() || buf.size == 0) return false;
    if (dest.kind != locator_kind::UDPv4) return false;

    const uint8_t sock = socket_for(dest);
    if (sock == RTPS_WIZNET_META_SOCKET && !meta_open_) return false;
    if (sock == RTPS_WIZNET_USER_SOCKET && !user_open_) return false;

    // Build W5500-style destination address (4 bytes, MSB-first).
    const uint32_t ip = locator_ipv4(dest);
    uint8_t addr[4] = {
        static_cast<uint8_t>(ip >> 24u),
        static_cast<uint8_t>(ip >> 16u),
        static_cast<uint8_t>(ip >>  8u),
        static_cast<uint8_t>(ip)
    };
    const uint16_t port = static_cast<uint16_t>(dest.port);

    const int32_t sent = wiznet_send(sock,
                                     addr,
                                     port,
                                     buf.data,
                                     static_cast<uint16_t>(buf.size));
    return (sent == static_cast<int32_t>(buf.size));
}

// ============================================================
// WiznetUdpTransport::send_multicast()
//
// §9.3.1.2, p. 186: send via UDP multicast.
//
// On the W5500 a multicast send is identical to a unicast send
// once the socket was opened with SF_MULTI_ENABLE; the hardware
// handles IGMP.
// ============================================================

bool WiznetUdpTransport::send_multicast(const BufRef&  buf,
                                        const Locator& dest) noexcept
{
    // Reuse the unicast path — the W5500 routes based on the
    // destination address, not the send call.
    if (!buf.valid() || buf.size == 0) return false;
    if (dest.kind != locator_kind::UDPv4) return false;

    const uint32_t ip = locator_ipv4(dest);
    uint8_t addr[4] = {
        static_cast<uint8_t>(ip >> 24u),
        static_cast<uint8_t>(ip >> 16u),
        static_cast<uint8_t>(ip >>  8u),
        static_cast<uint8_t>(ip)
    };
    const uint16_t port = static_cast<uint16_t>(dest.port);
    const int32_t sent = wiznet_mcast(RTPS_WIZNET_META_SOCKET,
                                      addr,
                                      port,
                                      buf.data,
                                      static_cast<uint16_t>(buf.size));
    return (sent == static_cast<int32_t>(buf.size));
}

#else   // RTPS_NO_WIZNET — stub for unit tests

bool WiznetUdpTransport::open() noexcept { return true; }
void WiznetUdpTransport::close() noexcept {}
uint32_t WiznetUdpTransport::poll_meta(Octet*, uint32_t, uint32_t&, uint16_t&) noexcept { return 0; }
uint32_t WiznetUdpTransport::poll_user(Octet*, uint32_t, uint32_t&, uint16_t&) noexcept { return 0; }
bool WiznetUdpTransport::send_unicast(const BufRef&, const Locator&) noexcept { return true; }
bool WiznetUdpTransport::send_multicast(const BufRef&, const Locator&) noexcept { return true; }

#endif // RTPS_NO_WIZNET

bool WiznetTransportRuntime::start() noexcept {
    return transport_.open();
}

void WiznetTransportRuntime::spin_some(Octet*           rx_buffer,
                                       uint32_t         rx_buffer_size,
                                       RTPSParticipant& participant) noexcept {
    uint32_t src_addr = 0;
    uint16_t src_port = 0;
    uint32_t n = 0;

    while ((n = transport_.poll_meta(rx_buffer,
                                     rx_buffer_size,
                                     src_addr,
                                     src_port)) > 0u) {
        participant.parse_received(rx_buffer, n);
    }

    while ((n = transport_.poll_user(rx_buffer,
                                     rx_buffer_size,
                                     src_addr,
                                     src_port)) > 0u) {
        participant.parse_received(rx_buffer, n);
    }
}


} // namespace rtps
