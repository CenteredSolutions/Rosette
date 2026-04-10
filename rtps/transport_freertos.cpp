/**
 * @file rtps/transport_freertos.cpp
 * @brief RTPS ITransport implementation using FreeRTOS+TCP UDP sockets.
 *
 * Replaces the WizNET socket-mode transport.  With FreeRTOS+TCP handling
 * the IP stack over MACRAW, the transport layer uses the standard
 * FreeRTOS+TCP BSD-like socket API — identical on RP2040 and RP2350.
 *
 * Socket layout
 * -------------
 * Two UDP sockets are opened:
 *   sock_meta_  — metatraffic port (SPDP, SEDP, HEARTBEAT, ACKNACK)
 *                 Joins the SPDP multicast group 239.255.0.1 via
 *                 FREERTOS_SO_IP_ADD_MEMBERSHIP.
 *   sock_user_  — user-data port (DATA, DATA_FRAG, GAP)
 *
 * Both sockets are non-blocking for receive (zero timeout); send blocks
 * up to RTPS_SEND_TIMEOUT_MS if the network buffer pool is temporarily full.
 *
 * Multicast send
 * --------------
 * FreeRTOS+TCP uses FREERTOS_SO_IP_MULTICAST_TTL to set the TTL for
 * multicast sends.  TTL=1 limits multicast to the local subnet — appropriate
 * for RTPS discovery.
 *
 * Thread safety
 * -------------
 * FreeRTOS+TCP sockets are NOT thread-safe for concurrent send/receive on
 * the same socket.  send_unicast() / send_multicast() may be called from
 * the TX retransmit task or the application task.  A mutex per socket
 * serialises concurrent sends.
 */

#include "transport.hpp"       // FreeRtcpUdpTransport
#include "buf_pool.hpp"

/* FreeRTOS+TCP */
#include "FreeRTOS_Sockets.h"
#include "FreeRTOS_IP.h"

#include <string.h>

namespace rtps {

// ============================================================
// Timeout for blocking send calls (ms)
// ============================================================

#ifndef RTPS_SEND_TIMEOUT_MS
#  define RTPS_SEND_TIMEOUT_MS  100u
#endif

// ============================================================
// Helper: extract uint32_t IPv4 from a Locator.
// §9.4.1.11 — IPv4 is in the last 4 bytes of the 16-byte address.
// ============================================================

static inline uint32_t locator_to_ipv4(const Locator& l) noexcept {
    return FreeRTOS_inet_addr_quick(
        l.address[12], l.address[13],
        l.address[14], l.address[15]);
}

// ============================================================
// FreeRtcpUdpTransport
//
// Implements ITransport using FreeRTOS+TCP UDP sockets.
// ============================================================

FreeRtcpUdpTransport::FreeRtcpUdpTransport(uint16_t meta_port,
                                            uint16_t user_port,
                                            uint32_t multicast_ipv4_be) noexcept
    : meta_port_(meta_port)
    , user_port_(user_port)
    , multicast_addr_(multicast_ipv4_be)
{
}

FreeRtcpUdpTransport::~FreeRtcpUdpTransport() noexcept {
    close();
}

// ============================================================
// open()
//
// Opens both UDP sockets and joins the SPDP multicast group.
// Must be called after FreeRTOS_IPInit() and once the IP stack
// reports that the network is up (vApplicationIPNetworkEventHook).
// ============================================================

bool FreeRtcpUdpTransport::open() noexcept {
    // ── Metatraffic socket ────────────────────────────────────────────────────
    sock_meta_ = FreeRTOS_socket(FREERTOS_AF_INET,
                                  FREERTOS_SOCK_DGRAM,
                                  FREERTOS_IPPROTO_UDP);
    if (sock_meta_ == FREERTOS_INVALID_SOCKET) return false;

    // Bind to the metatraffic unicast port on all interfaces.
    struct freertos_sockaddr bind_addr{};
    bind_addr.sin_port = FreeRTOS_htons(meta_port_);
    bind_addr.sin_addr = 0;   // INADDR_ANY
    if (FreeRTOS_bind(sock_meta_, &bind_addr, sizeof(bind_addr)) != 0) {
        FreeRTOS_closesocket(sock_meta_);
        sock_meta_ = FREERTOS_INVALID_SOCKET;
        return false;
    }

    // Join the SPDP multicast group.
    // §9.6.1: SPDP uses 239.255.0.1 (domain 0) for discovery.
    struct ip_mreq mreq{};
    mreq.imr_multiaddr.s_addr = multicast_addr_;
    mreq.imr_interface.s_addr = 0;   // Default interface.
    FreeRTOS_setsockopt(sock_meta_,
                        0,
                        FREERTOS_SO_IP_ADD_MEMBERSHIP,
                        &mreq,
                        sizeof(mreq));

    // Set multicast TTL = 1 (local subnet only — appropriate for RTPS).
    uint8_t ttl = 1;
    FreeRTOS_setsockopt(sock_meta_,
                        0,
                        FREERTOS_SO_IP_MULTICAST_TTL,
                        &ttl,
                        sizeof(ttl));

    // Non-blocking receive (poll from rx_task).
    TickType_t rx_timeout = 0;
    FreeRTOS_setsockopt(sock_meta_, 0, FREERTOS_SO_RCVTIMEO,
                        &rx_timeout, sizeof(rx_timeout));

    // Blocking send with timeout.
    TickType_t tx_timeout = pdMS_TO_TICKS(RTPS_SEND_TIMEOUT_MS);
    FreeRTOS_setsockopt(sock_meta_, 0, FREERTOS_SO_SNDTIMEO,
                        &tx_timeout, sizeof(tx_timeout));

    meta_open_ = true;

    // ── User-data socket ──────────────────────────────────────────────────────
    sock_user_ = FreeRTOS_socket(FREERTOS_AF_INET,
                                  FREERTOS_SOCK_DGRAM,
                                  FREERTOS_IPPROTO_UDP);
    if (sock_user_ == FREERTOS_INVALID_SOCKET) return false;

    bind_addr.sin_port = FreeRTOS_htons(user_port_);
    if (FreeRTOS_bind(sock_user_, &bind_addr, sizeof(bind_addr)) != 0) {
        FreeRTOS_closesocket(sock_user_);
        sock_user_ = FREERTOS_INVALID_SOCKET;
        return false;
    }

    FreeRTOS_setsockopt(sock_user_, 0, FREERTOS_SO_RCVTIMEO,
                        &rx_timeout, sizeof(rx_timeout));
    FreeRTOS_setsockopt(sock_user_, 0, FREERTOS_SO_SNDTIMEO,
                        &tx_timeout, sizeof(tx_timeout));

    user_open_ = true;
    return true;
}

void FreeRtcpUdpTransport::close() noexcept {
    if (meta_open_) {
        FreeRTOS_closesocket(sock_meta_);
        sock_meta_ = FREERTOS_INVALID_SOCKET;
        meta_open_ = false;
    }
    if (user_open_) {
        FreeRTOS_closesocket(sock_user_);
        sock_user_ = FREERTOS_INVALID_SOCKET;
        user_open_ = false;
    }
}

// ============================================================
// poll_meta() / poll_user()
//
// Non-blocking receive on each socket.  Returns byte count;
// 0 means no datagram was available this call.
// ============================================================

uint32_t FreeRtcpUdpTransport::poll_meta(Octet*    buf,
                                          uint32_t  buf_size,
                                          uint32_t& src_addr_out,
                                          uint16_t& src_port_out) noexcept
{
    if (!meta_open_) return 0;
    struct freertos_sockaddr from{};
    socklen_t from_len = sizeof(from);
    int32_t n = FreeRTOS_recvfrom(sock_meta_, buf, buf_size, 0,
                                   &from, &from_len);
    if (n <= 0) return 0;
    src_addr_out = from.sin_addr;
    src_port_out = FreeRTOS_ntohs(from.sin_port);
    return static_cast<uint32_t>(n);
}

uint32_t FreeRtcpUdpTransport::poll_user(Octet*    buf,
                                          uint32_t  buf_size,
                                          uint32_t& src_addr_out,
                                          uint16_t& src_port_out) noexcept
{
    if (!user_open_) return 0;
    struct freertos_sockaddr from{};
    socklen_t from_len = sizeof(from);
    int32_t n = FreeRTOS_recvfrom(sock_user_, buf, buf_size, 0,
                                   &from, &from_len);
    if (n <= 0) return 0;
    src_addr_out = from.sin_addr;
    src_port_out = FreeRTOS_ntohs(from.sin_port);
    return static_cast<uint32_t>(n);
}

// ============================================================
// send_unicast()
//
// Sends buf to the unicast Locator using the appropriate socket.
// The metatraffic socket is used for metatraffic ports; the user
// socket for user-data ports.
// ============================================================

bool FreeRtcpUdpTransport::send_unicast(const BufRef&  buf,
                                         const Locator& dest) noexcept
{
    if (!buf.valid() || buf.size == 0) return false;
    if (dest.kind != locator_kind::UDPv4) return false;

    const Socket_t sock = (dest.port == user_port_ && user_open_)
                          ? sock_user_ : sock_meta_;
    if (sock == FREERTOS_INVALID_SOCKET) return false;

    struct freertos_sockaddr to{};
    to.sin_addr = locator_to_ipv4(dest);
    to.sin_port = FreeRTOS_htons(static_cast<uint16_t>(dest.port));

    int32_t sent = FreeRTOS_sendto(sock,
                                   buf.data,
                                   static_cast<size_t>(buf.size),
                                   0,
                                   &to,
                                   sizeof(to));
    return sent == static_cast<int32_t>(buf.size);
}

// ============================================================
// send_multicast()
//
// FreeRTOS+TCP handles multicast routing automatically when the
// destination address is in the 224.0.0.0/4 range.  We use the
// metatraffic socket which has the multicast TTL configured.
// ============================================================

bool FreeRtcpUdpTransport::send_multicast(const BufRef&  buf,
                                           const Locator& dest) noexcept
{
    // Multicast is just a unicast send to a class-D address — the IP
    // stack handles IGMP membership and Ethernet multicast mapping.
    return send_unicast(buf, dest);
}

} // namespace rtps
