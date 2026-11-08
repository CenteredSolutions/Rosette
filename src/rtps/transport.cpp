// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file rtps/transport_freertos.cpp
 * @brief RTPS UDP transport implemented on top of the shared socket module.
 */

#include "transport.hpp"

#include "participant.hpp"
#include "log.hpp"

#include <cstring>

namespace rosette::rtps {

namespace {

#ifndef RTPS_SEND_TIMEOUT_MS
#  define RTPS_SEND_TIMEOUT_MS 100u
#endif

// Receive buffering the user-traffic socket asks for: the largest sample the
// buffer pool can reassemble (kPoolMaxBlockSize), whose fragments all arrive
// back to back, plus an eighth for each fragment's RTPS headers and the
// socket backend's per-datagram record (about 160 octets per 1.3 KB fragment).
#ifndef RTPS_USER_SOCKET_RECEIVE_BUFFER
#  define RTPS_USER_SOCKET_RECEIVE_BUFFER (kMaxSampleOctets + kMaxSampleOctets / 8u)
#endif

// Receive buffering the metatraffic socket asks for: a burst of SEDP
// announcements from a participant that has just matched, each a few
// hundred octets.
#ifndef RTPS_META_SOCKET_RECEIVE_BUFFER
#  define RTPS_META_SOCKET_RECEIVE_BUFFER 2048u
#endif

static uint32_t locator_to_ipv4_be(const Locator& locator) noexcept
{
    return locator.ipv4_address_be();
}

static bool locator_is_ipv4(const Locator& locator) noexcept
{
    return locator.kind == locator_kind::UDPv4;
}

static bool locator_is_ipv6(const Locator& locator) noexcept
{
    return locator.kind == locator_kind::UDPv6;
}

static bool locator_is_udp(const Locator& locator) noexcept
{
    return locator_is_ipv4(locator) || locator_is_ipv6(locator);
}

/// The four octets of a host-order IPv4 address, most significant first.
static std::array<uint8_t, 4> ipv4_octets(uint32_t ipv4) noexcept {
    return {static_cast<uint8_t>(ipv4 >> 24u), static_cast<uint8_t>(ipv4 >> 16u),
            static_cast<uint8_t>(ipv4 >> 8u), static_cast<uint8_t>(ipv4)};
}

}  // namespace

SocketUdpTransport::SocketUdpTransport(rosette::net::SocketProvider& provider,
                                       uint16_t meta_port,
                                       uint16_t user_port,
                                       uint32_t multicast_addr,
                                       uint16_t multicast_port) noexcept
    : SocketUdpTransport(
          provider,
          meta_port,
          user_port,
          make_udpv4_locator(multicast_addr, multicast_port))
{
}

SocketUdpTransport::SocketUdpTransport(rosette::net::SocketProvider& provider,
                                       uint16_t meta_port,
                                       uint16_t user_port,
                                       const Locator& multicast_locator) noexcept
    : provider_(provider)
    , meta_port_(meta_port)
    , user_port_(user_port)
    , multicast_locator_(multicast_locator)
{
}

SocketUdpTransport::~SocketUdpTransport() noexcept
{
    close();
}

std::unique_ptr<rosette::net::Socket> SocketUdpTransport::open_bound(uint16_t port,
                                                                     bool join_group,
                                                                     uint32_t receive_bytes) noexcept
{
    const rosette::net::SocketDomain domain =
        locator_is_ipv6(multicast_locator_) ? rosette::net::SocketDomain::ipv6
                                            : rosette::net::SocketDomain::ipv4;
    std::unique_ptr<rosette::net::Socket> socket = provider_.open(domain);
    if (!socket) {
        return nullptr;
    }

    // Before bind(): a WIZnet backend picks the hardware socket, and so the
    // buffer, then. A backend that does not size per socket declines, and a
    // burst larger than the grant is repaired fragment by fragment
    // (NACK_FRAG), only more slowly.
    if (receive_bytes > 0u && socket->set_receive_buffer_size(receive_bytes) == 0) {
        const std::size_t granted = socket->receive_buffer_size();
        if (granted < receive_bytes) {
            RTPS_LOG_W("transport", "port %u receive buffer %lu of %lu asked",
                       static_cast<unsigned>(port), static_cast<unsigned long>(granted),
                       static_cast<unsigned long>(receive_bytes));
        } else {
            RTPS_LOG_I("transport", "port %u receive buffer %lu", static_cast<unsigned>(port),
                       static_cast<unsigned long>(granted));
        }
    }

    // Every socket sends SPDP multicast from the interface the participant
    // advertises; a backend with one interface declines, which is fine.
    if (multicast_interface_ != 0u && locator_is_ipv4(multicast_locator_)) {
        const int rc = socket->set_multicast_interface(
            rosette::net::make_socket_address_v4(ipv4_octets(multicast_interface_), 0u));
        if (rc != 0 && rc != static_cast<int>(rosette::net::SocketError::not_supported)) {
            RTPS_LOG_E("transport", "multicast interface refused: port=%u error=%d",
                       static_cast<unsigned>(port), rc);
            socket->close();
            return nullptr;
        }
    }

    // Membership precedes bind(): WIZnet latches it at OPEN, and hosted
    // backends enable port sharing for group sockets at join time.
    if (join_group &&
        socket->join_multicast(locator_to_multicast_membership(multicast_locator_)) != 0) {
        RTPS_LOG_E("transport", "multicast join failed: port=%u", static_cast<unsigned>(port));
        socket->close();
        return nullptr;
    }
    if (socket->bind(any_bind_address(port)) != 0 ||
        socket->set_receive_timeout(0u) != 0) {
        socket->close();
        return nullptr;
    }
    return socket;
}

bool SocketUdpTransport::open() noexcept
{
    close();

    if (!locator_is_udp(multicast_locator_)) {
        return false;
    }

    // A board whose metatraffic port is the SPDP port listens for SPDP on
    // its metatraffic socket; otherwise the shared SPDP port gets its own
    // group socket and the metatraffic socket stays unicast-only.
    const uint16_t multicast_port = static_cast<uint16_t>(multicast_locator_.port);
    const bool shared_meta = (multicast_port == meta_port_);

    meta_socket_ = open_bound(meta_port_, shared_meta, RTPS_META_SOCKET_RECEIVE_BUFFER);
    user_socket_ = open_bound(user_port_, false, RTPS_USER_SOCKET_RECEIVE_BUFFER);
    if (!meta_socket_ || !user_socket_) {
        RTPS_LOG_D("transport", "bind failed: meta_port=%u user_port=%u",
                   static_cast<unsigned>(meta_port_), static_cast<unsigned>(user_port_));
        close();
        return false;
    }

    if (!shared_meta) {
        spdp_socket_ = open_bound(multicast_port, true, 0u);
        if (!spdp_socket_) {
            RTPS_LOG_E("transport", "bind failed: spdp_port=%u",
                       static_cast<unsigned>(multicast_port));
            close();
            return false;
        }
    }

    meta_open_ = true;
    user_open_ = true;
    return true;
}

void SocketUdpTransport::close() noexcept
{
    for (auto* socket : {&meta_socket_, &spdp_socket_, &user_socket_}) {
        if (*socket) {
            (*socket)->close();
            socket->reset();
        }
    }

    meta_open_ = false;
    user_open_ = false;
}

uint32_t SocketUdpTransport::receive_from(rosette::net::Socket* socket,
                                          Octet*    buf,
                                          uint32_t  buf_size,
                                          uint32_t& src_addr,
                                          uint16_t& src_port) noexcept
{
    if (socket == nullptr) {
        return 0u;
    }

    rosette::net::SocketAddress remote{};
    std::ptrdiff_t received = 0;
    for (;;) {
        received = socket->recv_from(buf, buf_size, &remote);
        if (received <= 0) {
            return 0u;
        }
        ++stats_.rx_datagrams;
        if (static_cast<uint32_t>(received) > stats_.max_rx_datagram) {
            stats_.max_rx_datagram = static_cast<uint32_t>(received);
        }
        if (static_cast<uint32_t>(received) <= max_datagram_) {
            break;
        }
        ++stats_.oversize_rx;
        if (!strict_link_) {
            break;
        }
        // Strict-link mode: this datagram could not have crossed the link
        // intact, so it is lost exactly as on the wire. Keep draining.
        ++stats_.dropped_rx;
    }
    if (static_cast<std::size_t>(received) > buf_size) {
        // The datagram did not fit. The complete submessages at its start
        // are still parsed; the rest is lost, so count it for diagnosis.
        ++stats_.truncated_rx;
        RTPS_LOG_W("transport", "datagram of %ld octets truncated to %lu",
                   static_cast<long>(received), static_cast<unsigned long>(buf_size));
        return buf_size;
    }

    if (remote.is_ipv4()) {
        src_addr = (static_cast<uint32_t>(remote.address[0]) << 24u) |
                   (static_cast<uint32_t>(remote.address[1]) << 16u) |
                   (static_cast<uint32_t>(remote.address[2]) << 8u) |
                   static_cast<uint32_t>(remote.address[3]);
    } else {
        src_addr = 0u;
    }
    src_port = remote.port;
    return static_cast<uint32_t>(received);
}

uint32_t SocketUdpTransport::poll_meta(Octet*    buf,
                                       uint32_t  buf_size,
                                       uint32_t& src_addr,
                                       uint16_t& src_port) noexcept
{
    if (!meta_open_) {
        return 0u;
    }
    const uint32_t unicast = receive_from(meta_socket_.get(), buf, buf_size, src_addr, src_port);
    if (unicast > 0u) {
        return unicast;
    }
    return receive_from(spdp_socket_.get(), buf, buf_size, src_addr, src_port);
}

uint32_t SocketUdpTransport::poll_user(Octet*    buf,
                                       uint32_t  buf_size,
                                       uint32_t& src_addr,
                                       uint16_t& src_port) noexcept
{
    if (!user_open_) {
        return 0u;
    }
    return receive_from(user_socket_.get(), buf, buf_size, src_addr, src_port);
}

bool SocketUdpTransport::send_unicast(const BufRef& buf,
                                      const Locator& dest) noexcept
{
    if (!buf.valid() || (buf.size == 0u) || !locator_is_udp(dest)) {
        RTPS_LOG_W("transport", "drop send_unicast: valid=%d size=%lu kind=%ld",
                   static_cast<int>(buf.valid()), static_cast<unsigned long>(buf.size),
                   static_cast<long>(dest.kind));
        return false;
    }

    rosette::net::Socket* socket = socket_for(dest);
    if (socket == nullptr) {
        RTPS_LOG_W("transport", "drop send_unicast: no socket for port=%lu",
                   static_cast<unsigned long>(dest.port));
        return false;
    }

    const auto address = locator_to_socket_address(dest);
    ++stats_.tx_datagrams;
    if (buf.size > stats_.max_tx_datagram) stats_.max_tx_datagram = buf.size;
    if (buf.size > max_datagram_) {
        ++stats_.oversize_tx;
        RTPS_LOG_W("transport", "datagram of %lu octets exceeds the %lu-octet limit",
                   static_cast<unsigned long>(buf.size), static_cast<unsigned long>(max_datagram_));
    }
    const std::ptrdiff_t sent = socket->send_to(buf.data, buf.size, address);
    if (sent != static_cast<std::ptrdiff_t>(buf.size)) {
        RTPS_LOG_W(
            "transport", "send_to failed: kind=%ld dst=%u.%u.%u.%u:%u len=%lu sent=%ld err=%d",
            static_cast<long>(dest.kind), static_cast<unsigned>(address.address[0]),
            static_cast<unsigned>(address.address[1]), static_cast<unsigned>(address.address[2]),
            static_cast<unsigned>(address.address[3]), static_cast<unsigned>(address.port),
            static_cast<unsigned long>(buf.size), static_cast<long>(sent),
            static_cast<int>(socket->last_error()));
        return false;
    }

    RTPS_LOG_D("transport", "tx kind=%ld dst=%u.%u.%u.%u:%u len=%lu", static_cast<long>(dest.kind),
               static_cast<unsigned>(address.address[0]), static_cast<unsigned>(address.address[1]),
               static_cast<unsigned>(address.address[2]), static_cast<unsigned>(address.address[3]),
               static_cast<unsigned>(address.port), static_cast<unsigned long>(buf.size));
    return true;
}

bool SocketUdpTransport::send_multicast(const BufRef& buf,
                                        const Locator& dest) noexcept
{
    return send_unicast(buf, dest);
}

rosette::net::Socket* SocketUdpTransport::socket_for(const Locator& dest) const noexcept
{
    const uint16_t port = static_cast<uint16_t>(dest.port);
    if ((port == user_port_) && user_open_ && user_socket_) {
        return user_socket_.get();
    }

    if (meta_open_ && meta_socket_) {
        return meta_socket_.get();
    }

    return nullptr;
}

rosette::net::SocketAddress
SocketUdpTransport::locator_to_socket_address(const Locator& dest) noexcept
{
    if (locator_is_ipv6(dest)) {
        std::array<uint8_t, 16> ipv6{};
        std::memcpy(ipv6.data(), dest.address, ipv6.size());
        return rosette::net::make_socket_address_v6(
            ipv6,
            static_cast<uint16_t>(dest.port));
    }

    const uint32_t ip = locator_to_ipv4_be(dest);
    return rosette::net::make_socket_address_v4(
        {
            static_cast<uint8_t>(ip >> 24u),
            static_cast<uint8_t>(ip >> 16u),
            static_cast<uint8_t>(ip >> 8u),
            static_cast<uint8_t>(ip),
        },
        static_cast<uint16_t>(dest.port));
}

rosette::net::SocketMulticastMembership SocketUdpTransport::locator_to_multicast_membership(
    const Locator& dest) const noexcept {
    if (locator_is_ipv6(dest)) {
        std::array<uint8_t, 16> ipv6{};
        std::memcpy(ipv6.data(), dest.address, ipv6.size());
        return rosette::net::make_multicast_membership_v6(
            ipv6,
            {},
            static_cast<uint16_t>(dest.port));
    }

    const uint32_t ip = locator_to_ipv4_be(dest);
    return rosette::net::make_multicast_membership_v4(
        {
            static_cast<uint8_t>(ip >> 24u),
            static_cast<uint8_t>(ip >> 16u),
            static_cast<uint8_t>(ip >> 8u),
            static_cast<uint8_t>(ip),
        },
        ipv4_octets(multicast_interface_), static_cast<uint16_t>(dest.port));
}

rosette::net::SocketAddress
SocketUdpTransport::any_bind_address(uint16_t port) const noexcept
{
    if (locator_is_ipv6(multicast_locator_)) {
        return rosette::net::make_socket_address_v6({}, port);
    }

    return rosette::net::make_socket_address_v4({0u, 0u, 0u, 0u}, port);
}

bool SocketTransportRuntime::start() noexcept
{
    return transport_.open();
}

void SocketTransportRuntime::spin_some(Octet*           rx_buffer,
                                       uint32_t         rx_buffer_size,
                                       RTPSParticipant& participant) noexcept
{
    uint32_t src_addr = 0u;
    uint16_t src_port = 0u;
    uint32_t received = 0u;

    while ((received = transport_.poll_meta(rx_buffer,
                                            rx_buffer_size,
                                            src_addr,
                                            src_port)) > 0u) {
        participant.parse_received(rx_buffer, received);
    }

    while ((received = transport_.poll_user(rx_buffer,
                                            rx_buffer_size,
                                            src_addr,
                                            src_port)) > 0u) {
        participant.parse_received(rx_buffer, received);
    }
}

}  // namespace rosette::rtps
