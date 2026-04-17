/**
 * @file rtps/transport_freertos.cpp
 * @brief RTPS UDP transport implemented on top of the shared socket module.
 */

#include "transport.hpp"

#include "participant.hpp"
#include "log.hpp"

#include <cstring>

namespace rtps {

namespace {

#ifndef RTPS_SEND_TIMEOUT_MS
#  define RTPS_SEND_TIMEOUT_MS 100u
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

}  // namespace

SocketUdpTransport::SocketUdpTransport(milkdds::net::SocketProvider& provider,
                                       uint16_t meta_port,
                                       uint16_t user_port,
                                       uint32_t multicast_addr) noexcept
    : SocketUdpTransport(
          provider,
          meta_port,
          user_port,
          make_udpv4_locator(multicast_addr, meta_port))
{
}

SocketUdpTransport::SocketUdpTransport(milkdds::net::SocketProvider& provider,
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

bool SocketUdpTransport::open() noexcept
{
    close();

    if (!locator_is_udp(multicast_locator_)) {
        return false;
    }

    const milkdds::net::SocketDomain domain =
        locator_is_ipv6(multicast_locator_) ? milkdds::net::SocketDomain::ipv6
                                            : milkdds::net::SocketDomain::ipv4;

    meta_socket_ = provider_.open(domain);
    if (!meta_socket_) {
        return false;
    }

    if (meta_socket_->join_multicast(locator_to_multicast_membership(multicast_locator_)) != 0) {
        close();
        return false;
    }

    if (meta_socket_->bind(any_bind_address(meta_port_)) != 0) {
        close();
        return false;
    }

    if (meta_socket_->set_receive_timeout(0u) != 0) {
        close();
        return false;
    }

    meta_open_ = true;

    user_socket_ = provider_.open(domain);
    if (!user_socket_) {
        close();
        return false;
    }

    if (user_socket_->bind(any_bind_address(user_port_)) != 0) {
        close();
        return false;
    }

    if (user_socket_->set_receive_timeout(0u) != 0) {
        close();
        return false;
    }

    user_open_ = true;
    return true;
}

void SocketUdpTransport::close() noexcept
{
    if (meta_socket_) {
        meta_socket_->close();
        meta_socket_.reset();
    }

    if (user_socket_) {
        user_socket_->close();
        user_socket_.reset();
    }

    meta_open_ = false;
    user_open_ = false;
}

uint32_t SocketUdpTransport::poll_meta(Octet*    buf,
                                       uint32_t  buf_size,
                                       uint32_t& src_addr,
                                       uint16_t& src_port) noexcept
{
    if (!meta_open_ || !meta_socket_) {
        return 0u;
    }

    milkdds::net::SocketAddress remote{};
    const std::ptrdiff_t received = meta_socket_->recv_from(buf, buf_size, &remote);
    if (received <= 0) {
        return 0u;
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

uint32_t SocketUdpTransport::poll_user(Octet*    buf,
                                       uint32_t  buf_size,
                                       uint32_t& src_addr,
                                       uint16_t& src_port) noexcept
{
    if (!user_open_ || !user_socket_) {
        return 0u;
    }

    milkdds::net::SocketAddress remote{};
    const std::ptrdiff_t received = user_socket_->recv_from(buf, buf_size, &remote);
    if (received <= 0) {
        return 0u;
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

bool SocketUdpTransport::send_unicast(const BufRef& buf,
                                      const Locator& dest) noexcept
{
    if (!buf.valid() || (buf.size == 0u) || !locator_is_udp(dest)) {
        RTPS_LOG_W("transport", "drop send_unicast: valid=%d size=%lu kind=%ld",
                   (int)buf.valid(),
                   (unsigned long)buf.size,
                   (long)dest.kind);
        return false;
    }

    milkdds::net::Socket* socket = socket_for(dest);
    if (socket == nullptr) {
        RTPS_LOG_W("transport", "drop send_unicast: no socket for port=%lu",
                   (unsigned long)dest.port);
        return false;
    }

    const auto address = locator_to_socket_address(dest);
    const std::ptrdiff_t sent = socket->send_to(buf.data, buf.size, address);
    if (sent != static_cast<std::ptrdiff_t>(buf.size)) {
        RTPS_LOG_W("transport", "send_to failed: kind=%ld dst=%u.%u.%u.%u:%u len=%lu sent=%ld err=%d",
                   (long)dest.kind,
                   (unsigned)address.address[0],
                   (unsigned)address.address[1],
                   (unsigned)address.address[2],
                   (unsigned)address.address[3],
                   (unsigned)address.port,
                   (unsigned long)buf.size,
                   (long)sent,
                   (int)socket->last_error());
        return false;
    }

    RTPS_LOG_D("transport", "tx kind=%ld dst=%u.%u.%u.%u:%u len=%lu",
               (long)dest.kind,
               (unsigned)address.address[0],
               (unsigned)address.address[1],
               (unsigned)address.address[2],
               (unsigned)address.address[3],
               (unsigned)address.port,
               (unsigned long)buf.size);
    return true;
}

bool SocketUdpTransport::send_multicast(const BufRef& buf,
                                        const Locator& dest) noexcept
{
    return send_unicast(buf, dest);
}

milkdds::net::Socket* SocketUdpTransport::socket_for(const Locator& dest) const noexcept
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

milkdds::net::SocketAddress
SocketUdpTransport::locator_to_socket_address(const Locator& dest) noexcept
{
    if (locator_is_ipv6(dest)) {
        std::array<uint8_t, 16> ipv6{};
        std::memcpy(ipv6.data(), dest.address, ipv6.size());
        return milkdds::net::make_socket_address_v6(
            ipv6,
            static_cast<uint16_t>(dest.port));
    }

    const uint32_t ip = locator_to_ipv4_be(dest);
    return milkdds::net::make_socket_address_v4(
        {
            static_cast<uint8_t>(ip >> 24u),
            static_cast<uint8_t>(ip >> 16u),
            static_cast<uint8_t>(ip >> 8u),
            static_cast<uint8_t>(ip),
        },
        static_cast<uint16_t>(dest.port));
}

milkdds::net::SocketMulticastMembership
SocketUdpTransport::locator_to_multicast_membership(const Locator& dest) noexcept
{
    if (locator_is_ipv6(dest)) {
        std::array<uint8_t, 16> ipv6{};
        std::memcpy(ipv6.data(), dest.address, ipv6.size());
        return milkdds::net::make_multicast_membership_v6(
            ipv6,
            {},
            static_cast<uint16_t>(dest.port));
    }

    const uint32_t ip = locator_to_ipv4_be(dest);
    return milkdds::net::make_multicast_membership_v4(
        {
            static_cast<uint8_t>(ip >> 24u),
            static_cast<uint8_t>(ip >> 16u),
            static_cast<uint8_t>(ip >> 8u),
            static_cast<uint8_t>(ip),
        },
        {0u, 0u, 0u, 0u},
        static_cast<uint16_t>(dest.port));
}

milkdds::net::SocketAddress
SocketUdpTransport::any_bind_address(uint16_t port) const noexcept
{
    if (locator_is_ipv6(multicast_locator_)) {
        return milkdds::net::make_socket_address_v6({}, port);
    }

    return milkdds::net::make_socket_address_v4({0u, 0u, 0u, 0u}, port);
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

}  // namespace rtps
