/**
 * @file rtps/transport_freertos.cpp
 * @brief RTPS UDP transport implemented on top of the shared socket module.
 */

#include "transport.hpp"

#include "participant.hpp"

#include <cstring>

namespace rtps {

namespace {

#ifndef RTPS_SEND_TIMEOUT_MS
#  define RTPS_SEND_TIMEOUT_MS 100u
#endif

static uint32_t locator_to_ipv4_be(const Locator& locator) noexcept
{
    return (static_cast<uint32_t>(locator.address[12]) << 24u) |
           (static_cast<uint32_t>(locator.address[13]) << 16u) |
           (static_cast<uint32_t>(locator.address[14]) << 8u) |
           static_cast<uint32_t>(locator.address[15]);
}

}  // namespace

SocketUdpTransport::SocketUdpTransport(milkdds::net::SocketProvider& provider,
                                       uint16_t meta_port,
                                       uint16_t user_port,
                                       uint32_t multicast_addr) noexcept
    : provider_(provider)
    , meta_port_(meta_port)
    , user_port_(user_port)
    , multicast_addr_(multicast_addr)
{
}

SocketUdpTransport::~SocketUdpTransport() noexcept
{
    close();
}

bool SocketUdpTransport::open() noexcept
{
    close();

    const milkdds::net::SocketMulticastMembershipV4 meta_membership{
        .group = {
            static_cast<uint8_t>(multicast_addr_ >> 24u),
            static_cast<uint8_t>(multicast_addr_ >> 16u),
            static_cast<uint8_t>(multicast_addr_ >> 8u),
            static_cast<uint8_t>(multicast_addr_),
        },
        .interface = {0u, 0u, 0u, 0u},
        .port = meta_port_,
    };

    meta_socket_ = provider_.open();
    if (!meta_socket_) {
        return false;
    }

    if (meta_socket_->join_multicast(meta_membership) != 0) {
        close();
        return false;
    }

    if (meta_socket_->bind({.address = {0u, 0u, 0u, 0u}, .port = meta_port_}) != 0) {
        close();
        return false;
    }

    if (meta_socket_->set_receive_timeout(0u) != 0) {
        close();
        return false;
    }

    meta_open_ = true;

    user_socket_ = provider_.open();
    if (!user_socket_) {
        close();
        return false;
    }

    if (user_socket_->bind({.address = {0u, 0u, 0u, 0u}, .port = user_port_}) != 0) {
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

    milkdds::net::SocketAddressV4 remote{};
    const std::ptrdiff_t received = meta_socket_->recv_from(buf, buf_size, &remote);
    if (received <= 0) {
        return 0u;
    }

    src_addr = (static_cast<uint32_t>(remote.address[0]) << 24u) |
               (static_cast<uint32_t>(remote.address[1]) << 16u) |
               (static_cast<uint32_t>(remote.address[2]) << 8u) |
               static_cast<uint32_t>(remote.address[3]);
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

    milkdds::net::SocketAddressV4 remote{};
    const std::ptrdiff_t received = user_socket_->recv_from(buf, buf_size, &remote);
    if (received <= 0) {
        return 0u;
    }

    src_addr = (static_cast<uint32_t>(remote.address[0]) << 24u) |
               (static_cast<uint32_t>(remote.address[1]) << 16u) |
               (static_cast<uint32_t>(remote.address[2]) << 8u) |
               static_cast<uint32_t>(remote.address[3]);
    src_port = remote.port;
    return static_cast<uint32_t>(received);
}

bool SocketUdpTransport::send_unicast(const BufRef& buf,
                                      const Locator& dest) noexcept
{
    if (!buf.valid() || (buf.size == 0u) || (dest.kind != locator_kind::UDPv4)) {
        return false;
    }

    milkdds::net::Socket* socket = socket_for(dest);
    if (socket == nullptr) {
        return false;
    }

    const auto address = locator_to_socket_address(dest);
    return socket->send_to(buf.data, buf.size, address) ==
           static_cast<std::ptrdiff_t>(buf.size);
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

milkdds::net::SocketAddressV4
SocketUdpTransport::locator_to_socket_address(const Locator& dest) noexcept
{
    const uint32_t ip = locator_to_ipv4_be(dest);
    return {
        .address = {
            static_cast<uint8_t>(ip >> 24u),
            static_cast<uint8_t>(ip >> 16u),
            static_cast<uint8_t>(ip >> 8u),
            static_cast<uint8_t>(ip),
        },
        .port = static_cast<uint16_t>(dest.port),
    };
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
