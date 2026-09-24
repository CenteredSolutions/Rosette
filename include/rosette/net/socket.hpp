#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>

namespace rosette::net {

enum class SocketDomain : uint8_t {
    ipv4 = 2u,
    ipv6 = 10u,
};

enum class SocketType : uint8_t {
    datagram = 2u,
};

enum class SocketProtocol : uint8_t {
    udp = 17u,
};

enum class SocketError : int {
    none = 0,
    invalid_argument = -22,
    io = -5,
    would_block = -11,
    closed = -9,
    timed_out = -110,
    not_supported = -95,
    no_buffer_space = -105,
    network_down = -100,
};

struct SocketAddress {
    SocketDomain domain = SocketDomain::ipv4;
    std::array<uint8_t, 16> address{};
    uint16_t port = 0u;
    uint32_t scope_id = 0u;

    bool is_ipv4() const noexcept
    {
        return domain == SocketDomain::ipv4;
    }

    bool is_ipv6() const noexcept
    {
        return domain == SocketDomain::ipv6;
    }

    std::array<uint8_t, 4> ipv4() const noexcept
    {
        return {address[0], address[1], address[2], address[3]};
    }
};

struct SocketMulticastMembership {
    SocketDomain domain = SocketDomain::ipv4;
    std::array<uint8_t, 16> group{};
    std::array<uint8_t, 16> interface{};
    uint16_t port = 0u;
    uint32_t interface_index = 0u;

    bool operator==(const SocketMulticastMembership& other) const noexcept
    {
        return (domain == other.domain) &&
               (group == other.group) &&
               (interface == other.interface) &&
               (port == other.port) &&
               (interface_index == other.interface_index);
    }
};

inline SocketAddress make_socket_address_v4(const std::array<uint8_t, 4>& address,
                                            uint16_t port) noexcept
{
    SocketAddress out{};
    out.domain = SocketDomain::ipv4;
    out.address[0] = address[0];
    out.address[1] = address[1];
    out.address[2] = address[2];
    out.address[3] = address[3];
    out.port = port;
    return out;
}

inline SocketAddress make_socket_address_v6(const std::array<uint8_t, 16>& address,
                                            uint16_t port,
                                            uint32_t scope_id = 0u) noexcept
{
    SocketAddress out{};
    out.domain = SocketDomain::ipv6;
    out.address = address;
    out.port = port;
    out.scope_id = scope_id;
    return out;
}

inline SocketMulticastMembership
make_multicast_membership_v4(const std::array<uint8_t, 4>& group,
                             const std::array<uint8_t, 4>& interface = {0u, 0u, 0u, 0u},
                             uint16_t port = 0u) noexcept
{
    SocketMulticastMembership out{};
    out.domain = SocketDomain::ipv4;
    out.group[0] = group[0];
    out.group[1] = group[1];
    out.group[2] = group[2];
    out.group[3] = group[3];
    out.interface[0] = interface[0];
    out.interface[1] = interface[1];
    out.interface[2] = interface[2];
    out.interface[3] = interface[3];
    out.port = port;
    return out;
}

inline SocketMulticastMembership
make_multicast_membership_v6(const std::array<uint8_t, 16>& group,
                             const std::array<uint8_t, 16>& interface = {},
                             uint16_t port = 0u,
                             uint32_t interface_index = 0u) noexcept
{
    SocketMulticastMembership out{};
    out.domain = SocketDomain::ipv6;
    out.group = group;
    out.interface = interface;
    out.port = port;
    out.interface_index = interface_index;
    return out;
}

class Socket {
public:
    virtual ~Socket() = default;

    virtual bool is_open() const noexcept = 0;
    /// Joins a multicast group. Call before bind(): some backends (WIZnet)
    /// latch membership when the socket opens, and hosted backends enable
    /// port sharing here so several participants can bind the group port.
    virtual int joinMulticastGroup(const SocketMulticastMembership& membership) noexcept = 0;
    virtual int leaveMulticastGroup(const SocketMulticastMembership& membership) noexcept = 0;
    virtual int bind(const SocketAddress& local) noexcept = 0;
    virtual int set_receive_timeout(uint32_t timeout_ms) noexcept = 0;
    virtual std::ptrdiff_t send_to(const void* data,
                                   std::size_t length,
                                   const SocketAddress& remote) noexcept = 0;
    virtual std::ptrdiff_t recv_from(void* data,
                                     std::size_t capacity,
                                     SocketAddress* remote) noexcept = 0;
    virtual void close() noexcept = 0;
    virtual SocketError last_error() const noexcept = 0;

    int join_multicast(const SocketMulticastMembership& membership) noexcept
    {
        return joinMulticastGroup(membership);
    }

    int leave_multicast(const SocketMulticastMembership& membership) noexcept
    {
        return leaveMulticastGroup(membership);
    }
};

class SocketProvider {
public:
    virtual ~SocketProvider() = default;

    virtual std::unique_ptr<Socket> open(
        SocketDomain domain = SocketDomain::ipv4,
        SocketType type = SocketType::datagram,
        SocketProtocol protocol = SocketProtocol::udp) noexcept = 0;
};

const char* socket_error_string(SocketError error) noexcept;

}  // namespace rosette::net
