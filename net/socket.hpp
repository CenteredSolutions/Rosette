#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>

namespace milkdds::net {

enum class SocketDomain : uint8_t {
    ipv4 = 2u,
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

struct SocketAddressV4 {
    std::array<uint8_t, 4> address{};
    uint16_t port = 0u;
};

struct SocketMulticastMembershipV4 {
    std::array<uint8_t, 4> group{};
    std::array<uint8_t, 4> interface{};
    uint16_t port = 0u;
};

class Socket {
public:
    virtual ~Socket() = default;

    virtual bool is_open() const noexcept = 0;
    virtual int join_multicast(const SocketMulticastMembershipV4& membership) noexcept = 0;
    virtual int bind(const SocketAddressV4& local) noexcept = 0;
    virtual int set_receive_timeout(uint32_t timeout_ms) noexcept = 0;
    virtual std::ptrdiff_t send_to(const void* data,
                                   std::size_t length,
                                   const SocketAddressV4& remote) noexcept = 0;
    virtual std::ptrdiff_t recv_from(void* data,
                                     std::size_t capacity,
                                     SocketAddressV4* remote) noexcept = 0;
    virtual void close() noexcept = 0;
    virtual SocketError last_error() const noexcept = 0;
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

}  // namespace milkdds::wiznet
