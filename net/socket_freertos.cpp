#include "socket_freertos.hpp"

#include <new>

extern "C" {
#include "FreeRTOS.h"
#include "FreeRTOS_Sockets.h"
}

namespace milkdds::net {

namespace {

bool is_supported_request(SocketDomain domain,
                          SocketType type,
                          SocketProtocol protocol) noexcept
{
    return (domain == SocketDomain::ipv4) &&
           (type == SocketType::datagram) &&
           (protocol == SocketProtocol::udp);
}

freertos_sockaddr to_freertos_address(const SocketAddressV4& address) noexcept
{
    freertos_sockaddr out{};
    out.sin_addr = FreeRTOS_inet_addr_quick(address.address[0],
                                            address.address[1],
                                            address.address[2],
                                            address.address[3]);
    out.sin_port = FreeRTOS_htons(address.port);
    return out;
}

SocketAddressV4 from_freertos_address(const freertos_sockaddr& address) noexcept
{
    const uint32_t host_order = FreeRTOS_ntohl(address.sin_addr);
    return SocketAddressV4{
        .address = {
            static_cast<uint8_t>((host_order >> 24) & 0xFFu),
            static_cast<uint8_t>((host_order >> 16) & 0xFFu),
            static_cast<uint8_t>((host_order >> 8) & 0xFFu),
            static_cast<uint8_t>(host_order & 0xFFu),
        },
        .port = FreeRTOS_ntohs(address.sin_port),
    };
}

SocketError map_freertos_error(BaseType_t result) noexcept
{
    if (result >= 0) {
        return SocketError::none;
    }

    switch (result) {
#ifdef pdFREERTOS_ERRNO_EWOULDBLOCK
    case -pdFREERTOS_ERRNO_EWOULDBLOCK:
        return SocketError::would_block;
#endif
#ifdef pdFREERTOS_ERRNO_ETIMEDOUT
    case -pdFREERTOS_ERRNO_ETIMEDOUT:
        return SocketError::timed_out;
#endif
#ifdef pdFREERTOS_ERRNO_ENOSPC
    case -pdFREERTOS_ERRNO_ENOSPC:
        return SocketError::no_buffer_space;
#endif
#ifdef pdFREERTOS_ERRNO_ENETDOWN
    case -pdFREERTOS_ERRNO_ENETDOWN:
        return SocketError::network_down;
#endif
#ifdef pdFREERTOS_ERRNO_EINVAL
    case -pdFREERTOS_ERRNO_EINVAL:
        return SocketError::invalid_argument;
#endif
    default:
        return SocketError::io;
    }
}
}

class FreeRtosDatagramSocket final : public Socket {
public:
    explicit FreeRtosDatagramSocket(Socket_t handle) noexcept
        : handle_(handle) {}

    ~FreeRtosDatagramSocket() override
    {
        close();
    }

    bool is_open() const noexcept override
    {
        return handle_ != FREERTOS_INVALID_SOCKET;
    }

    int join_multicast(const SocketMulticastMembershipV4& membership) noexcept override
    {
        if (!is_open()) {
            last_error_ = SocketError::closed;
            return -1;
        }

#ifdef FREERTOS_SO_IP_ADD_MEMBERSHIP
        struct FreeRtosIpV4Addr {
            uint32_t s_addr;
        };
        struct FreeRtosIpMreq {
            FreeRtosIpV4Addr imr_multiaddr;
            FreeRtosIpV4Addr imr_interface;
        };

        const uint32_t group =
            FreeRTOS_inet_addr_quick(membership.group[0],
                                     membership.group[1],
                                     membership.group[2],
                                     membership.group[3]);
        const uint32_t interface =
            FreeRTOS_inet_addr_quick(membership.interface[0],
                                     membership.interface[1],
                                     membership.interface[2],
                                     membership.interface[3]);

        const FreeRtosIpMreq mreq{
            .imr_multiaddr = {.s_addr = group},
            .imr_interface = {.s_addr = interface},
        };

        const BaseType_t rc =
            FreeRTOS_setsockopt(handle_,
                                0,
                                FREERTOS_SO_IP_ADD_MEMBERSHIP,
                                &mreq,
                                sizeof(mreq));
        last_error_ = map_freertos_error(rc);
        return rc == 0 ? 0 : -1;
#else
        (void)membership;
        last_error_ = SocketError::not_supported;
        return -1;
#endif
    }

    int bind(const SocketAddressV4& local) noexcept override
    {
        if (!is_open()) {
            last_error_ = SocketError::closed;
            return -1;
        }

        const freertos_sockaddr address = to_freertos_address(local);
        const BaseType_t rc = FreeRTOS_bind(handle_, &address, sizeof(address));
        last_error_ = map_freertos_error(rc);
        return rc == 0 ? 0 : -1;
    }

    int set_receive_timeout(uint32_t timeout_ms) noexcept override
    {
        if (!is_open()) {
            last_error_ = SocketError::closed;
            return -1;
        }

        const TickType_t timeout_ticks = pdMS_TO_TICKS(timeout_ms);
        const BaseType_t rc =
            FreeRTOS_setsockopt(handle_,
                                0,
                                FREERTOS_SO_RCVTIMEO,
                                &timeout_ticks,
                                sizeof(timeout_ticks));
        last_error_ = map_freertos_error(rc);
        return rc == 0 ? 0 : -1;
    }

    std::ptrdiff_t send_to(const void* data,
                           std::size_t length,
                           const SocketAddressV4& remote) noexcept override
    {
        if (!is_open()) {
            last_error_ = SocketError::closed;
            return -1;
        }

        const freertos_sockaddr address = to_freertos_address(remote);
        const BaseType_t rc =
            FreeRTOS_sendto(handle_, data, length, 0, &address, sizeof(address));
        last_error_ = map_freertos_error(rc);
        return rc < 0 ? -1 : static_cast<std::ptrdiff_t>(rc);
    }

    std::ptrdiff_t recv_from(void* data,
                             std::size_t capacity,
                             SocketAddressV4* remote) noexcept override
    {
        if (!is_open()) {
            last_error_ = SocketError::closed;
            return -1;
        }

        freertos_sockaddr from{};
        socklen_t from_length = sizeof(from);
        const BaseType_t rc =
            FreeRTOS_recvfrom(handle_, data, capacity, 0, &from, &from_length);
        last_error_ = map_freertos_error(rc);

        if (rc < 0) {
            return -1;
        }

        if (remote != nullptr) {
            *remote = from_freertos_address(from);
        }

        return static_cast<std::ptrdiff_t>(rc);
    }

    void close() noexcept override
    {
        if (handle_ != FREERTOS_INVALID_SOCKET) {
            FreeRTOS_closesocket(handle_);
            handle_ = FREERTOS_INVALID_SOCKET;
        }
        last_error_ = SocketError::none;
    }

    SocketError last_error() const noexcept override
    {
        return last_error_;
    }

private:
    Socket_t handle_ = FREERTOS_INVALID_SOCKET;
    SocketError last_error_ = SocketError::none;
};

}  // namespace

std::unique_ptr<Socket> FreeRtosSocketProvider::open(SocketDomain domain,
                                                     SocketType type,
                                                     SocketProtocol protocol) noexcept
{
    if (!is_supported_request(domain, type, protocol)) {
        return {};
    }

    const Socket_t handle = FreeRTOS_socket(FREERTOS_AF_INET,
                                            FREERTOS_SOCK_DGRAM,
                                            FREERTOS_IPPROTO_UDP);
    if (handle == FREERTOS_INVALID_SOCKET) {
        return {};
    }

    auto socket = std::unique_ptr<Socket>(new (std::nothrow) FreeRtosDatagramSocket(handle));
    if (!socket) {
        FreeRTOS_closesocket(handle);
    }
    return socket;
}

}  // namespace milkdds::wiznet
