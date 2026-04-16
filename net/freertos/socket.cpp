#include "net/freertos/socket.hpp"

#include <array>
#include <cstring>
#include <new>

#include "net/freertos/igmp.hpp"

extern "C" {
#include "FreeRTOS.h"
#include "FreeRTOS_IPv6.h"
#include "FreeRTOS_Sockets.h"
}

namespace milkdds::net::freertos {

namespace {

bool is_supported_request(SocketDomain domain,
                          SocketType type,
                          SocketProtocol protocol) noexcept
{
    return ((domain == SocketDomain::ipv4) ||
            (domain == SocketDomain::ipv6)) &&
           (type == SocketType::datagram) &&
           (protocol == SocketProtocol::udp);
}

freertos_sockaddr to_freertos_address(const SocketAddress& address) noexcept
{
    freertos_sockaddr out{};
    out.sin_len = static_cast<uint8_t>(sizeof(out));
    out.sin_family = static_cast<uint8_t>(
        address.domain == SocketDomain::ipv6 ? FREERTOS_AF_INET6 : FREERTOS_AF_INET);
    out.sin_port = FreeRTOS_htons(address.port);
    out.sin_flowinfo = address.scope_id;

    if (address.domain == SocketDomain::ipv6) {
        std::memcpy(out.sin_address.xIP_IPv6.ucBytes,
                    address.address.data(),
                    sizeof(out.sin_address.xIP_IPv6.ucBytes));
    } else {
        out.sin_address.ulIP_IPv4 =
            FreeRTOS_inet_addr_quick(address.address[0],
                                     address.address[1],
                                     address.address[2],
                                     address.address[3]);
    }
    return out;
}

SocketAddress from_freertos_address(const freertos_sockaddr& address) noexcept
{
    if (address.sin_family == FREERTOS_AF_INET6) {
        std::array<uint8_t, 16> bytes{};
        std::memcpy(bytes.data(),
                    address.sin_address.xIP_IPv6.ucBytes,
                    bytes.size());
        return make_socket_address_v6(bytes,
                                      FreeRTOS_ntohs(address.sin_port),
                                      address.sin_flowinfo);
    }

    const uint32_t host_order = FreeRTOS_ntohl(address.sin_address.ulIP_IPv4);
    return make_socket_address_v4(
        {
            static_cast<uint8_t>((host_order >> 24) & 0xFFu),
            static_cast<uint8_t>((host_order >> 16) & 0xFFu),
            static_cast<uint8_t>((host_order >> 8) & 0xFFu),
            static_cast<uint8_t>(host_order & 0xFFu),
        },
        FreeRTOS_ntohs(address.sin_port));
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

}  // namespace

class FreeRtosDatagramSocket final : public Socket {
public:
    explicit FreeRtosDatagramSocket(Socket_t handle) noexcept
        : handle_(handle) {}

    ~FreeRtosDatagramSocket() override
    {
        leave_all_multicast_groups();
        close();
    }

    bool is_open() const noexcept override
    {
        return handle_ != FREERTOS_INVALID_SOCKET;
    }

    int joinMulticastGroup(const SocketMulticastMembership& membership) noexcept override
    {
        if (!is_open()) {
            last_error_ = SocketError::closed;
            return -1;
        }

        if ((membership.domain != SocketDomain::ipv4) &&
            (membership.domain != SocketDomain::ipv6)) {
            last_error_ = SocketError::not_supported;
            return -1;
        }

        if (multicast_joined_ && multicast_membership_ == membership) {
            last_error_ = SocketError::none;
            return 0;
        }

        if (multicast_joined_) {
            last_error_ = SocketError::no_buffer_space;
            return -1;
        }

        if (freertos_multicast_service().join_group(membership) != 0) {
            last_error_ = SocketError::io;
            return -1;
        }

        multicast_membership_ = membership;
        multicast_joined_ = true;
        last_error_ = SocketError::none;
        return 0;
    }

    int leaveMulticastGroup(const SocketMulticastMembership& membership) noexcept override
    {
        if (!multicast_joined_ || !(multicast_membership_ == membership)) {
            last_error_ = SocketError::invalid_argument;
            return -1;
        }

        if (freertos_multicast_service().leave_group(membership) != 0) {
            last_error_ = SocketError::io;
            return -1;
        }

        multicast_membership_ = {};
        multicast_joined_ = false;
        last_error_ = SocketError::none;
        return 0;
    }

    int bind(const SocketAddress& local) noexcept override
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
                           const SocketAddress& remote) noexcept override
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
                             SocketAddress* remote) noexcept override
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
        leave_all_multicast_groups();

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
    void leave_all_multicast_groups() noexcept
    {
        if (multicast_joined_) {
            (void)freertos_multicast_service().leave_group(multicast_membership_);
            multicast_membership_ = {};
            multicast_joined_ = false;
        }
    }

    Socket_t handle_ = FREERTOS_INVALID_SOCKET;
    SocketMulticastMembership multicast_membership_{};
    bool multicast_joined_ = false;
    SocketError last_error_ = SocketError::none;
};

std::unique_ptr<Socket> FreeRtosSocketProvider::open(SocketDomain domain,
                                                     SocketType type,
                                                     SocketProtocol protocol) noexcept
{
    if (!is_supported_request(domain, type, protocol)) {
        return {};
    }

    const BaseType_t freertos_domain =
        (domain == SocketDomain::ipv6) ? FREERTOS_AF_INET6 : FREERTOS_AF_INET;
    const Socket_t handle = FreeRTOS_socket(freertos_domain,
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

}  // namespace milkdds::net
