#include "posix_socket.hpp"

#include <array>
#include <cerrno>
#include <cstdint>
#include <cstring>
#include <memory>
#include <new>

#include <arpa/inet.h>
#include <fcntl.h>
#include <net/if.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

namespace rosette::net::posix {

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

SocketError map_errno(const int err) noexcept
{
    switch (err) {
    case 0:
        return SocketError::none;
    case EINVAL:
        return SocketError::invalid_argument;
    case EWOULDBLOCK:
#if EAGAIN != EWOULDBLOCK
    case EAGAIN:
#endif
        return SocketError::would_block;
    case ETIMEDOUT:
        return SocketError::timed_out;
    case ENOBUFS:
        return SocketError::no_buffer_space;
    case ENETDOWN:
        return SocketError::network_down;
    case EBADF:
    case ENOTSOCK:
        return SocketError::closed;
    default:
        return SocketError::io;
    }
}

int family_for_domain(SocketDomain domain) noexcept;

/**
 * @brief Opens a non-blocking UDP socket for the requested address family.
 *
 * The POSIX backend underpins Rosette's host-side networking and needs socket
 * semantics that match the embedded transport adapters: datagram I/O with
 * non-blocking behavior surfaced through errno rather than blocking waits.
 *
 * Linux exposes `SOCK_NONBLOCK` as part of `socket()`, while Darwin requires a
 * follow-up `fcntl(F_SETFL, O_NONBLOCK)`. This helper centralizes that policy
 * so the higher-level socket provider can ask for a UDP socket without caring
 * which host platform supplies the flag directly.
 *
 * @param[in] domain Socket domain requested by the Rosette socket provider.
 * @return File descriptor for a non-blocking UDP socket, or `-1` if creation
 *     or non-blocking configuration fails.
 *
 * References:
 * - POSIX.1-2017, `socket()`
 * - POSIX.1-2017, `fcntl()`
 */
int open_nonblocking_datagram_socket(SocketDomain domain) noexcept
{
#ifdef SOCK_NONBLOCK
    return ::socket(family_for_domain(domain), SOCK_DGRAM | SOCK_NONBLOCK, IPPROTO_UDP);
#else
    const int fd = ::socket(family_for_domain(domain), SOCK_DGRAM, IPPROTO_UDP);
    if (fd < 0) {
        return -1;
    }

    const int current_flags = ::fcntl(fd, F_GETFL, 0);
    if (current_flags < 0 || ::fcntl(fd, F_SETFL, current_flags | O_NONBLOCK) < 0) {
        (void)::close(fd);
        return -1;
    }

    return fd;
#endif
}

bool to_sockaddr(const SocketAddress& source,
                 sockaddr_storage& destination,
                 socklen_t& destination_length) noexcept
{
    std::memset(&destination, 0, sizeof(destination));

    if (source.domain == SocketDomain::ipv6) {
        sockaddr_in6 in6{};
        in6.sin6_family = AF_INET6;
        in6.sin6_port = htons(source.port);
        in6.sin6_scope_id = source.scope_id;
        std::memcpy(&in6.sin6_addr, source.address.data(), sizeof(in6.sin6_addr));
        std::memcpy(&destination, &in6, sizeof(in6));
        destination_length = static_cast<socklen_t>(sizeof(in6));
        return true;
    }

    if (source.domain == SocketDomain::ipv4) {
        sockaddr_in in4{};
        in4.sin_family = AF_INET;
        in4.sin_port = htons(source.port);
        in4.sin_addr.s_addr =
            htonl((static_cast<uint32_t>(source.address[0]) << 24u) |
                  (static_cast<uint32_t>(source.address[1]) << 16u) |
                  (static_cast<uint32_t>(source.address[2]) << 8u) |
                  static_cast<uint32_t>(source.address[3]));
        std::memcpy(&destination, &in4, sizeof(in4));
        destination_length = static_cast<socklen_t>(sizeof(in4));
        return true;
    }

    destination_length = 0;
    return false;
}

SocketAddress from_sockaddr(const sockaddr_storage& source) noexcept
{
    if (source.ss_family == AF_INET6) {
        const auto* in6 = reinterpret_cast<const sockaddr_in6*>(&source);
        std::array<uint8_t, 16> bytes{};
        std::memcpy(bytes.data(), &in6->sin6_addr, bytes.size());
        return make_socket_address_v6(bytes, ntohs(in6->sin6_port), in6->sin6_scope_id);
    }

    const auto* in4 = reinterpret_cast<const sockaddr_in*>(&source);
    const uint32_t host_order = ntohl(in4->sin_addr.s_addr);
    return make_socket_address_v4(
        {
            static_cast<uint8_t>((host_order >> 24u) & 0xFFu),
            static_cast<uint8_t>((host_order >> 16u) & 0xFFu),
            static_cast<uint8_t>((host_order >> 8u) & 0xFFu),
            static_cast<uint8_t>(host_order & 0xFFu),
        },
        ntohs(in4->sin_port));
}

int family_for_domain(const SocketDomain domain) noexcept
{
    return domain == SocketDomain::ipv6 ? AF_INET6 : AF_INET;
}

/// Milliseconds on a monotonic clock, for bounding a receive wait.
int64_t monotonic_ms() noexcept
{
    timespec ts{};
    (void)::clock_gettime(CLOCK_MONOTONIC, &ts);
    return (static_cast<int64_t>(ts.tv_sec) * 1000) +
           (static_cast<int64_t>(ts.tv_nsec) / 1000000);
}

}  // namespace

class PosixDatagramSocket final : public Socket {
public:
    explicit PosixDatagramSocket(const int fd) noexcept
        : fd_(fd) {}

    ~PosixDatagramSocket() override
    {
        close();
    }

    bool is_open() const noexcept override
    {
        return fd_ >= 0;
    }

    /**
     * @brief Joins a multicast group and lets other sockets share its port.
     *
     * A multicast group port is shared by every participant on the host
     * (the RTPS SPDP port, the micro-ROS agent discovery port), so this
     * enables SO_REUSEADDR and SO_REUSEPORT before joining. Callers join
     * before bind(), which is when the kernel checks port sharing; unicast
     * sockets never get reuse, so a busy unicast port fails to bind instead
     * of silently splitting traffic between two owners.
     *
     * @param[in] membership Group, interface, and port to join.
     * @return 0 on success, -1 with last_error() set on failure.
     *
     * References:
     * - socket(7) (SO_REUSEADDR, SO_REUSEPORT), ip(7) (IP_ADD_MEMBERSHIP)
     */
    int joinMulticastGroup(const SocketMulticastMembership& membership) noexcept override
    {
        if (!is_open()) {
            last_error_ = SocketError::closed;
            return -1;
        }

        const int one = 1;
        (void)::setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
#ifdef SO_REUSEPORT
        (void)::setsockopt(fd_, SOL_SOCKET, SO_REUSEPORT, &one, sizeof(one));
#endif

        if (membership.domain == SocketDomain::ipv4) {
            ip_mreq req{};
            req.imr_multiaddr.s_addr =
                htonl((static_cast<uint32_t>(membership.group[0]) << 24u) |
                      (static_cast<uint32_t>(membership.group[1]) << 16u) |
                      (static_cast<uint32_t>(membership.group[2]) << 8u) |
                      static_cast<uint32_t>(membership.group[3]));
            req.imr_interface.s_addr =
                htonl((static_cast<uint32_t>(membership.interface[0]) << 24u) |
                      (static_cast<uint32_t>(membership.interface[1]) << 16u) |
                      (static_cast<uint32_t>(membership.interface[2]) << 8u) |
                      static_cast<uint32_t>(membership.interface[3]));

            if (::setsockopt(fd_, IPPROTO_IP, IP_ADD_MEMBERSHIP, &req, sizeof(req)) != 0) {
                last_error_ = map_errno(errno);
                return -1;
            }

            last_error_ = SocketError::none;
            return 0;
        }

        if (membership.domain == SocketDomain::ipv6) {
            ipv6_mreq req{};
            std::memcpy(req.ipv6mr_multiaddr.s6_addr,
                        membership.group.data(),
                        membership.group.size());
            req.ipv6mr_interface = membership.interface_index;

            if (::setsockopt(fd_, IPPROTO_IPV6, IPV6_JOIN_GROUP, &req, sizeof(req)) != 0) {
                last_error_ = map_errno(errno);
                return -1;
            }

            last_error_ = SocketError::none;
            return 0;
        }

        last_error_ = SocketError::not_supported;
        return -1;
    }

    /**
     * @brief Sends this socket's IPv4 multicast out of the interface holding `local`.
     *
     * Without it the kernel routes a group datagram like any other, through
     * the default route's interface on most hosts. See
     * Socket::set_multicast_interface() for the contract.
     *
     * @param[in] local IPv4 address of the interface; 0.0.0.0 restores the kernel's choice.
     * @return 0 on success; the negative SocketError otherwise (`invalid_argument` when
     *   no interface holds `local`, `not_supported` for IPv6, `closed`).
     *
     * Verification:
     * - Correct means: a group datagram sent afterwards reaches a member
     *   joined on that interface.
     * - Covered by: tests/host/posix_host_interface_test.cpp (TC-NET-IFC-006).
     *
     * References:
     * - Linux ip(7), IP_MULTICAST_IF
     */
    int set_multicast_interface(const SocketAddress& local) noexcept override {
        if (!is_open()) {
            last_error_ = SocketError::closed;
            return static_cast<int>(last_error_);
        }

        if (local.domain != SocketDomain::ipv4) {
            return static_cast<int>(SocketError::not_supported);
        }

        in_addr ifaddr{};
        ifaddr.s_addr = htonl((static_cast<uint32_t>(local.address[0]) << 24u)
                              | (static_cast<uint32_t>(local.address[1]) << 16u)
                              | (static_cast<uint32_t>(local.address[2]) << 8u)
                              | static_cast<uint32_t>(local.address[3]));
        if (::setsockopt(fd_, IPPROTO_IP, IP_MULTICAST_IF, &ifaddr, sizeof(ifaddr)) != 0) {
            last_error_ = errno == EADDRNOTAVAIL ? SocketError::invalid_argument : map_errno(errno);
            return static_cast<int>(last_error_);
        }

        last_error_ = SocketError::none;
        return 0;
    }

    int leaveMulticastGroup(const SocketMulticastMembership& membership) noexcept override
    {
        if (!is_open()) {
            last_error_ = SocketError::closed;
            return -1;
        }

        if (membership.domain == SocketDomain::ipv4) {
            ip_mreq req{};
            req.imr_multiaddr.s_addr =
                htonl((static_cast<uint32_t>(membership.group[0]) << 24u) |
                      (static_cast<uint32_t>(membership.group[1]) << 16u) |
                      (static_cast<uint32_t>(membership.group[2]) << 8u) |
                      static_cast<uint32_t>(membership.group[3]));
            req.imr_interface.s_addr =
                htonl((static_cast<uint32_t>(membership.interface[0]) << 24u) |
                      (static_cast<uint32_t>(membership.interface[1]) << 16u) |
                      (static_cast<uint32_t>(membership.interface[2]) << 8u) |
                      static_cast<uint32_t>(membership.interface[3]));

            if (::setsockopt(fd_, IPPROTO_IP, IP_DROP_MEMBERSHIP, &req, sizeof(req)) != 0) {
                last_error_ = map_errno(errno);
                return -1;
            }

            last_error_ = SocketError::none;
            return 0;
        }

        if (membership.domain == SocketDomain::ipv6) {
            ipv6_mreq req{};
            std::memcpy(req.ipv6mr_multiaddr.s6_addr,
                        membership.group.data(),
                        membership.group.size());
            req.ipv6mr_interface = membership.interface_index;

            if (::setsockopt(fd_, IPPROTO_IPV6, IPV6_LEAVE_GROUP, &req, sizeof(req)) != 0) {
                last_error_ = map_errno(errno);
                return -1;
            }

            last_error_ = SocketError::none;
            return 0;
        }

        last_error_ = SocketError::not_supported;
        return -1;
    }

    int bind(const SocketAddress& local) noexcept override
    {
        if (!is_open()) {
            last_error_ = SocketError::closed;
            return -1;
        }

        sockaddr_storage addr{};
        socklen_t addr_len = 0;
        if (!to_sockaddr(local, addr, addr_len)) {
            last_error_ = SocketError::invalid_argument;
            return -1;
        }

        if (::bind(fd_, reinterpret_cast<const sockaddr*>(&addr), addr_len) != 0) {
            last_error_ = map_errno(errno);
            return -1;
        }

        last_error_ = SocketError::none;
        return 0;
    }

    int set_receive_timeout(const uint32_t timeout_ms) noexcept override
    {
        if (!is_open()) {
            last_error_ = SocketError::closed;
            return -1;
        }

        // The fd is non-blocking, so SO_RCVTIMEO has no effect on its own. The
        // timeout is remembered and enforced by recv_from() via poll(); it is
        // also set on the socket so the semantics still hold if the fd is ever
        // switched to blocking.
        receive_timeout_ms_ = timeout_ms;

        timeval tv{};
        tv.tv_sec = static_cast<time_t>(timeout_ms / 1000u);
        tv.tv_usec = static_cast<suseconds_t>((timeout_ms % 1000u) * 1000u);

        if (::setsockopt(fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) != 0) {
            last_error_ = map_errno(errno);
            return -1;
        }

        last_error_ = SocketError::none;
        return 0;
    }

    std::ptrdiff_t send_to(const void* data,
                           const std::size_t length,
                           const SocketAddress& remote) noexcept override
    {
        if (!is_open()) {
            last_error_ = SocketError::closed;
            return -1;
        }

        sockaddr_storage addr{};
        socklen_t addr_len = 0;
        if (!to_sockaddr(remote, addr, addr_len)) {
            last_error_ = SocketError::invalid_argument;
            return -1;
        }

        const ssize_t rc =
            ::sendto(fd_, data, length, 0, reinterpret_cast<const sockaddr*>(&addr), addr_len);
        if (rc < 0) {
            last_error_ = map_errno(errno);
            return -1;
        }

        last_error_ = SocketError::none;
        return static_cast<std::ptrdiff_t>(rc);
    }

    /**
     * @brief Receive one datagram, waiting up to the configured receive timeout.
     *
     * The underlying fd is non-blocking. When a receive timeout has been set,
     * an empty socket does not fail immediately: recv_from() waits for the fd
     * to become readable with poll(), bounded by the time remaining against a
     * monotonic deadline, and retries. This gives the same "block up to the
     * timeout" contract that the FreeRTOS socket backend provides. With no
     * timeout configured the call returns at once, reporting `would_block`.
     *
     * @param[out] data     Buffer for the datagram payload.
     * @param[in]  capacity Size of @p data; a larger datagram is truncated.
     * @param[out] remote    Source address of the datagram, when not null.
     * @return The datagram's length, or -1 with `last_error()` set
     *     (`timed_out` when nothing arrived before the deadline). On Linux
     *     the length is the datagram's real size (MSG_TRUNC), so a value
     *     above @p capacity reports that only @p capacity bytes were stored.
     */
    std::ptrdiff_t recv_from(void* data,
                             const std::size_t capacity,
                             SocketAddress* remote) noexcept override
    {
        if (!is_open()) {
            last_error_ = SocketError::closed;
            return -1;
        }

        const int64_t deadline = (receive_timeout_ms_ > 0u)
            ? (monotonic_ms() + static_cast<int64_t>(receive_timeout_ms_))
            : 0;

        for (;;) {
            sockaddr_storage from{};
            socklen_t from_len = static_cast<socklen_t>(sizeof(from));
#ifdef __linux__
            // MSG_TRUNC: return the real datagram length even when it did not
            // fit, so callers can detect (and count) truncation.
            const int recv_flags = MSG_TRUNC;
#else
            const int recv_flags = 0;
#endif
            const ssize_t rc = ::recvfrom(fd_, data, capacity, recv_flags,
                                          reinterpret_cast<sockaddr*>(&from),
                                          &from_len);
            if (rc >= 0) {
                if (remote != nullptr) {
                    *remote = from_sockaddr(from);
                }
                last_error_ = SocketError::none;
                return static_cast<std::ptrdiff_t>(rc);
            }

            const int err = errno;
            const bool would_block =
                (err == EWOULDBLOCK) || (err == EAGAIN);
            if (!would_block || (receive_timeout_ms_ == 0u)) {
                last_error_ = map_errno(err);
                return -1;
            }

            const int64_t remaining = deadline - monotonic_ms();
            if (remaining <= 0) {
                last_error_ = SocketError::timed_out;
                return -1;
            }

            pollfd pfd{};
            pfd.fd = fd_;
            pfd.events = POLLIN;
            const int ready = ::poll(&pfd, 1u, static_cast<int>(remaining));
            if (ready == 0) {
                last_error_ = SocketError::timed_out;
                return -1;
            }
            if (ready < 0) {
                if (errno == EINTR) {
                    continue;
                }
                last_error_ = map_errno(errno);
                return -1;
            }
            // Readable (or an error is pending): loop and let recvfrom report it.
        }
    }

    void close() noexcept override
    {
        if (fd_ >= 0) {
            (void)::close(fd_);
            fd_ = -1;
        }
        last_error_ = SocketError::none;
    }

    SocketError last_error() const noexcept override
    {
        return last_error_;
    }

    /// @brief Underlying file descriptor, for PosixSocketProvider::wait_readable().
    int native_handle() const noexcept
    {
        return fd_;
    }

private:
    int fd_ = -1;
    SocketError last_error_ = SocketError::none;
    uint32_t receive_timeout_ms_ = 0u;
};

std::unique_ptr<Socket> PosixSocketProvider::open(SocketDomain domain,
                                                  SocketType type,
                                                  SocketProtocol protocol) noexcept
{
    if (!is_supported_request(domain, type, protocol)) {
        return {};
    }

    const int fd = open_nonblocking_datagram_socket(domain);
    if (fd < 0) {
        return {};
    }

    const int one = 1;
    if (domain == SocketDomain::ipv6) {
        (void)::setsockopt(fd, IPPROTO_IPV6, IPV6_V6ONLY, &one, sizeof(one));
    }

    auto socket = std::unique_ptr<Socket>(new (std::nothrow) PosixDatagramSocket(fd));
    if (!socket) {
        (void)::close(fd);
    }
    return socket;
}

void PosixSocketProvider::wait_readable(Socket* const* sockets, std::size_t count,
                                        rosette::platform::Event& wake,
                                        uint32_t timeout_ms) noexcept
{
    constexpr std::size_t kMaxSockets = 7u;
    pollfd fds[kMaxSockets + 1u] = {};
    nfds_t n = 0;
    fds[n++] = pollfd{wake.native_handle(), POLLIN, 0};
    for (std::size_t i = 0; i < count && n <= kMaxSockets; ++i) {
        // Every socket this provider opens is a PosixDatagramSocket.
        const auto* socket = static_cast<const PosixDatagramSocket*>(sockets[i]);
        if (socket != nullptr && socket->is_open()) {
            fds[n++] = pollfd{socket->native_handle(), POLLIN, 0};
        }
    }
    const int timeout = (timeout_ms == rosette::platform::kWaitForever)
        ? -1 : static_cast<int>(timeout_ms);
    int rc;
    do {
        rc = ::poll(fds, n, timeout);
    } while (rc < 0 && errno == EINTR);
    if (rc > 0 && (fds[0].revents & POLLIN) != 0) {
        (void)wake.consume();
    }
}

}  // namespace rosette::net::posix
