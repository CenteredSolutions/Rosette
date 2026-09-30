#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>

#include "rosette/platform/platform.hpp"

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

    /**
     * @brief Asks for at least `bytes` of receive buffering, as SO_RCVBUF does.
     *
     * A socket that receives bursts, such as every fragment of a large RTPS
     * sample sent back to back, loses the datagrams that arrive while its
     * buffer is full, so it asks for room for the whole burst. The request is
     * best effort: the backend grants what it can, and receive_buffer_size()
     * reports the result. Call it before bind(): a backend whose buffers are
     * fixed per hardware socket (WIZnet) picks the socket then.
     *
     * Backends whose buffering is not per socket (FreeRTOS+TCP's shared
     * network buffers) or already ample (POSIX) leave this default, which
     * reports not_supported and changes nothing.
     *
     * @param[in] bytes Receive buffering wanted, in bytes.
     * @return 0 when the request was applied (the grant may be smaller than
     *   `bytes`); otherwise the negative SocketError: not_supported, or
     *   invalid_argument after bind() on a backend that cannot resize then.
     *
     * Verification:
     * - Correct means: after a 0 return, receive_buffer_size() is the
     *   smallest available size of at least `bytes`, or the largest available
     *   when none is that large.
     * - Covered by: TC-NET-WIZ-LAYOUT-002 (WIZnet backend).
     *
     * References:
     * - POSIX.1-2017 setsockopt(), SO_RCVBUF
     */
    virtual int set_receive_buffer_size(std::size_t bytes) noexcept
    {
        (void)bytes;
        return static_cast<int>(SocketError::not_supported);
    }

    /// @brief Receive buffering this socket has, in bytes; 0 when the backend does not say.
    virtual std::size_t receive_buffer_size() const noexcept { return 0u; }
    virtual std::ptrdiff_t send_to(const void* data,
                                   std::size_t length,
                                   const SocketAddress& remote) noexcept = 0;
    /// Receives one datagram into `data`. Returns its length, or -1 with
    /// last_error() set. Backends that can report it return the datagram's
    /// real length, so a return above `capacity` means it was truncated to
    /// `capacity` bytes; backends that cannot never return above `capacity`.
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

    /**
     * @brief Registers the event signaled when a datagram arrives.
     *
     * Interrupt-driven backends (WIZnet INT pin, FreeRTOS+TCP socket
     * callbacks) call `event->signal_from_isr()` or `signal()` on receive, so
     * a thread sleeping in wait_readable() wakes as soon as data is there.
     * Backends that can block on their sockets directly (POSIX) ignore it.
     *
     * @param[in] event Event to signal; must outlive the provider, or be null.
     */
    virtual void set_receive_event(rosette::platform::Event* event) noexcept {
        receive_event_ = event;
    }

    /**
     * @brief The event registered with set_receive_event(), or null.
     *
     * Receive interrupt handlers read it on every interrupt, so the event can
     * be registered after the interrupt is attached, and by whichever driver
     * owns the provider at the time (an rmw context creates its participant
     * driver inside rcl_init()). The pointer is one aligned word, written only
     * from thread context and read whole by the handler.
     *
     * @return The event to signal on receive, or null when none is registered.
     */
    rosette::platform::Event* receive_event() const noexcept { return receive_event_; }

    /**
     * @brief Sleeps until one of `sockets` is readable, `wake` is signaled,
     * or `timeout_ms` passes.
     *
     * The participant driver calls this between protocol passes, so an idle
     * node sleeps (WFE on bare metal, blocked on FreeRTOS and POSIX) instead of
     * polling. The default waits on `wake` alone and relies on the backend
     * signaling the event registered with set_receive_event() when a datagram
     * arrives; register `wake` there. A return says only "check again": the
     * caller drains every socket afterwards.
     *
     * @param[in] sockets    Sockets opened by this provider; may contain nulls.
     * @param[in] count      Number of entries in `sockets`.
     * @param[in] wake       Event other threads signal to end the wait early.
     * @param[in] timeout_ms Longest wait; platform::kWaitForever blocks.
     *
     * Verification:
     * - Correct means: returns promptly (well under the timeout) after a
     *   datagram arrives on any listed socket or `wake` is signaled.
     */
    virtual void wait_readable(Socket* const* sockets, std::size_t count,
                               rosette::platform::Event& wake,
                               uint32_t timeout_ms) noexcept {
        (void)sockets;
        (void)count;
        (void)wake.wait_for(timeout_ms);
    }

protected:
    rosette::platform::Event* receive_event_ = nullptr;
};

const char* socket_error_string(SocketError error) noexcept;

}  // namespace rosette::net
