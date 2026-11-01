// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

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

/**
 * @brief Why the last call on a Socket failed, or `none` when it succeeded.
 *
 * Every Socket call except is_open(), receive_buffer_size(), and
 * last_error() leaves one of these in last_error(). Each value is the
 * negated Linux errno a POSIX sockets call reports for the same condition
 * (EINVAL is 22, so `invalid_argument` is -22), so a code logged as a number
 * reads like errno. Each backend maps its own failures onto this set; the
 * meanings below are what a caller may rely on, and each Socket method lists
 * the codes it sets and what leads to each.
 *
 * References:
 * - POSIX.1-2017 <errno.h>; Linux errno(3) for the numeric values
 */
enum class SocketError : int {
    /// The call succeeded.
    none = 0,
    /// The arguments or the socket's state rule the call out: an address the backend cannot
    /// represent, a multicast group not joined or outside 224.0.0.0/4, or a call the backend
    /// accepts only before bind() made after it.
    invalid_argument = -22,
    /// The backend or the hardware failed in a way it does not classify: the chip did not open
    /// or transfer on the socket, the network stack refused a multicast change, or a host call
    /// failed with an errno this set does not name (EADDRINUSE from bind(), for one).
    io = -5,
    /// Nothing could be done now, though the same call may succeed later: no datagram is waiting
    /// and no receive timeout is set, the host's send buffer is full, or send_to() dropped the
    /// datagram before the wire while its next hop is resolved.
    would_block = -11,
    /// The socket is not open: it was closed, or the backend has not opened it yet.
    closed = -9,
    /// recv_from() waited its whole receive timeout and no datagram arrived.
    timed_out = -110,
    /// The backend cannot do this at all: an address family it does not handle, a second
    /// multicast group, a membership change after bind() on hardware that fixes membership
    /// then, or sending or receiving on a hardware socket with no buffer in that direction.
    not_supported = -95,
    /// The network stack had no buffer for the datagram (ENOBUFS, ENOSPC), or a FreeRTOS+TCP
    /// socket that already holds its one multicast group was asked to join another.
    no_buffer_space = -105,
    /// The network stack reports its interface down (ENETDOWN).
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

/**
 * @brief One UDP socket on whichever network backend the board uses.
 *
 * Rosette's RTPS transport and its DHCP and SNTP clients reach the network
 * only through this interface, so each runs unchanged on POSIX sockets,
 * FreeRTOS+TCP, and WIZnet hardware sockets. A socket comes from
 * SocketProvider::open(); multicast membership and receive buffering are set
 * before bind(), then send_to() and recv_from() move datagrams until close().
 *
 * Errors follow the sockets convention: a call that fails returns a negative
 * value and leaves the reason in last_error(), and a call that succeeds
 * leaves `none` there, so last_error() describes the most recent call only.
 * The one success that carries a code is send_to() returning `length` with
 * `would_block`: the datagram was accepted and dropped before the wire. Each
 * method lists under Errors the codes it sets and what leads to each; a
 * backend with no cause for a code never sets it, and a code named for one
 * backend is set by that backend only.
 *
 * Example:
 * @code{.cpp}
 * auto socket = provider.open(SocketDomain::ipv4);
 * if (socket == nullptr || socket->bind(make_socket_address_v4({0u, 0u, 0u, 0u}, 12300u)) != 0) {
 *     return;
 * }
 *
 * if (socket->send_to(request.data(), request.size(), server) < 0) {
 *     std::printf("send failed: %s\n", socket_error_string(socket->last_error()));
 * }
 *
 * socket->close();
 * @endcode
 * Full program: src/net/sntp_client.cpp (SntpClient::exchange()).
 */
class Socket {
public:
    virtual ~Socket() = default;

    /// @brief Whether the socket can send and receive; false once closed (WIZnet: until bind()).
    virtual bool is_open() const noexcept = 0;

    /**
     * @brief Joins the multicast group `membership` names, so its datagrams reach this socket.
     *
     * Call before bind(): some backends (WIZnet) latch membership when the
     * socket opens, and hosted backends enable port sharing here so several
     * participants can bind the group port. Joining the group the socket
     * already holds succeeds and changes nothing.
     *
     * An IPv4 membership whose `interface` is 0.0.0.0 joins wherever the
     * stack chooses; any other address joins on the interface holding it,
     * which is how a caller on a multi-interface stack listens on the network
     * its peers are on (see set_multicast_interface() for the sending side).
     *
     * @param[in] membership Group, interface, and port to join.
     * @return 0 when the socket holds the group; -1 with last_error() set otherwise.
     *
     * Errors:
     * - `closed`: the socket was closed (POSIX, FreeRTOS+TCP).
     * - `invalid_argument`: the group is outside 224.0.0.0/4, or the socket is already bound
     *   (WIZnet); or the kernel rejected the request as EINVAL (POSIX).
     * - `not_supported`: the backend does not handle the group's address family (WIZnet: any
     *   but IPv4), or the socket already holds a different group (WIZnet).
     * - `no_buffer_space`: the socket already holds a different group (FreeRTOS+TCP), or the
     *   kernel had no memory for the membership (POSIX, ENOBUFS).
     * - `io`: the network stack refused the join.
     *
     * References:
     * - socket(7) (SO_REUSEADDR, SO_REUSEPORT), ip(7) (IP_ADD_MEMBERSHIP), ipv6(7)
     */
    virtual int joinMulticastGroup(const SocketMulticastMembership& membership) noexcept = 0;

    /**
     * @brief Leaves the multicast group `membership` names.
     *
     * The counterpart of joinMulticastGroup(), taking the same membership.
     * Hardware that latches membership when the socket opens (WIZnet) can
     * leave only before bind(); afterwards the socket keeps the group until
     * close(). Leaving when the socket holds no group succeeds (WIZnet).
     *
     * @param[in] membership The membership given to joinMulticastGroup().
     * @return 0 when the socket no longer holds the group; -1 with last_error() set otherwise.
     *
     * Errors:
     * - `closed`: the socket was closed (POSIX).
     * - `invalid_argument`: the socket does not hold `membership` (WIZnet, FreeRTOS+TCP); or
     *   the kernel rejected the request as EINVAL (POSIX).
     * - `not_supported`: the socket is already bound (WIZnet), or the backend does not handle
     *   the group's address family (POSIX).
     * - `io`: the network stack refused the change.
     *
     * References:
     * - ip(7) (IP_DROP_MEMBERSHIP), ipv6(7) (IPV6_LEAVE_GROUP)
     */
    virtual int leaveMulticastGroup(const SocketMulticastMembership& membership) noexcept = 0;

    /**
     * @brief Binds the socket to the local address and port `local`.
     *
     * Opens the socket for traffic: on WIZnet this is when the chip opens the
     * hardware socket, with any multicast group already joined. Port 0 lets
     * the backend choose a port (WIZnet picks one in the RFC 6335 dynamic
     * range, since the chip has no allocator of its own).
     *
     * @param[in] local Local address (all zeros for any interface) and port.
     * @return 0 when bound; -1 with last_error() set otherwise.
     *
     * Errors:
     * - `closed`: the socket was closed (POSIX, FreeRTOS+TCP).
     * - `invalid_argument`: the address cannot be represented (POSIX), or the socket is
     *   already bound (POSIX, FreeRTOS+TCP).
     * - `not_supported`: `local` is of another address family than the socket (WIZnet).
     * - `io`: the chip did not open the socket (WIZnet), or the network stack failed the bind
     *   for a reason this set does not name, such as the port being in use (EADDRINUSE).
     *
     * References:
     * - POSIX.1-2017 bind(); RFC 6335 §6 (dynamic ports)
     */
    virtual int bind(const SocketAddress& local) noexcept = 0;

    /**
     * @brief Sets how long recv_from() waits for a datagram, in milliseconds.
     *
     * A timeout of 0 makes recv_from() return at once when nothing is waiting.
     * Backends that cannot fail here (WIZnet) only remember the value.
     *
     * @param[in] timeout_ms Longest wait in recv_from(), in milliseconds; 0 for none.
     * @return 0 when set; -1 with last_error() set otherwise.
     *
     * Errors:
     * - `closed`: the socket was closed (POSIX, FreeRTOS+TCP).
     * - `invalid_argument`, `io`: the network stack rejected the option.
     *
     * References:
     * - socket(7) (SO_RCVTIMEO)
     */
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
     * returns not_supported, changes nothing, and leaves last_error() as it
     * was.
     *
     * @param[in] bytes Receive buffering wanted, in bytes.
     * @return 0 when the request was applied (the grant may be smaller than
     *   `bytes`); otherwise the negative SocketError listed under Errors.
     *
     * Errors:
     * - `not_supported`: the backend's buffering is not per socket (POSIX, FreeRTOS+TCP).
     * - `invalid_argument`: the socket is already bound (WIZnet).
     * - `closed`: the socket was closed (WIZnet).
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

    /**
     * @brief Sends this socket's multicast datagrams out of the interface holding `local`.
     *
     * On a stack with several interfaces, a datagram to a multicast group
     * leaves through whichever interface the stack picks, usually the default
     * route's, which need not be the network the caller's peers are on. RTPS
     * discovery announces a participant by multicast and asks peers to answer
     * at its unicast address, so both must use the same interface. The caller
     * names that interface by an IPv4 address it holds, the same address a
     * joinMulticastGroup() membership carries in `interface`. Each backend
     * maps the address to its own notion of an interface: a POSIX kernel
     * interface (IP_MULTICAST_IF), a FreeRTOS+TCP endpoint
     * (FreeRTOS_FindEndPointOnIP_IPv4()), or an lwIP netif
     * (udp_set_multicast_netif_addr()).
     *
     * A backend with one interface (WIZnet) has nothing to choose and leaves
     * this default, which returns not_supported, changes nothing, and leaves
     * last_error() as it was; the caller treats that as success.
     *
     * @param[in] local An IPv4 address of the interface to send through; its
     *   port is ignored. 0.0.0.0 restores the stack's own choice.
     * @return 0 when later multicast sends leave through that interface;
     *   otherwise the negative SocketError listed under Errors.
     *
     * Errors:
     * - `not_supported`: the backend sends through one interface (WIZnet, FreeRTOS+TCP),
     *   or `local` is not IPv4 (POSIX).
     * - `invalid_argument`: no interface holds `local` (POSIX).
     * - `closed`: the socket was closed (POSIX).
     *
     * Verification:
     * - Correct means: a datagram sent to a group after a 0 return leaves
     *   through the interface holding `local`, whatever the routing table
     *   says about the group.
     * - Covered by: tests/host/posix_host_interface_test.cpp (TC-NET-IFC-006,
     *   POSIX backend over loopback).
     *
     * References:
     * - Linux ip(7), IP_MULTICAST_IF; RFC 1112 §6.1
     */
    virtual int set_multicast_interface(const SocketAddress& local) noexcept {
        (void)local;
        return static_cast<int>(SocketError::not_supported);
    }

    /**
     * @brief Sends one datagram to `remote`.
     *
     * UDP gives no delivery guarantee, so a datagram the backend accepted but
     * that never reaches the wire looks to most callers like one lost on it.
     * A backend that resolves next hops itself (the WIZnet ARP cache) drops a
     * datagram whose next hop it is still resolving, returns `length` as for
     * a datagram sent, and sets last_error() to `would_block`; a caller that
     * paces its sends against a remote's limits (an SNTP client) reads that to
     * retry soon, since the remote saw nothing. Every other accepted datagram
     * leaves last_error() at `none`.
     *
     * @param[in] data   The payload; read before the call returns.
     * @param[in] length Payload bytes.
     * @param[in] remote The destination.
     * @return `length` when the datagram was accepted (see last_error() for
     *   one dropped before the wire); 0 with last_error() at `none` when the
     *   network stack had no buffer to take it (FreeRTOS+TCP); or -1 with
     *   last_error() set.
     *
     * Errors:
     * - `would_block` with a return of `length`: the datagram was dropped before the wire
     *   while its next hop is resolved, and no host received it (WIZnet).
     * - `would_block` with a return of -1: the kernel's send buffer is full (POSIX).
     * - `closed`: the socket was closed, or (WIZnet) is not yet bound.
     * - `invalid_argument`: `remote` cannot be represented (POSIX), or the network stack
     *   rejected the address (FreeRTOS+TCP).
     * - `not_supported`: `remote` is of another address family than the socket, or the
     *   hardware socket has no transmit buffer (WIZnet).
     * - `no_buffer_space`, `network_down`: the network stack had no buffer for the datagram,
     *   or its interface is down (POSIX).
     * - `io`: the chip did not take the datagram (WIZnet), or the network stack failed the
     *   send for a reason this set does not name.
     *
     * Verification:
     * - Correct means: a datagram dropped before the wire returns `length`
     *   with `would_block`, and a datagram sent returns `length` with `none`.
     * - Covered by: TC-NET-WIZ-ARP-010 (WIZnet backend).
     *
     * References:
     * - POSIX.1-2017 sendto(); EAGAIN / EWOULDBLOCK
     */
    virtual std::ptrdiff_t send_to(const void* data,
                                   std::size_t length,
                                   const SocketAddress& remote) noexcept = 0;

    /**
     * @brief Receives one datagram into `data`, waiting up to the receive timeout.
     *
     * With a receive timeout set, the call blocks the calling thread until a
     * datagram arrives or the timeout passes; with none, it returns at once.
     *
     * @param[out] data     Buffer for the payload; holds `capacity` bytes.
     * @param[in]  capacity Size of `data`, in bytes; a longer datagram is truncated.
     * @param[out] remote   The sender's address, when not null and a datagram was received.
     * @return The datagram's length, or -1 with last_error() set. Backends that
     *   can report it return the datagram's real length, so a return above
     *   `capacity` means it was truncated to `capacity` bytes; backends that
     *   cannot never return above `capacity`.
     *
     * Errors:
     * - `would_block`: no datagram is waiting and no receive timeout is set; FreeRTOS+TCP
     *   also reports a timeout that passed this way.
     * - `timed_out`: the receive timeout passed and no datagram arrived (POSIX, WIZnet).
     * - `closed`: the socket was closed, or (WIZnet) is not yet bound.
     * - `not_supported`: the hardware socket has no receive buffer (WIZnet).
     * - `invalid_argument`: the network stack rejected the socket (FreeRTOS+TCP).
     * - `io`: the chip read failed (WIZnet), or the network stack failed the receive for a
     *   reason this set does not name.
     *
     * References:
     * - POSIX.1-2017 recvfrom(); recv(2) (MSG_TRUNC)
     */
    virtual std::ptrdiff_t recv_from(void* data,
                                     std::size_t capacity,
                                     SocketAddress* remote) noexcept = 0;

    /// @brief Closes the socket and leaves its multicast group; safe to repeat. Sets `none`.
    virtual void close() noexcept = 0;

    /// @brief Why the most recent call failed, or `none` when it succeeded; see SocketError.
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

/// @brief The enumerator's name ("would_block"), or "unknown" for a value outside SocketError.
const char* socket_error_string(SocketError error) noexcept;

}  // namespace rosette::net
