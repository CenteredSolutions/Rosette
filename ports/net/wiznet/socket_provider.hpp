/**
 * @file net/wiznet/socket_provider.hpp
 * @brief Template `SocketProvider` over the WIZnet hardware UDP sockets.
 *
 * `WiznetSocketProvider<Chip, Bus>` hands out the chip's hardware
 * sockets as `rosette::net::Socket` objects. All of the register-level work
 * lives in `WiznetDevice<Chip, Bus>`; this layer maps the `Socket` vtable onto
 * it and tracks which hardware sockets are in use.
 *
 *   WiznetDevice<W5100S, Rp2SpiBus> device{bus};
 *   WiznetSocketProvider<W5100S, Rp2SpiBus> sockets{device};
 *   auto s = sockets.open(SocketDomain::ipv4);
 *   s->bind({.port = 0});
 *   s->send_to(data, len, remote);
 *
 * An `ipv6` socket is available on parts with hardware IPv6 (W6100, W6300,
 * `Chip::has_hw_ipv6()`): it binds the chip's UDP6 mode and exchanges IPv6
 * datagrams only, from the addresses the board's IPv6 address provider has
 * programmed (see `slaac_provider.hpp`). A host that wants both families opens
 * one socket of each, as it would with BSD sockets and `IPV6_V6ONLY`.
 *
 * Waiting is the same on every OS. A thread blocked in `recv_from()` sleeps on
 * its socket's `platform::Event`, and wait_readable() sleeps on the caller's
 * event. The chip's INT pin (attach_interrupt()) signals every such event on
 * each falling edge, and each woken thread reads its own sockets' registers:
 * the interrupt touches no bus. INT stays low while any socket's interrupt is
 * pending, so whoever is about to sleep or has just read first clears every
 * flagged socket's RECV interrupt (rearm_rx_interrupts()) and wakes the threads
 * that may have missed an edge; the next datagram then raises a fresh one.
 *
 * With an ARP resolver attached (attach_arp_resolver(), `arp_resolver.hpp`),
 * each IPv4 send carries its destination MAC from the board's ARP cache and
 * the chip never ARPs inline; a datagram whose next hop is not resolved yet
 * is dropped, as a lost datagram, and resolved in the background. A
 * multicast-joined socket on a part that picks the MAC per socket (W6x00)
 * keeps the chip's own ARP, since its Sn_DHAR holds the group address.
 */

#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <new>

#include "rosette/platform/gpio_interrupt.hpp"
#include "rosette/platform/platform.hpp"

#include "arp_resolver.hpp"
#include "device.hpp"
#include "rosette/net/socket.hpp"

namespace rosette::net::wiznet {

template <typename Chip, typename Bus>
class WiznetSocketProvider;

/**
 * @brief One hardware UDP socket, optionally bound to a multicast group.
 */
template <typename Chip, typename Bus>
class WiznetUdpSocket final : public Socket {
public:
    using device_type = WiznetDevice<Chip, Bus>;
    using provider_type = WiznetSocketProvider<Chip, Bus>;
    using socket_type = typename device_type::socket_type;
    using arp_resolver_type = WiznetArpResolver<Chip, Bus>;

    WiznetUdpSocket(device_type& device, provider_type& provider, socket_type socket,
                    SocketDomain domain) noexcept
        : device_(device), provider_(provider), socket_(socket), domain_(domain) {}

    ~WiznetUdpSocket() override
    {
        close();
    }

    bool is_open() const noexcept override
    {
        return open_;
    }

    int joinMulticastGroup(const SocketMulticastMembership& membership) noexcept override
    {
        if (open_) {
            last_error_ = SocketError::invalid_argument;
            return -1;
        }

        if (membership.domain != SocketDomain::ipv4) {
            last_error_ = SocketError::not_supported;
            return -1;
        }

        if ((membership.group[0] < 224u) || (membership.group[0] > 239u)) {
            last_error_ = SocketError::invalid_argument;
            return -1;
        }

        if (multicast_joined_ && !same_membership(membership, multicast_membership_)) {
            last_error_ = SocketError::not_supported;
            return -1;
        }

        multicast_membership_ = membership;
        multicast_joined_ = true;
        last_error_ = SocketError::none;
        return 0;
    }

    int leaveMulticastGroup(const SocketMulticastMembership& membership) noexcept override
    {
        if (!multicast_joined_) {
            last_error_ = SocketError::none;
            return 0;
        }

        if (!same_membership(membership, multicast_membership_)) {
            last_error_ = SocketError::invalid_argument;
            return -1;
        }

        if (open_) {
            last_error_ = SocketError::not_supported;
            return -1;
        }

        multicast_membership_ = {};
        multicast_joined_ = false;
        last_error_ = SocketError::none;
        return 0;
    }

    int bind(const SocketAddress& local) noexcept override
    {
        if (local.domain != domain_) {
            last_error_ = SocketError::not_supported;
            return -1;
        }

        // Port 0 means "any": the chip has no ephemeral-port allocator, and a
        // literal source port of 0 makes replying hosts drop the datagram
        // (Linux sendto() → EINVAL), so pick one in the RFC 6335 dynamic range.
        const uint16_t port = local.port != 0u ? local.port : ephemeral_port();

        // Sends take their MAC from the ARP cache unless the socket is a
        // multicast member on a part whose Sn_DHAR then holds the group. The
        // socket keeps the resolver it opened with until close(), so its chip
        // mode and its sends agree even if the provider's resolver changes.
        arp_resolver_type* const arp = provider_.arp_resolver();
        const bool resolves_mac = arp != nullptr && domain_ == SocketDomain::ipv4
                                  && !(multicast_joined_ && Chip::dest_mac_per_socket());

        if (multicast_joined_) {
            if (!device_.open_udp_multicast_socket(socket_,
                                                   port,
                                                   {multicast_membership_.group[0],
                                                    multicast_membership_.group[1],
                                                    multicast_membership_.group[2],
                                                    multicast_membership_.group[3]},
                                                   multicast_membership_.port != 0u
                                                       ? multicast_membership_.port
                                                       : port)) {
                last_error_ = SocketError::io;
                return -1;
            }
        } else if (!device_.open_udp_socket(socket_, port, domain_ == SocketDomain::ipv6,
                                            resolves_mac)) {
            last_error_ = SocketError::io;
            return -1;
        }

        arp_ = resolves_mac ? arp : nullptr;
        open_ = true;
        last_error_ = SocketError::none;
        return 0;
    }

    int set_receive_timeout(uint32_t timeout_ms) noexcept override
    {
        receive_timeout_ms_ = timeout_ms;
        last_error_ = SocketError::none;
        return 0;
    }

    /**
     * @brief Moves this socket to the hardware socket whose receive buffer best fits `bytes`.
     *
     * A WIZnet socket's receive buffer is fixed by its number
     * (WiznetDevice::set_rx_layout()), and the number is only claimed, not
     * opened on the chip, until bind(); so the request swaps the claim for
     * the free socket WiznetSocketProvider::reclaim() picks: the smallest
     * whose buffer holds `bytes`, otherwise the largest.
     *
     * @param[in] bytes Receive buffering wanted, in bytes.
     * @return 0; SocketError::invalid_argument once bound; SocketError::closed
     *   after close().
     *
     * Verification:
     * - Correct means: the socket ends on the best-fitting free buffer, and
     *   the socket it held is free again unless it was that best fit.
     * - Covered by: TC-NET-WIZ-LAYOUT-002.
     */
    int set_receive_buffer_size(std::size_t bytes) noexcept override
    {
        if (!claimed_) {
            last_error_ = SocketError::closed;
            return static_cast<int>(last_error_);
        }

        if (open_) {
            last_error_ = SocketError::invalid_argument;
            return static_cast<int>(last_error_);
        }

        socket_ = provider_.reclaim(socket_, bytes);
        last_error_ = SocketError::none;
        return 0;
    }

    std::size_t receive_buffer_size() const noexcept override
    {
        return device_.socket_rx_capacity(socket_);
    }

    std::ptrdiff_t send_to(const void* data,
                           std::size_t length,
                           const SocketAddress& remote) noexcept override
    {
        if (!open_) {
            last_error_ = SocketError::closed;
            return -1;
        }

        if (remote.domain != domain_) {
            last_error_ = SocketError::not_supported;
            return -1;
        }

        if (device_.socket_tx_capacity(socket_) == 0u) {
            last_error_ = SocketError::not_supported;
            return -1;
        }

        UdpEndpoint peer{};
        peer.ipv6 = remote.is_ipv6();
        peer.address = remote.address;
        peer.port = remote.port;
        std::array<uint8_t, 6> mac{};
        const std::array<uint8_t, 6>* dest_mac = nullptr;
        if (arp_ != nullptr) {
            using Route = typename arp_resolver_type::Route;
            const Route route = arp_->route(peer, mac);
            // Dropped while the next hop is resolved: to the caller it is a datagram
            // lost on the wire, and would_block tells one that cares it never left.
            if (route == Route::drop) {
                last_error_ = SocketError::would_block;
                return static_cast<std::ptrdiff_t>(length);
            }

            dest_mac = route == Route::to_mac ? &mac : nullptr;
        }

        const std::ptrdiff_t rc =
            device_.udp_send_to(socket_, static_cast<const uint8_t*>(data), length, peer, dest_mac);
        last_error_ = rc < 0 ? SocketError::io : SocketError::none;
        return rc;
    }

    /// @brief True when the chip holds a received datagram for this socket.
    bool rx_pending() noexcept { return open_ && device_.socket_rx_available(socket_) > 0u; }

    /**
     * @brief Reads one datagram, waiting up to the configured receive timeout.
     *
     * Between checks the calling thread sleeps on this socket's event, which
     * the provider signals from the chip's interrupt
     * (WiznetSocketProvider::wait_rx()). The loop owns the deadline and
     * re-checks the chip after each wake.
     */
    std::ptrdiff_t recv_from(void* data,
                             std::size_t capacity,
                             SocketAddress* remote) noexcept override
    {
        if (!open_) {
            last_error_ = SocketError::closed;
            return -1;
        }

        if (device_.socket_rx_capacity(socket_) == 0u) {
            last_error_ = SocketError::not_supported;
            return -1;
        }

        const uint32_t start = rosette::platform::monotonic_ms();
        typename provider_type::RxWaiter waiter(provider_, socket_, rx_event_);
        for (;;) {
            UdpEndpoint source{};
            const std::ptrdiff_t rc = device_.udp_receive_from(
                socket_, static_cast<uint8_t*>(data), capacity, &source);

            if (rc != 0) {
                provider_.rearm_rx_interrupts(&rx_event_);
                if (rc > 0 && provider_.arp_resolver() != nullptr) {
                    provider_.arp_resolver()->heard_from(source);
                }

                if ((rc >= 0) && (remote != nullptr)) {
                    *remote = source.ipv6 ? make_socket_address_v6(source.address, source.port)
                                          : make_socket_address_v4(source.ipv4(), source.port);
                }
                last_error_ = rc < 0 ? SocketError::io : SocketError::none;
                return rc;
            }

            if (receive_timeout_ms_ == 0u) {
                last_error_ = SocketError::would_block;
                return -1;
            }

            const uint32_t elapsed = rosette::platform::monotonic_ms() - start;
            if (elapsed >= receive_timeout_ms_) {
                last_error_ = SocketError::timed_out;
                return -1;
            }

            provider_.wait_rx(socket_, rx_event_, receive_timeout_ms_ - elapsed);
        }
    }

    void close() noexcept override
    {
        if (!claimed_) {
            return;
        }

        if (open_) {
            device_.close_socket(socket_);
        }

        provider_.release(socket_);
        open_ = false;
        arp_ = nullptr;
        claimed_ = false;
        multicast_joined_ = false;
        multicast_membership_ = {};
        last_error_ = SocketError::none;
    }

    SocketError last_error() const noexcept override
    {
        return last_error_;
    }

private:
    static bool same_membership(const SocketMulticastMembership& lhs,
                                const SocketMulticastMembership& rhs) noexcept
    {
        return (lhs.domain == rhs.domain) &&
               (lhs.group == rhs.group) &&
               (lhs.interface == rhs.interface) &&
               (lhs.port == rhs.port) &&
               (lhs.interface_index == rhs.interface_index);
    }

    /// A port in the RFC 6335 dynamic range, spread by hardware socket and boot
    /// time so a re-bind after a reset does not reuse an in-flight port.
    uint16_t ephemeral_port() const noexcept
    {
        const uint32_t spread =
            rosette::platform::monotonic_ms() + (static_cast<uint32_t>(socket_) * 2654435761u);
        return static_cast<uint16_t>(49152u + (spread % 16384u));
    }

    device_type& device_;
    provider_type& provider_;
    /// Signaled from the chip's interrupt while a recv_from() waits on this socket.
    rosette::platform::Event rx_event_;
    socket_type socket_ = 0u;
    SocketDomain domain_ = SocketDomain::ipv4;
    uint32_t receive_timeout_ms_ = 0u;
    SocketMulticastMembership multicast_membership_{};
    SocketError last_error_ = SocketError::none;
    bool open_ = false;
    /// The resolver this socket's sends route through, fixed at bind(); null when the chip ARPs.
    arp_resolver_type* arp_ = nullptr;
    bool claimed_ = true;
    bool multicast_joined_ = false;
};

/**
 * @brief Hands out the chip's hardware sockets as `Socket` objects.
 */
template <typename Chip, typename Bus>
class WiznetSocketProvider final : public SocketProvider {
public:
    using device_type = WiznetDevice<Chip, Bus>;
    using socket_type = typename device_type::socket_type;

    explicit WiznetSocketProvider(device_type& device) noexcept : device_(device) {}

    WiznetSocketProvider(const WiznetSocketProvider&) = delete;
    WiznetSocketProvider& operator=(const WiznetSocketProvider&) = delete;

    std::unique_ptr<Socket> open(
        SocketDomain domain = SocketDomain::ipv4,
        SocketType type = SocketType::datagram,
        SocketProtocol protocol = SocketProtocol::udp) noexcept override
    {
        const bool family_supported =
            (domain == SocketDomain::ipv4) ||
            (domain == SocketDomain::ipv6 && Chip::has_hw_ipv6());
        if (!family_supported ||
            (type != SocketType::datagram) ||
            (protocol != SocketProtocol::udp)) {
            return {};
        }

        if (!ensure_started()) {
            return {};
        }

        const socket_type socket = claim_socket();
        if (socket >= Chip::socket_count()) {
            return {};
        }

        auto instance = std::unique_ptr<Socket>(
            new (std::nothrow) WiznetUdpSocket<Chip, Bus>(device_, *this, socket, domain));
        if (!instance) {
            release(socket);
        }
        return instance;
    }

    /**
     * @brief Routes every IPv4 socket bound from now on through `resolver`'s ARP cache.
     *
     * Attach before the first socket binds: a socket's mode (SEND_MAC per
     * datagram on the W5100S, Sn_MR2.DHAM_MANUAL on the W6x00) is fixed when
     * it opens on the chip, and so is the resolver it routes through. A
     * socket already bound keeps the resolver it bound with until it closes,
     * so attaching another or detaching affects only sockets bound later.
     *
     * @param[in] resolver The board's resolver, outliving the provider and
     *   every socket it hands out; null detaches.
     *
     * Verification:
     * - Correct means: a socket bound after the call routes its unicast sends
     *   through `resolver`, carrying the cached MAC and sending nothing on a
     *   miss; a socket bound before it is unaffected.
     * - Covered by: TC-NET-WIZ-ARP-010, TC-NET-WIZ-ARP-014, TC-NET-WIZ-ARP-018.
     */
    void attach_arp_resolver(WiznetArpResolver<Chip, Bus>* resolver) noexcept { arp_ = resolver; }

    /// @brief The attached ARP resolver, or null.
    WiznetArpResolver<Chip, Bus>* arp_resolver() const noexcept { return arp_; }

    /**
     * @brief Routes the chip's INT pin to every thread waiting on this provider's sockets.
     *
     * Each falling edge signals the event registered with
     * set_receive_event() and the event of each socket blocked in
     * recv_from(). Attach once the chip answers; until then, and on a chip
     * port without GPIO interrupts, waits poll the chip's registers instead.
     *
     * @param[in] pin GPIO wired to the chip's active-low INT output.
     * @return Whether the interrupt is attached.
     *
     * Verification:
     * - Correct means: a datagram sent to the board ends a kWaitForever
     *   wait_readable(), or a blocking recv_from(), within interrupt latency.
     * - Covered by: examples/wiznet_udp_echo (wait_readable()) and its SNTP
     *   exchange (recv_from()), examples/rtps_chatter and examples/rclc_chatter.
     */
    bool attach_interrupt(uint32_t pin) noexcept {
        interrupt_attached_ = rosette::platform::attach_falling_edge(
            pin,
            [](void* provider) {
                static_cast<WiznetSocketProvider*>(provider)->interrupt_from_isr();
            },
            this);
        return interrupt_attached_;
    }

    /**
     * @brief Sleeps until a listed socket holds a datagram, `wake` is
     * signaled, or `timeout_ms` passes.
     *
     * The participant driver sleeps here between protocol passes. With the
     * chip's INT pin attached (attach_interrupt()) and `wake` registered
     * through set_receive_event(), every flagged RECV interrupt is cleared
     * (rearm_rx_interrupts()), each socket's received-size register read
     * once, and the core then sleeps on `wake` for the whole timeout: the next
     * datagram asserts INT, whose interrupt signals `wake`. Clearing before
     * reading means a datagram landing after the read still produces an edge,
     * so none is missed. Without the interrupt the registers are read again
     * every kRxPollSliceMs while sleeping on `wake` in between.
     *
     * @param[in] sockets    Sockets opened by this provider (entries may be
     *   null); each is a WiznetUdpSocket by construction.
     * @param[in] count      Number of entries in `sockets`.
     * @param[in] wake       Event that ends the wait early.
     * @param[in] timeout_ms Longest wait; platform::kWaitForever blocks until
     *   data or `wake`.
     *
     * Verification:
     * - Correct means: returns within interrupt latency (INT attached) or
     *   kRxPollSliceMs (no INT) of a datagram reaching any listed socket, and
     *   immediately when `wake` is signaled.
     * - Covered by: examples/rtps_chatter on the W5100S-EVB-Pico2 (command
     *   echo against Fast DDS).
     *
     * References: W5100S datasheet §Sn_IR, §INT pin.
     */
    void wait_readable(Socket* const* sockets, std::size_t count,
                       rosette::platform::Event& wake,
                       uint32_t timeout_ms) noexcept override
    {
        using udp_socket = WiznetUdpSocket<Chip, Bus>;
        const bool interrupt_driven = interrupt_attached_ && (receive_event_ == &wake);
        const uint32_t start = rosette::platform::monotonic_ms();
        for (;;) {
            if (interrupt_driven) {
                rearm_rx_interrupts(&wake);
            }

            for (std::size_t i = 0; i < count; ++i) {
                if (sockets[i] != nullptr && static_cast<udp_socket*>(sockets[i])->rx_pending()) {
                    return;
                }
            }

            if (interrupt_driven) {
                (void)wake.wait_for(timeout_ms);
                return;
            }

            const uint32_t elapsed = rosette::platform::monotonic_ms() - start;
            if (timeout_ms != rosette::platform::kWaitForever && elapsed >= timeout_ms) {
                return;
            }

            uint32_t slice = kRxPollSliceMs;
            if (timeout_ms != rosette::platform::kWaitForever && timeout_ms - elapsed < slice) {
                slice = timeout_ms - elapsed;
            }

            if (wake.wait_for(slice)) {
                return;
            }
        }
    }

    /**
     * @brief Registers a socket's event for the chip's interrupt while a recv_from() waits.
     *
     * The interrupt reads the registration on every edge, so the event is
     * signaled only while the waiter is in scope.
     */
    class RxWaiter {
    public:
        RxWaiter(WiznetSocketProvider& provider, socket_type socket,
                 rosette::platform::Event& event) noexcept
            : slot_(provider.rx_waiters_[socket]) {
            slot_.store(&event, std::memory_order_release);
        }

        ~RxWaiter() { slot_.store(nullptr, std::memory_order_release); }

        RxWaiter(const RxWaiter&) = delete;
        RxWaiter& operator=(const RxWaiter&) = delete;

    private:
        std::atomic<rosette::platform::Event*>& slot_;
    };

    /**
     * @brief Sleeps until `socket` may hold a datagram, or at most `timeout_ms`.
     *
     * Rearms the chip's interrupts before the last check for data, so a
     * datagram landing after the check signals `event`. The sleep is capped
     * at kRxBackstopMs with the interrupt attached, for the edge a datagram
     * cannot raise while another thread's unread socket holds INT low, and at
     * kRxPollSliceMs without it. The caller registers `event` (RxWaiter) and
     * re-checks the chip after the return.
     *
     * @param[in] socket     The socket the caller reads.
     * @param[in] event      The caller's event, registered for `socket`.
     * @param[in] timeout_ms Rest of the caller's receive timeout.
     */
    void wait_rx(socket_type socket, rosette::platform::Event& event,
                 uint32_t timeout_ms) noexcept {
        if (interrupt_attached_) {
            rearm_rx_interrupts(&event);
        }

        if (device_.socket_rx_available(socket) > 0u) {
            return;
        }

        const uint32_t cap = interrupt_attached_ ? kRxBackstopMs : kRxPollSliceMs;
        (void)event.wait_for(timeout_ms < cap ? timeout_ms : cap);
    }

    /**
     * @brief Clears every flagged RECV interrupt and wakes each waiter that may have missed its
     * edge.
     *
     * While any socket's interrupt is pending, INT stays low and no other
     * socket's datagram can raise a falling edge, so a thread about to sleep
     * clears them all (WiznetDevice::clear_rx_interrupts()). A flag it clears
     * may belong to a datagram that arrived while INT was already low, whose
     * waiter was never signaled, so every registered event but `self` is
     * signaled whenever a flag was found. Each woken thread re-reads its own
     * socket; a spurious wake costs one register read.
     *
     * @param[in] self The caller's own event, not signaled; may be null.
     */
    void rearm_rx_interrupts(const rosette::platform::Event* self) noexcept {
        if (!interrupt_attached_ || device_.clear_rx_interrupts() == 0u) {
            return;
        }

        rosette::platform::Event* const registered = receive_event_;
        if (registered != nullptr && registered != self) {
            registered->signal();
        }

        for (auto& slot : rx_waiters_) {
            rosette::platform::Event* const event = slot.load(std::memory_order_acquire);
            if (event != nullptr && event != self) {
                event->signal();
            }
        }
    }

    /**
     * @brief Claims a free hardware socket for a non-UDP mode (e.g. IPRAW6).
     *
     * Chip-level protocols that are not `Socket`s, such as the SLAAC
     * provider's router-advertisement reader, take a hardware socket here for
     * as long as they run and give it back with release(), so they share the
     * chip's sockets with every UDP user.
     *
     * @return Socket index, or `Chip::socket_count()` when none is free.
     */
    socket_type claim_raw_socket() noexcept
    {
        if (!ensure_started()) return Chip::socket_count();
        return claim_socket();
    }

    void release(socket_type socket) noexcept
    {
        if (socket < Chip::socket_count()) {
            rosette::platform::LockGuard guard(claim_lock_);
            claimed_[socket] = false;
        }
    }

    /**
     * @brief Trades the claimed, unopened socket `held` for the free socket that best fits `bytes`.
     *
     * Backs WiznetUdpSocket::set_receive_buffer_size(). `held` counts as
     * free, so it is kept when it is already the best fit.
     *
     * @param[in] held  A socket this provider handed out and the chip has not opened.
     * @param[in] bytes Receive buffering wanted, in bytes.
     * @return The socket now claimed in place of `held`.
     */
    socket_type reclaim(socket_type held, std::size_t bytes) noexcept
    {
        rosette::platform::LockGuard guard(claim_lock_);
        claimed_[held] = false;
        const socket_type best = best_free_socket(bytes);
        const socket_type taken = best < Chip::socket_count() ? best : held;
        claimed_[taken] = true;
        return taken;
    }

private:
    /**
     * @brief One-time chip init (socket mode).
     *
     * Called on the first `open()`. If the device is already in socket mode —
     * because `WiznetLink` (or the caller) ran bring-up — the chip init is
     * skipped so its addressing is not reset.
     */
    bool ensure_started() noexcept
    {
        if (device_.mode() != DeviceMode::socket) {
            const auto mac = device_.mac_address();
            auto result = device_.init_socket_mode(mac);
            if (std::holds_alternative<typename device_type::InvalidChipVersion>(result)) {
                return false;
            }
        }

        return true;
    }

    /// @brief Signals every registered event; runs in interrupt context and touches no bus.
    void interrupt_from_isr() noexcept {
        rosette::platform::Event* const registered = receive_event_;
        if (registered != nullptr) {
            registered->signal_from_isr();
        }

        for (auto& slot : rx_waiters_) {
            rosette::platform::Event* const event = slot.load(std::memory_order_acquire);
            if (event != nullptr) {
                event->signal_from_isr();
            }
        }
    }

    /**
     * @brief Claims the free hardware socket with the smallest receive buffer, or `Chip::socket_count()` if none.
     *
     * Sockets are sized unevenly (WiznetDevice::set_rx_layout()), and one
     * that asks for more (Socket::set_receive_buffer_size()) does so after
     * open(), so a socket that asks for nothing must leave the large buffers
     * free: whichever protocol happens to open first (DHCP, usually) would
     * otherwise take the one the RTPS user socket needs.
     */
    socket_type claim_socket() noexcept
    {
        rosette::platform::LockGuard guard(claim_lock_);
        const socket_type socket = best_free_socket(0u);
        if (socket < Chip::socket_count()) {
            claimed_[socket] = true;
        }

        return socket;
    }

    /**
     * @brief The free socket whose receive buffer best fits `bytes`, or `Chip::socket_count()` if none.
     *
     * The best fit is the smallest buffer of at least `bytes`, or, when none
     * is that large, the largest buffer; ties go to the lower socket number.
     * Sockets with no receive buffer, and socket 0 while the device is in
     * MACRAW mode, are never picked. The caller holds the critical section.
     */
    socket_type best_free_socket(std::size_t bytes) const noexcept
    {
        socket_type fit = Chip::socket_count();
        socket_type largest = Chip::socket_count();
        for (socket_type socket = 0; socket < Chip::socket_count(); ++socket) {
            const std::size_t size = device_.socket_rx_capacity(socket);
            const bool reserved =
                device_.mode() == DeviceMode::macraw && socket == Chip::macraw_socket();
            if (claimed_[socket] || reserved || size == 0u) {
                continue;
            }

            if (size >= bytes && (fit == Chip::socket_count() || size < device_.socket_rx_capacity(fit))) {
                fit = socket;
            }

            if (largest == Chip::socket_count() || size > device_.socket_rx_capacity(largest)) {
                largest = socket;
            }
        }

        return fit < Chip::socket_count() ? fit : largest;
    }

    /// Register re-read period for boards with no INT line (see wait_readable()).
    static constexpr uint32_t kRxPollSliceMs = 2u;
    /// Longest interrupt-driven sleep in recv_from() before re-reading the chip (see wait_rx()).
    static constexpr uint32_t kRxBackstopMs = 20u;

    device_type& device_;
    WiznetArpResolver<Chip, Bus>* arp_ = nullptr;
    /// Guards `claimed_` against concurrent open() / close().
    rosette::platform::Mutex claim_lock_;
    std::array<bool, Chip::socket_count()> claimed_{};
    /// Per socket, the event of the thread blocked in its recv_from(), or null.
    std::array<std::atomic<rosette::platform::Event*>, Chip::socket_count()> rx_waiters_{};
    /// Set once attach_interrupt() succeeds; written before the chip's first edge.
    bool interrupt_attached_ = false;
};

}  // namespace rosette::net::wiznet
