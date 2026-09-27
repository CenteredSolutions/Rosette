/**
 * @file net/wiznet/socket_provider.hpp
 * @brief Template `SocketProvider` over the WIZnet hardware UDP sockets.
 *
 * `WiznetSocketProvider<Chip, Bus, Executor>` hands out the chip's hardware
 * sockets as `rosette::net::Socket` objects. All of the register-level work
 * lives in `WiznetDevice<Chip, Bus>`; this layer maps the `Socket` vtable onto
 * it and tracks which hardware sockets are in use.
 *
 *   WiznetDevice<W5100S, PicoSpiBus> device{bus};
 *   WiznetSocketProvider<W5100S, PicoSpiBus> sockets{device};
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
 * The @p Executor policy (see `executor.hpp`) decides how `recv_from()` waits:
 * a cooperative poll in a bare superloop, or a task notification under
 * FreeRTOS. It defaults to whichever fits the build's platform (`ROSETTE_OS_FREERTOS`);
 * pass `PollingExecutor` explicitly to force poll mode even under an RTOS.
 */

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <new>

#include "rosette/platform/host_runtime.hpp"
#include "rosette/platform/platform.hpp"

#include "device.hpp"
#include "executor.hpp"
#include "../socket.hpp"

namespace rosette::net::wiznet {

template<typename Chip, typename Bus,
         template<typename> class ExecutorT = DefaultExecutor>
class WiznetSocketProvider;

/**
 * @brief One hardware UDP socket, optionally bound to a multicast group.
 */
template<typename Chip, typename Bus, template<typename> class ExecutorT>
class WiznetUdpSocket final : public Socket {
public:
    using device_type = WiznetDevice<Chip, Bus>;
    using provider_type = WiznetSocketProvider<Chip, Bus, ExecutorT>;
    using executor_type = ExecutorT<device_type>;
    using socket_type = typename device_type::socket_type;

    WiznetUdpSocket(device_type& device,
                    provider_type* provider,
                    executor_type& executor,
                    socket_type socket,
                    SocketDomain domain) noexcept
        : device_(device)
        , provider_(provider)
        , executor_(executor)
        , socket_(socket)
        , domain_(domain) {}

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
        } else if (!device_.open_udp_socket(socket_, port, domain_ == SocketDomain::ipv6)) {
            last_error_ = SocketError::io;
            return -1;
        }

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
        const std::ptrdiff_t rc =
            device_.udp_send_to(socket_, static_cast<const uint8_t*>(data), length, peer);
        last_error_ = rc < 0 ? SocketError::io : SocketError::none;
        return rc;
    }

    /**
     * @brief Reads one datagram, waiting up to the configured receive timeout.
     *
     * The wait itself is delegated to the @c Executor: a cooperative sleep in
     * poll mode, a task notification under FreeRTOS. Either way this loop owns
     * the deadline and re-checks the chip after each wake.
     */
    /**
     * @brief True when the chip holds a received datagram for this socket.
     *
     * @param[in] rearm Clear the socket's RECV interrupt first, so a datagram
     *   arriving after this check asserts INT again.
     */
    bool rx_pending(bool rearm) noexcept
    {
        if (!open_) return false;
        if (rearm) device_.rearm_socket_rx_interrupt(socket_);
        return device_.socket_rx_available(socket_) > 0u;
    }

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

        const uint32_t start = executor_type::now_ms();
        typename executor_type::RxScope rx_scope(device_, socket_);
        for (;;) {
            UdpEndpoint source{};
            const std::ptrdiff_t rc = device_.udp_receive_from(
                socket_, static_cast<uint8_t*>(data), capacity, &source);

            if (rc != 0) {
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

            const uint32_t elapsed = executor_type::now_ms() - start;
            if (elapsed >= receive_timeout_ms_) {
                last_error_ = SocketError::timed_out;
                return -1;
            }

            executor_.wait_rx(receive_timeout_ms_ - elapsed);
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

        if (provider_ != nullptr) {
            provider_->release(socket_);
        }

        open_ = false;
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
        const uint32_t spread = rosette_platform_get_ms() +
                                (static_cast<uint32_t>(socket_) * 2654435761u);
        return static_cast<uint16_t>(49152u + (spread % 16384u));
    }

    device_type& device_;
    provider_type* provider_ = nullptr;
    executor_type& executor_;
    socket_type socket_ = 0u;
    SocketDomain domain_ = SocketDomain::ipv4;
    uint32_t receive_timeout_ms_ = 0u;
    SocketMulticastMembership multicast_membership_{};
    SocketError last_error_ = SocketError::none;
    bool open_ = false;
    bool claimed_ = true;
    bool multicast_joined_ = false;
};

/**
 * @brief Hands out the chip's hardware sockets as `Socket` objects.
 */
template<typename Chip, typename Bus, template<typename> class ExecutorT>
class WiznetSocketProvider final : public SocketProvider {
public:
    using device_type = WiznetDevice<Chip, Bus>;
    using executor_type = ExecutorT<device_type>;
    using socket_type = typename device_type::socket_type;

    explicit WiznetSocketProvider(device_type& device) noexcept
        : device_(device)
        , executor_(device) {}

    ~WiznetSocketProvider() override
    {
        executor_.stop();
    }

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
            new (std::nothrow) WiznetUdpSocket<Chip, Bus, ExecutorT>(
                device_, this, executor_, socket, domain));
        if (!instance) {
            release(socket);
        }
        return instance;
    }

    /**
     * @brief Sleeps until a listed socket holds a datagram, `wake` is
     * signaled, or `timeout_ms` passes.
     *
     * The participant driver sleeps here between protocol passes. With the
     * chip's INT pin routed to `wake` (attach_rx_interrupt(), and the event
     * registered through set_receive_event()), each socket's RECV interrupt
     * is cleared and its received-size register read once, and the core then
     * sleeps on `wake` for the whole timeout: the next datagram asserts INT,
     * whose interrupt signals `wake`. Clearing before reading means a datagram
     * landing after the read still produces an edge, so none is missed.
     * A board without an INT line is served by reading the registers again
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
     * - Correct means: returns within interrupt latency (INT wired) or
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
        using udp_socket = WiznetUdpSocket<Chip, Bus, ExecutorT>;
        const bool interrupt_driven = (receive_event_ == &wake);
        const uint32_t start = executor_type::now_ms();
        for (;;) {
            for (std::size_t i = 0; i < count; ++i) {
                if (sockets[i] != nullptr &&
                    static_cast<udp_socket*>(sockets[i])->rx_pending(interrupt_driven)) {
                    return;
                }
            }
            if (interrupt_driven) {
                (void)wake.wait_for(timeout_ms);
                return;
            }
            const uint32_t elapsed = executor_type::now_ms() - start;
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
            typename executor_type::CriticalSection guard;
            claimed_[socket] = false;
        }
    }

private:
    /**
     * @brief One-time chip init (socket mode) and dispatch-lifecycle start.
     *
     * Called on the first `open()`. If the device is already in socket mode —
     * because `WiznetLink` (or the caller) ran bring-up — the chip init is
     * skipped so its addressing is not reset. Provider setup is expected to run
     * from a single context, so the started flag is not itself lock-guarded.
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

        if (!executor_started_) {
            if (!executor_.start()) {
                return false;
            }
            executor_started_ = true;
        }
        return true;
    }

    /**
     * @brief Claims a free hardware socket, or `Chip::socket_count()` if none.
     *
     * Socket 0 is reserved while the device is in MACRAW mode.
     */
    socket_type claim_socket() noexcept
    {
        typename executor_type::CriticalSection guard;
        for (socket_type socket = 0; socket < Chip::socket_count(); ++socket) {
            if ((device_.mode() == DeviceMode::macraw) &&
                (socket == Chip::macraw_socket())) {
                continue;
            }

            if (!claimed_[socket]) {
                claimed_[socket] = true;
                return socket;
            }
        }
        return Chip::socket_count();
    }

    /// Register re-read period for boards with no INT line (see wait_readable()).
    static constexpr uint32_t kRxPollSliceMs = 2u;

    device_type& device_;
    executor_type executor_;
    std::array<bool, Chip::socket_count()> claimed_{};
    bool executor_started_ = false;
};

}  // namespace rosette::net::wiznet
