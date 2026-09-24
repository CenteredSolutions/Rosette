/**
 * @file rosette/rtps/participant_driver.hpp
 * @brief Runs a participant: receive, tick, and sleep until something happens.
 *
 * An RTPSParticipant advances only when someone feeds it received datagrams
 * and calls tick(). ParticipantDriver is that someone, and it is the single
 * owner of the participant lock:
 *
 *  - mutex(): every access to the participant, its endpoints, and state its
 *    callbacks touch (rmw take rings, for example) happens with this held.
 *    RTPS objects themselves take no locks.
 *  - spin_until(ready, timeout): drain every socket, tick, and evaluate
 *    `ready` under the lock; then release it and sleep in the transport until
 *    a datagram arrives, wake() is called, or the participant's next timer
 *    (heartbeat, repair, SPDP announcement, lease expiry) is due, whichever
 *    is first. rmw_wait() and rmw_publisher_wait_for_all_acked() are both
 *    this loop with different predicates.
 *  - wake(): ends a sleeping spin_until() from any thread, core, or (via
 *    wake_event().signal_from_isr()) interrupt handler.
 *
 * An idle node therefore sleeps: WFE on bare metal, blocked on FreeRTOS,
 * blocked in poll() on POSIX, until data or a deadline needs it.
 */

#pragma once

#include "rosette/platform/platform.hpp"

#include "participant.hpp"
#include "transport.hpp"

#include <cstdint>

namespace rtps {

class ParticipantDriver {
public:
    /**
     * @brief Binds a driver to a participant and its transport.
     *
     * Registers the driver's wake event as the transport backend's receive
     * event, so interrupt-driven backends wake the driver on arrival.
     *
     * @param[in] participant Participant to run; must outlive the driver.
     * @param[in] transport   The participant's opened transport.
     * @param[in] provider    Socket backend the transport was opened on.
     * @param[in] rx_buffer   Receive buffer holding one full datagram; must
     *   outlive the driver.
     * @param[in] rx_capacity Size of `rx_buffer` in octets.
     */
    ParticipantDriver(RTPSParticipant& participant, SocketUdpTransport& transport,
                      rosette::net::SocketProvider& provider, Octet* rx_buffer,
                      uint32_t rx_capacity) noexcept;
    ~ParticipantDriver() noexcept;

    ParticipantDriver(const ParticipantDriver&) = delete;
    ParticipantDriver& operator=(const ParticipantDriver&) = delete;

    /// @brief The participant lock; hold it for any participant access.
    rosette::platform::Mutex& mutex() noexcept { return mutex_; }

    /// @brief Ends a sleeping spin_until() now; safe from any thread.
    void wake() noexcept { wake_.signal(); }

    /// @brief The wake event, for interrupt handlers and guard conditions.
    rosette::platform::Event& wake_event() noexcept { return wake_; }

    /**
     * @brief Runs the protocol until `ready()` holds or `timeout_ms` passes.
     *
     * Each pass, with the lock held: drains every datagram waiting on the
     * transport into the participant, calls tick(), and evaluates `ready()`.
     * Between passes the lock is released and the caller sleeps until the
     * sooner of the remaining timeout and the participant's next timer, or
     * until a datagram or wake() ends the sleep.
     *
     * @param[in] ready      Predicate evaluated under the lock; must not block.
     * @param[in] timeout_ms Longest total wait; 0 runs one pass,
     *   platform::kWaitForever waits until `ready()`.
     * @return `true` when `ready()` held, `false` on timeout.
     *
     * Verification:
     * - Correct means: returns within one sleep of a datagram, wake(), or the
     *   predicate becoming true; never sleeps past a participant timer.
     * - Covered by: rmw_rosette/test/test_guard_condition.cpp,
     *   rmw_rosette/test/test_pub_sub_loopback.cpp, tests/ros2 interop.
     */
    template <typename Ready>
    bool spin_until(Ready ready, uint32_t timeout_ms) noexcept {
        const uint32_t start_ms = rosette::platform::monotonic_ms();
        for (;;) {
            uint32_t now_ms;
            uint32_t next_timer_ms;
            {
                rosette::platform::LockGuard guard(mutex_);
                now_ms = pump_locked();
                if (ready()) return true;
                next_timer_ms = participant_.ms_until_next_timer(now_ms);
            }
            uint32_t sleep_ms = next_timer_ms;
            if (timeout_ms != rosette::platform::kWaitForever) {
                const uint32_t elapsed = now_ms - start_ms;
                if (elapsed >= timeout_ms) return false;
                const uint32_t remaining = timeout_ms - elapsed;
                if (remaining < sleep_ms) sleep_ms = remaining;
            }
            if (sleep_ms > 0u) transport_.wait_readable(wake_, sleep_ms);
        }
    }

private:
    /// Drains all waiting datagrams and ticks; caller holds mutex_.
    /// Returns the monotonic time the pass ran at.
    uint32_t pump_locked() noexcept;

    RTPSParticipant&              participant_;
    SocketUdpTransport&           transport_;
    rosette::net::SocketProvider& provider_;
    Octet*                        rx_buffer_;
    uint32_t                      rx_capacity_;
    rosette::platform::Mutex      mutex_;
    rosette::platform::Event      wake_;
};

}  // namespace rtps
