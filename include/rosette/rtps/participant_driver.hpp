// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

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
 *  - spin_until(ready, timeout): run one protocol pass (take waiting
 *    datagrams, tick) and evaluate `ready` under the lock; then release it
 *    and sleep in the transport until a datagram arrives, wake() is called,
 *    or the participant's next timer (heartbeat, repair, SPDP announcement,
 *    lease expiry) is due, whichever is first. rmw_wait() and
 *    rmw_publisher_wait_for_all_acked() are both this loop with different
 *    predicates.
 *  - wake(): ends a sleeping spin_until() from any thread, core, or (via
 *    wake_event().signal_from_isr()) interrupt handler.
 *
 * A pass is bounded in time, not by how much is waiting. It takes datagrams
 * alternately from the metatraffic and user-data sockets and stops taking
 * them once it has run RTPS_PUMP_PASS_BUDGET_US; datagrams still waiting are
 * taken by the next pass, which follows after RTPS_PUMP_BACKLOG_YIELD_MS. A
 * flood (a whole network of peers announcing or leaving at once) therefore
 * delays `ready`, the lock's other users, and the participant's timers by
 * about one budget and one yield, not by the time it takes to drain the
 * flood, and a discovery flood does not hold back the user data queued
 * behind it.
 *
 * The yield between backlogged passes is what keeps a flood that never
 * drains (a peer answering every ACKNACK with another HEARTBEAT, say) from
 * holding the CPU: without it the driving thread never blocks, and every
 * lower-priority thread, a watchdog's feeder among them, starves.
 * backlogged_passes() counts the passes the budget cut short. The yield does
 * not consult the participant's timers, so during a backlog a timer can fire
 * up to one yield late (on FreeRTOS, rounded up to whole ticks). That is the
 * price of letting lower-priority threads run; the RTPS timers it delays
 * (HEARTBEAT periods, response delays, resend deadlines) are advisory
 * schedules that tolerate a millisecond or a tick of lateness.
 *
 * An idle node therefore sleeps: WFE on bare metal, blocked on FreeRTOS,
 * blocked in poll() on POSIX, until data or a deadline needs it.
 *
 * On a board nothing else runs the participant, so it sends its first SPDP
 * announcement and reads its first discovery traffic in the first
 * rmw_wait(). (On a POSIX host rmw_rosette runs it from a thread of its own,
 * from rmw_init() on; see rmw_rosette/participant_pump.hpp.) A firmware
 * that creates all of its entities before it first waits (rclc does, before
 * the executor spins) is discovered with all of them, and every SEDP
 * announcement a peer replays to it arrives after its local endpoints exist.
 * That order is what lets a small EndpointMatchTable keep exactly the remote
 * endpoints the firmware matches on a network far larger than the table
 * (see Capacity in endpoint_match_table.hpp).
 */

#pragma once

#include "rosette/platform/platform.hpp"

#include "participant.hpp"
#include "transport.hpp"

#include <cstdint>

#ifndef RTPS_PUMP_PASS_BUDGET_US
/// Microseconds one protocol pass may spend taking datagrams before it ticks
/// and returns: about 20 metatraffic datagrams on a 133 MHz RP2040 with a
/// WIZnet chip on SPI, and the tick that follows sees timers at most this
/// late. 0 takes one datagram per pass.
#  define RTPS_PUMP_PASS_BUDGET_US 10000u
#endif

#ifndef RTPS_PUMP_BACKLOG_YIELD_MS
/// Milliseconds spin_until() sleeps after a pass its budget cut short, with
/// datagrams still waiting, so lower-priority threads run during a flood; on
/// FreeRTOS any nonzero value blocks for at least one tick. 0 runs the next
/// pass at once.
#  define RTPS_PUMP_BACKLOG_YIELD_MS 1u
#endif

namespace rosette::rtps {

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
     * @param[in] pass_budget_us Microseconds one pass may spend taking
     *   datagrams (RTPS_PUMP_PASS_BUDGET_US); a pass always takes at least one.
     */
    ParticipantDriver(RTPSParticipant& participant, SocketUdpTransport& transport,
                      rosette::net::SocketProvider& provider, Octet* rx_buffer,
                      uint32_t rx_capacity,
                      uint32_t pass_budget_us = RTPS_PUMP_PASS_BUDGET_US) noexcept;
    ~ParticipantDriver() noexcept;

    ParticipantDriver(const ParticipantDriver&) = delete;
    ParticipantDriver& operator=(const ParticipantDriver&) = delete;

    /// @brief The participant lock; hold it for any participant access.
    rosette::platform::Mutex& mutex() noexcept { return mutex_; }

    /// @brief Ends a sleeping spin_until() now; safe from any thread.
    void wake() noexcept { wake_.signal(); }

    /// @brief The wake event, for interrupt handlers and guard conditions.
    rosette::platform::Event& wake_event() noexcept { return wake_; }

    /// @brief Passes whose budget ran out with datagrams still waiting, since construction.
    uint32_t backlogged_passes() const noexcept { return backlogged_passes_; }

    /**
     * @brief Runs the protocol until `ready()` holds or `timeout_ms` passes.
     *
     * Each pass, with the lock held: feeds the participant the datagrams
     * waiting on the transport, up to the pass budget, calls tick(), and
     * evaluates `ready()`. Between passes the lock is released. A pass that
     * left datagrams waiting is followed by the next after a plain sleep of
     * RTPS_PUMP_BACKLOG_YIELD_MS, which the waiting datagrams do not cut
     * short; otherwise the caller sleeps until the sooner of the remaining
     * timeout and the participant's next timer, or until a datagram or
     * wake() ends the sleep.
     *
     * @param[in] ready      Predicate evaluated under the lock; must not block.
     * @param[in] timeout_ms Longest total wait; 0 runs one pass,
     *   platform::kWaitForever waits until `ready()`.
     * @return `true` when `ready()` held, `false` on timeout.
     *
     * Verification:
     * - Correct means: returns within one sleep of a datagram, wake(), or the
     *   predicate becoming true; with no datagram waiting, never sleeps past a
     *   participant timer; while a datagram it has not taken is waiting,
     *   sleeps only the backlog yield, at least RTPS_PUMP_BACKLOG_YIELD_MS
     *   and without regard to timers, once per backlogged pass, and counts
     *   that pass.
     * - Covered by: TC-DRV-PASS-001..003 (tests/spec), and
     *   rmw_rosette/test/test_guard_condition.cpp,
     *   rmw_rosette/test/test_pub_sub_loopback.cpp, tests/ros2 interop.
     */
    template <typename Ready>
    bool spin_until(Ready ready, uint32_t timeout_ms) noexcept {
        return spin_until(ready, timeout_ms, wake_);
    }

    /**
     * @brief spin_until(), sleeping on `wake`, and optionally not on the sockets.
     *
     * Lets two threads share the participant on a POSIX host
     * (rmw_rosette/participant_pump.hpp). A thread blocked in rmw_wait()
     * receives itself: it sleeps on the sockets and an event of its own, so
     * the datagram it waits for wakes it directly. Meanwhile the pump thread
     * sleeps on the driver's wake event and the participant's timers only,
     * with `receive` false, so it neither wakes for that datagram nor takes
     * the lock from the thread that does. Either thread's pass still drains
     * whatever is waiting on the sockets.
     *
     * @param[in] ready      As for spin_until().
     * @param[in] timeout_ms As for spin_until().
     * @param[in] wake       Event that ends this caller's sleep; owned by the
     *   caller and outliving the call.
     * @param[in] receive    true to also end the sleep when a datagram
     *   arrives; false to sleep until `wake`, a participant timer, or the
     *   timeout only.
     * @return As for spin_until().
     *
     * Verification:
     * - Correct means: the contract of spin_until(), with `wake` in place of
     *   wake(), and with datagrams ending the sleep only when `receive`
     *   holds; the backlog yield is the same plain sleep whatever `receive`
     *   and `wake` are; passes of two spinning threads never overlap.
     * - Covered by: rmw_rosette/test/test_participant_pump.cpp
     *   (TC-RMW-PUMP-001..004) and the rmw_rosette round-trip tests.
     */
    template <typename Ready>
    bool spin_until(Ready ready, uint32_t timeout_ms, rosette::platform::Event& wake,
                    bool receive = true) noexcept {
        const uint32_t start_ms = rosette::platform::monotonic_ms();
        for (;;) {
            Pass     pass;
            uint32_t sleep_ms;
            {
                rosette::platform::LockGuard guard(mutex_);
                pass = pump_locked();
                if (pass.backlog) {
                    ++backlogged_passes_;
                }

                if (ready()) {
                    return true;
                }

                sleep_ms = pass.backlog ? RTPS_PUMP_BACKLOG_YIELD_MS
                                        : participant_.ms_until_next_timer(pass.now_ms);
            }

            if (timeout_ms != rosette::platform::kWaitForever) {
                const uint32_t elapsed = pass.now_ms - start_ms;
                if (elapsed >= timeout_ms) {
                    return false;
                }

                const uint32_t remaining = timeout_ms - elapsed;
                if (remaining < sleep_ms) {
                    sleep_ms = remaining;
                }
            }

            if (sleep_ms == 0u) {
                continue;
            }

            // With datagrams waiting, wait_readable() would return at once
            // and the thread would never block; a plain sleep yields the CPU.
            if (pass.backlog) {
                rosette::platform::sleep_ms(sleep_ms);
            } else if (receive) {
                transport_.wait_readable(wake, sleep_ms);
            } else {
                (void)wake.wait_for(sleep_ms);
            }
        }
    }

private:
    /// The outcome of one protocol pass.
    struct Pass {
        uint32_t now_ms = 0u;      ///< Monotonic time the pass ticked at.
        bool     backlog = false;  ///< The budget ran out; datagrams may still be waiting.
    };

    /**
     * @brief One protocol pass: feeds the participant waiting datagrams, up
     *   to the pass budget, then ticks it. The caller holds mutex_.
     *
     * Takes one datagram at a time, alternating between the metatraffic
     * sockets (poll_meta()) and the user-data socket (poll_user()), and
     * carries the alternation over to the next pass, so neither kind waits
     * behind a flood of the other. Stops when both are empty, or after the
     * datagram during which the pass's budget ran out.
     *
     * @return The tick time, and whether the budget ended the pass.
     *
     * Verification:
     * - Correct means: every pass takes at least one waiting datagram; no
     *   pass takes another once pass_budget_us_ has passed since it began;
     *   with both kinds waiting, consecutive datagrams alternate kinds; a
     *   pass that emptied both sockets reports no backlog.
     * - Covered by: TC-DRV-PASS-001, TC-DRV-PASS-002, TC-DRV-PASS-003.
     */
    Pass pump_locked() noexcept;

    RTPSParticipant&              participant_;
    SocketUdpTransport&           transport_;
    rosette::net::SocketProvider& provider_;
    Octet*                        rx_buffer_;
    uint32_t                      rx_capacity_;
    uint32_t                      pass_budget_us_;
    uint32_t                      backlogged_passes_ = 0u;
    bool                          next_is_user_ = false;
    rosette::platform::Mutex      mutex_;
    rosette::platform::Event      wake_;
};

}  // namespace rosette::rtps
