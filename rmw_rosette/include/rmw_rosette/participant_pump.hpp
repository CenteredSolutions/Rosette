/**
 * @file rmw_rosette/participant_pump.hpp
 * @brief The host thread that runs a context's participant, and the waits it serves.
 *
 * A participant advances only while something runs its ParticipantDriver:
 * it receives, answers HEARTBEATs and ACKNACKs, repairs, announces itself and
 * sends what rmw_publish() queued only during a protocol pass. On a board the
 * executor is that something: rclc creates every entity, then spins
 * rmw_wait() for the life of the firmware, and rmw_wait() runs the passes
 * (participant_driver.hpp). On a host that is not enough. A ROS program may
 * publish from a thread that never waits, block in a service call, or stop
 * spinning between bursts, and a stock peer then sees a participant that
 * stops acknowledging, repairing and announcing. Fast DDS and Cyclone DDS
 * each run receive and event threads of their own for the same reason.
 *
 * ParticipantPump is that thread for rmw_rosette on a POSIX host. It starts in
 * rmw_init() and runs the driver until rmw_context_fini(), so the participant
 * advances whether or not any thread waits.
 *
 * A thread that waits (rmw_wait(), rmw_publisher_wait_for_all_acked()) runs
 * the driver too, as a firmware executor does: it sleeps on the sockets and
 * an event of its own, and the datagram it is waiting for wakes it and is
 * delivered by its own pass. While at least one thread waits this way, the
 * pump sleeps on the driver's wake event and the participant's timers only,
 * so it neither wakes for user data nor holds the participant lock when the
 * waiting thread needs it; the round trip pays no thread handoff. The first
 * waiter to arrive and the last to leave each call wake(), which moves the
 * pump off the sockets and back. Passes never overlap: every pass, on any
 * thread, runs under the participant lock.
 *
 * Each waiter also registers its readiness predicate. After every pass, on
 * the pump or on a waiting thread, the registered predicates are evaluated
 * under the participant lock, as ParticipantDriver::spin_until() evaluates its
 * own, and exactly the waiters whose predicate holds are signaled. A waiter
 * whose datagram another waiter drained is therefore still woken by the pass
 * that delivered it. The pump's own passes cover what only the pump observes:
 * a triggered guard condition and every rmw entry point that queues
 * protocol work wake the pump through the driver's wake event
 * (ContextUpdateLock in types.hpp), not the waiting threads. A waiter
 * therefore wakes for its own readiness or its own timeout, and an idle wait
 * still costs no CPU.
 *
 * The pump is chosen twice. The build option RMW_ROSETTE_PUMP_THREAD
 * (rmw_rosette_capacity.cmake) compiles it in: ON for a host build on the
 * POSIX OS port, whose rosette::platform::Thread runs the pump, and OFF for
 * firmware, where none of this is built. A host build then starts the pump in
 * rmw_init() unless the environment sets RMW_ROSETTE_PUMP_THREAD=0.
 *
 * Without the pump the participant runs as it does on a board: only inside
 * rmw_wait() and rmw_publisher_wait_for_all_acked(), on the calling thread
 * (ParticipantDriver::spin_until()). That is the deterministic choice for a
 * program shaped like rclc firmware, one executor thread spinning for the
 * life of the context: no second thread runs, so every pass happens on the
 * executor, between its callbacks, and a round trip pays no handoff at all.
 * Its contract is the firmware's too. A program that publishes without
 * spinning sends what it publishes but neither discovers, acknowledges nor
 * repairs until it spins again, and one thread at a time waits on a context:
 * the driver has a single wake event, so a guard condition triggered for one
 * of two concurrent waits may wake the other, and the first then sleeps
 * until the participant's next timer. Programs that wait from several
 * threads at once, as rclcpp's wait_for_service() beside a spinning
 * executor does, keep the pump.
 */

#pragma once

#include "rosette/platform/platform.hpp"

#include <atomic>
#include <cstdint>

#ifndef RMW_ROSETTE_PUMP_THREAD
#  define RMW_ROSETTE_PUMP_THREAD 0
#endif

#if RMW_ROSETTE_PUMP_THREAD

namespace rosette::rtps {
class ParticipantDriver;
}  // namespace rosette::rtps

namespace rmw_rosette {

/// @brief A readiness predicate evaluated under the participant lock; must not block.
using ReadyFunction = bool (*)(void* argument) noexcept;

class ParticipantPump {
public:
    ParticipantPump() noexcept = default;
    ParticipantPump(const ParticipantPump&) = delete;
    ParticipantPump& operator=(const ParticipantPump&) = delete;

    /**
     * @brief Starts the thread that runs `driver` until stop().
     *
     * Called by rmw_init() once the participant and its built-in writers
     * exist. The thread blocks every asynchronous signal
     * (rosette::platform::Thread), so SIGINT and its kind are delivered to
     * the program's own threads, where rclcpp and rclpy install their
     * handlers.
     *
     * @param[in] driver The context's driver; must outlive stop().
     * @return true when the thread runs; false when the OS refused to create
     *   it, and nothing was started.
     *
     * Verification:
     * - Correct means: after start(), the participant announces itself,
     *   matches, and acknowledges without any thread inside rmw_wait().
     * - Covered by: rmw_rosette/test/test_participant_pump.cpp (TC-RMW-PUMP-001),
     *   test_communication's rmw_rosette publisher and replier cases.
     *
     * References:
     * - ports/os/posix/rosette_os_port.hpp, rosette::platform::Thread
     */
    bool start(rosette::rtps::ParticipantDriver& driver) noexcept;

    /**
     * @brief Stops the thread and waits for it to exit.
     *
     * Called by rmw_context_fini() before the driver is destroyed. A pump
     * that was never started, or was stopped, is left as it is.
     *
     * Verification:
     * - Correct means: returns only after the thread has exited, so the
     *   driver it ran may be destroyed at once; init and fini can repeat.
     * - Covered by: test_rmw_implementation's TestInitShutdown.init_shutdown.
     */
    void stop() noexcept;

    /**
     * @brief Blocks until `ready(argument)` holds or `timeout_ms` passes.
     *
     * The pump's counterpart of ParticipantDriver::spin_until(), with the
     * same contract: `ready` is evaluated under the participant lock, at once
     * and after every pass, and the call returns as soon as it holds. The
     * caller runs passes itself, sleeping on the sockets and an event owned
     * by its thread, while the pump stays off the sockets; that event is
     * signaled when `ready` holds after a pass of the pump or of another
     * waiting thread. Valid only while running().
     *
     * @param[in] ready      Predicate; must not block, and may run on the pump
     *   thread while this call is in progress.
     * @param[in] argument   Passed to `ready`; must stay valid for the call.
     * @param[in] timeout_ms Longest wait; 0 checks once,
     *   rosette::platform::kWaitForever waits until `ready` holds.
     * @return true when `ready` held, false on timeout.
     *
     * Verification:
     * - Correct means: returns within one pass of `ready` becoming true;
     *   the pump never wakes its caller for a pass after which `ready` is
     *   false; the participant keeps discovering, matching and delivering
     *   while a thread is inside the call.
     * - Covered by: rmw_rosette/test/test_participant_pump.cpp (TC-RMW-PUMP-002,
     *   TC-RMW-PUMP-003, TC-RMW-PUMP-004), test_guard_condition.cpp,
     *   test_pub_sub_loopback.cpp; tests/perf/rmw_head_to_head for latency.
     *
     * References:
     * - rmw/rmw.h (Humble, Jazzy), `rmw_wait`
     */
    bool wait_until(ReadyFunction ready, void* argument, uint32_t timeout_ms) noexcept;

    /// @brief Whether start() succeeded and stop() has not run since.
    bool running() const noexcept { return driver_ != nullptr; }

private:
    /// A thread blocked in wait_until(); lives on that thread's stack while linked.
    struct Waiter {
        ReadyFunction             ready;
        void*                     argument;
        rosette::platform::Event* event;
        Waiter*                   next;
    };

    /// @brief Thread entry point; runs `self`'s driver until stop() is requested.
    static void run(void* self) noexcept;

    /// @brief Wakes every waiter but `skip` whose predicate holds; the caller holds the lock.
    void wake_ready_waiters_locked(const Waiter* skip = nullptr) noexcept;

    /// @brief Removes `waiter` from the list; the caller holds the lock.
    void unlink_locked(Waiter& waiter) noexcept;

    rosette::rtps::ParticipantDriver* driver_ = nullptr;
    rosette::platform::Thread         thread_;
    std::atomic<bool>                 stop_requested_{false};
    Waiter*                           waiters_ = nullptr;  ///< Guarded by the participant lock.
    /// Threads inside wait_until(); while any is, the pump leaves the sockets to them.
    std::atomic<uint32_t>             receiving_waiters_{0u};
};

}  // namespace rmw_rosette

#endif  // RMW_ROSETTE_PUMP_THREAD
