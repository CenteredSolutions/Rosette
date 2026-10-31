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
 * rmw_init() and runs the driver until rmw_context_fini(); it is the only
 * thread that runs passes or sleeps in the transport. A thread that waits
 * (rmw_wait(), rmw_publisher_wait_for_all_acked()) registers its readiness
 * predicate and sleeps on an event of its own. After every pass the pump
 * evaluates each registered predicate under the participant lock, as
 * ParticipantDriver::spin_until() evaluates its own, and wakes exactly the
 * waiters whose predicate holds. A waiter therefore wakes for its own
 * readiness or its own timeout, never for a pass that changed nothing it
 * waits on, and an idle wait still costs no CPU.
 *
 * Everything that makes a pass due wakes the pump through the driver's wake
 * event: a datagram, a triggered guard condition, and every rmw entry point
 * that queues protocol work (ContextUpdateLock in types.hpp).
 *
 * RMW_ROSETTE_PUMP_THREAD selects the pump. The build sets it
 * (rmw_rosette_capacity.cmake): 1 for a host build on the POSIX OS port,
 * whose rosette::platform::Thread runs the pump, and 0 for firmware, where
 * the executor runs the passes and none of this is built.
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
     * caller sleeps on an event owned by its thread, which the pump signals
     * when `ready` holds after a pass; the predicate is checked again by the
     * caller before it returns, under the lock.
     *
     * @param[in] ready      Predicate; must not block, and may run on the pump
     *   thread while this call is in progress.
     * @param[in] argument   Passed to `ready`; must stay valid for the call.
     * @param[in] timeout_ms Longest wait; 0 checks once,
     *   rosette::platform::kWaitForever waits until `ready` holds.
     * @return true when `ready` held, false on timeout.
     *
     * Verification:
     * - Correct means: returns within one pass of `ready` becoming true, and
     *   never wakes its caller for a pass after which `ready` is false.
     * - Covered by: rmw_rosette/test/test_participant_pump.cpp (TC-RMW-PUMP-002,
     *   TC-RMW-PUMP-003), test_guard_condition.cpp, test_pub_sub_loopback.cpp.
     *
     * References:
     * - rmw/rmw.h (Humble, Jazzy), `rmw_wait`
     */
    bool wait_until(ReadyFunction ready, void* argument, uint32_t timeout_ms) noexcept;

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

    /// @brief Wakes every waiter whose predicate holds; the caller holds the lock.
    void wake_ready_waiters_locked() noexcept;

    /// @brief Removes `waiter` from the list; the caller holds the lock.
    void unlink_locked(Waiter& waiter) noexcept;

    rosette::rtps::ParticipantDriver* driver_ = nullptr;
    rosette::platform::Thread         thread_;
    std::atomic<bool>                 stop_requested_{false};
    Waiter*                           waiters_ = nullptr;  ///< Guarded by the participant lock.
};

}  // namespace rmw_rosette

#endif  // RMW_ROSETTE_PUMP_THREAD
