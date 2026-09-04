#pragma once

/**
 * @file ros/timer.hpp
 * @brief Executor-backed periodic timers for the micro-ROS backend.
 *
 * Design goal
 * -----------
 * This module exposes a simple `create_timer(period_ms, callback)` surface that
 * mirrors Rosette's existing API while using the upstream `rclc` timer and
 * executor machinery under the hood.
 *
 * Example
 * -------
 *   auto timer = node.create_timer(100, []() noexcept {
 *       // Publish, poll hardware, or advance a small state machine.
 *   });
 *
 * Caveats
 * -------
 * - Timer callbacks run on the shared executor path, so they should stay short
 *   and deterministic.
 * - The timer period is configured in whole milliseconds and converted to
 *   nanoseconds for `rclc`.
 */

#include "common.hpp"
#include "context.hpp"

#include "rosette/support/inplace_function.hpp"

namespace rosette {

class Timer {
public:
    /// Type-erased timer callback.
    using Callback = InplaceFunction<void(), 16u>;

    Timer() noexcept = default;

    /**
     * @brief Constructs a periodic timer attached to a running context.
     *
     * The timer is registered with the shared `rclc_executor_t`, which invokes
     * the callback whenever the configured period elapses.
     *
     * @param[in] context Running micro-ROS context whose executor owns the timer.
     * @param[in] period_ms Timer period in milliseconds.
     * @param[in] callback Callable invoked by the executor on timer expiry.
     */
    template<typename F>
    Timer(Context* context,
          uint32_t period_ms,
          F&& callback) noexcept
        : context_(context)
        , callback_(static_cast<F&&>(callback))
    {
        if (context_ == nullptr || period_ms == 0u) {
            return;
        }

        timer_ = rcl_get_zero_initialized_timer();
        if (!ros_ok(rclc_timer_init_default(
                &timer_,
                &context_->support(),
                static_cast<uint64_t>(period_ms) * 1000000ull,
                &Timer::on_timer))) {
            timer_ = rcl_get_zero_initialized_timer();
            return;
        }

        if (!ros_ok(rclc_executor_add_timer(&context_->executor(), &timer_))) {
            (void)ros_ok(rcl_timer_fini(&timer_));
            timer_ = rcl_get_zero_initialized_timer();
            return;
        }

        if (!register_timer(&timer_, this)) {
            (void)ros_ok(rclc_executor_remove_timer(&context_->executor(), &timer_));
            (void)ros_ok(rcl_timer_fini(&timer_));
            timer_ = rcl_get_zero_initialized_timer();
            return;
        }
        valid_ = true;
    }

    Timer(const Timer&) = delete;
    Timer& operator=(const Timer&) = delete;
    Timer(Timer&&) = delete;
    Timer& operator=(Timer&&) = delete;

    ~Timer()
    {
        fini();
    }

    bool valid() const noexcept { return valid_; }

private:
    /**
     * @brief Bridges the plain-C `rclc` timer callback to the stored C++ callable.
     *
     * Humble's `rclc_timer_init_default()` does not expose a user context
     * parameter, and `rcl_timer_t` keeps its implementation opaque. The wrapper
     * therefore resolves the timer handle through a small static registry.
     *
     * @param[in] timer Timer instance provided by `rclc`.
     * @param[in] last_call_time Unused elapsed-time parameter from `rclc`.
     */
    static void on_timer(rcl_timer_t* timer, int64_t last_call_time) noexcept
    {
        (void)last_call_time;
        Timer* self = find_registered_timer(timer);
        if (self == nullptr) {
            return;
        }
        self->callback_();
    }

    /**
     * @brief Records the wrapper that owns an initialized `rcl_timer_t`.
     *
     * The registry is a small fixed-capacity table because timer callbacks in
     * Humble identify only the C timer handle. It is updated during timer
     * construction and teardown on the same cooperative executor path used by
     * the rest of this backend, so it assumes application code does not create
     * or destroy timers concurrently with `Context::tick()`.
     *
     * @param[in] timer Timer handle owned by the wrapper.
     * @param[in] owner C++ timer wrapper that should receive callbacks.
     * @return True when the timer was recorded or was already present.
     */
    static bool register_timer(rcl_timer_t* timer, Timer* owner) noexcept
    {
        if (timer == nullptr || owner == nullptr) {
            return false;
        }

        for (TimerRegistration& registration : registrations_) {
            if (registration.timer == timer) {
                registration.owner = owner;
                return true;
            }
        }
        for (TimerRegistration& registration : registrations_) {
            if (registration.timer == nullptr) {
                registration.timer = timer;
                registration.owner = owner;
                return true;
            }
        }
        return false;
    }

    /**
     * @brief Finds the wrapper that owns an `rcl_timer_t` callback handle.
     *
     * The executor calls this from the plain-C timer callback. A missing entry
     * means the timer is not owned by this wrapper type or teardown has already
     * removed it from the registry.
     *
     * @param[in] timer Timer handle supplied by `rcl`.
     * @return The owning timer wrapper, or `nullptr` when no registration exists.
     */
    static Timer* find_registered_timer(rcl_timer_t* timer) noexcept
    {
        if (timer == nullptr) {
            return nullptr;
        }
        for (TimerRegistration& registration : registrations_) {
            if (registration.timer == timer) {
                return registration.owner;
            }
        }
        return nullptr;
    }

    /**
     * @brief Removes a timer handle from the callback registry.
     *
     * Teardown unregisters before finalizing the underlying `rcl_timer_t` so a
     * later callback cannot dereference an owner that has released its handle.
     *
     * @param[in] timer Timer handle to remove from the registry.
     */
    static void unregister_timer(rcl_timer_t* timer) noexcept
    {
        if (timer == nullptr) {
            return;
        }
        for (TimerRegistration& registration : registrations_) {
            if (registration.timer == timer) {
                registration.timer = nullptr;
                registration.owner = nullptr;
                return;
            }
        }
    }

    /**
     * @brief Removes the timer from the executor and finalizes the `rcl_timer_t`.
     *
     * Timer teardown is symmetrical with setup: unregister from the executor
     * first, then release the timer handle.
     */
    void fini() noexcept
    {
        if (valid_) {
            unregister_timer(&timer_);
            (void)ros_ok(rclc_executor_remove_timer(&context_->executor(), &timer_));
            (void)ros_ok(rcl_timer_fini(&timer_));
        }
        timer_ = rcl_get_zero_initialized_timer();
        valid_ = false;
    }

    Context* context_ = nullptr;
    rcl_timer_t timer_ = rcl_get_zero_initialized_timer();
    Callback callback_{};
    bool valid_ = false;

    struct TimerRegistration {
        rcl_timer_t* timer;
        Timer* owner;
    };

    static constexpr size_t kMaxRegisteredTimers = 8u;
    inline static TimerRegistration registrations_[kMaxRegisteredTimers] = {};
};

}  // namespace rosette
