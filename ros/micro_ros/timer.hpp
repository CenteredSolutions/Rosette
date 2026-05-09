#pragma once

/**
 * @file ros/micro_ros/timer.hpp
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

namespace milkros::micro_ros {

template<typename CallbackT>
class Timer {
public:
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
    Timer(Context* context,
          uint32_t period_ms,
          CallbackT callback) noexcept
        : context_(context)
        , callback_(static_cast<CallbackT&&>(callback))
    {
        if (context_ == nullptr || period_ms == 0u) {
            return;
        }

        timer_ = rcl_get_zero_initialized_timer();
        if (rclc_timer_init_default2(
                &timer_,
                &context_->support(),
                static_cast<uint64_t>(period_ms) * 1000000ull,
                &Timer::on_timer,
                true) != RCL_RET_OK) {
            timer_ = rcl_get_zero_initialized_timer();
            return;
        }

        if (rclc_executor_add_timer(&context_->executor(), &timer_) != RCL_RET_OK) {
            (void)rcl_timer_fini(&timer_);
            timer_ = rcl_get_zero_initialized_timer();
            return;
        }

        timer_.impl->callback_data = this;
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
     * `rclc_timer_init_default2()` does not expose a user context parameter, so
     * the wrapper stores `this` in `callback_data` after successful
     * initialization and retrieves it here.
     *
     * @param[in] timer Timer instance provided by `rclc`.
     * @param[in] last_call_time Unused elapsed-time parameter from `rclc`.
     */
    static void on_timer(rcl_timer_t* timer, int64_t last_call_time) noexcept
    {
        (void)last_call_time;
        if (timer == nullptr || timer->impl == nullptr || timer->impl->callback_data == nullptr) {
            return;
        }
        auto* self = static_cast<Timer*>(timer->impl->callback_data);
        self->callback_();
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
            (void)rclc_executor_remove_timer(&context_->executor(), &timer_);
            (void)rcl_timer_fini(&timer_);
        }
        timer_ = rcl_get_zero_initialized_timer();
        valid_ = false;
    }

    Context* context_ = nullptr;
    rcl_timer_t timer_ = rcl_get_zero_initialized_timer();
    CallbackT callback_{};
    bool valid_ = false;
};

}  // namespace milkros::micro_ros
