/**
 * @file ros/timer.hpp
 * @brief Tick-driven wall-clock timer, analogous to rclcpp::Timer.
 *
 * Usage
 * -----
 *   auto* t = node.create_timer(500u, []() { publisher.publish(msg); });
 *
 * The returned pointer is stable (owned by the Context pool). Never delete it.
 *
 * Context::tick_timers(now_ms) drives all registered timers; call it from your
 * FreeRTOS tick task or main loop alongside tick_participant().
 *
 * Timer does NOT fire on the first tick — it waits one full period before the
 * first callback invocation, matching rclcpp::Timer behaviour.
 *
 * Drift compensation
 * ------------------
 * After each firing, last_fire_ms_ advances by period_ms_ (not by now_ms).
 * This keeps the phase stable even if tick() is called slightly late.
 *
 * Size configuration
 * ------------------
 * Context pre-allocates MILKROS_MAX_TIMERS slots of MILKROS_MAX_TIMER_SIZE
 * bytes each. Increase these if your lambdas capture large state:
 *
 *   #define MILKROS_MAX_TIMER_SIZE 128u  // before including ros.hpp
 *   #define MILKROS_MAX_TIMERS     16u
 */

#pragma once

#include <stdint.h>

#ifndef MILKROS_MAX_TIMER_SIZE
/// Maximum sizeof(Timer<CallbackT>) for any CallbackT stored in Context.
/// Default 64 B covers a lambda capturing up to ~4 pointers plus Timer overhead.
#  define MILKROS_MAX_TIMER_SIZE 64u
#endif
#ifndef MILKROS_MAX_TIMERS
/// Maximum number of timers that can be registered with one Context.
#  define MILKROS_MAX_TIMERS 8u
#endif

namespace milkros {

// ============================================================
// ITimer — type-erased base stored in Context pool
// ============================================================

class ITimer {
public:
    virtual void tick(uint32_t now_ms) noexcept = 0;
    virtual bool is_ready(uint32_t now_ms) const noexcept = 0;
    virtual uint32_t period_ms() const noexcept = 0;
    virtual void reset(uint32_t now_ms) noexcept = 0;
    virtual bool valid() const noexcept = 0;

protected:
    ~ITimer() = default;  // non-virtual dtor: Context owns lifetime, never deletes via ITimer*
};

// ============================================================
// Timer<CallbackT> — concrete implementation
// ============================================================

/**
 * @brief Wall-clock periodic timer.
 *
 * @tparam CallbackT  Zero-argument callable: void() noexcept.
 *
 * Instances are created via Node::create_timer() and owned by the Context
 * placement-new pool.  The returned raw pointer is valid for the lifetime of
 * the Context.  Never move or copy a Timer after it has been registered.
 */
template<typename CallbackT>
class Timer final : public ITimer {
public:
    Timer() noexcept = default;

    Timer(uint32_t period_ms, CallbackT callback) noexcept
        : period_ms_(period_ms)
        , callback_(static_cast<CallbackT&&>(callback))
    {}

    // Move-only — copying would silently create a second ITimer* alias.
    Timer(const Timer&)            = delete;
    Timer& operator=(const Timer&) = delete;

    Timer(Timer&& o) noexcept
        : period_ms_   (o.period_ms_)
        , last_fire_ms_(o.last_fire_ms_)
        , initialized_ (o.initialized_)
        , callback_    (static_cast<CallbackT&&>(o.callback_))
    {
        o.period_ms_   = 0u;
        o.initialized_ = false;
    }

    Timer& operator=(Timer&& o) noexcept {
        if (this != &o) {
            period_ms_    = o.period_ms_;
            last_fire_ms_ = o.last_fire_ms_;
            initialized_  = o.initialized_;
            callback_     = static_cast<CallbackT&&>(o.callback_);
            o.period_ms_  = 0u;
            o.initialized_= false;
        }
        return *this;
    }

    // ----------------------------------------------------------
    // ITimer interface
    // ----------------------------------------------------------

    /// Drive the timer.  Should be called from a single task (no locking needed).
    void tick(uint32_t now_ms) noexcept override {
        if (period_ms_ == 0u) return;

        if (!initialized_) {
            // First call: set baseline but do NOT fire yet (matches rclcpp behaviour).
            last_fire_ms_ = now_ms;
            initialized_  = true;
            return;
        }

        if ((now_ms - last_fire_ms_) >= period_ms_) {
            last_fire_ms_ += period_ms_;  // drift-compensating: preserve phase
            callback_();
        }
    }

    bool is_ready(uint32_t now_ms) const noexcept override {
        if (!initialized_) return false;
        return (now_ms - last_fire_ms_) >= period_ms_;
    }

    uint32_t period_ms() const noexcept override { return period_ms_; }

    void reset(uint32_t now_ms) noexcept override {
        last_fire_ms_ = now_ms;
        initialized_  = true;
    }

    bool valid() const noexcept override { return period_ms_ > 0u; }

private:
    uint32_t  period_ms_    = 0u;
    uint32_t  last_fire_ms_ = 0u;
    bool      initialized_  = false;
    CallbackT callback_{};
};

} // namespace milkros
