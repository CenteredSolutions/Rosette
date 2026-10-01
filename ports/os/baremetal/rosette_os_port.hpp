/**
 * @file ports/os/baremetal/rosette_os_port.hpp
 * @brief Bare-metal (no RTOS) OS layer of the platform contract, on the chip's sync hooks.
 *
 * platform.hpp includes this header by its fixed name when the build selects
 * ROSETTE_OS=baremetal (port.cmake beside it); see platform.hpp for the
 * contract each type satisfies. Without a scheduler there is still
 * concurrency to guard against: a second core, and interrupt handlers. Both
 * types keep their state behind the chip's critical section
 * (rosette/platform/chip_sync.hpp), which masks interrupts and excludes the
 * other cores, and sleep in chip::wait_for_event_until():
 *
 *  - Mutex is a flag: lock() takes it inside the critical section, or sleeps
 *    until an event and tries again; unlock() clears it and sends an event.
 *    Like any mutex it must not be taken in an interrupt handler.
 *  - Event is a pending flag: signal() and signal_from_isr() set it and send
 *    an event; wait_for() consumes it, or sleeps until the deadline. An event
 *    sent between the check and the sleep ends the sleep at once, so no
 *    wakeup is lost.
 *
 * The header uses nothing beyond the platform contract, so it runs on any
 * chip port that supplies chip_sync.hpp (the rp2 and stub ports do).
 */

#pragma once

#include <cstdint>

#include "rosette/platform/chip_sync.hpp"

namespace rosette::platform {

namespace detail {

/// @brief The monotonic_us() deadline `timeout_ms` from now; UINT64_MAX for kWaitForever.
inline uint64_t deadline_after_ms(uint32_t timeout_ms) noexcept {
    return timeout_ms == kWaitForever ? UINT64_MAX
                                      : monotonic_us() + static_cast<uint64_t>(timeout_ms) * 1000u;
}

}  // namespace detail

/**
 * @brief Cross-core mutex over the chip's critical section; thread (core) context only.
 */
class Mutex {
public:
    Mutex() noexcept = default;
    Mutex(const Mutex&) = delete;
    Mutex& operator=(const Mutex&) = delete;

    /// @brief Sleeps until the mutex is held by the calling core.
    void lock() noexcept {
        while (!try_lock()) {
            chip::wait_for_event_until(UINT64_MAX);
        }
    }

    /// @brief Releases a mutex the calling core holds.
    void unlock() noexcept {
        const uint32_t state = chip::critical_enter();
        locked_ = false;
        chip::critical_exit(state);
        chip::send_event();
    }

    /// @brief Takes the mutex if free; returns whether it was taken.
    bool try_lock() noexcept {
        const uint32_t state = chip::critical_enter();
        const bool taken = !locked_;
        locked_ = true;
        chip::critical_exit(state);
        return taken;
    }

private:
    volatile bool locked_ = false;
};

/**
 * @brief Auto-reset event; signal() is safe from any core and from interrupt handlers.
 */
class Event {
public:
    Event() noexcept = default;
    Event(const Event&) = delete;
    Event& operator=(const Event&) = delete;

    /// @brief Wakes one current or future wait, from any core.
    void signal() noexcept {
        const uint32_t state = chip::critical_enter();
        pending_ = true;
        chip::critical_exit(state);
        chip::send_event();
    }

    /// @brief Same as signal(); the critical section also masks interrupts.
    void signal_from_isr() noexcept { signal(); }

    /**
     * @brief Sleeps until signaled or the timeout passes, consuming the signal.
     *
     * @param[in] timeout_ms Longest wait; 0 polls, kWaitForever blocks.
     * @return `true` when signaled, `false` on timeout.
     */
    bool wait_for(uint32_t timeout_ms) noexcept {
        const uint64_t deadline = detail::deadline_after_ms(timeout_ms);
        for (;;) {
            if (consume()) {
                return true;
            }

            if (deadline != UINT64_MAX && monotonic_us() >= deadline) {
                return false;
            }

            chip::wait_for_event_until(deadline);
        }
    }

    /// @brief Clears a pending signal without waiting; returns whether one was pending.
    bool consume() noexcept {
        const uint32_t state = chip::critical_enter();
        const bool was = pending_;
        pending_ = false;
        chip::critical_exit(state);
        return was;
    }

private:
    volatile bool pending_ = false;
};

/**
 * @brief Sleeps the calling core for at least `ms` milliseconds.
 */
inline void sleep_ms(uint32_t ms) noexcept {
    const uint64_t deadline = detail::deadline_after_ms(ms);
    while (monotonic_us() < deadline) {
        chip::wait_for_event_until(deadline);
    }
}

}  // namespace rosette::platform
