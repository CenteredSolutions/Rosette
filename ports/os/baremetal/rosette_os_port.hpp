/**
 * @file rosette/platform/os_baremetal.hpp
 * @brief Bare-metal (no RTOS) OS layer of the platform contract, on the Pico SDK.
 *
 * Included through platform.hpp when ROSETTE_OS_BAREMETAL is defined; see
 * that header for the contract each type satisfies. Without a scheduler there
 * is still concurrency to guard against: the second core, and interrupt
 * handlers. The types use pico_sync primitives, which serialize across both
 * cores:
 *
 *  - Mutex wraps `mutex_t`: the owning core blocks (sleeping on WFE) until
 *    the other releases it. Like any mutex it must not be taken in an
 *    interrupt handler.
 *  - Event keeps its flag behind a `critical_section_t` (hardware spin lock
 *    plus interrupts off), so signal_from_isr() and either core can set it,
 *    and waits sleep in `best_effort_wfe_or_timeout()`. signal() issues SEV,
 *    and an event latched between the flag check and WFE makes WFE return at
 *    once, so no wakeup is lost.
 *
 * Bare-metal support currently requires the Pico SDK (ROSETTE_CHIP_RP2).
 */

#pragma once

#include <cstdint>

#include "hardware/sync.h"
#include "pico/critical_section.h"
#include "pico/mutex.h"
#include "pico/time.h"

#if !defined(ROSETTE_CHIP_RP2)
#  error "ROSETTE_OS=baremetal is implemented on the Pico SDK; set ROSETTE_CHIP=rp2"
#endif

namespace rosette::platform {

/**
 * @brief Cross-core mutex over pico_sync `mutex_t`; thread (core) context only.
 */
class Mutex {
public:
    Mutex() noexcept { mutex_init(&m_); }
    Mutex(const Mutex&) = delete;
    Mutex& operator=(const Mutex&) = delete;

    /// @brief Blocks (WFE) until the mutex is held by the calling core.
    void lock() noexcept { mutex_enter_blocking(&m_); }
    /// @brief Releases a mutex the calling core holds.
    void unlock() noexcept { mutex_exit(&m_); }
    /// @brief Takes the mutex if free; returns whether it was taken.
    bool try_lock() noexcept {
        uint32_t owner;
        return mutex_try_enter(&m_, &owner);
    }

private:
    mutex_t m_{};
};

/**
 * @brief Auto-reset event; signal() is safe from either core and from IRQs.
 */
class Event {
public:
    Event() noexcept { critical_section_init(&cs_); }
    ~Event() { critical_section_deinit(&cs_); }
    Event(const Event&) = delete;
    Event& operator=(const Event&) = delete;

    /// @brief Wakes one current or future wait, from either core.
    void signal() noexcept {
        critical_section_enter_blocking(&cs_);
        pending_ = true;
        critical_section_exit(&cs_);
        __sev();
    }

    /// @brief Same as signal(); the critical section also masks interrupts.
    void signal_from_isr() noexcept { signal(); }

    /**
     * @brief Sleeps (WFE) until signaled or the timeout passes, consuming the signal.
     *
     * @param[in] timeout_ms Longest wait; 0 polls, kWaitForever blocks.
     * @return `true` when signaled, `false` on timeout.
     */
    bool wait_for(uint32_t timeout_ms) noexcept {
        const absolute_time_t deadline = (timeout_ms == UINT32_MAX)
            ? at_the_end_of_time : make_timeout_time_ms(timeout_ms);
        for (;;) {
            if (consume()) return true;
            if (time_reached(deadline)) return false;
            (void)best_effort_wfe_or_timeout(deadline);
        }
    }

    /// @brief Clears a pending signal without waiting; returns whether one was pending.
    bool consume() noexcept {
        critical_section_enter_blocking(&cs_);
        const bool was = pending_;
        pending_ = false;
        critical_section_exit(&cs_);
        return was;
    }

private:
    critical_section_t cs_{};
    volatile bool pending_ = false;
};

/**
 * @brief Sleeps the calling core for at least `ms` milliseconds.
 */
inline void sleep_ms(uint32_t ms) noexcept { ::sleep_ms(ms); }

}  // namespace rosette::platform
