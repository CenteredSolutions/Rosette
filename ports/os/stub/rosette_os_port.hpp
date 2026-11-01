// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file ports/os/stub/rosette_os_port.hpp
 * @brief Compile-only OS layer: one thread, no interrupts, every wait a spin.
 *
 * The OS half of the stub port (ports/chip/stub/). It satisfies the OS
 * contract in rosette/platform/platform.hpp with no RTOS and no C library
 * beyond the standard headers, so the portable core builds for a target
 * Rosette has never seen. It is not a usable OS layer: Mutex does nothing,
 * because there is nothing to exclude, and Event::wait_for() and sleep_ms()
 * spin on monotonic_us(). A real port starts from a copy of this file.
 *
 * platform.hpp includes this header by its fixed name, after the chip-layer
 * declarations, when the build puts this directory on the include path
 * (port.cmake beside it).
 *
 * Verification:
 * - Correct means: tests/port_stub builds and links with no Pico SDK, no
 *   RTOS, and no POSIX headers.
 */

#pragma once

#include <cstdint>

namespace rosette::platform {

/**
 * @brief Mutex for a target with one thread and no interrupt handlers: every operation succeeds at
 * once.
 */
class Mutex {
public:
    Mutex() noexcept = default;
    Mutex(const Mutex&) = delete;
    Mutex& operator=(const Mutex&) = delete;

    /// @brief Returns at once: no other thread can hold it.
    void lock() noexcept {}

    /// @brief Returns at once.
    void unlock() noexcept {}

    /// @brief Returns `true`: the mutex is always free.
    bool try_lock() noexcept { return true; }
};

/**
 * @brief Auto-reset event whose waits spin on the monotonic clock.
 */
class Event {
public:
    Event() noexcept = default;
    Event(const Event&) = delete;
    Event& operator=(const Event&) = delete;

    /// @brief Wakes one current or future wait.
    void signal() noexcept { pending_ = true; }

    /// @brief Same as signal().
    void signal_from_isr() noexcept { signal(); }

    /**
     * @brief Spins until signaled or the timeout passes, consuming the signal.
     *
     * @param[in] timeout_ms Longest wait; 0 polls, kWaitForever waits forever.
     * @return `true` when signaled, `false` on timeout.
     */
    bool wait_for(uint32_t timeout_ms) noexcept {
        const uint64_t start = monotonic_us();
        for (;;) {
            if (consume()) {
                return true;
            }

            if (timeout_ms != kWaitForever
                && monotonic_us() - start >= static_cast<uint64_t>(timeout_ms) * 1000u) {
                return false;
            }
        }
    }

    /// @brief Clears a pending signal without waiting; returns whether one was pending.
    bool consume() noexcept {
        const bool was = pending_;
        pending_ = false;
        return was;
    }

private:
    volatile bool pending_ = false;
};

/**
 * @brief Spins for at least `ms` milliseconds.
 */
inline void sleep_ms(uint32_t ms) noexcept {
    const uint64_t start = monotonic_us();
    while (monotonic_us() - start < static_cast<uint64_t>(ms) * 1000u) {
    }
}

}  // namespace rosette::platform
