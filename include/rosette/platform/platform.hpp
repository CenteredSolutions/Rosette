// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file rosette/platform/platform.hpp
 * @brief The platform contract: everything Rosette needs from the OS and chip.
 *
 * Rosette runs on a POSIX host, under FreeRTOS, or bare-metal on an RP2040 or
 * RP2350, and exactly one platform exists in any build. The platform is
 * therefore chosen at build time, not through a runtime object: each board's
 * `board.cmake` sets two independent axes,
 *
 *   ROSETTE_OS    posix | freertos | baremetal | stub   scheduling and synchronization
 *   ROSETTE_CHIP  host  | rp2 | stub                   clock, device identity, flash
 *
 * Each name is a port directory, ports/os/<name>/ or ports/chip/<name>/ (or
 * the same layout under a directory in ROSETTE_PORT_DIRS), and the build
 * defines ROSETTE_OS_<NAME> and ROSETTE_CHIP_<NAME> for tests that report
 * which port they ran on. FreeRTOS on a Pico is OS=freertos, CHIP=rp2; a
 * two-core superloop is OS=baremetal, CHIP=rp2.
 *
 * This header is the whole contract. A port provides:
 *
 *  - from the OS layer (ports/os/<name>/rosette_os_port.hpp; header-only):
 *      Mutex        mutual exclusion between threads, tasks, and cores
 *      Event        auto-reset wakeup; signal() is safe from any thread, core,
 *                   and (via signal_from_isr()) interrupt handler
 *      sleep_ms()   yields the caller for a duration
 *      run_main_task(entry)
 *                   runs the application's main loop where the OS needs
 *                   it: under FreeRTOS a task, with the scheduler started
 *                   (never returns); elsewhere a plain call on the calling
 *                   thread. An application's main() calls board_init() and
 *                   then this, so the same main() serves every OS port.
 *  - from the chip layer (ports/chip/<name>/):
 *      monotonic_us()        the single time base; never steps
 *      device_unique_id()    8 octets unique per running Rosette instance
 *      boot_nonce()          32 bits that change at every boot or process start
 *      realtime_hint_us()    wall-clock time when the platform knows it
 *    and, in their own headers, flash (flash.hpp), GPIO interrupts
 *    (gpio_interrupt.hpp), and the critical section and event wait the
 *    bare-metal OS layer is built on (chip_sync.hpp).
 *
 * ports/chip/stub/ and ports/os/stub/ implement every item and are the
 * starting point for a new port.
 *
 * tests/platform/platform_contract_test.cpp exercises every item and is built
 * for each port, so a port that misses or mis-implements part of the contract
 * fails to link or fails that test.
 *
 * Concurrency rule: interrupt handlers may only call Event::signal_from_isr().
 * All protocol processing runs in thread context under a Mutex.
 */

#pragma once

#include <cstdint>

namespace rosette::platform {

/// Timeout value meaning "wait until signaled".
inline constexpr uint32_t kWaitForever = UINT32_MAX;

// ------------------------------------------------------------------
// Chip layer
// ------------------------------------------------------------------

/**
 * @brief Microseconds since an arbitrary fixed point (boot or process start).
 *
 * The one time base for timers, leases, and timeouts throughout Rosette. It
 * never steps or slews; wall-clock time is derived from it by WallClock
 * (wall_clock.hpp), so synchronizing to NTP or an agent never disturbs a
 * running timer.
 *
 * @return Monotonic microseconds; wraps after about 584,000 years.
 *
 * Verification:
 * - Correct means: successive reads never decrease, and advance by the
 *   elapsed real time within the chip's timer accuracy.
 * - Covered by: tests/platform/platform_contract_test.cpp (MonotonicAdvances).
 */
uint64_t monotonic_us() noexcept;

/**
 * @brief Milliseconds from the same time base as monotonic_us().
 *
 * RTPS timers compare these with unsigned subtraction, which stays correct
 * across the 49.7-day wrap for intervals shorter than that.
 *
 * @return Monotonic milliseconds, truncated to 32 bits.
 */
inline uint32_t monotonic_ms() noexcept {
    return static_cast<uint32_t>(monotonic_us() / 1000u);
}

/**
 * @brief Writes 8 octets that identify this Rosette instance on the network.
 *
 * RTPS GUID prefixes are built from it, so two instances that can be alive on
 * the same network at the same time must differ. On a microcontroller this is
 * the device's factory unique ID; on a host, where several processes share
 * one machine, it also covers the process.
 *
 * @param[out] out Receives the identifier.
 *
 * Verification:
 * - Correct means: stable for the life of the instance and distinct across
 *   devices and host processes.
 * - Covered by: tests/platform/platform_contract_test.cpp (UniqueIdStable).
 */
void device_unique_id(uint8_t out[8]) noexcept;

/**
 * @brief 32 bits drawn once per boot (or process start) that differ, with high
 * probability, from the previous boot's.
 *
 * RTPS GUID prefixes carry it beside the device's unique ID, so a device that
 * resets comes back as a new participant. Peers still hold the previous
 * participant's lease and its sequence numbers; were the GUID the same, they
 * would take the new SPDP announcement as a lease refresh and drop the new
 * participant's samples, numbered from 1 again, as ones already received.
 * `device_unique_id()` stays stable across boots, for anything that names the
 * device.
 *
 * @return The nonce; the same value for every call within one boot.
 *
 * Verification:
 * - Correct means: stable for the life of the instance; a reset of the device
 *   or a restart of the process draws a new value from a source that differs
 *   from boot to boot (a hardware entropy source, or the process ID and start
 *   time on a host).
 * - Covered by: tests/platform/platform_contract_test.cpp (BootNonceStable);
 *   TC-PART-8.5.1-003 (the nonce's place in the GUID prefix). That a reset
 *   draws a new value shows on hardware: a ROS 2 host that stays up
 *   rediscovers a device after it resets.
 *
 * References:
 * - OMG DDSI-RTPS 2.5 §8.2.4.2 (GuidPrefix_t), §8.5.3 (SPDP lease)
 */
uint32_t boot_nonce() noexcept;

/**
 * @brief Reports wall-clock time when the platform already knows it.
 *
 * WallClock uses this once to seed its offset. Hosts synchronize their clock
 * through the OS and report it; microcontrollers learn the time later, from
 * NTP or an agent, and return false.
 *
 * @param[out] unix_us Microseconds since the Unix epoch, written on success.
 * @return `true` when `unix_us` holds trustworthy wall-clock time.
 */
bool realtime_hint_us(int64_t& unix_us) noexcept;

}  // namespace rosette::platform

// ------------------------------------------------------------------
// OS layer: Mutex, Event, sleep_ms()
//
// The selected OS port supplies them under this fixed name; its port.cmake
// puts ports/os/<name>/ on the include path, and no other OS port is on it.
// ------------------------------------------------------------------
#include "rosette_os_port.hpp"

namespace rosette::platform {

/**
 * @brief Holds a Mutex for the enclosing scope.
 */
class LockGuard {
public:
    explicit LockGuard(Mutex& mutex) noexcept : mutex_(mutex) { mutex_.lock(); }
    ~LockGuard() { mutex_.unlock(); }
    LockGuard(const LockGuard&) = delete;
    LockGuard& operator=(const LockGuard&) = delete;

private:
    Mutex& mutex_;
};

}  // namespace rosette::platform
