// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file platform/chip_sync.hpp
 * @brief Chip contract: the critical section and event wait an OS-less build runs on.
 *
 * The bare-metal OS layer (ports/os/baremetal/) builds Mutex, Event, and
 * sleep_ms() from these four hooks alone, so it runs on any chip whose port
 * supplies them. Under an RTOS or a host OS nothing calls them, and a chip
 * port that never runs without one may leave them out.
 *
 * The hooks follow the Arm WFE / SEV pattern: a core sleeps until an event
 * or an interrupt, and an event sent before the sleep is not lost (it ends
 * the next sleep at once). A chip without such a latch must provide the same
 * guarantee some other way, or wake on a short period.
 */

#pragma once

#include <cstdint>

namespace rosette::platform::chip {

/**
 * @brief Masks interrupts on the calling core and excludes every other core.
 *
 * Held for a few instructions at a time; not re-entrant.
 *
 * @return State to pass to critical_exit().
 */
uint32_t critical_enter() noexcept;

/**
 * @brief Ends the critical section critical_enter() began.
 *
 * @param[in] state The value critical_enter() returned.
 */
void critical_exit(uint32_t state) noexcept;

/**
 * @brief Sleeps the calling core until an event, an interrupt, or `deadline_us`.
 *
 * May return early and without cause; callers re-check their condition.
 *
 * @param[in] deadline_us monotonic_us() value to wake by; UINT64_MAX waits
 *   for an event or interrupt only.
 */
void wait_for_event_until(uint64_t deadline_us) noexcept;

/// @brief Ends the current or next wait_for_event_until() on every core.
void send_event() noexcept;

}  // namespace rosette::platform::chip
