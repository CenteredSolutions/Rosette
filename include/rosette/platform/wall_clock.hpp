// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file rosette/platform/wall_clock.hpp
 * @brief Wall-clock time derived from the platform's single monotonic time base.
 *
 * There is one clock in Rosette: platform::monotonic_us(). Wall-clock (Unix)
 * time is that clock plus one offset held here, so everything that needs "now"
 * agrees, and synchronizing never steps a running timer. The offset is seeded
 * from platform::realtime_hint_us() when the platform knows the time (hosts),
 * and otherwise set by a time-sync service (NTP, micro-ROS agent sync) through
 * set_offset_us(); a sync service slews by setting small successive offsets.
 *
 * Readers: RTPS source timestamps (INFO_TS), ROS message stamps, logging.
 */

#pragma once

#include <cstdint>

namespace rosette::platform {

/**
 * @brief Process-wide wall clock: monotonic_us() + a synchronized offset.
 */
class WallClock {
public:
    /**
     * @brief Microseconds since the Unix epoch.
     *
     * Before any synchronization on a platform without a realtime hint this
     * is the monotonic time itself (a small number near 1970), which peers
     * interpret as "unsynchronized" rather than as a plausible date.
     *
     * @return Current wall-clock time in microseconds.
     *
     * Verification:
     * - Correct means: equals monotonic_us() + offset_us() at one instant
     *   during the call, the tick and the offset read together, so a time
     *   source that sets the offset mid-call never yields a time the clock
     *   did not show; on hosts, within a millisecond of CLOCK_REALTIME.
     * - Covered by: tests/platform/platform_contract_test.cpp (WallClock);
     *   tests/host/wall_clock_tick_test.cpp (TC-PLAT-WC-003).
     */
    static int64_t now_us() noexcept;

    /**
     * @brief Reads the wall time only when it is synchronized, in one reading of the offset.
     *
     * The read for a caller that must tell real time from the monotonic
     * fallback, such as a ROS message stamp: synchronized() followed by
     * now_us() reads the offset twice, and a time source could set it between
     * the two; this reads it once and answers both. Like every WallClock
     * read it takes no lock (src/platform/wall_clock.cpp, the sequence lock).
     *
     * @param[out] unix_us Microseconds since the Unix epoch; written only when
     *   the call returns true.
     * @return True when the clock is synchronized and @p unix_us holds the
     *   time; false before the first synchronization, leaving @p unix_us as
     *   it was.
     *
     * Verification:
     * - Correct means: returns synchronized() and, when true, now_us() as one
     *   reading of the offset and the tick.
     * - Covered by: tests/host/ros_time_stamp_test.cpp (TC-ROS-TIME-001);
     *   tests/host/wall_clock_concurrency_test.cpp (TC-PLAT-WC-002);
     *   tests/host/wall_clock_tick_test.cpp (TC-PLAT-WC-003).
     */
    static bool try_now_us(int64_t& unix_us) noexcept;

    /// @brief Current offset added to monotonic_us(), in microseconds.
    static int64_t offset_us() noexcept;

    /**
     * @brief Sets the offset from a time-sync measurement.
     *
     * @param[in] offset_us Unix time minus monotonic_us(), in microseconds.
     */
    static void set_offset_us(int64_t offset_us) noexcept;

    /// @brief True once the offset came from the platform hint or a sync service.
    static bool synchronized() noexcept;
};

}  // namespace rosette::platform
