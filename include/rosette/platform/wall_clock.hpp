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
     * - Correct means: equals monotonic_us() + offset_us() at the instant of
     *   the call; on hosts, within a millisecond of CLOCK_REALTIME.
     * - Covered by: tests/platform/platform_contract_test.cpp (WallClock).
     */
    static int64_t now_us() noexcept;

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
