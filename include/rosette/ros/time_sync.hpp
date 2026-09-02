/**
 * @file ros/time_sync.hpp
 * @brief Wall-clock time synchronized with the micro-ROS Agent.
 *
 * The Agent's host has real (typically NTP-disciplined) wall time; the MCU has
 * only a monotonic tick. `rmw_uros_sync_session()` runs a short round-trip with
 * the Agent to estimate the offset, after which `epoch_nanos()` /
 * `epoch_millis()` return Agent-aligned Unix time — the right value for
 * `std_msgs/Header` stamps and cross-device correlation.
 *
 * `rosette::App` performs the sync automatically on every (re)connect
 * (`AppOptions::sync_time_on_connect`) and can re-sync on an interval
 * (`AppOptions::time_resync_period_ms`); these helpers are what module
 * `publish()` code calls to stamp messages.
 */

#pragma once

#include <rcl/error_handling.h>
#include <rmw_microros/time_sync.h>

#include <stdint.h>

namespace rosette {

/**
 * @brief Whether the epoch clock has been synchronized with the Agent.
 */
inline bool time_synced() noexcept
{
    return rmw_uros_epoch_synchronized();
}

/**
 * @brief Milliseconds since the Unix epoch, Agent-aligned.
 *
 * @return Synchronized wall time in ms, or a value derived from the local
 * monotonic clock when `time_synced()` is false.
 */
inline int64_t epoch_millis() noexcept
{
    return rmw_uros_epoch_millis();
}

/**
 * @brief Nanoseconds since the Unix epoch, Agent-aligned.
 */
inline int64_t epoch_nanos() noexcept
{
    return rmw_uros_epoch_nanos();
}

/**
 * @brief A ROS time stamp (`sec` / `nanosec`) for the current instant.
 *
 * Fill a header directly:
 *   auto [sec, nsec] = rosette::time_stamp();
 *   msg.header.stamp.sec = sec;
 *   msg.header.stamp.nanosec = nsec;
 */
struct Stamp {
    int32_t sec;
    uint32_t nanosec;
};

inline Stamp time_stamp() noexcept
{
    const int64_t ns = rmw_uros_epoch_nanos();
    return Stamp{static_cast<int32_t>(ns / 1000000000ll),
                 static_cast<uint32_t>(ns % 1000000000ll)};
}

/**
 * @brief Runs one synchronization round-trip with the Agent.
 *
 * @param[in] timeout_ms Time to wait for the Agent's reply.
 * @return `true` when the exchange completed and the offset was updated.
 */
inline bool sync_time(int timeout_ms = 1000) noexcept
{
    if (rmw_uros_sync_session(timeout_ms) == RMW_RET_OK) {
        return true;
    }
    // A failed sync (Agent slow/unreachable) still leaves rcl error state set;
    // clear it so the next connect's rcl calls start clean.
    rcl_reset_error();
    return false;
}

}  // namespace rosette
