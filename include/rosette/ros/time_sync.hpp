#pragma once

/**
 * @file ros/time_sync.hpp
 * @brief Wall-clock (Unix epoch) time on a board that has only a monotonic tick.
 *
 * A board boots with no idea of the date, yet `std_msgs/Header` stamps and
 * cross-device correlation need Unix time. Rosette keeps one wall clock,
 * `platform::WallClock` (the monotonic tick plus a synchronized offset), and
 * these functions read it. On the micro-ROS agent backend, `AgentSessionGate`
 * runs `rmw_uros_sync_session()` against the agent's host clock when a
 * session comes up and every `AgentGatePolicy::time_resync_period_ms` after.
 *
 * The same WallClock feeds rcl's system clock (`rcutils_system_time_now()`,
 * through the stack's `rosette_micro_ros_epoch_time_us()` hook) and RTPS
 * source timestamps, so every stamp the firmware produces agrees.
 *
 * Until the first successful synchronization, `time_synced()` is false and
 * the epoch functions return 0, the ROS convention for "time not set", so
 * an unsynchronized stamp is recognizable rather than plausible. (A host
 * knows the time from boot: there WallClock starts synchronized.)
 *
 *     const rosette::ros::Stamp now = rosette::ros::time_stamp();
 *     msg.header.stamp.sec = now.sec;
 *     msg.header.stamp.nanosec = now.nanosec;
 */

#include <cstdint>

namespace rosette::ros {

/// A ROS `builtin_interfaces/Time` value: whole seconds and nanoseconds since the Unix epoch.
struct Stamp {
    int32_t sec;
    uint32_t nanosec;
};

/**
 * @brief Whether the wall clock has been synchronized with a reference at least once.
 *
 * @return True after the first successful synchronization; it
 *   stays true while sessions are torn down and rebuilt, since WallClock
 *   keeps the last offset and the local tick carries it forward.
 */
bool time_synced() noexcept;

/**
 * @brief Nanoseconds since the Unix epoch.
 *
 * @return The synchronized wall time, or 0 while `time_synced()` is false.
 *
 * Verification:
 * - Correct means: after a sync, the value tracks the reference's clock to
 *   within the exchange's round-trip time and advances with the local tick.
 * - Covered by: tests/agent/run_agent_live.sh (TC-ROS-AGT-LIVE-001, the
 *   agent's host clock).
 *
 * References:
 * - micro-ROS rmw_microxrcedds `rmw_microros/time_sync.h` (Humble)
 * - RFC 5905 §8 (the round-trip offset estimate)
 */
int64_t epoch_nanos() noexcept;

/// @brief Milliseconds since the Unix epoch, or 0 while `time_synced()` is false.
inline int64_t epoch_millis() noexcept {
    return epoch_nanos() / 1000000;
}

/// @brief The current instant as a ROS time stamp; zero while `time_synced()` is false.
inline Stamp time_stamp() noexcept {
    const int64_t ns = epoch_nanos();
    return Stamp{static_cast<int32_t>(ns / 1000000000), static_cast<uint32_t>(ns % 1000000000)};
}

}  // namespace rosette::ros
