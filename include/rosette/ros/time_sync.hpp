// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

#pragma once

/**
 * @file ros/time_sync.hpp
 * @brief Wall-clock (Unix epoch) time on a board that has only a monotonic tick.
 *
 * A board boots with no idea of the date, yet `std_msgs/Header` stamps and
 * cross-device correlation need Unix time. Rosette keeps one wall clock,
 * `platform::WallClock` (the monotonic tick plus a synchronized offset), and
 * these functions read it. Each ROS backend has its time source:
 *
 * - The micro-ROS agent: `AgentSessionGate` runs `rmw_uros_sync_session()`
 *   against the agent's host clock when a session comes up and every
 *   `AgentGatePolicy::time_resync_period_ms` after. On the agent backend
 *   this is the only time source by default.
 * - SNTP (net/sntp_client.hpp): on rmw_rosette, the board network
 *   synchronizes with an NTP server it has learned of, a configured one
 *   (`NetworkConfig::time.server`, for instance from a stored
 *   `DeviceConfig`) or one DHCP offered, as soon as the board holds an IPv4
 *   address, and keeps the clock on UTC. With no server known, the board
 *   has no wall clock.
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
 * @return True after the first successful SNTP or agent synchronization; it
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
 *   agent's host clock); tests/net/sntp_live_probe.cpp (a public NTP server);
 *   tests/host/sntp_client_test.cpp (the SNTP discipline).
 *
 * References:
 * - micro-ROS rmw_microxrcedds `rmw_microros/time_sync.h` (Humble)
 * - RFC 4330 §5, RFC 5905 §8 (the round-trip offset estimate both sources use)
 */
int64_t epoch_nanos() noexcept;

/// @brief Milliseconds since the Unix epoch, or 0 while `time_synced()` is false.
int64_t epoch_millis() noexcept;

/**
 * @brief The current instant as a ROS time stamp, for a message's `header.stamp`.
 *
 * The one call a publisher makes per stamped message. It reads the wall
 * clock once (WallClock::try_now_us(), lock-free) and splits the microseconds
 * into seconds and nanoseconds with a single 64-bit divide by 10^6, which
 * yields the quotient and the remainder together; the remainder is below
 * 10^6, so scaling it to nanoseconds is a 32-bit multiply. On a 32-bit core
 * the divide is the one runtime-library call (`__aeabi_uldivmod`); on the
 * RP2040 the Pico SDK's version, which uses the hardware divider, is faster
 * than short division by 15625 in 16-bit digits. A stamp takes about 2.8 us
 * on a 125 MHz RP2040.
 *
 * @return Seconds and nanoseconds since the Unix epoch, nanosec below 10^9;
 *   all zero while `time_synced()` is false (the ROS convention for "time not
 *   set") and for an instant before the epoch, which no time source sets.
 *
 * Verification:
 * - Correct means: sec * 10^9 + nanosec equals the wall clock's microseconds
 *   times 1000 at the instant of the call, with nanosec < 10^9; zero when
 *   unsynchronized.
 * - Covered by: tests/host/ros_time_stamp_test.cpp (TC-ROS-TIME-001 to
 *   TC-ROS-TIME-004).
 *
 * References:
 * - ROS 2 `builtin_interfaces/msg/Time` (sec int32, nanosec uint32 in [0, 10^9))
 *
 * Example:
 * @code{.cpp}
 * std_msgs__msg__Header header{};
 * const rosette::ros::Stamp now = rosette::ros::time_stamp();
 * header.stamp.sec = now.sec;          // 0 until SNTP or the agent sets the clock
 * header.stamp.nanosec = now.nanosec;
 * @endcode
 * Full program: tests/host/ros_time_stamp_test.cpp.
 */
Stamp time_stamp() noexcept;

}  // namespace rosette::ros
