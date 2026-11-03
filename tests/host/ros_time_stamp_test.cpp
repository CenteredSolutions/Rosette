// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file tests/host/ros_time_stamp_test.cpp
 * @brief ros::time_stamp() and the epoch readers over WallClock, at instants the test sets.
 *
 * The chip layer here is a fake: monotonic_us() returns g_monotonic_us and
 * there is no realtime hint, so WallClock starts unsynchronized, as on a
 * board, and each check sets the exact instant it reads.
 *
 * - TC-ROS-TIME-001: before synchronization, WallClock::try_now_us() is false
 *   and leaves its output alone, and every ROS reader returns zero.
 * - TC-ROS-TIME-002: after synchronization, the stamp, milliseconds, and
 *   nanoseconds all agree with the wall clock's microseconds.
 * - TC-ROS-TIME-003: the stamp splits whole seconds and the last microsecond
 *   of a second exactly, and matches the nanosecond divide-and-modulus split
 *   across a sweep of instants.
 * - TC-ROS-TIME-004: an instant before the Unix epoch stamps as zero.
 */

#include <cstdint>

#include "rosette/platform/platform.hpp"
#include "rosette/platform/wall_clock.hpp"
#include "rosette/ros/time_sync.hpp"
#include "test_util.hpp"

namespace pf = rosette::platform;
namespace ros = rosette::ros;

namespace {

uint64_t g_monotonic_us = 5'000'000u;

/// 2023-11-14 22:13:20 UTC, in seconds since the Unix epoch: microseconds past 2^32.
constexpr int64_t kUnixSecond = 1'700'000'000;
constexpr int64_t kUsPerSecond = 1'000'000;

/// Sets the wall clock so that it reads @p unix_us at the current fake tick.
void set_wall_clock(int64_t unix_us) {
    pf::WallClock::set_offset_us(unix_us - static_cast<int64_t>(g_monotonic_us));
}

/// The stamp the nanosecond split gives: the reference time_stamp() must equal.
ros::Stamp reference_stamp(int64_t unix_us) {
    const int64_t ns = unix_us * 1000;
    return ros::Stamp{static_cast<int32_t>(ns / 1'000'000'000),
                      static_cast<uint32_t>(ns % 1'000'000'000)};
}

bool same(const ros::Stamp& a, const ros::Stamp& b) {
    return a.sec == b.sec && a.nanosec == b.nanosec;
}

}  // namespace

namespace rosette::platform {

uint64_t monotonic_us() noexcept {
    return g_monotonic_us;
}

bool realtime_hint_us(int64_t& unix_us) noexcept {
    (void)unix_us;
    return false;
}

}  // namespace rosette::platform

int main() {
    int64_t          untouched = -7;
    const ros::Stamp unset = ros::time_stamp();
    check(!pf::WallClock::try_now_us(untouched) && untouched == -7,
          "TC-ROS-TIME-001 unsynchronized: try_now_us() is false and writes nothing");
    check(!ros::time_synced() && unset.sec == 0 && unset.nanosec == 0u && ros::epoch_millis() == 0
              && ros::epoch_nanos() == 0,
          "TC-ROS-TIME-001 unsynchronized: every ROS reader returns zero");

    const int64_t instant = kUnixSecond * kUsPerSecond + 123'456;
    set_wall_clock(instant);
    int64_t          read_us = 0;
    const ros::Stamp stamp = ros::time_stamp();
    check(pf::WallClock::try_now_us(read_us) && read_us == instant,
          "TC-ROS-TIME-002 synchronized: try_now_us() reads the wall clock");
    check(stamp.sec == kUnixSecond && stamp.nanosec == 123'456'000u,
          "TC-ROS-TIME-002 the stamp holds the seconds and the nanoseconds");
    check(ros::epoch_millis() == instant / 1000 && ros::epoch_nanos() == instant * 1000,
          "TC-ROS-TIME-002 epoch_millis() and epoch_nanos() agree with the stamp");

    set_wall_clock(kUnixSecond * kUsPerSecond);
    const ros::Stamp whole = ros::time_stamp();
    set_wall_clock(kUnixSecond * kUsPerSecond + kUsPerSecond - 1);
    const ros::Stamp last = ros::time_stamp();
    check(whole.sec == kUnixSecond && whole.nanosec == 0u,
          "TC-ROS-TIME-003 a whole second stamps with zero nanoseconds");
    check(last.sec == kUnixSecond && last.nanosec == 999'999'000u,
          "TC-ROS-TIME-003 the last microsecond of a second stays in that second");

    bool sweep_matches = true;
    for (int64_t step = 0; step < 4096; ++step) {
        const int64_t unix_us = kUnixSecond * kUsPerSecond + step * 7'919'777 + step % 1000;
        set_wall_clock(unix_us);
        const ros::Stamp got = ros::time_stamp();
        sweep_matches =
            sweep_matches && same(got, reference_stamp(unix_us)) && got.nanosec < 1'000'000'000u;
    }

    check(sweep_matches, "TC-ROS-TIME-003 4096 instants match the nanosecond split");

    set_wall_clock(-1);
    const ros::Stamp before_epoch = ros::time_stamp();
    check(before_epoch.sec == 0 && before_epoch.nanosec == 0u,
          "TC-ROS-TIME-004 an instant before the epoch stamps as zero");
    return test_summary();
}
