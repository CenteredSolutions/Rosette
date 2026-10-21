/**
 * @file tests/host/rtps_timestamp_test.cpp
 * @brief rtps::Timestamp::now() and its microsecond-to-fraction conversion, against exact
 * arithmetic.
 *
 * The chip layer is a fake: monotonic_us() returns g_monotonic_us and there
 * is no realtime hint, so each check sets the wall-clock instant it reads.
 *
 * - TC-RTPS-TIME-001: fraction_of_microseconds() equals
 *   floor(sub_us * 2^32 / 10^6) for every one of the 10^6 inputs.
 * - TC-RTPS-TIME-002: now() holds the wall clock's whole seconds and the
 *   exact fraction of the rest, at a second boundary, the last microsecond
 *   of a second, and a sweep of instants past 2^32 microseconds; an instant
 *   before the epoch is zero.
 *
 * References: OMG DDSI-RTPS 2.5 §9.3.2 (Time_t: seconds and 2^-32 fractions).
 */

#include <cstdint>

#include "rosette/platform/platform.hpp"
#include "rosette/platform/wall_clock.hpp"
#include "rosette/rtps/types.hpp"
#include "test_util.hpp"

namespace pf = rosette::platform;
namespace rtps = rosette::rtps;

namespace {

uint64_t g_monotonic_us = 5'000'000u;

/// 2023-11-14 22:13:20 UTC, in seconds since the Unix epoch: microseconds past 2^32.
constexpr int64_t kUnixSecond = 1'700'000'000;
constexpr int64_t kUsPerSecond = 1'000'000;

/// The fraction by a 64-bit divide: the reference fraction_of_microseconds() must equal.
uint32_t exact_fraction(uint32_t sub_us) {
    return static_cast<uint32_t>((static_cast<uint64_t>(sub_us) << 32) / 1'000'000u);
}

/// Sets the wall clock so that it reads @p unix_us at the current fake tick.
void set_wall_clock(int64_t unix_us) {
    pf::WallClock::set_offset_us(unix_us - static_cast<int64_t>(g_monotonic_us));
}

bool stamps_as(int64_t unix_us) {
    set_wall_clock(unix_us);
    const rtps::Timestamp now = rtps::Timestamp::now();
    const uint32_t        sub_us = static_cast<uint32_t>(unix_us % kUsPerSecond);
    return now.seconds == static_cast<rtps::Long>(unix_us / kUsPerSecond)
           && now.fraction == exact_fraction(sub_us);
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
    uint32_t wrong = 0u;
    for (uint32_t sub_us = 0u; sub_us < 1'000'000u; ++sub_us) {
        if (rtps::Timestamp::fraction_of_microseconds(sub_us) != exact_fraction(sub_us)) {
            ++wrong;
        }
    }

    check(wrong == 0u, "TC-RTPS-TIME-001 every microsecond converts to the exact 2^-32 fraction");

    check(stamps_as(kUnixSecond * kUsPerSecond),
          "TC-RTPS-TIME-002 a whole second has a zero fraction");
    check(stamps_as(kUnixSecond * kUsPerSecond + kUsPerSecond - 1),
          "TC-RTPS-TIME-002 the last microsecond of a second stays in that second");

    bool sweep = true;
    for (int64_t step = 0; step < 4096; ++step) {
        sweep = sweep && stamps_as(kUnixSecond * kUsPerSecond + step * 7'919'777 + step % 1000);
    }

    check(sweep, "TC-RTPS-TIME-002 4096 instants hold their seconds and exact fractions");

    set_wall_clock(-1);
    const rtps::Timestamp before_epoch = rtps::Timestamp::now();
    check(before_epoch.seconds == 0 && before_epoch.fraction == 0u,
          "TC-RTPS-TIME-002 an instant before the epoch is zero");
    return test_summary();
}
