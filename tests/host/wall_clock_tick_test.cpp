/**
 * @file tests/host/wall_clock_tick_test.cpp
 * @brief WallClock reads the tick and the offset together, even when a time source sets the
 * offset mid-read.
 *
 * The chip layer here is a fake whose monotonic_us() can, once, act as a time source that runs
 * in the middle of a read: it sets the offset to kCorrectedOffsetUs at tick kCorrectionTickUs and
 * returns the later tick kResumeTickUs, as if the reader were preempted across the correction.
 * During such a call the clock showed kOffsetUs plus a tick before the correction, then
 * kCorrectedOffsetUs plus a tick from the correction on. A reader that checks the offset and then
 * samples the tick adds the replaced offset to the later tick, kOffsetUs + kResumeTickUs, a time
 * the clock never showed.
 *
 * - TC-PLAT-WC-003: now_us() and try_now_us() return a time the clock showed during the call
 *   when the offset is set mid-read.
 */

#include <cstdint>

#include "rosette/platform/platform.hpp"
#include "rosette/platform/wall_clock.hpp"
#include "test_util.hpp"

namespace pf = rosette::platform;

namespace {

constexpr int64_t  kSecondUs = 1'000'000;
/// The offset when a read begins, and the tick it begins at.
constexpr int64_t  kOffsetUs = 10 * kSecondUs;
constexpr uint64_t kStartTickUs = 1 * kSecondUs;
/// The time source sets kCorrectedOffsetUs at kCorrectionTickUs; the reader resumes at
/// kResumeTickUs.
constexpr int64_t  kCorrectedOffsetUs = 0;
constexpr uint64_t kCorrectionTickUs = 2 * kSecondUs;
constexpr uint64_t kResumeTickUs = 3 * kSecondUs;

uint64_t g_tick_us = kStartTickUs;
bool     g_correct_on_next_tick = false;

/// Starts a read at kStartTickUs with kOffsetUs set, and arms the mid-read correction.
void arm_correction() {
    g_tick_us = kStartTickUs;
    pf::WallClock::set_offset_us(kOffsetUs);
    g_correct_on_next_tick = true;
}

/// True when the clock showed @p unix_us at some tick from kStartTickUs to kResumeTickUs.
bool clock_showed(int64_t unix_us) {
    const int64_t start = static_cast<int64_t>(kStartTickUs);
    const int64_t correction = static_cast<int64_t>(kCorrectionTickUs);
    const int64_t resume = static_cast<int64_t>(kResumeTickUs);
    const bool before_correction = unix_us >= kOffsetUs + start && unix_us < kOffsetUs + correction;
    const bool after_correction =
        unix_us >= kCorrectedOffsetUs + correction && unix_us <= kCorrectedOffsetUs + resume;
    return before_correction || after_correction;
}

}  // namespace

namespace rosette::platform {

uint64_t monotonic_us() noexcept {
    if (g_correct_on_next_tick) {
        g_correct_on_next_tick = false;
        g_tick_us = kCorrectionTickUs;
        WallClock::set_offset_us(kCorrectedOffsetUs);
        g_tick_us = kResumeTickUs;
    }

    return g_tick_us;
}

bool realtime_hint_us(int64_t& unix_us) noexcept {
    (void)unix_us;
    return false;
}

}  // namespace rosette::platform

int main() {
    arm_correction();
    const int64_t now = pf::WallClock::now_us();
    std::printf("now_us()=%lld\n", static_cast<long long>(now));
    check(!g_correct_on_next_tick, "the read sampled the tick");
    check(clock_showed(now), "TC-PLAT-WC-003 now_us() returns a time the clock showed");

    arm_correction();
    int64_t    stamp_us = 0;
    const bool synced = pf::WallClock::try_now_us(stamp_us);
    std::printf("try_now_us()=%lld\n", static_cast<long long>(stamp_us));
    check(synced && clock_showed(stamp_us),
          "TC-PLAT-WC-003 try_now_us() returns a time the clock showed");
    return test_summary();
}
