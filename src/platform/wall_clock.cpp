// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file wall_clock.cpp
 * @brief WallClock: one offset over the platform monotonic clock, and rcl's view of it.
 *
 * The offset is 64 bits, which the RP2040's Cortex-M0+ cannot read or write
 * in one access, and it is read far more often than it is written: every
 * ROS stamp, RTPS timestamp, and rcl clock read takes it, while a time source
 * writes it every few minutes. It is published under a sequence lock, the
 * scheme Linux timekeeping uses for the same data: a writer makes the
 * sequence number odd, stores the offset, and makes it even again; a reader
 * takes the sequence, the offset, and the sequence again, and keeps the
 * offset when both readings match and are even. Readers take no lock, so a
 * read costs a few loads and barriers instead of a Mutex round trip.
 *
 * A reader that wants the time samples monotonic_us() between its two
 * sequence loads, so the tick and the offset it adds them to come from one
 * interval in which no writer stored. Sampled after the check, a reader
 * preempted between the two could add an offset already replaced to a later
 * tick and return a time the clock never showed.
 *
 * Writers (set_offset_us() and the seeding from the realtime hint) still
 * serialize on a Mutex. A reader that sees a write overlap its read
 * kReadAttempts times in a row takes that Mutex instead: on a single-core
 * RTOS a reader that preempted a writer would otherwise spin while the
 * writer cannot run, and the Mutex's priority inheritance lets the writer
 * finish. WallClock is therefore callable from thread context on any core,
 * not from interrupt handlers.
 *
 * The firmware ROS stacks' rcutils (cmake/micro_ros_bootstrap.sh.in) take
 * `rcutils_system_time_now()` from a weak `rosette_micro_ros_epoch_time_us()`
 * hook. The strong definition here makes rcl's system clock this WallClock
 * on both ROS backends. `clock_gettime()` stays on the monotonic tick,
 * because the XRCE client times its waits with it (`uxr_millis()`) and a
 * clock that steps at synchronization would cut those waits short.
 */

#include "rosette/platform/wall_clock.hpp"

#include <atomic>

#include "rosette/platform/platform.hpp"

namespace rosette::platform {
namespace {

/// Set once the offset holds its first value, from the realtime hint or a time source.
constexpr uint32_t kSeeded = 1u;
/// Set once the offset came from the realtime hint or a time source.
constexpr uint32_t kSynchronized = 2u;
/// Lock-free reads a reader tries before it waits on the writers' Mutex.
constexpr int      kReadAttempts = 4;

/// The published clock state. Constant-initialized, so the read path has no static guard.
struct PublishedClock {
    std::atomic<uint32_t> sequence{0u};  ///< Odd while a writer is storing.
    std::atomic<uint32_t> offset_low{0u};
    std::atomic<uint32_t> offset_high{0u};
    std::atomic<uint32_t> flags{0u};  ///< kSeeded and kSynchronized.
};

PublishedClock g_clock;

/// One consistent reading of the published state, and the tick it applies to.
struct ClockSnapshot {
    int64_t  offset_us;
    uint32_t flags;
    uint64_t tick_us;  ///< monotonic_us() inside the reading; 0 when not sampled.
};

/// Serializes writers, and readers that gave up on the lock-free path.
Mutex& writer_mutex() noexcept {
    static Mutex mutex;
    return mutex;
}

/**
 * @brief Loads the published fields; consistent only under the sequence check or writer_mutex().
 *
 * Acquire loads keep the reader's second sequence load after them, and pair
 * with the writer's release stores, without a standalone fence, which
 * ThreadSanitizer does not model (-Wtsan).
 */
ClockSnapshot load_fields() noexcept {
    const uint32_t low = g_clock.offset_low.load(std::memory_order_acquire);
    const uint32_t high = g_clock.offset_high.load(std::memory_order_acquire);
    const uint32_t flags = g_clock.flags.load(std::memory_order_acquire);
    const uint64_t bits = (static_cast<uint64_t>(high) << 32) | low;
    return ClockSnapshot{static_cast<int64_t>(bits), flags, 0u};
}

/**
 * @brief Publishes a new offset and flags under the sequence lock.
 *
 * The caller holds writer_mutex(). Each field store is a release, so the odd
 * sequence number is visible before it; the release store of the even
 * number keeps the fields before it. A reader that sees the same even number
 * on both sides of its loads therefore saw no store of this write (Boehm,
 * "Can Seqlocks Get Along with Programming Language Memory Models?", 2012,
 * the variant with no standalone fences).
 */
void publish_locked(int64_t offset_us, uint32_t flags) noexcept {
    const uint32_t sequence = g_clock.sequence.load(std::memory_order_relaxed);
    g_clock.sequence.store(sequence + 1u, std::memory_order_relaxed);

    const uint64_t bits = static_cast<uint64_t>(offset_us);
    g_clock.offset_low.store(static_cast<uint32_t>(bits), std::memory_order_release);
    g_clock.offset_high.store(static_cast<uint32_t>(bits >> 32), std::memory_order_release);
    g_clock.flags.store(flags, std::memory_order_release);
    g_clock.sequence.store(sequence + 2u, std::memory_order_release);
}

/// Seeds the offset from the platform hint the first time it is needed; caller holds
/// writer_mutex().
void seed_locked() noexcept {
    if ((load_fields().flags & kSeeded) != 0u) {
        return;
    }

    int64_t unix_us = 0;
    if (realtime_hint_us(unix_us)) {
        publish_locked(unix_us - static_cast<int64_t>(monotonic_us()), kSeeded | kSynchronized);
    } else {
        publish_locked(0, kSeeded);
    }
}

/**
 * @brief Reads the published offset and flags, seeding them on first use.
 *
 * Tries the lock-free read kReadAttempts times; each attempt is kept when
 * the sequence number is even and unchanged across the tick and field loads,
 * and the state is seeded. Otherwise it takes writer_mutex(), which also
 * seeds an unseeded clock, and samples the tick under it.
 *
 * @param[in] sample_tick True to sample monotonic_us() inside the reading,
 *   for a caller that adds the offset to it; false leaves tick_us at 0.
 * @return The offset and flags from one interval with no write, and the tick
 *   sampled in that interval.
 *
 * Verification:
 * - Correct means: the offset and tick of a reading were current together:
 *   a write that lands between them makes the reading retry.
 * - Covered by: tests/host/wall_clock_tick_test.cpp (TC-PLAT-WC-003);
 *   tests/host/wall_clock_concurrency_test.cpp (TC-PLAT-WC-001, TC-PLAT-WC-002).
 */
ClockSnapshot read_clock(bool sample_tick) noexcept {
    for (int attempt = 0; attempt < kReadAttempts; ++attempt) {
        const uint32_t before = g_clock.sequence.load(std::memory_order_acquire);
        const uint64_t tick_us = sample_tick ? monotonic_us() : 0u;
        ClockSnapshot  snapshot = load_fields();
        const uint32_t after = g_clock.sequence.load(std::memory_order_relaxed);
        if (before == after && (before & 1u) == 0u && (snapshot.flags & kSeeded) != 0u) {
            snapshot.tick_us = tick_us;
            return snapshot;
        }
    }

    LockGuard guard(writer_mutex());
    seed_locked();
    ClockSnapshot snapshot = load_fields();
    snapshot.tick_us = sample_tick ? monotonic_us() : 0u;
    return snapshot;
}

}  // namespace

int64_t WallClock::now_us() noexcept {
    const ClockSnapshot clock = read_clock(true);
    return static_cast<int64_t>(clock.tick_us) + clock.offset_us;
}

bool WallClock::try_now_us(int64_t& unix_us) noexcept {
    const ClockSnapshot clock = read_clock(true);
    if ((clock.flags & kSynchronized) == 0u) {
        return false;
    }

    unix_us = static_cast<int64_t>(clock.tick_us) + clock.offset_us;
    return true;
}

int64_t WallClock::offset_us() noexcept {
    return read_clock(false).offset_us;
}

void WallClock::set_offset_us(int64_t offset_us) noexcept {
    LockGuard guard(writer_mutex());
    publish_locked(offset_us, kSeeded | kSynchronized);
}

bool WallClock::synchronized() noexcept {
    return (read_clock(false).flags & kSynchronized) != 0u;
}

}  // namespace rosette::platform

/**
 * @brief rcl's system time on firmware: WallClock::now_us(), in Unix microseconds.
 *
 * Overrides the weak definition in the firmware ROS stacks' rcutils time
 * source, which returns the bare monotonic tick.
 *
 * @return Microseconds since the Unix epoch once synchronized; before that,
 *   the monotonic time itself (near 1970), as the weak default returns.
 *
 * Verification:
 * - Correct means: `rcutils_system_time_now()` equals `WallClock::now_us() * 1000`.
 * - Covered by: nm on a firmware image shows this definition, not the weak one.
 *
 * References:
 * - cmake/micro_ros_bootstrap.sh.in (the rcutils time source with the weak hook)
 */
extern "C" int64_t rosette_micro_ros_epoch_time_us(void) {
    return rosette::platform::WallClock::now_us();
}
