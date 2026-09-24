/**
 * @file wall_clock.cpp
 * @brief WallClock: one offset over the platform monotonic clock.
 *
 * The offset is 64 bits, which the RP2040's Cortex-M0+ cannot read or write
 * atomically, so it sits behind a platform Mutex; WallClock is therefore
 * callable from thread context on any core, not from interrupt handlers.
 */

#include "rosette/platform/wall_clock.hpp"

#include "rosette/platform/platform.hpp"

namespace rosette::platform {
namespace {

struct WallClockState {
    Mutex   mutex;
    int64_t offset_us = 0;
    bool    seeded = false;         ///< Offset initialized (from hint or sync).
    bool    synchronized = false;
};

WallClockState& state() noexcept {
    static WallClockState s;
    return s;
}

/// Seeds the offset from the platform hint the first time it is needed.
/// Caller holds state().mutex.
void seed_locked(WallClockState& s) noexcept {
    if (s.seeded) return;
    s.seeded = true;
    int64_t unix_us = 0;
    if (realtime_hint_us(unix_us)) {
        s.offset_us = unix_us - static_cast<int64_t>(monotonic_us());
        s.synchronized = true;
    }
}

}  // namespace

int64_t WallClock::now_us() noexcept {
    WallClockState& s = state();
    LockGuard guard(s.mutex);
    seed_locked(s);
    return static_cast<int64_t>(monotonic_us()) + s.offset_us;
}

int64_t WallClock::offset_us() noexcept {
    WallClockState& s = state();
    LockGuard guard(s.mutex);
    seed_locked(s);
    return s.offset_us;
}

void WallClock::set_offset_us(int64_t offset_us) noexcept {
    WallClockState& s = state();
    LockGuard guard(s.mutex);
    s.seeded = true;
    s.offset_us = offset_us;
    s.synchronized = true;
}

bool WallClock::synchronized() noexcept {
    WallClockState& s = state();
    LockGuard guard(s.mutex);
    seed_locked(s);
    return s.synchronized;
}

}  // namespace rosette::platform
