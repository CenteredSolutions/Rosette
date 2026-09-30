#pragma once

/**
 * @file tests/host/fake_clock.hpp
 * @brief A controllable platform clock for host tests of the address layer.
 *
 * Defines rosette::platform::monotonic_us() in place of the host chip port, so
 * a test that includes this header must not also link chip_host.cpp, and must
 * include it from one translation unit only. The clock does not tick on its
 * own: `fake::advance()` / `fake::set_now()`.
 */

#include <cstdint>

#include "rosette/platform/platform.hpp"

namespace fake {
inline uint32_t g_now_ms = 0u;

inline void reset() {
    g_now_ms = 0u;
}

inline void set_now(uint32_t ms) {
    g_now_ms = ms;
}

inline void advance(uint32_t ms) {
    g_now_ms += ms;
}

inline uint32_t now() {
    return g_now_ms;
}
}  // namespace fake

namespace rosette::platform {
uint64_t monotonic_us() noexcept {
    return static_cast<uint64_t>(fake::g_now_ms) * 1000u;
}
}  // namespace rosette::platform
