#pragma once

/**
 * @file tests/host/fake_runtime.hpp
 * @brief Controllable `rosette_platform_*` for host tests of the address layer.
 *
 * The clock does not tick on its own — `fake::advance()` / `fake::set_now()`.
 * `rosette_platform_delay_ms()` / `_wait_for_event()` advance it so cooperative
 * loops (`NetworkAddress::wait()`) terminate.
 */

#include <array>
#include <cstdint>
#include <cstring>

namespace fake {
inline uint32_t g_now_ms = 0u;
inline uint32_t g_ipv4 = 0u;
inline std::array<uint8_t, 16> g_ipv6{};

inline void reset() {
    g_now_ms = 0u;
    g_ipv4 = 0u;
    g_ipv6.fill(0u);
}
inline void set_now(uint32_t ms) { g_now_ms = ms; }
inline void advance(uint32_t ms) { g_now_ms += ms; }
inline uint32_t now() { return g_now_ms; }
inline uint32_t ipv4() { return g_ipv4; }
inline std::array<uint8_t, 16> ipv6() { return g_ipv6; }
}  // namespace fake

extern "C" {
inline uint32_t rosette_platform_get_ms(void) { return fake::g_now_ms; }
inline void rosette_platform_delay_ms(uint32_t ms) { fake::g_now_ms += ms; }
inline void rosette_platform_wait_for_event(uint32_t ms) { fake::g_now_ms += ms; }
inline uint32_t rosette_platform_get_ipv4_addr(void) { return fake::g_ipv4; }
inline void rosette_platform_set_ipv4_addr(uint32_t a) { fake::g_ipv4 = a; }
inline bool rosette_platform_get_ipv6_addr(uint8_t out[16]) {
    std::memcpy(out, fake::g_ipv6.data(), 16u);
    for (int i = 0; i < 16; ++i) {
        if (fake::g_ipv6[static_cast<std::size_t>(i)] != 0u) {
            return true;
        }
    }
    return false;
}
inline void rosette_platform_set_ipv6_addr(const uint8_t a[16]) {
    if (a != nullptr) {
        std::memcpy(fake::g_ipv6.data(), a, 16u);
    } else {
        fake::g_ipv6.fill(0u);
    }
}
}  // extern "C"
