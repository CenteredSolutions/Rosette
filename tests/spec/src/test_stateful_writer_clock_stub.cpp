/**
 * @file test_stateful_writer_clock_stub.cpp
 * @brief Test chip layer for StatefulWriter and SEDP tests: a clock the tests set explicitly.
 *
 * Replaces ports/chip/host/chip_host.cpp in this test binary, so
 * platform::monotonic_ms() returns g_stateful_writer_test_now_ms and timer behavior is
 * deterministic.
 */

#include "rosette/platform/platform.hpp"

#include <stdint.h>

namespace spec_test {
uint32_t g_stateful_writer_test_now_ms = 0u;
}

namespace rosette::platform {

uint64_t monotonic_us() noexcept {
    return static_cast<uint64_t>(spec_test::g_stateful_writer_test_now_ms) * 1000u;
}

void device_unique_id(uint8_t out[8]) noexcept {
    for (int i = 0; i < 8; ++i) out[i] = static_cast<uint8_t>(0x10 + i);
}

uint32_t boot_nonce() noexcept {
    return 0x10203040u;
}

bool realtime_hint_us(int64_t& unix_us) noexcept {
    (void)unix_us;
    return false;
}

}  // namespace rosette::platform
