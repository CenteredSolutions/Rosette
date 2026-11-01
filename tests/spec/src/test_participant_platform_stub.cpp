// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file test_participant_platform_stub.cpp
 * @brief Test chip layer for participant tests: a clock the tests set explicitly.
 *
 * Replaces ports/chip/host/chip_host.cpp in this test binary, so
 * platform::monotonic_ms() returns g_participant_test_now_ms and timer behavior is deterministic.
 */

#include "rosette/platform/platform.hpp"

#include <stdint.h>

namespace spec_test {
uint32_t g_participant_test_now_ms = 0u;
}

namespace rosette::platform {

uint64_t monotonic_us() noexcept {
    return static_cast<uint64_t>(spec_test::g_participant_test_now_ms) * 1000u;
}

void device_unique_id(uint8_t out[8]) noexcept {
    static const uint8_t id[8] = {0xA1u, 0xB2u, 0xC3u, 0x04u, 0x05u, 0x06u, 0x07u, 0x08u};
    for (int i = 0; i < 8; ++i) out[i] = id[i];
}

uint32_t boot_nonce() noexcept {
    return 0xB007C0DEu;
}

bool realtime_hint_us(int64_t& unix_us) noexcept {
    (void)unix_us;
    return false;
}

}  // namespace rosette::platform
