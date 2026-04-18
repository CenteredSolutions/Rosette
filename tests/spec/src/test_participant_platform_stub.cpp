#include "rtps/types.hpp"

#include <stdint.h>

namespace spec_test {
uint32_t g_participant_test_now_ms = 0u;
}

extern "C" uint32_t rtps_platform_get_ms(void) {
    return spec_test::g_participant_test_now_ms;
}

extern "C" rtps::GuidPrefix rtps_platform_make_guid_prefix(uint8_t participant_id) noexcept {
    rtps::GuidPrefix gp{};
    gp.value[0] = 0xA1u;
    gp.value[1] = 0xB2u;
    gp.value[2] = 0xC3u;
    gp.value[11] = participant_id;
    return gp;
}

extern "C" uint32_t rtps_platform_get_ipv4_addr(void) noexcept {
    return 0xC0A8010Au;
}

extern "C" bool rtps_platform_get_ipv6_addr(uint8_t out[16]) noexcept {
    if (out != nullptr) {
        for (int i = 0; i < 16; ++i) out[i] = 0u;
    }
    return false;
}
