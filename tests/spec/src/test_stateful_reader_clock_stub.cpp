#include <stdint.h>

namespace spec_test {
uint32_t g_stateful_reader_test_now_ms = 0u;
}

extern "C" uint32_t rtps_platform_get_ms(void) {
    return spec_test::g_stateful_reader_test_now_ms;
}
