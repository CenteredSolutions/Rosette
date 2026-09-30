/**
 * @file ports/chip/stub/chip_stub.cpp
 * @brief Compile-only chip layer: every hook of the chip contract, doing nothing useful.
 *
 * The stub port is the smallest chip port that links. It exists so the
 * portable core is built for a chip that is neither the RP2040 / RP2350 nor a
 * host (tests/port_stub, run in CI): a portable file that calls into the Pico
 * SDK or a host OS then fails to link here, where an include check alone
 * would miss it. It also shows a new port the whole contract it must supply,
 * in one file:
 *
 *  - rosette/platform/platform.hpp: monotonic_us(), device_unique_id(),
 *    realtime_hint_us();
 *  - rosette/platform/flash.hpp: the storage flash region and its
 *    erase/program/read primitives;
 *  - rosette/platform/littlefs_platform.hpp: mounting the LittleFS volume
 *    (stub_littlefs.cpp, compiled into rosette_storage, which has lfs.h);
 *  - rosette/platform/host_runtime.hpp: the older runtime API, until it is
 *    removed (docs/design/ports_move_plan.md, "Order", step 6).
 *
 * The clock counts calls rather than time, so waits that spin on it
 * (ports/os/stub) still end.
 *
 * Verification:
 * - Correct means: tests/port_stub links, so every symbol the portable core
 *   needs from a chip is defined here and nowhere else is needed.
 */

#include "rosette/platform/flash.hpp"
#include "rosette/platform/host_runtime.hpp"
#include "rosette/platform/platform.hpp"

namespace rosette::platform {

uint64_t monotonic_us() noexcept {
    static uint64_t ticks = 0u;
    return ++ticks;
}

void device_unique_id(uint8_t out[8]) noexcept {
    for (int i = 0; i < 8; ++i) {
        out[i] = 0u;
    }
}

bool realtime_hint_us(int64_t& unix_us) noexcept {
    (void)unix_us;
    return false;
}

}  // namespace rosette::platform

extern "C" {

rosette_flash_region rosette_platform_flash_region(void) {
    return rosette_flash_region{};
}

uint32_t rosette_platform_flash_sector_size(void) {
    return 4096u;
}

uint32_t rosette_platform_flash_page_size(void) {
    return 256u;
}

void rosette_platform_flash_erase(uint32_t addr, uint32_t size) {
    (void)addr;
    (void)size;
}

void rosette_platform_flash_write(uint32_t addr, const void* data, uint32_t size) {
    (void)addr;
    (void)data;
    (void)size;
}

void rosette_platform_flash_read(uint32_t addr, void* data, uint32_t size) {
    (void)addr;
    (void)data;
    (void)size;
}

uint32_t rosette_platform_get_ms(void) {
    return rosette::platform::monotonic_ms();
}

uint32_t rosette_platform_get_ipv4_addr(void) {
    return 0u;
}

bool rosette_platform_get_ipv6_addr(uint8_t out[16]) {
    (void)out;
    return false;
}

void rosette_platform_delay_ms(uint32_t ms) {
    rosette::platform::sleep_ms(ms);
}

void rosette_platform_wait_for_event(uint32_t max_wait_ms) {
    rosette::platform::sleep_ms(max_wait_ms);
}

void rosette_platform_set_ipv4_addr(uint32_t addr_host_order) {
    (void)addr_host_order;
}

void rosette_platform_set_ipv6_addr(const uint8_t addr[16]) {
    (void)addr;
}

}  // extern "C"
