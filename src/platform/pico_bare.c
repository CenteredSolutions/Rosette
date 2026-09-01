/**
 * @file platform/pico_bare.c
 * @brief RP2040 / RP2350 platform runtime for a no-RTOS superloop build.
 *
 * Provides the `rosette_platform_*` primitives (clock, delay, local address,
 * flash) for boards that run a bare `for(;;)` loop over `rosette::App::spin_once()`
 * instead of FreeRTOS. This is the default for the W5100S-EVB-Pico2, where the
 * WIZnet chip does the TCP/IP work in hardware and no RTOS is needed.
 *
 * The FreeRTOS build gets the same symbols from `platform/pico_freertos.c`.
 */

#include "rosette/platform/host_runtime.hpp"

#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/flash.h"
#include "hardware/sync.h"

#include <string.h>

// Set by the network bring-up layer once an address is assigned.
static volatile uint32_t s_local_ipv4_host_order = 0u;

uint32_t rosette_platform_get_ms(void)
{
    return to_ms_since_boot(get_absolute_time());
}

void rosette_platform_delay_ms(uint32_t ms)
{
    sleep_ms(ms);
}

void rosette_platform_set_ipv4_addr(uint32_t addr_host_order)
{
    s_local_ipv4_host_order = addr_host_order;
}

uint32_t rosette_platform_get_ipv4_addr(void)
{
    return s_local_ipv4_host_order;
}

bool rosette_platform_get_ipv6_addr(uint8_t out[16])
{
    if (out != NULL) {
        memset(out, 0, 16u);
    }
    return false;
}

// ------------------------------------------------------------
// Flash — bare-metal (single core, no scheduler to suspend).
//
// flash_range_erase / flash_range_program run with interrupts disabled and
// XIP paused; on a single-core superloop that is all the protection needed.
// The reserved-region getter and geometry live in pico_littlefs.cpp.
// See platform/flash.hpp for the timing caveat callers must respect.
// ------------------------------------------------------------

void rosette_platform_flash_erase(uint32_t addr, uint32_t size)
{
    const uint32_t save = save_and_disable_interrupts();
    flash_range_erase(addr, size);
    restore_interrupts(save);
}

void rosette_platform_flash_write(uint32_t addr, const void* data, uint32_t size)
{
    const uint32_t save = save_and_disable_interrupts();
    flash_range_program(addr, (const uint8_t*)data, size);
    restore_interrupts(save);
}

void rosette_platform_flash_read(uint32_t addr, void* data, uint32_t size)
{
    memcpy(data, (const void*)(XIP_BASE + addr), size);
}
