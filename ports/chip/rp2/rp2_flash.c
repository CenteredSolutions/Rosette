/**
 * @file platform/pico_bare.c
 * @brief RP2040 / RP2350 flash hooks and newlib time shims for a no-RTOS build.
 *
 * Provides `rosette_platform_flash_*` (flash.hpp) for boards that run a bare
 * superloop instead of FreeRTOS, and backs newlib's `clock_gettime()` /
 * `_gettimeofday()` with the hardware timer. The FreeRTOS build gets the
 * flash hooks from `platform/pico_freertos.c`.
 */

#include "pico/stdlib.h"
#include "pico/time.h"
#include "hardware/flash.h"
#include "hardware/sync.h"

#include <string.h>
#include <time.h>
#include <sys/time.h>

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

// ------------------------------------------------------------
// POSIX time shims for newlib.
//
// The Micro XRCE-DDS client's time source (uxr_millis / uxr_nanos) calls
// clock_gettime(), which bare newlib does not implement. Back it with the
// microsecond hardware timer; micro-ROS then synchronizes this monotonic
// base to the Agent's wall clock via rmw_uros_sync_session().
// ------------------------------------------------------------
int clock_gettime(clockid_t clk_id, struct timespec* tp)
{
    (void)clk_id;
    if (tp == NULL) {
        return -1;
    }
    const uint64_t us = time_us_64();
    tp->tv_sec = (time_t)(us / 1000000ull);
    tp->tv_nsec = (long)((us % 1000000ull) * 1000ull);
    return 0;
}

int _gettimeofday(struct timeval* tv, void* tz)
{
    (void)tz;
    if (tv == NULL) {
        return -1;
    }
    const uint64_t us = time_us_64();
    tv->tv_sec = (time_t)(us / 1000000ull);
    tv->tv_usec = (suseconds_t)(us % 1000000ull);
    return 0;
}
