/**
 * @file platform/pico_freertos.c
 * @brief Flash hooks (flash.hpp) for RP2350 and RP2040 under FreeRTOS.
 *
 * Implements:
 *
 *   rosette_platform_flash_erase()         — RP2350/RP2040 flash erase
 *   rosette_platform_flash_write()         — RP2350/RP2040 flash program
 *   rosette_platform_flash_read()          — RP2350/RP2040 flash read
 *
 * All hooks are board-agnostic (no GPIO pin dependencies) and compile
 * identically for RP2040 and RP2350 from the same source file.
 *
 * Flash layout note
 * -----------------
 * The RP2350/RP2040 execute-in-place flash starts at XIP_BASE (0x10000000).
 * rosette_platform_flash_erase/write operate on addresses relative to the
 * start of flash (i.e. offset 0 = XIP_BASE).  The caller (persistence.cpp)
 * must pass offsets that do not overlap with the firmware image.
 *
 * Recommended layout: place the RTPS persistence region at the top of flash.
 * For a 2 MB flash with 4 KB sectors and 4 writers × 4 KB = 16 KB needed:
 *   RTPS persistence region: flash[0x1F0000 .. 0x1FFFFF] (last 64 KB)
 *
 * The SDK's flash_range_erase() and flash_range_program() require that
 * execution not be running from the flash region being modified.  Both
 * functions briefly disable XIP and run from SRAM.  Interrupts that
 * might trigger flash reads (ISRs in flash) must be disabled during the
 * operation — the SDK functions do this internally via __compiler_barrier
 * and NVIC masking.
 */

/* Pico SDK */
#include "pico/stdlib.h"
#include "hardware/flash.h"
#include "hardware/sync.h"

/* FreeRTOS */
#include "FreeRTOS.h"
#include "task.h"

#include <string.h>
#include <stdint.h>

/* The RTPS stack declares these as extern "C". */
#ifdef __cplusplus
extern "C" {
#endif

// ============================================================
// Flash persistence hooks
//
// The RP2350/RP2040 Pico SDK provides:
//   flash_range_erase(offset, count)   — erase `count` bytes at `offset`
//   flash_range_program(offset, data, count) — program `count` bytes
//
// `offset` is relative to the start of flash (not XIP_BASE).
//
// IMPORTANT: These functions disable interrupts and cannot be called
// from an ISR.  They must also not be called while other cores are
// executing from flash (use multicore_lockout if applicable).
//
// On a single-core FreeRTOS build with no multicore usage this is safe
// as long as we suspend the FreeRTOS scheduler during the operation to
// prevent ISR-driven flash reads.
// ============================================================

void rosette_platform_flash_erase(uint32_t addr, uint32_t size) {
    /* addr is the flash byte offset (relative to flash start).
     * FLASH_SECTOR_SIZE = 4096 on RP2350/RP2040. */
    configASSERT((addr % FLASH_SECTOR_SIZE) == 0);
    configASSERT((size % FLASH_SECTOR_SIZE) == 0);

    /* Suspend the scheduler so no FreeRTOS timer ISR triggers a flash read. */
    vTaskSuspendAll();
    uint32_t save = save_and_disable_interrupts();

    flash_range_erase(addr, size);

    restore_interrupts(save);
    xTaskResumeAll();
}

void rosette_platform_flash_write(uint32_t addr,
                                const void* data,
                                uint32_t size) {
    /* addr must be 256-byte aligned (flash page size). */
    configASSERT((addr % FLASH_PAGE_SIZE) == 0);
    configASSERT((size % FLASH_PAGE_SIZE) == 0);

    vTaskSuspendAll();
    uint32_t save = save_and_disable_interrupts();

    flash_range_program(addr, (const uint8_t*)data, size);

    restore_interrupts(save);
    xTaskResumeAll();
}

void rosette_platform_flash_read(uint32_t addr, void* data, uint32_t size) {
    /* Flash is memory-mapped at XIP_BASE; read via memcpy. */
    memcpy(data, (const void*)(XIP_BASE + addr), size);
}

#ifdef __cplusplus
}
#endif
