/**
 * @file platform/rtps_platform.c
 * @brief Platform hook implementations for RP2350 and RP2040.
 *
 * Implements the three extern "C" hooks declared in the RTPS stack:
 *
 *   rtps_platform_get_ms()              — monotonic millisecond counter
 *   rtps_platform_make_guid_prefix()    — unique participant GUID prefix
 *   rtps_platform_flash_erase()         — RP2350/RP2040 flash erase
 *   rtps_platform_flash_write()         — RP2350/RP2040 flash program
 *   rtps_platform_flash_read()          — RP2350/RP2040 flash read
 *
 * All hooks are board-agnostic (no GPIO pin dependencies) and compile
 * identically for RP2040 and RP2350 from the same source file.
 *
 * Flash layout note
 * -----------------
 * The RP2350/RP2040 execute-in-place flash starts at XIP_BASE (0x10000000).
 * rtps_platform_flash_erase/write operate on addresses relative to the
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
#include "pico/unique_id.h"
#include "hardware/flash.h"
#include "hardware/sync.h"

/* FreeRTOS */
#include "FreeRTOS.h"
#include "task.h"

#if defined(MILKDDS_FREERTOS_TCP)
#include "FreeRTOS_Routing.h"
#include "FreeRTOS_IP_Common.h"
#endif

#include <string.h>
#include <stdint.h>

/* The RTPS stack declares these as extern "C". */
#ifdef __cplusplus
extern "C" {
#endif

// ============================================================
// rtps_platform_get_ms()
//
// Returns a monotonically increasing millisecond counter derived
// from the FreeRTOS tick count.  Wraps at ~49.7 days (uint32_t).
//
// portTICK_PERIOD_MS = 1000 / configTICK_RATE_HZ.
// With configTICK_RATE_HZ = 1000, this is exactly 1 ms per tick.
// With configTICK_RATE_HZ = 100, multiply by 10.
// ============================================================

uint32_t rtps_platform_get_ms(void) {
    return (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
}

// ============================================================
// rtps_platform_make_guid_prefix()
//
// Derives a 12-byte GUID prefix unique to this device.
//
// Format:
//   bytes 0-1  : vendor code 0xEE, 0xC0 (project-specific; not OMG-assigned)
//   byte  2    : participant_id
//   bytes 3-9  : 7 bytes from the RP2350/RP2040 64-bit unique board ID
//   bytes 10-11: 0x00, 0x00 (reserved)
//
// The pico_unique_board_id is derived from the flash chip's 64-bit UID
// via the QSPI flash RUID command — unique per device.
// ============================================================

typedef struct { uint8_t value[12]; } rtps_GuidPrefix;  /* mirrors GuidPrefix */

rtps_GuidPrefix rtps_platform_make_guid_prefix(uint8_t participant_id) {
    pico_unique_board_id_t uid;
    pico_get_unique_board_id(&uid);

    rtps_GuidPrefix prefix;
    prefix.value[0]  = 0xEEu;           /* Project vendor byte 0 */
    prefix.value[1]  = 0xC0u;           /* Project vendor byte 1 */
    prefix.value[2]  = participant_id;
    prefix.value[3]  = uid.id[1];
    prefix.value[4]  = uid.id[2];
    prefix.value[5]  = uid.id[3];
    prefix.value[6]  = uid.id[4];
    prefix.value[7]  = uid.id[5];
    prefix.value[8]  = uid.id[6];
    prefix.value[9]  = uid.id[7];
    prefix.value[10] = 0x00u;
    prefix.value[11] = 0x00u;
    return prefix;
}

uint32_t rtps_platform_get_ipv4_addr(void) {
#if defined(MILKDDS_FREERTOS_TCP)
    for (NetworkEndPoint_t *ep = FreeRTOS_FirstEndPoint(NULL);
         ep != NULL;
         ep = FreeRTOS_NextEndPoint(NULL, ep)) {
        if ((ENDPOINT_IS_IPv4(ep) != 0) && (FreeRTOS_IsEndPointUp(ep) != 0)) {
            return ep->ipv4_settings.ulIPAddress;
        }
    }
    return 0u;
#else
    return 0u;
#endif
}

bool rtps_platform_get_ipv6_addr(uint8_t out[16]) {
    if (out == NULL) {
        return false;
    }

    memset(out, 0, 16u);

#if defined(MILKDDS_FREERTOS_TCP)
    NetworkEndPoint_t *preferred = NULL;

    for (NetworkEndPoint_t *ep = FreeRTOS_FirstEndPoint(NULL);
         ep != NULL;
         ep = FreeRTOS_NextEndPoint(NULL, ep)) {
        if ((ENDPOINT_IS_IPv6(ep) == 0) || (FreeRTOS_IsEndPointUp(ep) == 0)) {
            continue;
        }

        preferred = ep;
        if ((ep->ipv6_settings.xIPAddress.ucBytes[0] != 0xFEu) ||
            ((ep->ipv6_settings.xIPAddress.ucBytes[1] & 0xC0u) != 0x80u)) {
            break;
        }
    }

    if (preferred == NULL) {
        return false;
    }

    memcpy(out, preferred->ipv6_settings.xIPAddress.ucBytes, 16u);
    return true;
#endif

    return false;
}

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

void rtps_platform_flash_erase(uint32_t addr, uint32_t size) {
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

void rtps_platform_flash_write(uint32_t addr,
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

void rtps_platform_flash_read(uint32_t addr, void* data, uint32_t size) {
    /* Flash is memory-mapped at XIP_BASE; read via memcpy. */
    memcpy(data, (const void*)(XIP_BASE + addr), size);
}

#ifdef __cplusplus
}
#endif
