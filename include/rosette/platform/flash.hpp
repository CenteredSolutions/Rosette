// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file platform/flash.hpp
 * @brief Board-provided raw flash primitives and the reserved storage region.
 *
 * Rosette keeps a small LittleFS volume in a region of on-board flash that the
 * application image never touches. These `extern "C"` primitives are the only
 * flash operations the storage layer needs; each chip port implements them:
 *
 *  - `ports/chip/rp2/rp2_flash.c` and `rp2_littlefs_region.cpp` — RP2040 /
 *    RP2350 via the Pico SDK
 *  - `ports/chip/host/host_flash_file.cpp` — a file stand-in for host builds
 *  - `ports/chip/stub/` — the hooks a new chip port must supply
 *
 * Offsets are byte offsets from the start of flash, not absolute addresses.
 * `rosette_platform_flash_region()` reports where the storage volume lives and
 * how large it is; the values come from the board's `board_config.h`
 * (`ROSETTE_STORAGE_FLASH_OFFSET` / `ROSETTE_STORAGE_FLASH_SIZE`).
 *
 * Timing note: on RP2040 an erase or program call pauses execute-in-place and
 * runs with interrupts disabled for the duration (tens of milliseconds for a
 * multi-page write). Callers on a live micro-ROS session must budget for that
 * stall or defer the write to a point in the loop where a missed Agent ping is
 * acceptable.
 */

#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Describes the flash region reserved for the storage volume.
 */
typedef struct {
    uint32_t offset;  ///< Byte offset from the start of flash.
    uint32_t size;    ///< Region length in bytes; a multiple of the erase unit.
} rosette_flash_region;

/**
 * @brief Returns the flash region reserved for the Rosette storage volume.
 *
 * @return The board's storage region. `size == 0` means the board declares no
 * persistent storage and the storage layer should stay unmounted.
 */
rosette_flash_region rosette_platform_flash_region(void);

/**
 * @brief Returns the flash erase unit (sector) size in bytes.
 *
 * @return Erase granularity; 4096 on RP2040 / RP2350.
 */
uint32_t rosette_platform_flash_sector_size(void);

/**
 * @brief Returns the flash program unit (page) size in bytes.
 *
 * @return Program granularity; 256 on RP2040 / RP2350.
 */
uint32_t rosette_platform_flash_page_size(void);

/**
 * @brief Erases @p size bytes of flash starting at @p addr.
 *
 * @param[in] addr Sector-aligned byte offset from the start of flash.
 * @param[in] size Sector-aligned length in bytes.
 */
void rosette_platform_flash_erase(uint32_t addr, uint32_t size);

/**
 * @brief Programs @p size bytes at @p addr from @p data.
 *
 * The target range must already be erased.
 *
 * @param[in] addr Page-aligned byte offset from the start of flash.
 * @param[in] data Source buffer of at least @p size bytes.
 * @param[in] size Page-aligned length in bytes.
 */
void rosette_platform_flash_write(uint32_t addr, const void* data, uint32_t size);

/**
 * @brief Reads @p size bytes from flash at @p addr into @p data.
 *
 * @param[in] addr Byte offset from the start of flash.
 * @param[out] data Destination buffer of at least @p size bytes.
 * @param[in] size Number of bytes to read.
 */
void rosette_platform_flash_read(uint32_t addr, void* data, uint32_t size);

#ifdef __cplusplus
}  // extern "C"
#endif
