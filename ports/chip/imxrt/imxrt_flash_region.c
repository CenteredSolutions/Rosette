/**
 * @file ports/chip/imxrt/imxrt_flash_region.c
 * @brief The flash hooks (rosette/platform/flash.hpp) on the i.MX RT: no storage region.
 *
 * The i.MX RT port reserves no flash for the Rosette storage volume:
 * rosette_platform_flash_region() reports an empty region, so
 * rosette::storage stays unmounted (imxrt_littlefs_unmounted.cpp), and erase
 * and program do nothing. Reads are served from the memory-mapped FlexSPI
 * window, which is how every byte of flash is reachable while the image
 * executes in place. The geometry is that of the QSPI NOR flash the boot
 * block describes (imxrt_flexspi_nor_config.c).
 *
 * Verification:
 * - Correct means: the storage layer sees `size == 0` and never erases or
 *   programs; a read returns the flash contents at `addr`.
 * - Covered by: tests/platform/platform_contract_test.cpp (flash region query).
 *
 * References: i.MX RT1060 Reference Manual §2.3 (FlexSPI AHB window at 0x60000000).
 */

#include <string.h>

#include "rosette/platform/flash.hpp"

/// Start of the FlexSPI memory-mapped (AHB) read window.
#define IMXRT_FLEXSPI_AMBA_BASE 0x60000000u

rosette_flash_region rosette_platform_flash_region(void) {
    const rosette_flash_region none = {0u, 0u};
    return none;
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
    memcpy(data, (const void*)(uintptr_t)(IMXRT_FLEXSPI_AMBA_BASE + addr), size);
}
