// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file ports/chip/rp2/rp2_flash.c
 * @brief RP2040 / RP2350 flash hooks (flash.hpp), for every OS.
 *
 * Erasing or programming the flash stops execute-in-place, so nothing may run
 * from flash meanwhile: not an interrupt handler on this core, and not the
 * other core. The SDK's flash_safe_execute() arranges that for whatever the
 * image runs: interrupts off on a single core, a multicore lockout when the
 * other core has opted in (flash_safe_execute_core_init()), and a handshake
 * task under FreeRTOS SMP. When it cannot (the other core runs but never
 * opted in), the operation runs with interrupts off on this core only, which
 * is what a single-core image needs and what this port did before.
 *
 * Addresses are byte offsets from the start of flash (XIP_BASE); erases are
 * whole sectors and programs whole pages (flash.hpp).
 *
 * References: Pico SDK pico_flash flash_safe_execute(), hardware_flash
 * flash_range_erase() / flash_range_program(); RP2350 datasheet §5.1.
 */

#include <stdint.h>
#include <string.h>

#include "hardware/flash.h"
#include "hardware/sync.h"
#include "pico/flash.h"

/** One erase or program, as flash_safe_execute() hands it to flash_op(). */
struct flash_request {
    uint32_t addr;
    const uint8_t* data; /**< NULL to erase. */
    uint32_t size;
};

static void flash_op(void* param) {
    const struct flash_request* request = (const struct flash_request*)param;
    if (request->data == NULL) {
        flash_range_erase(request->addr, request->size);
    } else {
        flash_range_program(request->addr, request->data, request->size);
    }
}

/** Runs `request` with nothing executing from flash; see the file comment. */
static void run_flash_op(struct flash_request* request) {
    if (flash_safe_execute(flash_op, request, UINT32_MAX) == PICO_OK) {
        return;
    }

    const uint32_t save = save_and_disable_interrupts();
    flash_op(request);
    restore_interrupts(save);
}

void rosette_platform_flash_erase(uint32_t addr, uint32_t size) {
    struct flash_request request = {addr, NULL, size};
    run_flash_op(&request);
}

void rosette_platform_flash_write(uint32_t addr, const void* data, uint32_t size) {
    struct flash_request request = {addr, (const uint8_t*)data, size};
    run_flash_op(&request);
}

void rosette_platform_flash_read(uint32_t addr, void* data, uint32_t size) {
    memcpy(data, (const void*)(XIP_BASE + addr), size);
}
