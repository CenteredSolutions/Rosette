// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

#pragma once

/**
 * @file ports/chip/imxrt/imxrt_memory.h
 * @brief Where a board puts large objects on an i.MX RT10xx: DTCM by default, or the OCRAM.
 *
 * Everything the linker places by default (.data, .bss, the heap, the main
 * stack) is in the DTCM, which is not cached and which the ENET's DMA
 * reaches, so DMA rings and buffers belong there. The OCRAM (128 KiB on the
 * RT1021) is cached and otherwise unused; ROSETTE_IMXRT_OCRAM_BSS moves an
 * uninitialised object there (imxrt_ocram.ld). The startup code does not
 * zero it, so it suits storage its owner initialises itself, such as the
 * FreeRTOS heap.
 *
 * Example:
 *   ROSETTE_IMXRT_OCRAM_BSS uint8_t ucHeap[configTOTAL_HEAP_SIZE];
 */

#define ROSETTE_IMXRT_OCRAM_BSS __attribute__((section(".ocram.bss"), aligned(8)))
