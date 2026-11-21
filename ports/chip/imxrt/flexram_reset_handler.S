/* SPDX-FileCopyrightText: 2026 Centered Solutions, LLC */
/* SPDX-License-Identifier: Apache-2.0 */

/*
 * ports/chip/imxrt/flexram_reset_handler.S — the reset handler for a board
 * that sets its own FlexRAM layout.
 *
 * The FlexRAM is one pool of 32 KiB banks that serve as ITCM, DTCM, or OCRAM.
 * The boot ROM applies the layout the fuses select, then loads the main stack
 * pointer from the vector table and jumps here. The vector table's stack top
 * is the end of the DTCM in the board's layout (port.cmake writes the linker
 * MEMORY regions from the same layout), which can lie past the end of the
 * fuse layout's DTCM. So nothing may touch the stack until the board's layout
 * is in place: this handler writes the bank assignment with registers alone,
 * and only then runs the startup sequence.
 *
 * The sequence after the layout is the SDK's startup_<device>.S
 * Reset_Handler as port.cmake builds it (__STARTUP_CLEAR_BSS, no RAM
 * functions, no initialised non-cacheable data): set VTOR and the main stack
 * pointer, SystemInit(), copy .data from flash, clear .bss, unmask
 * interrupts, and enter newlib's _start. The SDK defines its Reset_Handler
 * weak, so this strong definition takes the vector table's reset entry.
 *
 * Inputs (compile definitions from port.cmake):
 *   ROSETTE_IMXRT_FLEXRAM_BANK_CFG  IOMUXC_GPR17.FLEXRAM_BANK_CFG: two bits
 *                                   per bank, 01 OCRAM, 10 DTCM, 11 ITCM
 *
 * Verification:
 * - Correct means: after reset, GPR17 holds ROSETTE_IMXRT_FLEXRAM_BANK_CFG,
 *   GPR16.FLEXRAM_BANK_CFG_SEL is 1, and every octet of the board's DTCM and
 *   OCRAM regions holds what is written to it, with no aliasing between
 *   banks; images then run as on the fuse layout.
 * - Covered by: tools/imxrt_flexram_check.py on the MIMXRT1020-EVK (register
 *   readback and an address-tagged pattern over both regions), then
 *   rosette_platform_contract and the udp_echo probe.
 *
 * References:
 * - i.MX RT1020 Processor Reference Manual, FlexRAM chapter (RAM bank
 *   allocation) and IOMUXC_GPR GPR16 (FLEXRAM_BANK_CFG_SEL) and GPR17
 *   (FLEXRAM_BANK_CFG).
 * - MCUXpresso SDK MIMXRT1021 fsl_flexram_allocate.c FLEXRAM_AllocateRam():
 *   the same two register writes, in the same order.
 * - ARM Cortex-M7 Devices Generic User Guide §2.1.3 (main stack pointer),
 *   §2.3.2 (vector table); ARM v7-M Architecture Reference Manual §A3.8.3
 *   (DSB, ISB).
 */

    .syntax unified
    .arch   armv7e-m
    .thumb

/* IOMUXC_GPR on the RT1021 (MIMXRT1021.h IOMUXC_GPR_BASE, GPR16, GPR17); */
/* chip_imxrt.cpp checks these against the device header. */
    .equ    IOMUXC_GPR16, 0x400AC040
    .equ    IOMUXC_GPR17, 0x400AC044
    .equ    GPR16_FLEXRAM_BANK_CFG_SEL, 0x4
    .equ    SCB_VTOR, 0xE000ED08

    .section .text.Reset_Handler, "ax", %progbits
    .align  2
/* The second name is the one port.cmake passes to -u: the SDK's weak */
/* definition already satisfies Reset_Handler, so naming it would not pull */
/* this object out of the archive. */
    .globl  Reset_Handler
    .globl  rosette_imxrt_flexram_reset_handler
    .type   Reset_Handler, %function
    .type   rosette_imxrt_flexram_reset_handler, %function
    .thumb_func
Reset_Handler:
    .thumb_func
rosette_imxrt_flexram_reset_handler:
    cpsid   i

/* The board's layout, before anything uses the stack. */
    ldr     r0, =IOMUXC_GPR17
    ldr     r1, =ROSETTE_IMXRT_FLEXRAM_BANK_CFG
    str     r1, [r0]
    ldr     r0, =IOMUXC_GPR16
    ldr     r1, [r0]
    orr     r1, r1, #GPR16_FLEXRAM_BANK_CFG_SEL
    str     r1, [r0]
    dsb
    isb

/* The vector table, and the main stack at the top of the board's DTCM. */
    ldr     r0, =SCB_VTOR
    ldr     r1, =__isr_vector
    str     r1, [r0]
    ldr     r2, [r1]
    msr     msp, r2

    ldr     r0, =SystemInit
    blx     r0

/* .data from its load image in flash. */
    ldr     r1, =__etext
    ldr     r2, =__data_start__
    ldr     r3, =__data_end__
.Lcopy_data:
    cmp     r2, r3
    ittt    lt
    ldrlt   r0, [r1], #4
    strlt   r0, [r2], #4
    blt     .Lcopy_data

/* .bss to zero. */
    ldr     r1, =__bss_start__
    ldr     r2, =__bss_end__
    movs    r0, #0
.Lclear_bss:
    cmp     r1, r2
    itt     lt
    strlt   r0, [r1], #4
    blt     .Lclear_bss

    cpsie   i
    ldr     r0, =_start
    blx     r0

    .pool
    .size   Reset_Handler, . - Reset_Handler
