/**
 * @file ports/chip/imxrt/memory_use.cpp
 * @brief The i.MX RT's main-stack and C-heap reports: main_stack_use() and c_heap_use().
 *
 * The SDK's linker script reserves both regions at the end of the DTCM: the
 * newlib heap from __HeapBase to __HeapLimit (__heap_size__,
 * ROSETTE_IMXRT_HEAP_BYTES) and the main stack from __StackLimit to
 * __StackTop (__stack_size__, ROSETTE_IMXRT_STACK_BYTES). The main stack runs
 * main() until an RTOS starts and every interrupt after, so its peak is the
 * deepest of those.
 *
 * The stack's peak is found by painting: an early constructor fills the stack
 * below the current stack pointer with a pattern, and main_stack_use() scans
 * up from __StackLimit for the first word that no longer holds it. The heap's
 * peak is newlib's arena, the octets malloc() has taken from _sbrk(), which
 * only grows.
 *
 * References: GNU ld, symbol assignments in the SDK's
 * MIMXRT1021xxxxx_flexspi_nor.ld (.heap, .stack); newlib mallinfo(); ARM
 * Cortex-M7 Devices Generic User Guide §2.1.3 (the main stack pointer).
 */

#include <malloc.h>

#include <cstddef>
#include <cstdint>

#include "rosette/platform/platform.hpp"

extern "C" {
extern uint32_t __StackLimit[];
extern uint32_t __StackTop[];
extern uint8_t  __HeapBase[];
extern uint8_t  __HeapLimit[];
}

namespace rosette::platform {
namespace {

/// The pattern unused main-stack words hold; a word that differs has been used.
constexpr uint32_t kStackPaint = 0xA5A5A5A5u;

/// Words below the stack pointer left unpainted, so painting never writes under its own frame.
constexpr std::ptrdiff_t kPaintMarginWords = 16;

/**
 * @brief Fills the unused main stack with kStackPaint before main() runs.
 *
 * Runs from the init array with the other early constructors, on the main
 * stack, before the application or an RTOS has used it deeply; what the
 * startup code used before this is above the stack pointer and counts as
 * used.
 *
 * Verification:
 * - Correct means: every word from __StackLimit to kPaintMarginWords below
 *   the stack pointer holds kStackPaint when main() starts.
 * - Covered by: tests/platform/platform_contract_test.cpp (MemoryReports) on
 *   the MIMXRT1020-EVK: the reported peak is above zero and below the
 *   reservation.
 */
__attribute__((constructor(101))) void paint_main_stack() noexcept {
    uint32_t* sp = nullptr;
    __asm volatile("mov %0, sp" : "=r"(sp));

    volatile uint32_t* word = __StackLimit;
    while (word < sp - kPaintMarginWords) {
        *word++ = kStackPaint;
    }
}

}  // namespace

RegionUse main_stack_use() noexcept {
    const volatile uint32_t* word = __StackLimit;
    while (word < __StackTop && *word == kStackPaint) {
        ++word;
    }

    const uintptr_t top = reinterpret_cast<uintptr_t>(__StackTop);
    return {top - reinterpret_cast<uintptr_t>(__StackLimit),
            top - reinterpret_cast<uintptr_t>(word)};
}

RegionUse c_heap_use() noexcept {
    const struct mallinfo heap = mallinfo();
    return {static_cast<std::size_t>(__HeapLimit - __HeapBase),
            static_cast<std::size_t>(heap.arena)};
}

}  // namespace rosette::platform
