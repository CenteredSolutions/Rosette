/**
 * @file ports/os/freertos/rosette_freertos_hooks.c
 * @brief The kernel's failure hooks: a failed configASSERT, a task stack overflow, and an
 * exhausted heap.
 *
 * Each prints what failed on the console and halts with interrupts masked, so
 * the last line of the boot log names the fault and a debugger attached
 * afterwards finds the core stopped at the failure. A board's
 * FreeRTOSConfig.h routes configASSERT() to rosette_freertos_assert_failed();
 * the kernel calls the two vApplication hooks itself when
 * configCHECK_FOR_STACK_OVERFLOW and configUSE_MALLOC_FAILED_HOOK are set.
 *
 * References: FreeRTOS "Stack Usage and Stack Overflow Checking"
 * (vApplicationStackOverflowHook); FreeRTOS "Memory Management"
 * (vApplicationMallocFailedHook); FreeRTOS "configASSERT".
 */

#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"

/// Masks interrupts and stops; nothing after a kernel fault can be trusted to run.
static void rosette_freertos_halt(void) {
    taskDISABLE_INTERRUPTS();
    for (;;) {
    }
}

void rosette_freertos_assert_failed(const char* file, int line) {
    printf("[freertos] assert failed: %s:%d\n", file, line);
    rosette_freertos_halt();
}

void vApplicationStackOverflowHook(TaskHandle_t task, char* name) {
    (void)task;
    printf("[freertos] stack overflow in task '%s'\n", name != NULL ? name : "?");
    rosette_freertos_halt();
}

void vApplicationMallocFailedHook(void) {
    printf("[freertos] heap exhausted (configTOTAL_HEAP_SIZE %u)\n",
           (unsigned)configTOTAL_HEAP_SIZE);
    rosette_freertos_halt();
}
