/**
 * @file boards/mimxrt1020-evk/FreeRTOSConfig.h
 * @brief FreeRTOS kernel configuration for the MIMXRT1020-EVK (i.MX RT1021, Cortex-M7).
 *
 * The kernel runs on its GCC_ARM_CM4F port (board.cmake). The tick is SysTick
 * at the core clock, 1 kHz, so RTPS timing constants expressed in
 * milliseconds map to whole ticks. Rosette's own time base stays the chip
 * port's PIT lifetime timer (monotonic_us()); the tick only schedules tasks.
 *
 * Interrupt priorities: the RT1021 implements four NVIC priority bits. An
 * interrupt that calls a FromISR API (the ENET receive interrupt, a GPIO
 * edge that signals a platform::Event) runs at a priority numerically at
 * least configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY.
 *
 * References: FreeRTOS "Customisation" (FreeRTOSConfig.h); FreeRTOS "Running
 * the RTOS on an ARM Cortex-M core" (configMAX_SYSCALL_INTERRUPT_PRIORITY);
 * i.MX RT1020 Reference Manual §4 (interrupts, __NVIC_PRIO_BITS = 4).
 */

#pragma once

/* FreeRTOS+TCP's FreeRTOSIPConfigDefaults.h checks this name to confirm the
 * kernel configuration was included first. */
#define FREERTOS_CONFIG_H

#if defined(__ASSEMBLER__) == 0
#  include <stdint.h>
extern uint32_t SystemCoreClock;

/// Prints the failed assertion's location and halts; ports/os/freertos/rosette_freertos_hooks.c.
void rosette_freertos_assert_failed(const char* file, int line);
#endif

// ── Tick and scheduling ───────────────────────────────────────────────────────
#define configUSE_PREEMPTION 1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION 1
#define configUSE_TICKLESS_IDLE 0
#define configCPU_CLOCK_HZ (SystemCoreClock)
#define configTICK_RATE_HZ ((TickType_t)1000)
#define configMAX_PRIORITIES 8
#define configMINIMAL_STACK_SIZE ((uint16_t)256)
#define configIDLE_SHOULD_YIELD 1
#define configUSE_TIME_SLICING 1

// ── Heap ──────────────────────────────────────────────────────────────────────
// heap_4: first fit with block merging. Task stacks and kernel objects only;
// FreeRTOS+TCP's network buffers are a static array (BufferAllocation_1).
#define configTOTAL_HEAP_SIZE ((size_t)(24 * 1024))

// ── Synchronisation ───────────────────────────────────────────────────────────
#define configUSE_MUTEXES 1
#define configUSE_RECURSIVE_MUTEXES 0
#define configUSE_COUNTING_SEMAPHORES 1
#define configUSE_TASK_NOTIFICATIONS 1
#define configTASK_NOTIFICATION_ARRAY_ENTRIES 1
#define configUSE_QUEUE_SETS 0
#define configQUEUE_REGISTRY_SIZE 0

// ── Timers ────────────────────────────────────────────────────────────────────
#define configUSE_TIMERS 1
#define configTIMER_TASK_PRIORITY (configMAX_PRIORITIES - 1)
#define configTIMER_QUEUE_LENGTH 8
#define configTIMER_TASK_STACK_DEPTH 256

// ── Debug ─────────────────────────────────────────────────────────────────────
#define configUSE_TRACE_FACILITY 0
#define configUSE_STATS_FORMATTING_FUNCTIONS 0
#define configGENERATE_RUN_TIME_STATS 0
#define configCHECK_FOR_STACK_OVERFLOW 2
#define configUSE_MALLOC_FAILED_HOOK 1
#define configASSERT(x)                                         \
    do {                                                        \
        if (!(x)) {                                             \
            rosette_freertos_assert_failed(__FILE__, __LINE__); \
        }                                                       \
    } while (0)

// ── Cortex-M interrupt priority ───────────────────────────────────────────────
#define configPRIO_BITS 4
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY 15
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 5
#define configKERNEL_INTERRUPT_PRIORITY \
    (configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))
#define configMAX_SYSCALL_INTERRUPT_PRIORITY \
    (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))

// ── Allocation ────────────────────────────────────────────────────────────────
#define configSUPPORT_DYNAMIC_ALLOCATION 1
#define configSUPPORT_STATIC_ALLOCATION 0
#define configTICK_TYPE_WIDTH_IN_BITS TICK_TYPE_WIDTH_32_BITS

// ── Hooks ─────────────────────────────────────────────────────────────────────
#define configUSE_IDLE_HOOK 0
#define configUSE_TICK_HOOK 0
#define configUSE_DAEMON_TASK_STARTUP_HOOK 0

// ── API inclusion ─────────────────────────────────────────────────────────────
#define INCLUDE_vTaskDelay 1
#define INCLUDE_vTaskDelete 1
#define INCLUDE_xTaskGetSchedulerState 1
#define INCLUDE_xTaskGetCurrentTaskHandle 1
#define INCLUDE_uxTaskPriorityGet 1
#define INCLUDE_vTaskSuspend 1

// ── Vectors: the SDK startup's names for the port's handlers ─────────────────
#define vPortSVCHandler SVC_Handler
#define xPortPendSVHandler PendSV_Handler
#define xPortSysTickHandler SysTick_Handler
