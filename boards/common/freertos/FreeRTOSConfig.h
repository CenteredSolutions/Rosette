/**
 * @file boards/common/freertos/FreeRTOSConfig.h
 * @brief FreeRTOS kernel configuration for the RTPS stack on RP2040/RP2350.
 *
 * This file is included by FreeRTOS-Kernel headers.  The values below are
 * tuned for the RTPS workload on a single-core FreeRTOS build.
 *
 * Key choices and rationale:
 *   - configTICK_RATE_HZ = 1000: 1 ms tick granularity matches RTPS timing
 *     constants (nackResponseDelay, heartbeatPeriod) which are expressed in ms.
 *   - configTOTAL_HEAP_SIZE: FreeRTOS+TCP allocates its own buffer pool from
 *     the heap.  Allow enough for ipconfigNUM_NETWORK_BUFFER_DESCRIPTORS
 *     buffers (typically 1536 bytes each) plus task stacks.
 *   - configUSE_MUTEXES = 1: required for StatefulWriter / StatefulReader locks.
 *   - configUSE_TASK_NOTIFICATIONS = 1: required for write_reliable() blocking.
 */

#pragma once

#include "board_config.h"   /* BOARD_IS_RP2350 for port selection */

// ── Processor / port ──────────────────────────────────────────────────────────
#if BOARD_IS_RP2350
#  define configMINIMAL_STACK_SIZE   ((uint16_t)256)
#else
#  define configMINIMAL_STACK_SIZE   ((uint16_t)128)
#endif

// ── Tick and scheduling ───────────────────────────────────────────────────────
#define configUSE_PREEMPTION                    1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION 0
#define configUSE_TICKLESS_IDLE                 0
#define configCPU_CLOCK_HZ                      (SystemCoreClock)
#define configTICK_RATE_HZ                      ((TickType_t)1000)
#define configMAX_PRIORITIES                    8
#define configIDLE_SHOULD_YIELD                 1
#define configUSE_TIME_SLICING                  1

// ── Heap ──────────────────────────────────────────────────────────────────────
// heap_4 provides first-fit with block merging — best for FreeRTOS+TCP.
#define configHEAP_IMPLEMENTATION               4
// 64 KB: enough for FreeRTOS+TCP buffers + task stacks.
// Increase if ipconfigNUM_NETWORK_BUFFER_DESCRIPTORS > 8.
#define configTOTAL_HEAP_SIZE                   ((size_t)(64 * 1024))

// ── Mutexes and synchronisation ───────────────────────────────────────────────
#define configUSE_MUTEXES                       1
#define configUSE_RECURSIVE_MUTEXES             0
#define configUSE_COUNTING_SEMAPHORES           1
#define configUSE_TASK_NOTIFICATIONS            1   /* Required by write_reliable() */
#define configTASK_NOTIFICATION_ARRAY_ENTRIES   1

// ── Queues ────────────────────────────────────────────────────────────────────
#define configUSE_QUEUE_SETS                    0
#define configQUEUE_REGISTRY_SIZE               8

// ── Timers ────────────────────────────────────────────────────────────────────
#define configUSE_TIMERS                        1
#define configTIMER_TASK_PRIORITY               (configMAX_PRIORITIES - 1)
#define configTIMER_QUEUE_LENGTH                8
#define configTIMER_TASK_STACK_DEPTH            256

// ── Debug / trace ─────────────────────────────────────────────────────────────
#define configUSE_TRACE_FACILITY                0
#define configUSE_STATS_FORMATTING_FUNCTIONS    0
#define configGENERATE_RUN_TIME_STATS           0
#define configCHECK_FOR_STACK_OVERFLOW         2    /* Method 2: pattern check */
#define configUSE_MALLOC_FAILED_HOOK            1

// ── Assert ────────────────────────────────────────────────────────────────────
// Halt on assertion failure — safer than silently continuing on embedded.
#define configASSERT(x)  do { if (!(x)) { portDISABLE_INTERRUPTS(); for(;;); } } while(0)

// ── Cortex-M interrupt priority ───────────────────────────────────────────────
// FreeRTOS requires NVIC priorities >= configMAX_SYSCALL_INTERRUPT_PRIORITY
// for any ISR that calls a FreeRTOS API.
#define configPRIO_BITS                         4    /* RP2350/RP2040 NVIC */
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY   15
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 5
#define configKERNEL_INTERRUPT_PRIORITY  \
    (configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))
#define configMAX_SYSCALL_INTERRUPT_PRIORITY  \
    (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))

// ── Required by FreeRTOS+TCP ──────────────────────────────────────────────────
#define configSUPPORT_DYNAMIC_ALLOCATION        1
#define configSUPPORT_STATIC_ALLOCATION         0
#define configUSE_16_BIT_TICKS                  0

// ── Hook functions ────────────────────────────────────────────────────────────
#define configUSE_IDLE_HOOK                     0
#define configUSE_TICK_HOOK                     0
#define configUSE_DAEMON_TASK_STARTUP_HOOK      0

// ── API inclusion ────────────────────────────────────────────────────────────
#define INCLUDE_vTaskDelay                      1
#define INCLUDE_xTaskGetSchedulerState          1
#define INCLUDE_xTaskGetCurrentTaskHandle       1
#define INCLUDE_uxTaskPriorityGet               1
#define INCLUDE_vTaskSuspend                    1

// ── Map FreeRTOS port interrupt vectors ───────────────────────────────────────
#define vPortSVCHandler     isr_svcall
#define xPortPendSVHandler  isr_pendsv
#define xPortSysTickHandler isr_systick
