# SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
# SPDX-License-Identifier: Apache-2.0

# ports/os/freertos/rp2_kernel_ram_placement.cmake — runs the FreeRTOS
# kernel's interrupt path from RAM on the RP2040 kernel port (GCC_RP2040).
#
# The RP2040 executes in place from QSPI flash through a 16 KB XIP cache. An
# interrupt that misses the cache stalls for the QSPI fetch, and on a dual-core
# image the two cores miss on each other's behalf. Rosette's own interrupt
# handlers are ROSETTE_RAM_FUNCTION, but the kernel calls they make (give a
# semaphore, switch context, tick) and the kernel's own interrupts (PendSV,
# SysTick, the cross-core FIFO) are FreeRTOS sources Rosette does not edit.
# This file places those functions in RAM from the linker script instead.
#
# The kernel library is compiled with -ffunction-sections, so each function is
# its own input section, .text.<name>. GNU ld assigns an input section to the
# first rule that matches it in script order and has no exclusion by section
# name, so a `.data` rule naming those sections never sees them while the
# SDK's `.text` rule `*(.text*)` comes first. rosette_os_add_image_outputs()
# therefore rewrites the target's linker script (the SDK default, or the
# script the application already set with pico_set_linker_script()):
#
# @verbatim
#   FLASH                                RAM
#   +---------------------------+        +----------------------------+
#   | .text  (excludes          |        | .data                      |
#   |   libfreertos_kernel.a)   |        |   .time_critical*          |
#   +---------------------------+  copy  |   kernel interrupt path  <-+-- named sections
#   | .data load image          | -----> |   other .text (excludes    |
#   +---------------------------+  crt0  |     libfreertos_kernel.a)  |
#   | .freertos_kernel_text     |        |   .data                    |
#   |   rest of the kernel  <---+-- every other kernel .text*         |
#   +---------------------------+        +----------------------------+
# @endverbatim
#
# .freertos_kernel_text follows the .data and .tdata load images, so the one
# contiguous copy crt0 makes (__etext to __data_start__.. __data_end__) is
# unchanged. libfreertos_kernel.a also carries the SDK objects the kernel port
# compiles in; their .time_critical sections still match .data's own rule, and
# their other code follows the kernel into .freertos_kernel_text.
#
# The functions placed in RAM are every kernel function an interrupt reaches
# on this port, read from the disassembly of a MinSizeRel image. At -Os the
# compiler keeps a `static inline` helper with several callers out of line (each
# translation unit its own copy, all matched by name), so the port's lock helper
# is listed; a function it does inline has no section and needs no entry:
#
# | Source           | Functions                                          | Reached from                |
# |------------------|----------------------------------------------------|-----------------------------|
# | portable port.c  | xPortPendSVHandler, xPortSysTickHandler,           | the exceptions and the      |
# |                  | prvFIFOInterruptHandler, vYieldCore,               | cross-core yield            |
# |                  | ulSetInterruptMaskFromISR,                         |                             |
# |                  | vClearInterruptMaskFromISR                         |                             |
# | portmacro.h      | vPortRecursiveLock                                 | every ISR critical section  |
# | tasks.c          | vTaskSwitchContext, xTaskGetCurrentTaskHandle      | PendSV                      |
# | tasks.c          | xTaskIncrementTick, prvAddCurrentTaskToDelayedList,| SysTick                     |
# |                  | prvResetNextTaskUnblockTime                        |                             |
# | tasks.c          | vTaskEnterCriticalFromISR, vTaskExitCriticalFromISR| SysTick, xQueueGiveFromISR  |
# | tasks.c          | xTaskRemoveFromEventList, prvYieldForTask,         | xQueueGiveFromISR           |
# |                  | uxTaskGetNumberOfTasks                             |                             |
# | queue.c          | xQueueGiveFromISR                                  | Event::signal_from_isr()    |
# | list.c           | vListInsertEnd, vListInsert, uxListRemove          | all of the above            |
#
# Two calls on these paths stay in flash: vApplicationStackOverflowHook, which
# runs only on a detected overflow, and the SDK's __wrap_memset that
# xTaskIncrementTick calls to clear its per-core yield flags, which is outside
# the kernel library and forwards to the boot ROM.
#
# Cost: about 2.7 KB of RAM in .data; flash shrinks, since per-function sections
# let --gc-sections drop the kernel functions an image never calls. Only the
# default (execute-in-place) binary type is rewritten; copy_to_ram and no_flash
# images already run everything from RAM.

# One input section per function, which the placement rules name.
target_compile_options(freertos_kernel PRIVATE -ffunction-sections)
target_compile_options(freertos_kernel_port PRIVATE -ffunction-sections)

# Replaces `anchor` in the variable named `script_var`, or stops the configure:
# a script that no longer has the anchor would otherwise link the kernel
# entirely from flash without a word.
function(_rosette_kernel_ram_replace script_var anchor replacement source)
  string(FIND "${${script_var}}" "${anchor}" _at)
  if(_at EQUAL -1)
    message(FATAL_ERROR "rp2_kernel_ram_placement: ${source} has no line\n  ${anchor}\n"
      "so the FreeRTOS interrupt path cannot be placed in RAM.")
  endif()

  string(REPLACE "${anchor}" "${replacement}" _out "${${script_var}}")
  set(${script_var} "${_out}" PARENT_SCOPE)
endfunction()

# Links <target> with its linker script rewritten as drawn above. Called by
# rosette_add_image_outputs(), after the application has chosen its script.
function(rosette_os_add_image_outputs target)
  get_target_property(_type ${target} PICO_TARGET_BINARY_TYPE)
  if(NOT _type)
    set(_type "${PICO_DEFAULT_BINARY_TYPE}")
  endif()

  if(_type AND NOT _type STREQUAL "default")
    return()
  endif()

  get_target_property(_source ${target} PICO_TARGET_LINKER_SCRIPT)
  if(NOT _source)
    set(_source "${PICO_SDK_PATH}/src/rp2_common/pico_crt0/rp2040/memmap_default.ld")
  endif()

  file(READ "${_source}" _script)

  # The table above; set here because a function sees its caller's variables.
  set(_functions
    xPortPendSVHandler xPortSysTickHandler prvFIFOInterruptHandler vYieldCore
    ulSetInterruptMaskFromISR vClearInterruptMaskFromISR vPortRecursiveLock
    vTaskSwitchContext xTaskGetCurrentTaskHandle
    xTaskIncrementTick prvAddCurrentTaskToDelayedList prvResetNextTaskUnblockTime
    vTaskEnterCriticalFromISR vTaskExitCriticalFromISR
    xTaskRemoveFromEventList prvYieldForTask uxTaskGetNumberOfTasks
    xQueueGiveFromISR
    vListInsertEnd vListInsert uxListRemove)
  list(TRANSFORM _functions PREPEND ".text." OUTPUT_VARIABLE _sections)
  list(JOIN _sections " " _sections)
  set(_kernel "*libfreertos_kernel.a:")

  _rosette_kernel_ram_replace(_script
    "*(EXCLUDE_FILE(*libgcc.a: *libc.a:*lib_a-mem*.o *libm.a:) .text*)"
    "*(EXCLUDE_FILE(*libgcc.a: *libc.a:*lib_a-mem*.o *libm.a: ${_kernel}) .text*)"
    "${_source}")
  _rosette_kernel_ram_replace(_script
    "        *(.time_critical*)\n"
    "        *(.time_critical*)\n        ${_kernel}(${_sections})\n"
    "${_source}")
  _rosette_kernel_ram_replace(_script
    "        *(.text*)\n"
    "        *(EXCLUDE_FILE(${_kernel}) .text*)\n"
    "${_source}")
  _rosette_kernel_ram_replace(_script
    "    __etext = LOADADDR(.data);\n"
    "    __etext = LOADADDR(.data);\n\n    .freertos_kernel_text : {\n        ${_kernel}(.text*)\n    } > FLASH\n"
    "${_source}")

  set(_out "${CMAKE_CURRENT_BINARY_DIR}/${target}_freertos_ram.ld")
  file(WRITE "${_out}"
    "/* Generated by ports/os/freertos/rp2_kernel_ram_placement.cmake from ${_source}. */\n"
    "${_script}")
  pico_set_linker_script(${target} "${_out}")
endfunction()
