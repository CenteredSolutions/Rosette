// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

#pragma once

/**
 * @file rosette/platform/memory_report.hpp
 * @brief The platform's memory reports as text: each reservation's peak use against its size.
 *
 * A firmware prints this on its console or status port while it runs a
 * representative workload, then sizes each reservation (main stack, C heap,
 * RTOS heap, task stacks) from the peaks, with margin, instead of guessing
 * (docs/mcu_memory_model.md, "Measuring"). The numbers come from
 * main_stack_use(), c_heap_use(), rtos_heap_use(), and visit_task_stacks() in
 * the platform contract; this file only formats them.
 */

#include <cstddef>

namespace rosette::platform {

/**
 * @brief Writes the platform's memory use as text lines of `name=peak/reserved` octets.
 *
 * Two lines at most: `mem` with the main stack, C heap, and RTOS heap, and
 * `tasks` with each task's stack. A region the platform does not track is
 * left out, and so is a line with nothing in it, so a host writes nothing.
 * Task names with spaces (FreeRTOS's "Tmr Svc") keep them; each entry ends at
 * the next ` name=`.
 *
 * Reads the task list into records on the caller's stack (visit_task_stacks()):
 * call it from a task with about 1 KiB of stack to spare.
 *
 * @param[out] out Text buffer; always terminated when @p cap > 0.
 * @param[in]  cap Its size in octets.
 * @return Octets written, excluding the terminator; at most cap - 1, with the
 *   text cut short when the buffer is too small.
 *
 * Verification:
 * - Correct means: each figure is the one the contract function returned at
 *   the time of the call, and the text never overruns @p cap.
 * - Covered by: tests/platform/platform_contract_test.cpp (MemoryReports);
 *   examples/udp_echo on the MIMXRT1020-EVK prints it every five seconds.
 *
 * Example:
 * @code{.cpp}
 * char text[384];
 * if (rosette::platform::format_memory_report(text, sizeof(text)) > 0) {
 *     std::fputs(text, stdout);  // mem main-stack=1840/16384 c-heap=1232/16388 ...
 * }
 * @endcode
 * Full program: examples/udp_echo/main.cpp.
 */
std::size_t format_memory_report(char* out, std::size_t cap) noexcept;

}  // namespace rosette::platform
