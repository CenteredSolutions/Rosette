// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file src/platform/memory_report.cpp
 * @brief format_memory_report(): the platform contract's memory reports as text.
 */

#include "rosette/platform/memory_report.hpp"

#include <cstdarg>
#include <cstdio>

#include "rosette/platform/platform.hpp"

namespace rosette::platform {
namespace {

/// A text buffer that appends with printf formatting and stops, terminated, when full.
struct TextOut {
    char*       out;
    std::size_t cap;
    std::size_t used = 0u;

    __attribute__((format(printf, 2, 3))) void append(const char* format, ...) noexcept {
        if (used + 1u >= cap) {
            return;
        }

        va_list args;
        va_start(args, format);
        const int n = std::vsnprintf(out + used, cap - used, format, args);
        va_end(args);
        if (n > 0) {
            used += static_cast<std::size_t>(n) < cap - used ? static_cast<std::size_t>(n)
                                                             : cap - used - 1u;
        }
    }

    /// Appends ` name=peak/reserved` when the platform tracks the region.
    void region(const char* name, const RegionUse& use) noexcept {
        if (use.reserved > 0u) {
            append(" %s=%lu/%lu", name, static_cast<unsigned long>(use.peak),
                   static_cast<unsigned long>(use.reserved));
        }
    }
};

}  // namespace

std::size_t format_memory_report(char* out, std::size_t cap) noexcept {
    if (out == nullptr || cap == 0u) {
        return 0u;
    }

    out[0] = '\0';
    TextOut text{out, cap};

    const RegionUse stack = main_stack_use();
    const RegionUse c_heap = c_heap_use();
    const RegionUse rtos_heap = rtos_heap_use();
    if (stack.reserved > 0u || c_heap.reserved > 0u || rtos_heap.reserved > 0u) {
        text.append("mem");
        text.region("main-stack", stack);
        text.region("c-heap", c_heap);
        text.region("rtos-heap", rtos_heap);
        text.append("\n");
    }

    const std::size_t line_start = text.used;
    text.append("tasks");
    const std::size_t tasks = visit_task_stacks(
        [](void* context, const char* name, const RegionUse& use) {
            static_cast<TextOut*>(context)->region(name, use);
        },
        &text);
    if (tasks == 0u) {
        text.used = line_start;
        out[line_start] = '\0';
    } else {
        text.append("\n");
    }

    return text.used;
}

}  // namespace rosette::platform
