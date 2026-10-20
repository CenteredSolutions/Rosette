// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file platform_contract_test.cpp
 * @brief Conformance test for a platform port (platform.hpp).
 *
 * Built once per port: on the host with ROSETTE_OS_POSIX / ROSETTE_CHIP_HOST,
 * and for boards with their OS and chip. It uses no test framework so the same
 * binary can run on a microcontroller and report over stdio. Every contract
 * item is exercised; a port missing an item fails to link, and one that
 * misbehaves fails a check here.
 *
 * Exit status is 0 only when every check passes.
 */

#include "rosette/platform/memory_report.hpp"
#include "rosette/platform/platform.hpp"
#include "rosette/platform/wall_clock.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstring>

#if defined(ROSETTE_OS_POSIX)
#  include <thread>
#endif
#if defined(ROSETTE_CHIP_RP2)
#  include "pico/multicore.h"
#  include "pico/stdlib.h"
#endif
#if defined(ROSETTE_CHIP_IMXRT)
#  include "board.h"
#endif
#if defined(ROSETTE_OS_FREERTOS)
#  include "FreeRTOS.h"
#  include "task.h"
#endif

namespace pf = rosette::platform;

namespace {

int g_failures = 0;

void check(bool ok, const char* name, const char* what) {
    std::printf("[%s] %s: %s\n", ok ? "PASS" : "FAIL", name, what);
    if (!ok) ++g_failures;
}

/// Clock: monotonic, correct rate, and ms derived from the same base.
void test_monotonic_advances() {
    const uint64_t t0 = pf::monotonic_us();
    pf::sleep_ms(20);
    const uint64_t t1 = pf::monotonic_us();
    check(t1 >= t0 + 19000u, "MonotonicAdvances", "sleep_ms(20) advances monotonic_us by >= 19 ms");
    check(t1 < t0 + 1000000u, "MonotonicAdvances", "sleep_ms(20) returns within 1 s");
    const uint32_t ms = pf::monotonic_ms();
    check(static_cast<uint32_t>(pf::monotonic_us() / 1000u) - ms <= 1u, "MonotonicAdvances",
          "monotonic_ms() is monotonic_us() / 1000");
}

/// Identity: stable across calls and not blank.
void test_unique_id_stable() {
    uint8_t a[8] = {}, b[8] = {};
    pf::device_unique_id(a);
    pf::device_unique_id(b);
    bool same = true, blank = true;
    for (int i = 0; i < 8; ++i) {
        same = same && a[i] == b[i];
        blank = blank && a[i] == 0u;
    }
    check(same, "UniqueIdStable", "repeated calls return the same identifier");
    check(!blank, "UniqueIdStable", "identifier is not all zeros");
}

/// Boot nonce: one value for the whole boot. That it changes at the next boot
/// is checked on hardware, by a peer rediscovering a device that reset.
void test_boot_nonce_stable() {
    const uint32_t first = pf::boot_nonce();
    check(pf::boot_nonce() == first, "BootNonceStable", "repeated calls return the same nonce");
}

/// Mutex: exclusive and non-recursive.
void test_mutex() {
    pf::Mutex m;
    m.lock();
    check(!m.try_lock(), "Mutex", "try_lock fails while held");
    m.unlock();
    check(m.try_lock(), "Mutex", "try_lock succeeds when free");
    m.unlock();
    {
        pf::LockGuard guard(m);
        check(!m.try_lock(), "Mutex", "LockGuard holds the mutex for its scope");
    }
    check(m.try_lock(), "Mutex", "LockGuard releases on scope exit");
    m.unlock();
}

/// Event: latched, auto-reset, timeout honored.
void test_event() {
    pf::Event e;
    e.signal();
    e.signal();
    check(e.wait_for(0), "Event", "a signal before the wait is not lost");
    check(!e.wait_for(0), "Event", "signals coalesce and the event auto-resets");

    const uint64_t t0 = pf::monotonic_us();
    check(!e.wait_for(30), "Event", "wait_for times out without a signal");
    const uint64_t waited = pf::monotonic_us() - t0;
    check(waited >= 29000u && waited < 1000000u, "Event", "timeout lasts about as long as requested");

    e.signal_from_isr();
    check(e.consume(), "Event", "signal_from_isr sets the event; consume clears it");
    check(!e.consume(), "Event", "consume reports nothing pending afterwards");
}

#if defined(ROSETTE_OS_POSIX)
/// Event: a signal from another thread wakes a blocked wait promptly.
void test_event_cross_thread() {
    pf::Event e;
    const uint64_t t0 = pf::monotonic_us();
    std::thread signaler([&e] {
        pf::sleep_ms(20);
        e.signal();
    });
    const bool woke = e.wait_for(pf::kWaitForever);
    const uint64_t waited = pf::monotonic_us() - t0;
    signaler.join();
    check(woke && waited < 500000u, "EventCrossThread", "another thread's signal ends a blocking wait");
}
#endif

#if defined(ROSETTE_OS_FREERTOS)
/// Event: a signal from another task wakes a blocked wait promptly.
void test_event_cross_task() {
    static pf::Event e;
    const uint64_t   t0 = pf::monotonic_us();
    TaskHandle_t     signaler = nullptr;
    const BaseType_t created = xTaskCreate(
        [](void*) {
            pf::sleep_ms(20);
            e.signal();
            vTaskDelete(nullptr);
        },
        "signaler", configMINIMAL_STACK_SIZE, nullptr, tskIDLE_PRIORITY + 1, &signaler);
    const bool     woke = (created == pdPASS) && e.wait_for(1000u);
    const uint64_t waited = pf::monotonic_us() - t0;
    check(woke && waited >= 19000u && waited < 500000u, "EventCrossTask",
          "another task's signal ends a blocking wait after its 20 ms sleep");
}
#endif

// The bare-metal OS layer's cross-core guarantees, on the dual-core RP2040 /
// RP2350; a single-core chip has no second core to test against.
#if defined(ROSETTE_OS_BAREMETAL) && defined(ROSETTE_CHIP_RP2)
pf::Event* g_core1_event = nullptr;

void core1_signal_after_delay() {
    pf::sleep_ms(20);
    g_core1_event->signal();
}

/// Event: a signal from the other core wakes a WFE-sleeping wait promptly.
void test_event_cross_core() {
    pf::Event e;
    g_core1_event = &e;
    const uint64_t t0 = pf::monotonic_us();
    multicore_launch_core1(core1_signal_after_delay);
    const bool woke = e.wait_for(2000);
    const uint64_t waited = pf::monotonic_us() - t0;
    multicore_reset_core1();
    check(woke && waited < 500000u, "EventCrossCore", "core 1's signal ends core 0's WFE wait");
}

pf::Mutex g_shared_lock;
volatile uint32_t g_shared_count = 0u;
volatile bool g_core1_done = false;
constexpr uint32_t kIncrementsPerCore = 100000u;

/// Increments the shared count under the mutex, as many times as core 0 does.
void core1_increment() {
    for (uint32_t i = 0; i < kIncrementsPerCore; ++i) {
        pf::LockGuard guard(g_shared_lock);
        g_shared_count = g_shared_count + 1u;
    }

    g_core1_done = true;
}

/// Mutex: both cores incrementing one count under it lose no increment.
void test_mutex_cross_core() {
    g_shared_count = 0u;
    g_core1_done = false;
    multicore_launch_core1(core1_increment);
    for (uint32_t i = 0; i < kIncrementsPerCore; ++i) {
        pf::LockGuard guard(g_shared_lock);
        g_shared_count = g_shared_count + 1u;
    }

    const uint64_t deadline = pf::monotonic_us() + 2000000u;
    while (!g_core1_done && pf::monotonic_us() < deadline) {
    }

    multicore_reset_core1();
    check(g_core1_done && g_shared_count == 2u * kIncrementsPerCore, "MutexCrossCore",
          "two cores incrementing under the mutex lose no increment");
}
#endif

/// WallClock: tracks the monotonic base plus the offset it was given.
void test_wall_clock() {
    int64_t hint = 0;
    const bool has_hint = pf::realtime_hint_us(hint);
    if (has_hint) {
        const int64_t diff = pf::WallClock::now_us() - hint;
        check(pf::WallClock::synchronized(), "WallClock", "a realtime hint synchronizes the clock");
        check(diff > -5000 && diff < 5000, "WallClock", "now_us() matches the platform's realtime");
    }
    // now_us() reads the monotonic clock internally, so bracket it with reads
    // on either side: the offset-free value must fall inside that window.
    const int64_t before = pf::WallClock::offset_us();
    pf::WallClock::set_offset_us(before + 1000000);
    const int64_t lo = static_cast<int64_t>(pf::monotonic_us());
    const int64_t now = pf::WallClock::now_us();
    const int64_t hi = static_cast<int64_t>(pf::monotonic_us());
    const int64_t base = now - before - 1000000;
    check(base >= lo && base <= hi, "WallClock", "set_offset_us moves now_us() by the offset");
    check(pf::WallClock::synchronized(), "WallClock", "setting an offset marks the clock synchronized");
    pf::WallClock::set_offset_us(before);
}

/// Checks one region report: a tracked region's peak is used and within its reservation.
void check_region(const char* what, const pf::RegionUse& use) {
    if (use.reserved == 0u) {
        std::printf("[SKIP] MemoryReports: %s is not tracked here\n", what);
        return;
    }

    char text[96];
    std::snprintf(text, sizeof(text), "%s peak %lu is within its %lu octets", what,
                  static_cast<unsigned long>(use.peak), static_cast<unsigned long>(use.reserved));
    check(use.peak <= use.reserved, "MemoryReports", text);
}

/// Memory reports: peaks within their reservations, never falling, and the text bounded.
void test_memory_reports() {
    const pf::RegionUse stack = pf::main_stack_use();
    check_region("main stack", stack);
    if (stack.reserved > 0u) {
        check(stack.peak > 0u, "MemoryReports", "the main stack has been used");
        check(pf::main_stack_use().peak >= stack.peak, "MemoryReports",
              "the main stack's peak never falls");
    }

    const pf::RegionUse before = pf::c_heap_use();
    check_region("C heap", before);
    if (before.reserved > 0u) {
        void* const         block = std::malloc(256u);
        const pf::RegionUse after = pf::c_heap_use();
        check(block != nullptr && after.peak >= before.peak && after.peak >= 256u, "MemoryReports",
              "a malloc() of 256 octets shows in the C heap's peak");
        std::free(block);
    }

    check_region("RTOS heap", pf::rtos_heap_use());

    struct Visit {
        std::size_t calls = 0u;
        bool        sane = true;
    } visit;
    const std::size_t tasks = pf::visit_task_stacks(
        [](void* context, const char* name, const pf::RegionUse& use) {
            auto& v = *static_cast<Visit*>(context);
            ++v.calls;
            v.sane = v.sane && name != nullptr && use.peak > 0u && use.peak <= use.reserved;
        },
        &visit);
    check(visit.calls == tasks && visit.sane, "MemoryReports",
          "each task is visited once, with 0 < peak <= its stack");
#if defined(ROSETTE_OS_FREERTOS) && (configUSE_TRACE_FACILITY == 1) \
    && (configRECORD_STACK_HIGH_ADDRESS == 1)
    check(tasks >= 2u, "MemoryReports", "the contract task and the idle task are reported");
#endif

    char       text[384];
    const auto n = pf::format_memory_report(text, sizeof(text));
    check(n < sizeof(text) && text[n] == '\0', "MemoryReports",
          "the report is terminated and fits");
    char       tiny[8];
    const auto cut = pf::format_memory_report(tiny, sizeof(tiny));
    check(cut < sizeof(tiny) && std::strlen(tiny) == cut, "MemoryReports",
          "a report cut short by a small buffer stays terminated");
    if (n > 0u) {
        std::printf("%s", text);
    }
}

int run_contract() {
    g_failures = 0;
    test_monotonic_advances();
    test_unique_id_stable();
    test_boot_nonce_stable();
    test_mutex();
    test_event();
#if defined(ROSETTE_OS_POSIX)
    test_event_cross_thread();
#endif
#if defined(ROSETTE_OS_FREERTOS)
    test_event_cross_task();
#endif
#if defined(ROSETTE_OS_BAREMETAL) && defined(ROSETTE_CHIP_RP2)
    test_event_cross_core();
    test_mutex_cross_core();
#endif
    test_wall_clock();
    test_memory_reports();
    std::printf("platform contract: %s (%d failure%s)\n", g_failures ? "FAIL" : "PASS",
                g_failures, g_failures == 1 ? "" : "s");
    return g_failures;
}

}  // namespace

#if defined(ROSETTE_CHIP_RP2)
/// On a board the result goes to USB serial: wait for a terminal, run once,
/// then repeat the run every few seconds so a late-attaching terminal sees it.
int main() {
    stdio_init_all();
    for (int i = 0; i < 100 && !stdio_usb_connected(); ++i) sleep_ms(100);
    for (;;) {
        run_contract();
        sleep_ms(5000);
    }
}
#elif defined(ROSETTE_OS_FREERTOS)
/// Under FreeRTOS the contract runs in a task, since sleep_ms() and Event
/// waits block in the scheduler; the run repeats every few seconds so a
/// terminal that opens the console late still sees a result.
int main() {
#  if defined(ROSETTE_CHIP_IMXRT)
    board_init();
#  endif
    xTaskCreate(
        [](void*) {
            for (;;) {
                run_contract();
                pf::sleep_ms(5000);
            }
        },
        "contract", 1024u, nullptr, tskIDLE_PRIORITY + 2, nullptr);
    vTaskStartScheduler();
    std::printf("platform contract: the scheduler did not start\n");
    return EXIT_FAILURE;
}
#else
int main() {
#  if defined(ROSETTE_CHIP_IMXRT)
    board_init();
#  endif
    return run_contract() ? EXIT_FAILURE : EXIT_SUCCESS;
}
#endif
