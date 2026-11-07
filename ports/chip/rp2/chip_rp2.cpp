// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file ports/chip/rp2/chip_rp2.cpp
 * @brief RP2040 / RP2350 chip layer of the platform contract.
 *
 * The monotonic clock is the 64-bit microsecond hardware timer, which runs
 * from reset and is the same whether or not an RTOS is present, so FreeRTOS
 * and bare-metal builds share one time base. The unique ID is the flash
 * chip's 64-bit ID (pico_unique_id); the boot nonce comes from pico_rand,
 * which seeds from the ring oscillator and the bus performance counters, so
 * it differs from one reset to the next. The chip has no wall-clock source; time
 * arrives later through NTP or an agent (WallClock).
 *
 * Falling-edge GPIO interrupts use the SDK's raw per-pin handlers, so the
 * application keeps the single shared GPIO callback
 * (gpio_set_irq_enabled_with_callback()) for its own pins.
 *
 * The bare-metal sync hooks (chip_sync.hpp) hold hardware spin lock
 * PICO_SPINLOCK_ID_OS1, which the SDK reserves for an OS layer; only the
 * bare-metal OS port calls them, so no RTOS shares the lock. Waits sleep in
 * WFE, and send_event() is SEV, which reaches both cores.
 *
 * References: Pico SDK hardware_gpio gpio_add_raw_irq_handler(),
 * hardware_sync spin_lock_blocking(), pico_time best_effort_wfe_or_timeout().
 */

#include <cstddef>

#include "rosette/platform/chip_sync.hpp"
#include "rosette/platform/gpio_interrupt.hpp"
#include "rosette/platform/platform.hpp"

#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/sync.h"
#include "hardware/timer.h"
#include "pico/rand.h"
#include "pico/time.h"
#include "pico/unique_id.h"

namespace rosette::platform {

namespace {

/// One attach_falling_edge() registration; a null handler marks the slot free.
struct FallingEdgeSlot {
    uint32_t pin = 0u;
    GpioInterruptHandler volatile handler = nullptr;
    void* volatile context = nullptr;
};

FallingEdgeSlot g_falling_edge[4];

/// The raw handler for slot `N`: the SDK passes a raw handler no argument.
template <std::size_t N>
void falling_edge_irq() {
    FallingEdgeSlot& slot = g_falling_edge[N];
    if ((gpio_get_irq_event_mask(slot.pin) & GPIO_IRQ_EDGE_FALL) == 0u) {
        return;
    }

    gpio_acknowledge_irq(slot.pin, GPIO_IRQ_EDGE_FALL);
    const GpioInterruptHandler handler = slot.handler;
    if (handler != nullptr) {
        handler(slot.context);
    }
}

constexpr irq_handler_t kFallingEdgeIrqs[] = {&falling_edge_irq<0>, &falling_edge_irq<1>,
                                              &falling_edge_irq<2>, &falling_edge_irq<3>};
static_assert(sizeof(kFallingEdgeIrqs) / sizeof(kFallingEdgeIrqs[0])
              == sizeof(g_falling_edge) / sizeof(g_falling_edge[0]));

}  // namespace

bool attach_falling_edge(uint32_t pin, GpioInterruptHandler handler, void* context) noexcept {
    if (handler == nullptr || pin >= NUM_BANK0_GPIOS) {
        return false;
    }

    for (FallingEdgeSlot& slot : g_falling_edge) {
        if (slot.handler != nullptr && slot.pin == pin) {
            gpio_set_irq_enabled(pin, GPIO_IRQ_EDGE_FALL, false);
            slot.context = context;
            slot.handler = handler;
            gpio_set_irq_enabled(pin, GPIO_IRQ_EDGE_FALL, true);
            return true;
        }
    }

    for (std::size_t i = 0; i < sizeof(g_falling_edge) / sizeof(g_falling_edge[0]); ++i) {
        FallingEdgeSlot& slot = g_falling_edge[i];
        if (slot.handler != nullptr) {
            continue;
        }

        slot.pin = pin;
        slot.context = context;
        slot.handler = handler;
        gpio_init(pin);
        gpio_set_dir(pin, GPIO_IN);
        gpio_pull_up(pin);
        gpio_add_raw_irq_handler(pin, kFallingEdgeIrqs[i]);
        gpio_set_irq_enabled(pin, GPIO_IRQ_EDGE_FALL, true);
        irq_set_enabled(IO_IRQ_BANK0, true);
        return true;
    }

    return false;
}

uint64_t monotonic_us() noexcept {
    return time_us_64();
}

void device_unique_id(uint8_t out[8]) noexcept {
    pico_unique_board_id_t id;
    pico_get_unique_board_id(&id);
    for (int i = 0; i < 8; ++i) out[i] = id.id[i];
}

uint32_t boot_nonce() noexcept {
    static const uint32_t nonce = get_rand_32();
    return nonce;
}

bool realtime_hint_us(int64_t& unix_us) noexcept {
    (void)unix_us;
    return false;
}

/// @brief Not tracked on the RP2040 / RP2350 yet: reports {0, 0}.
RegionUse main_stack_use() noexcept {
    return {};
}

/// @brief Not tracked on the RP2040 / RP2350 yet: reports {0, 0}.
RegionUse c_heap_use() noexcept {
    return {};
}

}  // namespace rosette::platform

namespace rosette::platform::chip {

uint32_t critical_enter() noexcept {
    return spin_lock_blocking(spin_lock_instance(PICO_SPINLOCK_ID_OS1));
}

void critical_exit(uint32_t state) noexcept {
    spin_unlock(spin_lock_instance(PICO_SPINLOCK_ID_OS1), state);
}

void wait_for_event_until(uint64_t deadline_us) noexcept {
    if (deadline_us == UINT64_MAX) {
        __wfe();
        return;
    }

    (void)best_effort_wfe_or_timeout(from_us_since_boot(deadline_us));
}

void send_event() noexcept {
    __sev();
}

}  // namespace rosette::platform::chip
