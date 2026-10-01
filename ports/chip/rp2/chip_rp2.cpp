/**
 * @file chip_rp2.cpp
 * @brief RP2040 / RP2350 chip layer of the platform contract.
 *
 * The monotonic clock is the 64-bit microsecond hardware timer, which runs
 * from reset and is the same whether or not an RTOS is present, so FreeRTOS
 * and bare-metal builds share one time base. The unique ID is the flash
 * chip's 64-bit ID (pico_unique_id). The chip has no wall-clock source; time
 * arrives later through NTP or an agent (WallClock).
 *
 * Falling-edge GPIO interrupts use the SDK's raw per-pin handlers, so the
 * application keeps the single shared GPIO callback
 * (gpio_set_irq_enabled_with_callback()) for its own pins.
 *
 * References: Pico SDK hardware_gpio gpio_add_raw_irq_handler().
 */

#include <cstddef>

#include "rosette/platform/gpio_interrupt.hpp"
#include "rosette/platform/platform.hpp"

#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/timer.h"
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

bool realtime_hint_us(int64_t& unix_us) noexcept {
    (void)unix_us;
    return false;
}

}  // namespace rosette::platform
