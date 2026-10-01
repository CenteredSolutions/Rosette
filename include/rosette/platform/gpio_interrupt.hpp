/**
 * @file platform/gpio_interrupt.hpp
 * @brief Chip contract: call a handler from interrupt context on a GPIO's falling edge.
 *
 * Portable drivers that wake on an interrupt line, such as a WIZnet chip's
 * active-low INT, attach to it here instead of calling a vendor SDK, so the
 * driver is the same on every chip. The chip port implements the one hook;
 * a chip with no GPIO interrupts (the host, the stub port) returns false and
 * the driver falls back to polling.
 *
 * The handler runs in interrupt context: it may only do what an ISR may, such
 * as platform::Event::signal_from_isr(). It must not touch a bus.
 */

#pragma once

#include <cstdint>

namespace rosette::platform {

/// Interrupt-context callback; `context` is the pointer given to attach_falling_edge().
using GpioInterruptHandler = void (*)(void* context);

/**
 * @brief Calls `handler(context)` from interrupt context on each falling edge of `pin`.
 *
 * Configures `pin` as an input with its pull-up enabled, which suits an
 * active-low, open-drain interrupt output. Attaching the same pin again
 * replaces its handler and context.
 *
 * @param[in] pin     Chip GPIO number.
 * @param[in] handler Called once per falling edge; must not be null.
 * @param[in] context Passed to `handler` unchanged; must outlive the attachment.
 * @return `false` when the chip has no GPIO interrupts, `pin` is not valid,
 *   or the chip port has no room for another handler.
 */
bool attach_falling_edge(uint32_t pin, GpioInterruptHandler handler, void* context) noexcept;

}  // namespace rosette::platform
