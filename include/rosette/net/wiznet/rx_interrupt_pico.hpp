/**
 * @file net/wiznet/rx_interrupt_pico.hpp
 * @brief Routes the WIZnet INT pin to the socket provider's receive event.
 *
 * The chip drives INT low while any enabled socket interrupt is pending;
 * every UDP socket enables its RECV interrupt when opened. A falling edge on
 * INT therefore means "a datagram arrived" and ends the participant driver's
 * sleep (WFE on bare metal, a blocked semaphore under FreeRTOS) through
 * Event::signal_from_isr() on the event the driver registered with the
 * provider (SocketProvider::set_receive_event()). The handler looks the event
 * up on each interrupt, so the interrupt can be attached during board
 * bring-up, before the driver exists: an rmw context creates its driver inside
 * rcl_init(). The handler touches no SPI: the thread that wakes
 * reads the chip. WiznetSocketProvider::wait_readable() clears each socket's
 * RECV flag before it checks for data, so INT deasserts and the next arrival
 * produces a fresh edge; no datagram can arrive unnoticed between the check
 * and the sleep.
 *
 * A raw per-pin handler is used so the application keeps the Pico SDK's
 * single shared GPIO callback for its own pins.
 *
 * References: W5100S datasheet §INT pin, Sn_IR, Sn_IMR, SIMR; Pico SDK
 * hardware_gpio gpio_add_raw_irq_handler().
 */

#pragma once

#include "hardware/gpio.h"
#include "hardware/irq.h"

#include "rosette/net/socket.hpp"
#include "rosette/platform/platform.hpp"

namespace rosette::net::wiznet {

namespace detail {
inline uint g_rx_interrupt_pin = 0u;
inline const rosette::net::SocketProvider* g_rx_interrupt_provider = nullptr;

inline void rx_interrupt_handler() {
    if (gpio_get_irq_event_mask(g_rx_interrupt_pin) & GPIO_IRQ_EDGE_FALL) {
        gpio_acknowledge_irq(g_rx_interrupt_pin, GPIO_IRQ_EDGE_FALL);
        rosette::platform::Event* event =
            g_rx_interrupt_provider != nullptr ? g_rx_interrupt_provider->receive_event() : nullptr;
        if (event != nullptr) event->signal_from_isr();
    }
}
}  // namespace detail

/**
 * @brief Signals `provider`'s receive event on each falling edge of INT.
 *
 * WiznetSocketProvider::wait_readable() sleeps without polling whenever the
 * waiting driver's event is the registered receive event, so every board with
 * INT wired must attach it before the first driver waits.
 *
 * @param[in] pin      GPIO wired to the chip's active-low INT output.
 * @param[in] provider The chip's socket provider; must outlive the program.
 *   An interrupt with no event registered is acknowledged and ignored.
 *
 * Verification:
 * - Correct means: a datagram sent to the board ends a kWaitForever
 *   wait_readable() within interrupt latency, with no polling.
 * - Covered by: examples/rtps_chatter and examples/rclc_chatter (command
 *   echo against Fast DDS).
 */
inline void attach_rx_interrupt(uint pin, const rosette::net::SocketProvider& provider) {
    detail::g_rx_interrupt_pin = pin;
    detail::g_rx_interrupt_provider = &provider;
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_IN);
    gpio_pull_up(pin);
    gpio_add_raw_irq_handler(pin, detail::rx_interrupt_handler);
    gpio_set_irq_enabled(pin, GPIO_IRQ_EDGE_FALL, true);
    irq_set_enabled(IO_IRQ_BANK0, true);
}

}  // namespace rosette::net::wiznet
