// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file ports/chip/imxrt/chip_imxrt.cpp
 * @brief i.MX RT10xx chip layer of the platform contract.
 *
 * The chip port owns the clock tree and the time base, and brings both up in
 * a constructor that runs before any other static constructor and before
 * main() (start_chip()), so monotonic_us() is valid from the first line of
 * application code and a board never configures clocks itself:
 *
 *  - The clock tree is the MCUXpresso SDK's BOARD_BootClockRUN() from the
 *    reference board the board names (ROSETTE_IMXRT_SDK_BOARD; on the
 *    RT1062, evkmimxrt1060: 600 MHz core, 150 MHz IPG, the FlexSPI clock left
 *    as the boot ROM set it, since the image executes from that flash). The
 *    CCM stays in RUN mode on WFI/WFE, so a wait never gates the clocks the
 *    time base and the peripherals run on. The instruction and data caches
 *    are on.
 *  - The monotonic clock is the PIT's 64-bit lifetime timer (channels 0 and
 *    1 chained), counting down from all ones at the 24 MHz crystal (PERCLK
 *    from OSC), so it never wraps and needs no interrupt to extend it.
 *  - PIT channel 2 is the one-shot that ends wait_for_event_until() at its
 *    deadline.
 *
 * The unique ID is the 64-bit fuse UID (OCOTP CFG0/CFG1); the boot nonce is
 * drawn from the TRNG once per boot. There is no wall-clock source: time
 * arrives through NTP or an agent (WallClock).
 *
 * Falling-edge GPIO interrupts take the standard GPIO1..GPIO5 ports' combined
 * interrupt vectors, so a pin on a fast GPIO port (GPIO6..GPIO9, selected per
 * pin in IOMUXC_GPR26..29) cannot take one; out of reset every pin is on its
 * standard port.
 *
 * The bare-metal sync hooks mask interrupts with PRIMASK; the RT10xx has one
 * core, so that is the whole critical section. Waits sleep in WFE, which an
 * interrupt, SEV, or the PIT one-shot ends.
 *
 * References:
 * - i.MX RT1060 Reference Manual §14 (CCM), §53 (PIT, lifetime timer),
 *   §12 (GPIO), §23 (OCOTP), §45 (TRNG)
 * - Armv7-M Architecture Reference Manual §B1.5.18 (WFE wake-up events)
 * - MCUXpresso SDK 2.16: fsl_pit, fsl_gpio, fsl_trng, fsl_clock
 */

#include <cstddef>
#include <cstdint>

#include "rosette/platform/chip_sync.hpp"
#include "rosette/platform/gpio_interrupt.hpp"
#include "rosette/platform/platform.hpp"

#include "clock_config.h"
#include "fsl_clock.h"
#include "fsl_gpio.h"
#include "fsl_pit.h"
#include "fsl_trng.h"
#include "imxrt_gpio_pin.h"

#ifdef ROSETTE_IMXRT_FLEXRAM_BANK_CFG
// flexram_reset_handler.S runs before any C, so it spells these out; they
// must be the device header's.
static_assert(IOMUXC_GPR_BASE + offsetof(IOMUXC_GPR_Type, GPR16) == 0x400AC040u);
static_assert(IOMUXC_GPR_BASE + offsetof(IOMUXC_GPR_Type, GPR17) == 0x400AC044u);
static_assert(IOMUXC_GPR_GPR16_FLEXRAM_BANK_CFG_SEL_MASK == 0x4u);
static_assert((ROSETTE_IMXRT_FLEXRAM_BANK_CFG & ~IOMUXC_GPR_GPR17_FLEXRAM_BANK_CFG_MASK) == 0u);
#endif

namespace rosette::platform {

namespace {

/// PIT input clock: the 24 MHz crystal through PERCLK, undivided.
constexpr uint64_t kTicksPerUs = 24u;

/// PIT channel that ends a wait_for_event_until() at its deadline.
constexpr pit_chnl_t kWakeChannel = kPIT_Chnl_2;

/// One attach_falling_edge() registration; a null handler marks the slot free.
struct FallingEdgeSlot {
    uint32_t pin = 0u;
    GpioInterruptHandler volatile handler = nullptr;
    void* volatile context = nullptr;
};

FallingEdgeSlot g_falling_edge[4];

/// @brief The combined interrupt that serves `pin`: one per port half (bits 0..15, 16..31).
IRQn_Type falling_edge_irq(uint32_t pin) noexcept {
#if defined(GPIO4)
    constexpr IRQn_Type kGpio4Low = GPIO4_Combined_0_15_IRQn;
    constexpr IRQn_Type kGpio4High = GPIO4_Combined_16_31_IRQn;
#else
    // A part without GPIO4 (the RT1021): rosette_imxrt_gpio_port() rejects
    // its pins before this table is read.
    constexpr IRQn_Type kGpio4Low = NotAvail_IRQn;
    constexpr IRQn_Type kGpio4High = NotAvail_IRQn;
#endif
    static constexpr IRQn_Type kLow[] = {GPIO1_Combined_0_15_IRQn, GPIO2_Combined_0_15_IRQn,
                                         GPIO3_Combined_0_15_IRQn, kGpio4Low,
                                         GPIO5_Combined_0_15_IRQn};
    static constexpr IRQn_Type kHigh[] = {GPIO1_Combined_16_31_IRQn, GPIO2_Combined_16_31_IRQn,
                                          GPIO3_Combined_16_31_IRQn, kGpio4High,
                                          GPIO5_Combined_16_31_IRQn};
    const uint32_t             port = (pin >> 5) - 1u;
    return rosette_imxrt_gpio_bit(pin) < 16u ? kLow[port] : kHigh[port];
}

/**
 * @brief Calls the handlers whose pins on `port` have a falling edge latched.
 *
 * Shared by the ten combined GPIO vectors below. Each slot's flag is
 * acknowledged before its handler runs, so an edge during the handler
 * latches again and is not lost.
 *
 * @param[in] port_number GPIO port, 1..5, whose vector fired.
 */
void dispatch_falling_edges(uint32_t port_number) noexcept {
    GPIO_Type* const port = rosette_imxrt_gpio_port(port_number << 5);
    const uint32_t   flags = GPIO_PortGetInterruptFlags(port);
    for (FallingEdgeSlot& slot : g_falling_edge) {
        const uint32_t             mask = 1u << rosette_imxrt_gpio_bit(slot.pin);
        const GpioInterruptHandler handler = slot.handler;
        if (handler == nullptr || (slot.pin >> 5) != port_number || (flags & mask) == 0u) {
            continue;
        }

        GPIO_PortClearInterruptFlags(port, mask);
        handler(slot.context);
    }

    // Arm erratum 838869: a store that clears an interrupt source must
    // complete before the exception returns, or the interrupt re-enters.
    __DSB();
}

}  // namespace

/**
 * @brief Calls `handler(context)` on each falling edge of `pin`, a ROSETTE_IMXRT_GPIO() number.
 *
 * Sets the GPIO to an input with falling-edge detection (ICR) and enables its
 * port-half's combined interrupt. The pad's mux (GPIO) and pull-up are the
 * board's (board_init()), since the GPIO number does not name a pad; see
 * imxrt_gpio_pin.h. Attaching the same pin again replaces its handler.
 *
 * @param[in] pin     ROSETTE_IMXRT_GPIO(port, bit), port 1..5.
 * @param[in] handler Called from interrupt context; must not be null.
 * @param[in] context Passed to `handler`; must outlive the attachment.
 * @return `false` when `pin` is not on GPIO1..GPIO5 or all four slots are taken.
 *
 * Verification:
 * - Correct means: each falling edge on the pin calls the handler once, and
 *   a second attach of the pin replaces the first.
 * - Covered by: examples/udp_echo on the micromod-teensy-w5500 board
 *   (the boot log does not print "INT not attached").
 *
 * References: i.MX RT1060 Reference Manual §12.5 (GPIO_ICR1/2, GPIO_IMR, GPIO_ISR).
 */
bool attach_falling_edge(uint32_t pin, GpioInterruptHandler handler, void* context) noexcept {
    GPIO_Type* const port = rosette_imxrt_gpio_port(pin);
    if (handler == nullptr || port == nullptr) {
        return false;
    }

    const uint32_t bit = rosette_imxrt_gpio_bit(pin);
    for (FallingEdgeSlot& slot : g_falling_edge) {
        if (slot.handler != nullptr && slot.pin == pin) {
            GPIO_PortDisableInterrupts(port, 1u << bit);
            slot.context = context;
            slot.handler = handler;
            GPIO_PortEnableInterrupts(port, 1u << bit);
            return true;
        }
    }

    for (FallingEdgeSlot& slot : g_falling_edge) {
        if (slot.handler != nullptr) {
            continue;
        }

        slot.pin = pin;
        slot.context = context;
        slot.handler = handler;
        const gpio_pin_config_t config = {kGPIO_DigitalInput, 0u, kGPIO_IntFallingEdge};
        GPIO_PinInit(port, bit, &config);
        GPIO_PortClearInterruptFlags(port, 1u << bit);
        GPIO_PortEnableInterrupts(port, 1u << bit);
        EnableIRQ(falling_edge_irq(pin));
        return true;
    }

    return false;
}

uint64_t monotonic_us() noexcept {
    // The lifetime timer counts down from all ones; its complement is the
    // number of ticks since start_chip().
    return ~PIT_GetLifetimeTimerCount(PIT) / kTicksPerUs;
}

void device_unique_id(uint8_t out[8]) noexcept {
    const uint32_t words[2] = {OCOTP->CFG0, OCOTP->CFG1};
    for (int i = 0; i < 8; ++i) {
        out[i] = static_cast<uint8_t>(words[i / 4] >> (24 - 8 * (i % 4)));
    }
}

/**
 * @brief A 32-bit value that differs from one boot to the next, drawn once from the TRNG.
 *
 * The TRNG is started, read for four bytes, and stopped on the first call;
 * later calls return the same value. Should the TRNG fail, the value is the
 * time base at that moment mixed with the unique ID, which still differs
 * across boots because the time of the first call varies.
 *
 * @return The boot nonce.
 *
 * Verification:
 * - Correct means: equal within a boot, different across resets.
 * - Covered by: tests/platform/platform_contract_test.cpp (stable within a boot).
 *
 * References: i.MX RT1060 Reference Manual §45 (TRNG); MCUXpresso SDK fsl_trng.
 */
uint32_t boot_nonce() noexcept {
    static const uint32_t nonce = [] {
        uint32_t      value = 0u;
        trng_config_t config;
        const bool    drawn = TRNG_GetDefaultConfig(&config) == kStatus_Success
                           && TRNG_Init(TRNG, &config) == kStatus_Success
                           && TRNG_GetRandomData(TRNG, &value, sizeof(value)) == kStatus_Success;
        TRNG_Deinit(TRNG);
        if (!drawn) {
            value = static_cast<uint32_t>(monotonic_us()) ^ OCOTP->CFG0 ^ (OCOTP->CFG1 << 7);
        }

        return value;
    }();
    return nonce;
}

bool realtime_hint_us(int64_t& unix_us) noexcept {
    (void)unix_us;
    return false;
}

}  // namespace rosette::platform

namespace rosette::platform::chip {

uint32_t critical_enter() noexcept {
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

void critical_exit(uint32_t state) noexcept {
    __set_PRIMASK(state);
}

/**
 * @brief Sleeps in WFE until an event, an interrupt, or `deadline_us`.
 *
 * A finite deadline arms PIT channel 2 as a one-shot for the remaining time
 * (at most 2^32 ticks, about 179 s; a longer wait returns early, which the
 * contract allows). If the one-shot or any other interrupt fires before the
 * WFE, its exception return sets the event register and the WFE returns at
 * once, so no wake is lost.
 *
 * @param[in] deadline_us monotonic_us() value to wake by; UINT64_MAX waits
 *   for an event or interrupt only.
 *
 * Verification:
 * - Correct means: returns no later than the deadline plus interrupt latency,
 *   and at once when send_event() preceded it.
 * - Covered by: tests/platform/platform_contract_test.cpp (sleep_ms, Event
 *   timeouts on the bare-metal OS layer).
 *
 * References: Armv7-M ARM §B1.5.18 (WFE, the event register); i.MX RT1060
 * Reference Manual §53.6 (PIT TCTRL, TFLG).
 */
void wait_for_event_until(uint64_t deadline_us) noexcept {
    if (deadline_us == UINT64_MAX) {
        __WFE();
        return;
    }

    const uint64_t now = monotonic_us();
    if (deadline_us <= now) {
        return;
    }

    uint64_t ticks = (deadline_us - now) * kTicksPerUs;
    if (ticks > UINT32_MAX) {
        ticks = UINT32_MAX;
    }

    PIT_StopTimer(PIT, kWakeChannel);
    PIT_ClearStatusFlags(PIT, kWakeChannel, kPIT_TimerFlag);
    PIT_SetTimerPeriod(PIT, kWakeChannel, static_cast<uint32_t>(ticks));
    PIT_EnableInterrupts(PIT, kWakeChannel, kPIT_TimerInterruptEnable);
    PIT_StartTimer(PIT, kWakeChannel);
    __DSB();
    __WFE();
    PIT_StopTimer(PIT, kWakeChannel);
    PIT_DisableInterrupts(PIT, kWakeChannel, kPIT_TimerInterruptEnable);
}

void send_event() noexcept {
    __SEV();
}

}  // namespace rosette::platform::chip

/**
 * @brief Brings up the clock tree, caches, and time base before any other constructor runs.
 *
 * Runs from the C runtime's init array (priority 101, the first one
 * available to programs), after .data and .bss are set up and before main().
 * Applies the SDK clock tree, keeps the CCM in RUN mode through WFI/WFE,
 * enables the data cache (the instruction cache is on from SystemInit()),
 * switches PERCLK to the 24 MHz crystal, and starts the PIT's chained
 * lifetime timer.
 *
 * The boot ROM uses the PIT itself and leaves its channels running, with
 * their interrupt enables and flags as it last set them. Every channel is
 * stopped, its interrupt enable cleared, and its flag cleared before the
 * port claims the PIT, so the PIT interrupt fires only for the wake one-shot
 * that PIT_IRQHandler() clears.
 *
 * Verification:
 * - Correct means: monotonic_us() advances at 1 µs per µs from before main(),
 *   SystemCoreClock reports the configured core clock, and main() is reached
 *   after a boot through the ROM (the board's boot log prints).
 * - Covered by: tests/platform/platform_contract_test.cpp (clock rate check).
 *
 * References: i.MX RT1060 Reference Manual §14.7.18 (CSCMR1 PERCLK_PODF /
 * PERCLK_CLK_SEL), §14.7.20 (CLPCR LPM), §53.5.2 (lifetime timer), §53.6.5
 * (TCTRL), §53.6.6 (TFLG); PJRC Teensyduino cores/teensy4/startup.c
 * ResetHandler() ("Undo PIT timer usage by ROM startup").
 */
extern "C" __attribute__((constructor(101))) void rosette_imxrt_start_chip(void) {
    BOARD_BootClockRUN();
    CLOCK_SetMode(kCLOCK_ModeRun);
    SCB_EnableDCache();

    CLOCK_SetMux(kCLOCK_PerclkMux, 1u);
    CLOCK_SetDiv(kCLOCK_PerclkDiv, 0u);

    pit_config_t config;
    PIT_GetDefaultConfig(&config);
    PIT_Init(PIT, &config);
    for (uint32_t channel = 0u; channel < FSL_FEATURE_PIT_TIMER_COUNT; ++channel) {
        PIT->CHANNEL[channel].TCTRL = 0u;
        PIT->CHANNEL[channel].TFLG = PIT_TFLG_TIF_MASK;
    }

    NVIC_ClearPendingIRQ(PIT_IRQn);
    PIT_SetTimerPeriod(PIT, kPIT_Chnl_0, UINT32_MAX);
    PIT_SetTimerPeriod(PIT, kPIT_Chnl_1, UINT32_MAX);
    PIT_SetTimerChainMode(PIT, kPIT_Chnl_1, true);
    PIT_StartTimer(PIT, kPIT_Chnl_1);
    PIT_StartTimer(PIT, kPIT_Chnl_0);
    EnableIRQ(PIT_IRQn);

    CLOCK_EnableClock(kCLOCK_Ocotp);
}

extern "C" {

/// @brief monotonic_us() for the C system calls (imxrt_newlib_syscalls.c).
uint64_t rosette_imxrt_monotonic_us(void) {
    return rosette::platform::monotonic_us();
}

/// @brief Ends a wait_for_event_until() one-shot; the exception itself is the wake.
void PIT_IRQHandler(void) {
    PIT_ClearStatusFlags(PIT, kPIT_Chnl_2, kPIT_TimerFlag);
    PIT_StopTimer(PIT, kPIT_Chnl_2);
    __DSB();
}

void GPIO1_Combined_0_15_IRQHandler(void) {
    rosette::platform::dispatch_falling_edges(1u);
}

void GPIO1_Combined_16_31_IRQHandler(void) {
    rosette::platform::dispatch_falling_edges(1u);
}

void GPIO2_Combined_0_15_IRQHandler(void) {
    rosette::platform::dispatch_falling_edges(2u);
}

void GPIO2_Combined_16_31_IRQHandler(void) {
    rosette::platform::dispatch_falling_edges(2u);
}

void GPIO3_Combined_0_15_IRQHandler(void) {
    rosette::platform::dispatch_falling_edges(3u);
}

void GPIO3_Combined_16_31_IRQHandler(void) {
    rosette::platform::dispatch_falling_edges(3u);
}

#if defined(GPIO4)
void GPIO4_Combined_0_15_IRQHandler(void) {
    rosette::platform::dispatch_falling_edges(4u);
}

void GPIO4_Combined_16_31_IRQHandler(void) {
    rosette::platform::dispatch_falling_edges(4u);
}
#endif

void GPIO5_Combined_0_15_IRQHandler(void) {
    rosette::platform::dispatch_falling_edges(5u);
}

void GPIO5_Combined_16_31_IRQHandler(void) {
    rosette::platform::dispatch_falling_edges(5u);
}

}  // extern "C"
