// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file ports/chip/imxrt/imxrt_usb_console.c
 * @brief The i.MX RT port's USB console: TinyUSB CDC ACM on USB_OTG1, serviced from its interrupt.
 *
 * The console is a USB serial port (CDC ACM) that printf() writes to through
 * newlib's _write() (imxrt_newlib_syscalls.c), beside or instead of an LPUART.
 *
 * Execution model. A no-RTOS firmware has no thread to run tud_task(), and
 * the console must not need the application's loop to call it. The USB
 * interrupt does both halves of TinyUSB's work: tud_int_handler() turns the
 * controller's status into events, and tud_task() then runs them, all at the
 * lowest interrupt priority, so the network's GPIO and timer interrupts
 * preempt it. A writer in thread context masks only the USB interrupt in the
 * NVIC while it calls into TinyUSB, which makes it the sole user of the stack
 * for that moment; an interrupt that arrives meanwhile stays pending and runs
 * once the mask lifts. Every event TinyUSB acts on starts in that interrupt,
 * so no event waits for a poll.
 *
 * Writes never block on a missing terminal. With no terminal holding the
 * port open (DTR low) the transmit FIFO keeps the newest octets
 * (tusb_config.h); when a terminal opens it, the backlog is sent. With a
 * terminal open and the FIFO full, a write waits for the host to drain it,
 * up to kStallLimitUs without progress, then drops the rest, so a stalled
 * host costs the firmware a bounded delay per write.
 *
 * Reboot request. A host that sets the line to 134 baud asks the board to
 * enter its bootloader (the Teensyduino convention); the console calls
 * board_enter_bootloader(), which a board without one leaves as a no-op.
 *
 * References: i.MX RT1060 Reference Manual §42 (USB), §43 (USBPHY), §14.8
 * (CCM_ANALOG PLL_USB1); TinyUSB src/device/usbd.h, src/osal/osal_none.h,
 * hw/bsp/imxrt/family.c (PHY bring-up); PJRC Teensyduino cores/teensy4/usb.c
 * (controller reset after the bootloader, 134-baud reboot).
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include "fsl_clock.h"
#include "fsl_device_registers.h"
#include "imxrt_console.h"
#include "tusb.h"

/// Longest a write waits for the host to take data before dropping the rest.
#define kStallLimitUs 20000u

/// Time the host's SET_LINE_CODING status stage needs before the reboot (PJRC: 80 microframes).
#define kRebootDelayUs 10000u

extern uint64_t rosette_imxrt_monotonic_us(void);

static bool g_started;

__attribute__((weak)) void board_enter_bootloader(void) {}

/**
 * @brief Brings USBPHY1 and the USB_OTG1 clocks up from whatever state the boot left them in.
 *
 * The Teensy bootloader runs its own USB device on this controller and
 * leaves it running, so the controller and PHY are reset first, as
 * Teensyduino's usb_init() does. PLL3 (USB1 PLL, 480 MHz) is already
 * running when the image executes in place, since the FlexSPI clock derives
 * from it; CLOCK_EnableUsbhs0PhyPllClock() only enables its USB outputs.
 * The PHY's transmit calibration is the value NXP's SDK and TinyUSB use for
 * the RT10xx parts.
 */
static void usb_phy_start(void) {
    CLOCK_EnableClock(kCLOCK_UsbOh3);
    USBPHY1->CTRL_SET = USBPHY_CTRL_SFTRST_MASK;
    USB1->USBCMD &= ~USB_USBCMD_RS_MASK;
    USB1->USBCMD |= USB_USBCMD_RST_MASK;
    while ((USB1->USBCMD & USB_USBCMD_RST_MASK) != 0u) {
    }

    NVIC_ClearPendingIRQ(USB_OTG1_IRQn);

    CLOCK_EnableUsbhs0PhyPllClock(kCLOCK_Usbphy480M, 480000000u);
    CLOCK_EnableUsbhs0Clock(kCLOCK_Usb480M, 480000000u);

    uint32_t tx = USBPHY1->TX;
    tx &= ~(USBPHY_TX_D_CAL_MASK | USBPHY_TX_TXCAL45DM_MASK | USBPHY_TX_TXCAL45DP_MASK);
    tx |= USBPHY_TX_D_CAL(0x0Cu) | USBPHY_TX_TXCAL45DP(0x06u) | USBPHY_TX_TXCAL45DM(0x06u);
    USBPHY1->TX = tx;
}

bool rosette_imxrt_usb_console_init(void) {
    usb_phy_start();
    NVIC_SetPriority(USB_OTG1_IRQn, (1u << __NVIC_PRIO_BITS) - 1u);

    const tusb_rhport_init_t init = {
        .role = TUSB_ROLE_DEVICE,
        .speed = TUSB_SPEED_HIGH,
    };
    if (!tusb_init(0, &init)) {
        return false;
    }

    g_started = true;
    setvbuf(stdout, NULL, _IONBF, 0);
    return true;
}

/// @brief Runs TinyUSB: the controller's events, then the device stack's work for them.
void USB_OTG1_IRQHandler(void) {
    tud_int_handler(0);
    tud_task();
    __DSB();
}

void rosette_imxrt_usb_console_write(const char* data, size_t length) {
    if (!g_started) {
        return;
    }

    uint64_t last_progress_us = rosette_imxrt_monotonic_us();
    while (length > 0u) {
        NVIC_DisableIRQ(USB_OTG1_IRQn);
        const uint32_t written = tud_cdc_write(data, (uint32_t)length);
        (void)tud_cdc_write_flush();
        NVIC_EnableIRQ(USB_OTG1_IRQn);

        data += written;
        length -= written;
        const uint64_t now_us = rosette_imxrt_monotonic_us();
        if (written > 0u) {
            last_progress_us = now_us;
        } else if (now_us - last_progress_us >= kStallLimitUs) {
            return;
        }
    }
}

/**
 * @brief Sends the backlog when a terminal opens the port (DTR rises).
 *
 * TinyUSB calls it from tud_task(), in the USB interrupt.
 */
void tud_cdc_line_state_cb(uint8_t itf, bool dtr, bool rts) {
    (void)itf;
    (void)rts;
    if (dtr) {
        (void)tud_cdc_write_flush();
    }
}

/**
 * @brief Enters the board's bootloader when the host sets the line to 134 baud.
 *
 * TinyUSB calls it from tud_task(), in the USB interrupt, once the
 * SET_LINE_CODING data has arrived; the wait lets the status stage finish
 * before the board leaves the bus.
 */
void tud_cdc_line_coding_cb(uint8_t itf, const cdc_line_coding_t* line_coding) {
    (void)itf;
    if (line_coding->bit_rate != 134u) {
        return;
    }

    const uint64_t start_us = rosette_imxrt_monotonic_us();
    while (rosette_imxrt_monotonic_us() - start_us < kRebootDelayUs) {
    }

    board_enter_bootloader();
}
