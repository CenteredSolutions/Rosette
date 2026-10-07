/**
 * @file ports/chip/imxrt/imxrt_console.h
 * @brief Routes the C library's stdout and stderr to an LPUART, a USB serial port, or both.
 *
 * A board starts each console it has from board_init(); printf() output goes
 * to every console started (imxrt_newlib_syscalls.c). The USB console exists
 * on boards that set ROSETTE_IMXRT_USB_CONSOLE in board.cmake.
 */

#ifndef ROSETTE_IMXRT_CONSOLE_H
#define ROSETTE_IMXRT_CONSOLE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "fsl_device_registers.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initializes `uart` at `baud`, 8N1, and sends printf() output to it from then on.
 *
 * The board calls it from board_init() after muxing the UART's pads; output
 * written earlier is discarded. See imxrt_newlib_syscalls.c.
 *
 * @param[in] uart            LPUART instance, e.g. LPUART6.
 * @param[in] baud            Bits per second, e.g. 115200.
 * @param[in] source_clock_hz The LPUART root clock (UART_CLK_ROOT), e.g.
 *   BOARD_BOOTCLOCKRUN_UART_CLK_ROOT from the SDK's clock_config.h.
 * @return `false` when the LPUART cannot make `baud` from the clock.
 *
 * Verification:
 * - Correct means: printf("x\n") emits "x\r\n" on the UART's TX pin.
 * - Covered by: the boot log of every example on an i.MX RT board.
 *
 * References: MCUXpresso SDK fsl_lpuart LPUART_Init().
 */
bool rosette_imxrt_console_init(LPUART_Type* uart, uint32_t baud, uint32_t source_clock_hz);

/**
 * @brief Starts USB_OTG1 as a CDC ACM serial port and sends printf() output to it from then on.
 *
 * The board calls it from board_init(), first, so the boot log is captured:
 * output waits in the transmit backlog (4 KiB, newest kept) until a terminal
 * opens the port. The USB interrupt services the device from then on; the
 * application calls nothing (imxrt_usb_console.c).
 *
 * @return `false` when TinyUSB could not start the controller; printf()
 *   output then goes only to the other consoles.
 *
 * Verification:
 * - Correct means: the host enumerates a CDC ACM device (`lsusb` lists
 *   1209:0001, /dev/ttyACM* appears) within a second of reset, and a
 *   terminal that opens it later receives the boot log from the first line.
 * - Covered by: the boot log of every example on a board with the USB console.
 *
 * References: TinyUSB tusb_init(); USB CDC 1.2 / PSTN 1.2 (ACM).
 *
 * Example:
 * @code{.c}
 * void board_init(void) {
 *     CLOCK_EnableClock(kCLOCK_Iomuxc);
 *     (void)rosette_imxrt_usb_console_init();
 *     printf("[board] up\n");  // reaches the terminal when it opens the port
 * }
 * @endcode
 * Full program: boards/micromod-teensy-w5500/board_init.c
 */
bool rosette_imxrt_usb_console_init(void);

/**
 * @brief Queues `length` octets of `data` on the USB console, as is (no newline translation).
 *
 * Called by _write(). Returns at once when the console is not started or no
 * terminal holds the port open (the backlog keeps the newest octets); with a
 * terminal open, waits while the host drains the FIFO, and drops the rest
 * after 20 ms without progress.
 *
 * @param[in] data   Octets to send; not retained.
 * @param[in] length Count of octets.
 */
void rosette_imxrt_usb_console_write(const char* data, size_t length);

/**
 * @brief Board hook: leaves the firmware for the board's USB bootloader.
 *
 * The USB console calls it, in the USB interrupt, when the host sets the
 * line to 134 baud (`stty -F /dev/ttyACM0 134`), the Teensyduino request
 * that teensy_loader_cli and the Arduino IDE use to reprogram without the
 * button. The chip port's weak definition does nothing; a board with a
 * bootloader defines it and does not return.
 *
 * References: PJRC Teensyduino cores/teensy4/usb.c (134-baud reboot, `bkpt #251`).
 */
void board_enter_bootloader(void);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // ROSETTE_IMXRT_CONSOLE_H
