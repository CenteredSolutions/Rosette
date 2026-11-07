// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file boards/micromod-teensy-w5500/board.h
 * @brief Board bring-up entry points for the SparkFun MicroMod Teensy with the Ethernet Function
 * Board.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Starts the USB and UART consoles, flashes the LED, initializes LPSPI4, and resets the
 * W5500.
 *
 * Prints the reset cause (SRC_SRSR) and a line once the W5500's reset pulse
 * is done, so the boot log shows how far the board got.
 * Call once at the top of `main()`, before constructing a `WiznetLink`. The
 * clock tree and the time base are already running: the chip port brings
 * them up before main() (ports/chip/imxrt/chip_imxrt.cpp).
 */
void board_init(void);

/**
 * @brief Fills @p mac with this board's 6-byte hardware address.
 *
 * @param[out] mac Six-byte buffer.
 */
void board_get_mac_address(uint8_t mac[6]);

/**
 * @brief Drives the user LED; does nothing on this board.
 *
 * The MicroMod Teensy's status LED is on SPI_SCK (Teensy pin 13), which
 * clocks the W5500, so the board leaves it to the SPI bus.
 *
 * @param[in] on Ignored.
 */
void board_set_led(bool on);

#ifdef __cplusplus
}  // extern "C"
#endif
