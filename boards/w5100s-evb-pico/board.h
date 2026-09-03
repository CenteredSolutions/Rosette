/**
 * @file boards/w5100s-evb-pico/board.h
 * @brief Board bring-up entry points for the WIZnet W5100S-EVB-Pico (RP2040).
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initializes SPI, GPIO, the W5100S reset, and the ioLibrary callbacks.
 *
 * Call once at the top of `main()`, before constructing a `WiznetLink`.
 */
void board_init(void);

/**
 * @brief Fills @p mac with this board's 6-byte hardware address.
 *
 * @param[out] mac Six-byte buffer.
 */
void board_get_mac_address(uint8_t mac[6]);

/**
 * @brief Drives the on-board user LED.
 *
 * @param[in] on `true` to light the LED.
 */
void board_set_led(bool on);

#ifdef __cplusplus
}  // extern "C"
#endif
