/**
 * @file boards/w6300-evb-pico2/board.h
 * @brief Board bring-up entry points for the WIZnet W6300-EVB-Pico2.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initializes GPIO and the W6300 hardware-reset sequence.
 *
 * The W6300 bus is `rosette::platform::PicoQspiBus` (single-lane bit-bang for
 * now); it owns the SCK / IO0 / IO1 / CS pins itself, so `board_init()` only
 * handles the reset line, the interrupt line and the LED.
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
