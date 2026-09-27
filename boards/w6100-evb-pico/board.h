/**
 * @file boards/w6100-evb-pico/board.h
 * @brief Board bring-up entry points for the WIZnet W6100-EVB-Pico.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initializes SPI0, the GPIO pin functions, and the W6100 hardware reset.
 *
 * Call once at the top of `main()`, before constructing a `WiznetLink`. The
 * register-level driver (`rosette::net::wiznet`) owns everything past the SPI
 * peripheral and reset pin — there are no ioLibrary callbacks to register.
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
