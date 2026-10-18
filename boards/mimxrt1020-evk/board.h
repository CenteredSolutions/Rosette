/**
 * @file boards/mimxrt1020-evk/board.h
 * @brief Board bring-up entry points for the NXP MIMXRT1020-EVK.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Starts the console, readies the Ethernet PHY, and flashes the user LED.
 *
 * Call once at the top of `main()`. The clock tree and the time base are
 * already running: the chip port brings them up before main()
 * (ports/chip/imxrt/chip_imxrt.cpp). The console is LPUART1, which the
 * DAPLink probe bridges to its USB virtual serial port. Ethernet: muxes the
 * RMII and MDIO pads, starts the ENET PLL's 50 MHz reference clock on
 * ENET_REF_CLK, and pulses the KSZ8081's reset; the ENET itself starts when
 * FreeRTOS+TCP initialises the interface (enet_zero_copy_interface.cpp).
 */
void board_init(void);

/**
 * @brief Fills @p mac with this board's 6-byte hardware address.
 *
 * @param[out] mac Six-byte buffer.
 */
void board_get_mac_address(uint8_t mac[6]);

/**
 * @brief Lights or darkens the user LED (D5).
 *
 * @param[in] on True lights the LED.
 */
void board_set_led(bool on);

#ifdef __cplusplus
}  // extern "C"
#endif
