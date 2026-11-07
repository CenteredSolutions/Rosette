// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file ports/chip/imxrt/imxrt_gpio_pin.h
 * @brief The i.MX RT port's GPIO numbering: one integer naming a GPIO port and bit.
 *
 * Rosette's chip contract names a pin with one integer
 * (rosette/platform/gpio_interrupt.hpp). On the i.MX RT a pin is a bit of one
 * of the standard GPIO ports (GPIO1..GPIO5); the port packs the two as
 * `(port << 5) | bit`, so GPIO4_IO06 is `ROSETTE_IMXRT_GPIO(4, 6)`. A board
 * names its pins this way in board_config.h, and the chip port and buses
 * decode them with the helpers below.
 *
 * The number names the GPIO, not the pad: the pad's IOMUXC mux (ALT5, GPIO)
 * and its pull and drive settings are the board's to set in board_init(),
 * since the pad that carries a given GPIO is a per-package table.
 *
 * References: i.MX RT1060 Reference Manual §12 (GPIO), §11 (IOMUXC).
 */

#ifndef ROSETTE_IMXRT_GPIO_PIN_H
#define ROSETTE_IMXRT_GPIO_PIN_H

#include <stdint.h>

#include "fsl_device_registers.h"

/// The pin number of GPIO<port>_IO<bit>; port 1..5, bit 0..31.
#define ROSETTE_IMXRT_GPIO(port, bit) ((uint32_t)(((port) << 5) | (bit)))

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief The GPIO port a pin number names.
 *
 * @param[in] pin A ROSETTE_IMXRT_GPIO() value.
 * @return The port's registers; NULL when the port is not GPIO1..GPIO5.
 */
static inline GPIO_Type* rosette_imxrt_gpio_port(uint32_t pin) {
    switch (pin >> 5) {
        case 1u:
            return GPIO1;
        case 2u:
            return GPIO2;
        case 3u:
            return GPIO3;
#if defined(GPIO4)
        case 4u:
            return GPIO4;
#endif
        case 5u:
            return GPIO5;
        default:
            return 0;
    }
}

/// @brief The bit within its port that a pin number names.
static inline uint32_t rosette_imxrt_gpio_bit(uint32_t pin) {
    return pin & 31u;
}

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // ROSETTE_IMXRT_GPIO_PIN_H
