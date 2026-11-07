// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file ports/chip/rp2/wiznet_board_bus.hpp
 * @brief The rp2 port's WIZnet bus for a board: hardware SPI, or QSPI on the W6300.
 *
 * The shared WIZnet composition (boards/common/wiznet/wiznet_board_types.hpp)
 * includes this header by its fixed name from the selected chip port's
 * directory, so the composition is the same on every chip and the bus is the
 * chip's. It reads the board's `board_config.h`:
 *
 *  - `ROSETTE_WIZNET_BUS_QSPI` defined: the W6300's QSPI wiring
 *    (rp2_qspi_bus.hpp, single lane), on `ROSETTE_PIN_SPI_SCK`, `_TX` (IO0),
 *    `_RX` (IO1), and `_CS`;
 *  - otherwise the hardware SPI peripheral `ROSETTE_SPI_INST` with the
 *    GPIO chip select `ROSETTE_PIN_SPI_CS` (rp2_spi_bus.hpp).
 *
 * The board's board_init() configures the peripheral and pins first;
 * make_board_bus() only binds them.
 */

#pragma once

#include "board_config.h"

#if defined(ROSETTE_WIZNET_BUS_QSPI)
#  include "rp2_qspi_bus.hpp"
#else
#  include "rp2_spi_bus.hpp"
#endif

namespace rosette::net::wiznet {

#if defined(ROSETTE_WIZNET_BUS_QSPI)
using BoardBus = rosette::platform::Rp2QspiBus;

/// @brief Binds the QSPI pins named in `board_config.h` (single-lane driver).
inline BoardBus make_board_bus() noexcept {
    return BoardBus{{
        ROSETTE_PIN_SPI_SCK,
        ROSETTE_PIN_SPI_TX,  // IO0 / MOSI
        ROSETTE_PIN_SPI_RX,  // IO1 / MISO
        ROSETTE_PIN_SPI_CS,
    }};
}
#else
using BoardBus = rosette::platform::Rp2SpiBus;

/// @brief Binds the SPI instance and CS pin named in `board_config.h`.
inline BoardBus make_board_bus() noexcept {
    return BoardBus{ROSETTE_SPI_INST, ROSETTE_PIN_SPI_CS};
}
#endif

}  // namespace rosette::net::wiznet
