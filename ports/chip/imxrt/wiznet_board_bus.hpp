/**
 * @file ports/chip/imxrt/wiznet_board_bus.hpp
 * @brief The i.MX RT port's WIZnet bus for a board: an LPSPI with a GPIO chip select.
 *
 * The shared WIZnet composition (boards/common/wiznet/wiznet_board_types.hpp)
 * includes this header by its fixed name from the selected chip port's
 * directory, so the composition is the same on every chip and the bus is the
 * chip's. It reads the board's `board_config.h`: the LPSPI instance
 * `ROSETTE_LPSPI_INST` (e.g. LPSPI4) and the chip select
 * `ROSETTE_PIN_SPI_CS`, a ROSETTE_IMXRT_GPIO() number. The board's
 * board_init() initializes the LPSPI and muxes the pads first;
 * make_board_bus() only binds them.
 */

#pragma once

#include "board_config.h"
#include "imxrt_lpspi_bus.hpp"

namespace rosette::net::wiznet {

using BoardBus = rosette::platform::ImxrtLpspiBus;

/// @brief Binds the LPSPI instance and chip-select GPIO named in `board_config.h`.
inline BoardBus make_board_bus() noexcept {
    return BoardBus{{ROSETTE_LPSPI_INST, ROSETTE_PIN_SPI_CS}};
}

}  // namespace rosette::net::wiznet
