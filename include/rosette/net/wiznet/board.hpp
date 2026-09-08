/**
 * @file net/wiznet/board.hpp
 * @brief Board-configured WIZnet stack: chip + bus + socket provider.
 *
 * One include resolves the concrete types for the current board from
 * `board_config.h` (`ROSETTE_WIZNET_CHIP`, `ROSETTE_SPI_INST`,
 * `ROSETTE_PIN_SPI_CS`):
 *
 *     #include "rosette/net/wiznet/board.hpp"
 *     namespace wiz = rosette::net::wiznet;
 *
 *     auto bus = wiz::make_board_bus();
 *     wiz::BoardDevice device{bus};
 *     wiz::BoardSocketProvider sockets{device};
 *     auto s = sockets.open(rosette::net::SocketDomain::ipv4);
 *
 * The executor is chosen automatically — `PollingExecutor` on a no-RTOS build,
 * `FreeRtosExecutor` otherwise. To pin it, name the third template argument on
 * `WiznetSocketProvider<BoardChip, BoardBus, PollingExecutor>` directly.
 *
 * SPI peripheral setup (baud rate, pin alt-functions, reset sequencing) stays
 * in board bring-up; `make_board_bus()` only binds the configured pins.
 *
 * The bus is chosen by `board_config.h`: define `ROSETTE_WIZNET_BUS_QSPI` for
 * the W6300's QSPI wiring (single-lane for now, see `bus_pico_qspi.hpp`);
 * otherwise the hardware SPI peripheral `ROSETTE_SPI_INST` is used.
 */

#pragma once

#include "board_config.h"

#include "chip_select.hpp"
#include "device.hpp"
#include "executor.hpp"
#include "socket_provider.hpp"

#if defined(ROSETTE_WIZNET_BUS_QSPI)
#  include "rosette/platform/bus_pico_qspi.hpp"
#else
#  include "rosette/platform/bus_pico_spi.hpp"
#endif

namespace rosette::net::wiznet {

#if defined(ROSETTE_WIZNET_BUS_QSPI)
using BoardBus = rosette::platform::PicoQspiBus;

/// Binds the QSPI pins named in `board_config.h` (single-lane driver for now).
inline BoardBus make_board_bus() noexcept
{
    return BoardBus{{
        ROSETTE_PIN_SPI_SCK,
        ROSETTE_PIN_SPI_TX,  // IO0 / MOSI
        ROSETTE_PIN_SPI_RX,  // IO1 / MISO
        ROSETTE_PIN_SPI_CS,
    }};
}
#else
using BoardBus = rosette::platform::PicoSpiBus;

/// Binds the SPI instance and CS pin named in `board_config.h`.
inline BoardBus make_board_bus() noexcept
{
    return BoardBus{ROSETTE_SPI_INST, ROSETTE_PIN_SPI_CS};
}
#endif

using BoardDevice = WiznetDevice<BoardChip, BoardBus>;
using BoardSocketProvider = WiznetSocketProvider<BoardChip, BoardBus>;

}  // namespace rosette::net::wiznet
