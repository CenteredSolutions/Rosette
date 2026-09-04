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
 * in board bring-up; `make_board_bus()` only binds the configured instance and
 * CS pin.
 */

#pragma once

#include "board_config.h"

#include "chip_select.hpp"
#include "device.hpp"
#include "executor.hpp"
#include "socket_provider.hpp"

#include "rosette/platform/bus_pico_spi.hpp"

namespace rosette::net::wiznet {

using BoardBus = rosette::platform::PicoSpiBus;
using BoardDevice = WiznetDevice<BoardChip, BoardBus>;
using BoardSocketProvider = WiznetSocketProvider<BoardChip, BoardBus>;

/// Binds the SPI instance and CS pin named in `board_config.h`.
inline BoardBus make_board_bus() noexcept
{
    return BoardBus{ROSETTE_SPI_INST, ROSETTE_PIN_SPI_CS};
}

}  // namespace rosette::net::wiznet
