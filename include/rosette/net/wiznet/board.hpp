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
 * SPI peripheral setup (baud rate, pin alt-functions, reset sequencing) stays
 * in board bring-up; `make_board_bus()` only binds the configured pins.
 *
 * The bus is chosen by `board_config.h`: define `ROSETTE_WIZNET_BUS_QSPI` for
 * the W6300's QSPI wiring (single-lane for now, see `rp2_qspi_bus.hpp`);
 * otherwise the hardware SPI peripheral `ROSETTE_SPI_INST` is used.
 */

#pragma once

#include "board_config.h"

#include "chip_select.hpp"
#include "device.hpp"
#include "socket_provider.hpp"

#if defined(ROSETTE_WIZNET_BUS_QSPI)
#include "rp2_qspi_bus.hpp"
#else
#include "rp2_spi_bus.hpp"
#endif

namespace rosette::net::wiznet {

#if defined(ROSETTE_WIZNET_BUS_QSPI)
using BoardBus = rosette::platform::Rp2QspiBus;

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
using BoardBus = rosette::platform::Rp2SpiBus;

/// Binds the SPI instance and CS pin named in `board_config.h`.
inline BoardBus make_board_bus() noexcept
{
    return BoardBus{ROSETTE_SPI_INST, ROSETTE_PIN_SPI_CS};
}
#endif

using BoardDevice = WiznetDevice<BoardChip, BoardBus>;
using BoardSocketProvider = WiznetSocketProvider<BoardChip, BoardBus>;

/**
 * @brief The board's receive-buffer KiB per hardware socket (WiznetDevice::set_rx_layout()).
 *
 * The chip's default (BoardChip::default_rx_layout_kib(): half the memory on
 * one bulk socket, 2 KiB on a medium one, the least on the rest), unless the
 * build sets `ROSETTE_WIZNET_RX_LAYOUT_KIB` to a comma-separated list, one
 * entry per socket (CMake cache variable of the same name, `;` or `,`
 * separated). The RTPS user socket asks for the bulk buffer, so a build whose
 * largest sample (RMW_ROSETTE_MAX_SAMPLE_SIZE) outgrows it has its fragment
 * bursts repaired fragment by fragment instead of arriving whole.
 */
inline constexpr BoardDevice::RxLayout kBoardRxLayoutKib =
#if defined(ROSETTE_WIZNET_RX_LAYOUT_KIB)
    {ROSETTE_WIZNET_RX_LAYOUT_KIB};
#else
    BoardChip::default_rx_layout_kib();
#endif
static_assert(BoardDevice::is_valid_rx_layout(kBoardRxLayoutKib),
              "ROSETTE_WIZNET_RX_LAYOUT_KIB: each entry must be 0 or a power of two, and the "
              "entries must fit the chip's receive memory");

}  // namespace rosette::net::wiznet
