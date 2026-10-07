// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file boards/common/wiznet/wiznet_board_types.hpp
 * @brief The WIZnet boards' driver types: chip + bus + socket provider.
 *
 * One include resolves the concrete types for the current board from
 * `board_config.h` (`ROSETTE_WIZNET_CHIP`), over the portable driver
 * (ports/net/wiznet/) and the bus the selected chip port supplies in its
 * `wiznet_board_bus.hpp` (`BoardBus`, `make_board_bus()`; for example
 * ports/chip/rp2/ and ports/chip/imxrt/):
 *
 *     #include "wiznet_board_types.hpp"
 *     namespace wiz = rosette::net::wiznet;
 *
 *     auto bus = wiz::make_board_bus();
 *     wiz::BoardDevice device{bus};
 *     wiz::BoardSocketProvider sockets{device};
 *     auto s = sockets.open(rosette::net::SocketDomain::ipv4);
 *
 * Peripheral setup (baud rate, pin functions, reset sequencing) stays in
 * board bring-up; `make_board_bus()` only binds the configured pins.
 *
 * `board_config.h` names the fitted part as the register trait's name in
 * `rosette::net::wiznet` (`w5100s.hpp`, `w5500.hpp`, `w6x00.hpp`):
 *
 *     #define ROSETTE_WIZNET_CHIP  W5100S
 */

#pragma once

#include "board_config.h"

#include <cstddef>

#include "ports/net/wiznet/arp_resolver.hpp"
#include "ports/net/wiznet/device.hpp"
#include "ports/net/wiznet/link.hpp"
#include "ports/net/wiznet/socket_provider.hpp"
#include "ports/net/wiznet/w5100s.hpp"
#include "ports/net/wiznet/w5500.hpp"
#include "ports/net/wiznet/w6x00.hpp"
#include "wiznet_board_bus.hpp"

#ifndef ROSETTE_WIZNET_CHIP
#error "board_config.h must define ROSETTE_WIZNET_CHIP (e.g. W5100S)"
#endif

namespace rosette::net::wiznet {

/// The register trait for the part this board is built for.
using BoardChip = ROSETTE_WIZNET_CHIP;

using BoardDevice = WiznetDevice<BoardChip, BoardBus>;
using BoardSocketProvider = WiznetSocketProvider<BoardChip, BoardBus>;
using BoardLink = WiznetLink<BoardChip, BoardBus>;
using BoardArpResolver = WiznetArpResolver<BoardChip, BoardBus>;

/**
 * @brief Next hops the board's ARP cache holds (ports/net/wiznet/arp_cache.hpp).
 *
 * `ROSETTE_WIZNET_ARP_CACHE_ENTRIES` (CMake cache variable of the same name),
 * which bsp.cmake sets by default to the build's RTPS peer limit plus four:
 * one host per active peer, plus the NTP, DHCP and TFTP servers and the
 * gateway. Idle entries stay in the table as `stale`, so a full table is the
 * usual state and its size, not a timer, decides which next hops keep their
 * MAC. 0 attaches no resolver, and the chip ARPs whenever a socket's
 * destination changes; that is always the case on a part without socket-less
 * ARP (`BoardChip::has_socketless_arp()`, false on the W5500). The fallback
 * serves builds that bypass bsp.cmake.
 */
#if defined(ROSETTE_WIZNET_ARP_CACHE_ENTRIES)
inline constexpr std::size_t kBoardArpCacheEntries =
    BoardChip::has_socketless_arp() ? ROSETTE_WIZNET_ARP_CACHE_ENTRIES : 0u;
#else
inline constexpr std::size_t kBoardArpCacheEntries = BoardChip::has_socketless_arp() ? 8u : 0u;
#endif

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
