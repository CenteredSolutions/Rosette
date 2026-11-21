// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

#pragma once

/**
 * @file ports/net/freertos_tcp/board_interface.hpp
 * @brief The one hook a board composition supplies to the FreeRTOS+TCP port: its network interface.
 *
 * The FreeRTOS+TCP port does not know which MAC a board carries. A board
 * composition instantiates NetworkInterface<Device> over its MAC provider
 * (boards/common/wiznet/wiznet_macraw_interface.cpp does, over the WIZnet
 * chip in MACRAW mode) and implements this function; the port's board
 * network (ports/net/freertos_tcp/board_network.cpp) calls it, then adds
 * endpoints and starts the IP task.
 */

extern "C" {
#include "FreeRTOS.h"
#include "FreeRTOS_IP.h"
}

#include "rosette/net/mac.hpp"

namespace rosette::net::freertos {

/**
 * @brief Registers the board's network interface with FreeRTOS+TCP and reports its station address.
 *
 * FreeRTOS+TCP V4 start-up is: add each interface, add endpoints to it
 * (FreeRTOS_FillEndPoint()), then FreeRTOS_IPInit_Multi(). The board network
 * (ports/net/freertos_tcp/board_network.cpp) runs that sequence and calls this for the
 * first step, because only the board knows its MAC, the MAC's address, and
 * how the MAC signals a received frame (a GPIO edge for a SPI controller, an
 * NVIC interrupt for an on-chip MAC).
 *
 * @param[out] mac The station hardware address the interface receives on:
 *   the board's fused or derived address (board_get_mac_address()). The
 *   board network gives it to every endpoint it fills.
 * @return The registered descriptor, owned by the board for the program's
 *   lifetime; null when the board has no interface to offer, in which case
 *   the network never comes up. Called once, before FreeRTOS_IPInit_Multi().
 *
 * Verification:
 * - Correct means: after FreeRTOS_IPInit_Multi(), the stack initialises the
 *   returned interface, frames sent to @p mac reach the IP task, and the
 *   board's MAC transmits the stack's frames.
 * - Covered by: the board network's bring-up on hardware (a DHCP lease is
 *   only possible when both directions work).
 *
 * References:
 * - FreeRTOS+TCP V4 multiple-interface API: FreeRTOS_AddNetworkInterface(),
 *   FreeRTOS_FillEndPoint(), FreeRTOS_IPInit_Multi()
 */
NetworkInterface_t* add_board_network_interface(MacAddress& mac) noexcept;

}  // namespace rosette::net::freertos
