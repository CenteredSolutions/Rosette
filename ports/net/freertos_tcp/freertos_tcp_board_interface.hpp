// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

#pragma once

/**
 * @file ports/net/freertos_tcp/freertos_tcp_board_interface.hpp
 * @brief The one hook a board composition supplies to the FreeRTOS+TCP port: its network interface.
 *
 * The FreeRTOS+TCP port does not know which MAC a board carries. A board
 * composition instantiates NetworkInterface<Device> over its MAC provider
 * (boards/common/wiznet/wiznet_macraw_interface.cpp does, over the WIZnet
 * chip in MACRAW mode) and implements this function; the port's start-up
 * code calls it, then adds endpoints and starts the IP task.
 */

extern "C" {
#include "FreeRTOS.h"
#include "FreeRTOS_IP.h"
}

#include "rosette/net/mac.hpp"

namespace rosette::net::freertos {

/**
 * @brief Registers the board's network interface with FreeRTOS+TCP and wires its interrupt.
 *
 * FreeRTOS+TCP V4 start-up is: add each interface, add endpoints to it
 * (FreeRTOS_FillEndPoint()), then FreeRTOS_IPInit_Multi(). This is the first
 * step, done by the board because only the board knows the MAC provider and
 * the GPIO its interrupt is wired to.
 *
 * @param[in] mac Station hardware address; the same address the caller then
 *   gives FreeRTOS_FillEndPoint().
 * @return The registered descriptor, owned by the board for the program's
 *   lifetime; call once, before FreeRTOS_IPInit_Multi().
 *
 * Verification:
 * - Correct means: after FreeRTOS_IPInit_Multi(), the stack initialises the
 *   returned interface, frames sent to @p mac reach the IP task, and the
 *   board's MAC transmits the stack's frames.
 * - Covered by: no automated test; nothing yet links an image on FreeRTOS+TCP.
 *
 * References:
 * - FreeRTOS+TCP V4 multiple-interface API: FreeRTOS_AddNetworkInterface(),
 *   FreeRTOS_FillEndPoint(), FreeRTOS_IPInit_Multi()
 */
NetworkInterface_t* add_board_network_interface(const MacAddress& mac) noexcept;

}  // namespace rosette::net::freertos
