// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file boards/mimxrt1020-evk/enet_copying_interface.cpp
 * @brief The EVK's FreeRTOS+TCP interface with ROSETTE_IMXRT_ENET_ZERO_COPY=OFF: the ENET copying
 * through its own buffers.
 *
 * Composition only: the FreeRTOS+TCP port supplies NetworkInterface<Device>
 * (ports/net/freertos_tcp/interface.hpp), the chip port supplies the MAC
 * (ports/chip/imxrt/imxrt_enet_mac_device.hpp), and this file sizes the DMA
 * rings, names the PHY, and implements
 * ports/net/freertos_tcp/board_interface.hpp over them. board_init() has
 * already muxed the RMII pads, started the 50 MHz reference clock, and
 * released the PHY from reset (board_init_ethernet()). The zero-copy
 * composition beside it (enet_zero_copy_interface.cpp) is the default; this one
 * is kept to measure against it.
 */

#include <cstdint>

#include "FreeRTOS.h"
#include "board.h"
#include "board_config.h"
#include "imxrt_enet_mac_device.hpp"
#include "ksz8081_phy.hpp"
#include "ports/net/freertos_tcp/board_interface.hpp"
#include "ports/net/freertos_tcp/interface.hpp"

namespace rosette::net::freertos {
namespace {

using imxrt::ImxrtEnetMacDevice;

// In .bss, which the linker places in the DTCM: not cached, and reachable by the ENET's DMA.
imxrt::ImxrtEnetRings<ROSETTE_ENET_RX_BUFFERS, ROSETTE_ENET_TX_BUFFERS> g_rings;

NetworkInterface<ImxrtEnetMacDevice> g_enet0{
    MacAddress{}, g_rings,
    imxrt::ImxrtEnetConfig{ROSETTE_PHY_MDIO_ADDRESS, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY,
                           &configure_ksz8081}};

NetworkInterface_t g_enet0_descriptor;

}  // namespace

NetworkInterface_t* add_board_network_interface(MacAddress& mac) noexcept {
    board_get_mac_address(mac.data());
    g_enet0.set_station_mac_address(mac);
    return g_enet0.add_to_freertos(g_enet0_descriptor, "enet0");
}

}  // namespace rosette::net::freertos
