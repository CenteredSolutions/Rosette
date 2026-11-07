// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file boards/mimxrt1020-evk/enet_zero_copy_interface.cpp
 * @brief The EVK's FreeRTOS+TCP interface: the RT1021's ENET moving frames in the stack's own
 * network buffers.
 *
 * Composition only: the FreeRTOS+TCP port supplies
 * ZeroCopyNetworkInterface<Device> and ZeroCopyBufferPool
 * (ports/net/freertos_tcp/zero_copy_interface.hpp, zero_copy_buffer_pool.hpp), the
 * chip port supplies the MAC (ports/chip/imxrt/imxrt_enet_zero_copy_device.hpp),
 * and this file sizes the descriptor rings, places the stack's network
 * buffers, names the PHY, and implements
 * ports/net/freertos_tcp/board_interface.hpp over them. board_init() has
 * already muxed the RMII pads, started the 50 MHz reference clock, and
 * released the PHY from reset (board_init_ethernet()).
 *
 * Memory: the network buffers are the only frame storage on the board. All
 * ipconfigNUM_NETWORK_BUFFER_DESCRIPTORS of them live in g_pool, in the
 * DTCM (DMA-reachable and not cached); ROSETTE_ENET_RX_BUFFERS of them sit in
 * the receive ring at any time.
 */

#include <cstdint>

#include "FreeRTOS.h"
#include "board.h"
#include "board_config.h"
#include "imxrt_enet_zero_copy_device.hpp"
#include "ksz8081_phy.hpp"
#include "ports/net/freertos_tcp/board_interface.hpp"
#include "ports/net/freertos_tcp/zero_copy_buffer_pool.hpp"
#include "ports/net/freertos_tcp/zero_copy_interface.hpp"

namespace rosette::net::freertos {
namespace {

using imxrt::ImxrtEnetZeroCopyDevice;
using Pool = ZeroCopyBufferPool<ImxrtEnetZeroCopyDevice::kFrameOffset,
                                ImxrtEnetZeroCopyDevice::kBufferAlignment>;

static_assert(Pool::kRegionBytes >= ImxrtEnetZeroCopyDevice::kBufferBytes,
              "a network buffer must hold the ENET's receive buffer size (MRBR)");
static_assert(Pool::kCount > ROSETTE_ENET_RX_BUFFERS,
              "the receive ring keeps its buffers; the stack needs some of its own");

// In .bss, which the linker places in the DTCM: not cached, and reachable by the ENET's DMA.
Pool                                                                                g_pool;
imxrt::ImxrtEnetZeroCopyRings<ROSETTE_ENET_RX_BUFFERS, ROSETTE_ENET_TX_DESCRIPTORS> g_rings;

ZeroCopyNetworkInterface<ImxrtEnetZeroCopyDevice> g_enet0{
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

/// @brief BufferAllocation_1's storage hook: every network buffer in g_pool, in the ENET's layout.
extern "C" void vNetworkInterfaceAllocateRAMToBuffers(
    NetworkBufferDescriptor_t pxNetworkBuffers[ipconfigNUM_NETWORK_BUFFER_DESCRIPTORS]) {
    rosette::net::freertos::g_pool.assign(pxNetworkBuffers);
}
