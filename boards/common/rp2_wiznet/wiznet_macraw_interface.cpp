/**
 * @file boards/common/rp2_wiznet/wiznet_macraw_interface.cpp
 * @brief The WIZnet EVB-Pico boards' FreeRTOS+TCP interface: the chip in MACRAW mode.
 *
 * Composition only: the FreeRTOS+TCP port supplies NetworkInterface<Device>
 * (ports/net/freertos_tcp/freertos_tcp_interface.hpp), the WIZnet port
 * supplies the MAC (ports/net/wiznet/macraw_device.hpp), and this file picks
 * the board's chip, bus, and interrupt pin from `board_config.h` and
 * implements freertos_tcp_board_interface.hpp over them.
 */

#include "freertos_tcp_board_interface.hpp"

#include <cstdio>

#include "freertos_tcp_interface.hpp"
#include "ports/net/wiznet/macraw_device.hpp"
#include "wiznet_board_types.hpp"

namespace rosette::net::freertos {
namespace {

using BoardMacrawDevice = wiznet::WiznetMacrawDevice<wiznet::BoardChip, wiznet::BoardBus>;

// The interface owns the device, and the device owns the bus, so one
// file-scope object carries the whole chain with its construction order fixed.
NetworkInterface<BoardMacrawDevice> g_wiz0{MacAddress ROSETTE_MAC_ADDR_DEFAULT,
                                           wiznet::make_board_bus()};

NetworkInterface_t g_wiz0_descriptor;

}  // namespace

NetworkInterface_t* add_board_network_interface(const MacAddress& mac) noexcept {
    g_wiz0.set_station_mac_address(mac);
    NetworkInterface_t* descriptor = g_wiz0.add_to_freertos(g_wiz0_descriptor, "wiz0");
    if (!g_wiz0.attach_rx_interrupt(ROSETTE_PIN_WIZNET_INT)) {
        std::printf("[wiznet] INT pin %u not attached; receive polls\n",
                    static_cast<unsigned>(ROSETTE_PIN_WIZNET_INT));
    }

    return descriptor;
}

}  // namespace rosette::net::freertos
