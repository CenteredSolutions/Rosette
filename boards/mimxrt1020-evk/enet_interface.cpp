/**
 * @file boards/mimxrt1020-evk/enet_interface.cpp
 * @brief The EVK's FreeRTOS+TCP interface: the RT1021's ENET and the KSZ8081RNB PHY.
 *
 * Composition only: the FreeRTOS+TCP port supplies NetworkInterface<Device>
 * (ports/net/freertos_tcp/interface.hpp), the chip port supplies the MAC
 * (ports/chip/imxrt/imxrt_enet_mac_device.hpp), and this file sizes the DMA
 * rings, names the PHY, and implements
 * ports/net/freertos_tcp/board_interface.hpp over them. board_init() has
 * already muxed the RMII pads, started the 50 MHz reference clock, and
 * released the PHY from reset (board_init_ethernet()).
 */

#include <cstdint>

#include "FreeRTOS.h"
#include "board.h"
#include "board_config.h"
#include "imxrt_enet_mac_device.hpp"
#include "ports/net/freertos_tcp/board_interface.hpp"
#include "ports/net/freertos_tcp/interface.hpp"

namespace rosette::net::freertos {
namespace {

using imxrt::ImxrtEnetMacDevice;

// ── KSZ8081 vendor registers (KSZ8081RNA/RNB datasheet, register 1Fh) ──
constexpr uint8_t  kKsz8081Control2 = 0x1Fu;
constexpr uint16_t kKsz8081RmiiClock50MHz = 1u << 7;

/**
 * @brief Puts the KSZ8081 in RMII 50 MHz clock mode: the RT1021 drives its XI pin with
 * ENET_REF_CLK.
 *
 * @param[in,out] device The ENET, whose MDIO bus reaches the PHY.
 * @return False when an MDIO transaction failed.
 *
 * The bit leaves reset clear (register 1Fh reads 0x8100), and with it clear
 * the PHY hears its link partner (energy detect) but never completes
 * auto-negotiation.
 *
 * Verification:
 * - Correct means: the PHY negotiates 100BASE-TX full duplex with the bench
 *   Pi from reset (carrier on its eth0) and frames pass in both directions.
 * - Covered by: bring-up on the MIMXRT1020-EVK (DHCP lease, ping, and
 *   tools/udp_echo_probe.py).
 *
 * References: KSZ8081RNA/RNB datasheet, register 1Fh bit 7 (RMII reference
 * clock select); MCUXpresso SDK fsl_phyksz8081.c (FSL_FEATURE_PHYKSZ8081_USE_RMII50M_MODE).
 */
bool configure_ksz8081(ImxrtEnetMacDevice& device) noexcept {
    uint16_t control2 = 0u;
    return device.phy_read(kKsz8081Control2, control2)
           && device.phy_write(kKsz8081Control2,
                               static_cast<uint16_t>(control2 | kKsz8081RmiiClock50MHz));
}

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
