// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

#pragma once

/**
 * @file boards/mimxrt1020-evk/ksz8081_phy.hpp
 * @brief The EVK's PHY set-up: the KSZ8081RNB's vendor register, for either ENET device.
 */

#include "imxrt_enet_link.hpp"

namespace rosette::net::freertos {

/**
 * @brief Puts the KSZ8081 in RMII 50 MHz clock mode: the RT1021 drives its XI pin with
 * ENET_REF_CLK.
 *
 * The ENET's ImxrtEnetConfig::configure_phy hook: runs after the PHY's
 * software reset and before auto-negotiation starts. The bit leaves reset
 * clear (register 1Fh reads 0x8100), and with it clear the PHY hears its
 * link partner (energy detect) but never completes auto-negotiation.
 *
 * @param[in,out] link The ENET, whose MDIO bus reaches the PHY.
 * @return False when an MDIO transaction failed.
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
bool configure_ksz8081(imxrt::ImxrtEnetLink& link) noexcept;

}  // namespace rosette::net::freertos
