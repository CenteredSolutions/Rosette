// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file ports/chip/imxrt/imxrt_enet_link.cpp
 * @brief The i.MX RT ENET's control half: bring-up, PHY status, multicast filter, interrupt.
 *
 * See imxrt_enet_link.hpp for the design. The PHY is driven through
 * the IEEE 802.3 clause 22 registers alone:
 *
 *   reg  name    used for
 *   0    BMCR    software reset; enable and restart auto-negotiation
 *   1    BMSR    link status (latched low, so read twice), negotiation done
 *   4    ANAR    what this end advertises: 10/100, half and full duplex
 *   5    ANLPAR  what the partner advertised; ANAR & ANLPAR picks the mode
 */

#include "imxrt_enet_link.hpp"

#include <cstring>

#include "fsl_clock.h"

namespace rosette::net::imxrt {
namespace {

namespace pf = rosette::platform;

// ── Clause 22 registers and bits (IEEE 802.3-2018 §22.2.4) ──
constexpr uint8_t  kBmcr = 0u;
constexpr uint8_t  kBmsr = 1u;
constexpr uint8_t  kAnar = 4u;
constexpr uint8_t  kAnlpar = 5u;
constexpr uint16_t kBmcrReset = 1u << 15;
constexpr uint16_t kBmcrAutoNegEnable = 1u << 12;
constexpr uint16_t kBmcrAutoNegRestart = 1u << 9;
constexpr uint16_t kBmsrLinkUp = 1u << 2;
constexpr uint16_t kBmsrAutoNegDone = 1u << 5;
constexpr uint16_t kAbility100Full = 1u << 8;
constexpr uint16_t kAbility100Half = 1u << 7;
constexpr uint16_t kAbility10Full = 1u << 6;
constexpr uint16_t kAbility10Half = 1u << 5;
constexpr uint16_t kSelector8023 = 0x0001u;

/// Longest wait for the PHY's software reset to finish (802.3 §22.2.4.1.1 allows 0.5 s).
constexpr uint32_t kPhyResetTimeoutMs = 500u;

/**
 * @brief The ENET's 64-bucket multicast hash: the top six bits of the address's CRC-32.
 *
 * The ENET hashes a destination address with the Ethernet CRC-32 (reflected,
 * initial value all ones, no final inversion) and indexes GAUR:GALR with the
 * CRC's six most significant bits; fsl_enet's ENET_AddMulticastGroup()
 * computes the same value.
 *
 * @param[in] address A multicast MAC address.
 * @return The bucket, 0..63: bits 32..63 live in GAUR, 0..31 in GALR.
 *
 * References: i.MX RT1020 Reference Manual, ENET chapter: GAUR / GALR and
 * "Hash table algorithm"; fsl_enet.c ENET_AddMulticastGroup().
 */
uint8_t multicast_bucket(const MacAddress& address) noexcept {
    uint32_t crc = 0xFFFFFFFFu;
    for (uint8_t octet : address) {
        for (int bit = 0; bit < 8; ++bit) {
            const bool feedback = ((octet ^ crc) & 1u) != 0u;
            crc >>= 1;
            octet >>= 1;
            if (feedback) {
                crc ^= 0xEDB88320u;
            }
        }
    }

    return static_cast<uint8_t>(crc >> 26);
}

/// @brief The best mode both ends advertised (802.3 Annex 28B.3 priority order).
void resolve_mode(uint16_t common, PhyStatus& status) noexcept {
    if ((common & kAbility100Full) != 0u) {
        status.speed = PhySpeed::mbps100;
        status.duplex = PhyDuplex::full;
    } else if ((common & kAbility100Half) != 0u) {
        status.speed = PhySpeed::mbps100;
        status.duplex = PhyDuplex::half;
    } else if ((common & kAbility10Full) != 0u) {
        status.speed = PhySpeed::mbps10;
        status.duplex = PhyDuplex::full;
    } else {
        status.speed = PhySpeed::mbps10;
        status.duplex = PhyDuplex::half;
    }
}

}  // namespace

/**
 * @brief Ungates the ENET clock and configures the MAC over the device's rings.
 *
 * fsl_enet ENET_Init() resets the ENET, installs the descriptor ring bases,
 * the receive buffer size (MRBR), the accelerators, and the interrupt
 * callback; this then programs the multicast filter from any groups added
 * before it. Reception does not start until start_receiving(), so a device
 * that fills its own receive descriptors does so in between.
 *
 * @param[in] mac     Station address for the MAC's address filter.
 * @param[in] buffers The device's rings; with null data buffers fsl_enet
 *   leaves the receive descriptors untouched.
 * @param[in] mode    Interrupt sources and accelerators.
 * @return False when fsl_enet rejects the configuration; the ENET is left
 *   as it was.
 *
 * Verification:
 * - Correct means: after start() and start_receiving() the link partner sees
 *   carrier within the auto-negotiation time (about 2 s), and frames to
 *   @p mac and to broadcast reach the device's receive ring.
 * - Covered by: bring-up on the MIMXRT1020-EVK with either device (carrier
 *   on the bench Pi's eth0, then a DHCP lease).
 *
 * References: i.MX RT1020 Reference Manual, ENET chapter ("Initialization
 * sequence", RACC / TACC); fsl_enet.h ENET_Init().
 */
bool ImxrtEnetLink::start(const MacAddress& mac, const enet_buffer_config_t& buffers,
                          const ImxrtEnetMode& mode) noexcept {
    enet_config_t config;
    ENET_GetDefaultConfig(&config);
    config.miiMode = kENET_RmiiMode;
    config.miiSpeed = kENET_MiiSpeed100M;
    config.miiDuplex = kENET_MiiFullDuplex;
    config.interrupt = mode.interrupts;
    config.rxAccelerConfig = mode.rx_accelerators;
    config.txAccelerConfig = mode.tx_accelerators;
    config.callback = &ImxrtEnetLink::on_enet_event;
    config.userData = this;

    // Before ENET_Init() enables the interrupt, so it never runs at a
    // priority the kernel forbids for an interrupt that signals an event.
    NVIC_SetPriority(ENET_IRQn, config_.irq_priority);

    MacAddress station = mac;
    if (ENET_Init(ENET, &handle_, &config, &buffers, station.data(), CLOCK_GetFreq(kCLOCK_IpgClk))
        != kStatus_Success) {
        return false;
    }

    enet_clocked_ = true;
    pf::LockGuard guard(control_mutex_);
    program_multicast_filter();
    return true;
}

bool ImxrtEnetLink::start_receiving() noexcept {
    ENET_ActiveRead(ENET);
    if (!start_phy()) {
        ENET_Deinit(ENET);
        enet_clocked_ = false;
        return false;
    }

    started_ = true;
    return true;
}

/**
 * @brief Resets the PHY, runs the board's set-up, and starts auto-negotiation for 10/100.
 *
 * @return False when the PHY does not answer on MDIO, does not leave reset
 *   within kPhyResetTimeoutMs, or the board's configure_phy fails.
 */
bool ImxrtEnetLink::start_phy() noexcept {
    if (!phy_write(kBmcr, kBmcrReset)) {
        return false;
    }

    const uint32_t start = pf::monotonic_ms();
    uint16_t       bmcr = kBmcrReset;
    while ((bmcr & kBmcrReset) != 0u) {
        if (!phy_read(kBmcr, bmcr) || pf::monotonic_ms() - start > kPhyResetTimeoutMs) {
            return false;
        }
    }

    if (config_.configure_phy != nullptr && !config_.configure_phy(*this)) {
        return false;
    }

    const uint16_t advertise =
        kAbility100Full | kAbility100Half | kAbility10Full | kAbility10Half | kSelector8023;
    return phy_write(kAnar, advertise)
           && phy_write(kBmcr, kBmcrAutoNegEnable | kBmcrAutoNegRestart);
}

void ImxrtEnetLink::set_mac_address(const MacAddress& mac) noexcept {
    if (started_) {
        MacAddress station = mac;
        ENET_SetMacAddr(ENET, station.data());
    }
}

/**
 * @brief Reads link and negotiated mode from the PHY, and moves the MAC to that mode when it
 * changed.
 *
 * Verification:
 * - Correct means: forcing the link partner to 10 Mbit/s half duplex
 *   (`ethtool -s eth0 speed 10 duplex half autoneg off` on the bench Pi)
 *   reports mbps10/half and frames keep flowing.
 * - Covered by: the bench procedure above on the MIMXRT1020-EVK.
 *
 * References: IEEE 802.3-2018 §22.2.4.2 (BMSR link status latches low),
 * Annex 28B.3; fsl_enet.h ENET_SetMII().
 */
PhyStatus ImxrtEnetLink::phy_status() noexcept {
    PhyStatus status{};
    if (!started_) {
        return status;
    }

    uint16_t bmsr = 0u;
    if (!phy_read(kBmsr, bmsr) || !phy_read(kBmsr, bmsr)) {
        return status;
    }

    status.link_up = (bmsr & kBmsrLinkUp) != 0u;
    status.auto_negotiation_complete = (bmsr & kBmsrAutoNegDone) != 0u;
    uint16_t anar = 0u;
    uint16_t anlpar = 0u;
    if (!status.link_up || !status.auto_negotiation_complete || !phy_read(kAnar, anar)
        || !phy_read(kAnlpar, anlpar)) {
        return status;
    }

    resolve_mode(static_cast<uint16_t>(anar & anlpar), status);
    if (status.speed != programmed_.speed || status.duplex != programmed_.duplex) {
        ENET_SetMII(ENET,
                    status.speed == PhySpeed::mbps100 ? kENET_MiiSpeed100M : kENET_MiiSpeed10M,
                    status.duplex == PhyDuplex::full ? kENET_MiiFullDuplex : kENET_MiiHalfDuplex);
        programmed_ = status;
    }

    return status;
}

bool ImxrtEnetLink::set_receive_event(pf::Event* event) noexcept {
    rx_event_ = event;
    return true;
}

void ImxrtEnetLink::add_multicast(const MacAddress& group) noexcept {
    pf::LockGuard guard(control_mutex_);
    uint8_t&      refs = multicast_refs_[multicast_bucket(group)];
    if (refs < UINT8_MAX) {
        ++refs;
    }

    program_multicast_filter();
}

void ImxrtEnetLink::remove_multicast(const MacAddress& group) noexcept {
    pf::LockGuard guard(control_mutex_);
    uint8_t&      refs = multicast_refs_[multicast_bucket(group)];
    if (refs > 0u) {
        --refs;
    }

    program_multicast_filter();
}

/// @brief Writes GAUR:GALR from the per-bucket counts; a no-op while the ENET clock is gated.
void ImxrtEnetLink::program_multicast_filter() noexcept {
    if (!enet_clocked_) {
        return;
    }

    uint32_t upper = 0u;
    uint32_t lower = 0u;
    for (uint8_t bucket = 0u; bucket < multicast_refs_.size(); ++bucket) {
        if (multicast_refs_[bucket] == 0u) {
            continue;
        }

        if (bucket >= 32u) {
            upper |= 1u << (bucket - 32u);
        } else {
            lower |= 1u << bucket;
        }
    }

    ENET->GAUR = upper;
    ENET->GALR = lower;
}

bool ImxrtEnetLink::phy_read(uint8_t reg, uint16_t& value) noexcept {
    pf::LockGuard guard(control_mutex_);
    return ENET_MDIORead(ENET, config_.phy_address, reg, &value) == kStatus_Success;
}

bool ImxrtEnetLink::phy_write(uint8_t reg, uint16_t value) noexcept {
    pf::LockGuard guard(control_mutex_);
    return ENET_MDIOWrite(ENET, config_.phy_address, reg, value) == kStatus_Success;
}

/// @brief fsl_enet's callback, in the ENET interrupt: wakes the consumer for each frame received or
/// sent.
void ImxrtEnetLink::on_enet_event(ENET_Type*, enet_handle_t*, enet_event_t event,
                                  enet_frame_info_t*, void* user_data) {
    auto*            self = static_cast<ImxrtEnetLink*>(user_data);
    pf::Event* const rx_event = self->rx_event_;
    if ((event == kENET_RxEvent || event == kENET_TxEvent) && rx_event != nullptr) {
        rx_event->signal_from_isr();
    }
}

}  // namespace rosette::net::imxrt
