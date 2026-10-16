/**
 * @file ports/chip/imxrt/imxrt_enet_mac_device.cpp
 * @brief The i.MX RT ENET MacDevice: bring-up, frame copy, PHY status, multicast filter.
 *
 * See imxrt_enet_mac_device.hpp for the design. The PHY is driven through
 * the IEEE 802.3 clause 22 registers alone:
 *
 *   reg  name    used for
 *   0    BMCR    software reset; enable and restart auto-negotiation
 *   1    BMSR    link status (latched low, so read twice), negotiation done
 *   4    ANAR    what this end advertises: 10/100, half and full duplex
 *   5    ANLPAR  what the partner advertised; ANAR & ANLPAR picks the mode
 */

#include "imxrt_enet_mac_device.hpp"

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

/// Longest a transmit waits for a TX descriptor the DMA still owns: several full frames' wire time.
constexpr uint64_t kTxBusyTimeoutUs = 1000u;

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
 * @brief Starts the ENET and the PHY on the first call; later calls only latch the address.
 *
 * The first successful call ungates the ENET clock, installs the rings and
 * the receive interrupt (fsl_enet ENET_Init()), programs the multicast
 * filter from any groups added before it, starts receiving, then resets the
 * PHY, runs the board's PHY set-up, and starts auto-negotiation. The link
 * comes up asynchronously, a second or more later; phy_status() reports it.
 *
 * @param[in] mac Station address for the MAC's address filter.
 * @return False when fsl_enet rejects the rings or the PHY does not answer
 *   on MDIO; the ENET is shut down again and the next call starts over.
 *
 * Verification:
 * - Correct means: after a true return the link partner sees carrier within
 *   the auto-negotiation time (about 2 s), and frames to @p mac and to
 *   broadcast reach the RX ring.
 * - Covered by: bring-up on the MIMXRT1020-EVK (carrier on the bench Pi's
 *   eth0, then a DHCP lease).
 *
 * References: i.MX RT1020 Reference Manual, ENET chapter ("Initialization
 * sequence"); fsl_enet.h ENET_Init(), ENET_ActiveRead().
 */
bool ImxrtEnetMacDevice::initialise(const MacAddress& mac) noexcept {
    if (started_) {
        set_mac_address(mac);
        return true;
    }

    if (!start_enet(mac)) {
        return false;
    }

    if (!start_phy()) {
        ENET_Deinit(ENET);
        enet_clocked_ = false;
        return false;
    }

    started_ = true;
    return true;
}

bool ImxrtEnetMacDevice::start_enet(const MacAddress& mac) noexcept {
    enet_config_t config;
    ENET_GetDefaultConfig(&config);
    config.miiMode = kENET_RmiiMode;
    config.miiSpeed = kENET_MiiSpeed100M;
    config.miiDuplex = kENET_MiiFullDuplex;
    config.interrupt = kENET_RxFrameInterrupt;
    config.callback = &ImxrtEnetMacDevice::on_enet_event;
    config.userData = this;

    // Before ENET_Init() enables the interrupt, so it never runs at a
    // priority the kernel forbids for an interrupt that signals an event.
    NVIC_SetPriority(ENET_IRQn, config_.irq_priority);

    MacAddress station = mac;
    if (ENET_Init(ENET, &handle_, &config, &buffers_, station.data(), CLOCK_GetFreq(kCLOCK_IpgClk))
        != kStatus_Success) {
        return false;
    }

    enet_clocked_ = true;
    {
        pf::LockGuard guard(control_mutex_);
        program_multicast_filter();
    }

    ENET_ActiveRead(ENET);
    return true;
}

/**
 * @brief Resets the PHY, runs the board's set-up, and starts auto-negotiation for 10/100.
 *
 * @return False when the PHY does not answer on MDIO, does not leave reset
 *   within kPhyResetTimeoutMs, or the board's configure_phy fails.
 */
bool ImxrtEnetMacDevice::start_phy() noexcept {
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

void ImxrtEnetMacDevice::set_mac_address(const MacAddress& mac) noexcept {
    if (started_) {
        MacAddress station = mac;
        ENET_SetMacAddr(ENET, station.data());
    }
}

/**
 * @brief Copies one frame into a free TX buffer and starts its transmission.
 *
 * Waits up to kTxBusyTimeoutUs for the DMA to free a descriptor when the
 * ring is full, which only happens when the IP task sends faster than the
 * wire drains.
 *
 * @param[in] data   Frame from the destination MAC, without FCS.
 * @param[in] length 1..max_frame_size() octets.
 * @return False before initialise(), or when no descriptor freed in time.
 */
bool ImxrtEnetMacDevice::send_frame(const uint8_t* data, std::size_t length) noexcept {
    if (!started_) {
        return false;
    }

    pf::LockGuard  guard(tx_mutex_);
    const uint64_t start = pf::monotonic_us();
    for (;;) {
        const status_t status =
            ENET_SendFrame(ENET, &handle_, data, static_cast<uint32_t>(length), 0u, false, nullptr);
        if (status == kStatus_Success) {
            return true;
        }

        if (status != kStatus_ENET_TxFrameBusy || pf::monotonic_us() - start > kTxBusyTimeoutUs) {
            return false;
        }
    }
}

/**
 * @brief Length of the next good frame; frames the MAC flagged as errors are dropped on the way.
 *
 * @return Octets without FCS, or 0 when the RX ring holds no complete frame.
 */
std::size_t ImxrtEnetMacDevice::peek_frame_size() noexcept {
    if (!started_) {
        return 0u;
    }

    for (;;) {
        uint32_t       length = 0u;
        const status_t status = ENET_GetRxFrameSize(&handle_, &length, 0u);
        if (status == kStatus_Success) {
            return length;
        }

        if (status != kStatus_ENET_RxFrameError) {
            return 0u;
        }

        // A CRC, overrun, or length error: hand the descriptors back to the DMA.
        (void)ENET_ReadFrame(ENET, &handle_, nullptr, 0u, 0u, nullptr);
    }
}

std::size_t ImxrtEnetMacDevice::receive_frame(uint8_t* buffer, std::size_t capacity) noexcept {
    const std::size_t length = peek_frame_size();
    if (length == 0u) {
        return 0u;
    }

    if (length > capacity) {
        (void)ENET_ReadFrame(ENET, &handle_, nullptr, 0u, 0u, nullptr);
        return 0u;
    }

    const status_t status =
        ENET_ReadFrame(ENET, &handle_, buffer, static_cast<uint32_t>(length), 0u, nullptr);
    return status == kStatus_Success ? length : 0u;
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
PhyStatus ImxrtEnetMacDevice::phy_status() noexcept {
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

bool ImxrtEnetMacDevice::set_receive_event(pf::Event* event) noexcept {
    rx_event_ = event;
    return true;
}

void ImxrtEnetMacDevice::add_multicast(const MacAddress& group) noexcept {
    pf::LockGuard guard(control_mutex_);
    uint8_t&      refs = multicast_refs_[multicast_bucket(group)];
    if (refs < UINT8_MAX) {
        ++refs;
    }

    program_multicast_filter();
}

void ImxrtEnetMacDevice::remove_multicast(const MacAddress& group) noexcept {
    pf::LockGuard guard(control_mutex_);
    uint8_t&      refs = multicast_refs_[multicast_bucket(group)];
    if (refs > 0u) {
        --refs;
    }

    program_multicast_filter();
}

/// @brief Writes GAUR:GALR from the per-bucket counts; a no-op while the ENET clock is gated.
void ImxrtEnetMacDevice::program_multicast_filter() noexcept {
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

bool ImxrtEnetMacDevice::phy_read(uint8_t reg, uint16_t& value) noexcept {
    pf::LockGuard guard(control_mutex_);
    return ENET_MDIORead(ENET, config_.phy_address, reg, &value) == kStatus_Success;
}

bool ImxrtEnetMacDevice::phy_write(uint8_t reg, uint16_t value) noexcept {
    pf::LockGuard guard(control_mutex_);
    return ENET_MDIOWrite(ENET, config_.phy_address, reg, value) == kStatus_Success;
}

/// @brief fsl_enet's callback, in the ENET interrupt: wakes the receive task for each frame.
void ImxrtEnetMacDevice::on_enet_event(ENET_Type*, enet_handle_t*, enet_event_t event,
                                       enet_frame_info_t*, void* user_data) {
    auto*            self = static_cast<ImxrtEnetMacDevice*>(user_data);
    pf::Event* const rx_event = self->rx_event_;
    if (event == kENET_RxEvent && rx_event != nullptr) {
        rx_event->signal_from_isr();
    }
}

}  // namespace rosette::net::imxrt
