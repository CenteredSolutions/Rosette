/**
 * @file ports/chip/imxrt/imxrt_enet_mac_device.cpp
 * @brief The copying i.MX RT ENET MacDevice: frames copied through fsl_enet's own rings.
 *
 * See imxrt_enet_mac_device.hpp for the design; bring-up, the PHY, and the
 * multicast filter are ImxrtEnetLink's (imxrt_enet_link.cpp).
 */

#include "imxrt_enet_mac_device.hpp"

namespace rosette::net::imxrt {
namespace {

namespace pf = rosette::platform;

/// Longest a transmit waits for a TX descriptor the DMA still owns: several full frames' wire time.
constexpr uint64_t kTxBusyTimeoutUs = 1000u;

}  // namespace

/**
 * @brief Starts the ENET and the PHY on the first call; later calls only latch the address.
 *
 * fsl_enet fills the descriptors from the board's ImxrtEnetRings, and only
 * the receive interrupt is enabled. The link comes up asynchronously, a
 * second or more later; phy_status() reports it.
 *
 * @param[in] mac Station address for the MAC's address filter.
 * @return False when the ENET or the PHY does not start; the next call
 *   starts over.
 *
 * Verification:
 * - Correct means: as ImxrtEnetLink::start(): carrier at the link partner
 *   and frames to @p mac reach the RX ring.
 * - Covered by: bring-up on the MIMXRT1020-EVK with
 *   ROSETTE_IMXRT_ENET_LENDING=OFF (DHCP lease, tools/udp_echo_probe.py).
 *
 * References: fsl_enet.h ENET_Init(), ENET_ActiveRead().
 */
bool ImxrtEnetMacDevice::initialise(const MacAddress& mac) noexcept {
    if (link_.started()) {
        link_.set_mac_address(mac);
        return true;
    }

    return link_.start(mac, buffers_, ImxrtEnetMode{}) && link_.start_receiving();
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
    if (!link_.started()) {
        return false;
    }

    pf::LockGuard  guard(tx_mutex_);
    const uint64_t start = pf::monotonic_us();
    for (;;) {
        const status_t status = ENET_SendFrame(ENET, &link_.handle(), data,
                                               static_cast<uint32_t>(length), 0u, false, nullptr);
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
    if (!link_.started()) {
        return 0u;
    }

    for (;;) {
        uint32_t       length = 0u;
        const status_t status = ENET_GetRxFrameSize(&link_.handle(), &length, 0u);
        if (status == kStatus_Success) {
            return length;
        }

        if (status != kStatus_ENET_RxFrameError) {
            return 0u;
        }

        // A CRC, overrun, or length error: hand the descriptors back to the DMA.
        (void)ENET_ReadFrame(ENET, &link_.handle(), nullptr, 0u, 0u, nullptr);
    }
}

std::size_t ImxrtEnetMacDevice::receive_frame(uint8_t* buffer, std::size_t capacity) noexcept {
    const std::size_t length = peek_frame_size();
    if (length == 0u) {
        return 0u;
    }

    if (length > capacity) {
        (void)ENET_ReadFrame(ENET, &link_.handle(), nullptr, 0u, 0u, nullptr);
        return 0u;
    }

    const status_t status =
        ENET_ReadFrame(ENET, &link_.handle(), buffer, static_cast<uint32_t>(length), 0u, nullptr);
    return status == kStatus_Success ? length : 0u;
}

}  // namespace rosette::net::imxrt
