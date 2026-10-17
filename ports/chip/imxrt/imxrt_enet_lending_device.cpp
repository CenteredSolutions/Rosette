/**
 * @file ports/chip/imxrt/imxrt_enet_lending_device.cpp
 * @brief The lending i.MX RT ENET device: descriptor rings over buffers the IP stack lends.
 *
 * See imxrt_enet_lending_device.hpp for the design. The legacy descriptor
 * fields this file uses (i.MX RT1020 RM, ENET "Legacy buffer descriptors"):
 *
 *   receive   E  (empty: the DMA owns it)   W (wrap)   L (last buffer of a frame)
 *             the error bits LG NO CR OV TR and CE (ENET_BUFFDESCRIPTOR_RX_ERR_MASK)
 *   transmit  R  (ready: the DMA owns it)   W (wrap)   L (last)   TC (append the CRC)
 *
 * The DMA walks each ring in order and stops at the first descriptor it does
 * not own; writing RDAR or TDAR sends it looking again. A receive slot whose
 * buffer the consumer has taken therefore simply stays not-empty, and the
 * DMA waits there until a fresh buffer is lent into it.
 */

#include "imxrt_enet_lending_device.hpp"

namespace rosette::net::imxrt {
namespace {

namespace pf = rosette::platform;

/// The ENET reports a received frame's length with the SHIFT16 pad included,
/// and expects a transmit length to include it.
constexpr uint16_t kShiftPad = static_cast<uint16_t>(ImxrtEnetLendingDevice::kLentFrameOffset);

}  // namespace

bool ImxrtEnetLendingDevice::aligned(const uint8_t* data) const noexcept {
    const uintptr_t region = reinterpret_cast<uintptr_t>(data) - kLentFrameOffset;
    return data != nullptr && region % kLentBufferAlignment == 0u;
}

/**
 * @brief Starts the ENET and the PHY over a full receive ring; later calls only latch the address.
 *
 * The receive descriptors already hold the consumer's buffers
 * (lend_receive_buffer()), so fsl_enet is given the rings without data
 * buffers and leaves them alone. Both interrupts are on: a received frame
 * and a sent frame each wake the consumer, the second so it collects the
 * sent buffer.
 *
 * @param[in] mac Station address for the MAC's address filter.
 * @return False until the consumer has filled the receive ring, or when the
 *   ENET or the PHY does not start; the next call starts over.
 *
 * Verification:
 * - Correct means: as ImxrtEnetLink::start(), with every received frame
 *   landing in a lent buffer two octets past its aligned start.
 * - Covered by: bring-up on the MIMXRT1020-EVK (DHCP lease,
 *   tools/udp_echo_probe.py).
 *
 * References: i.MX RT1020 Reference Manual, ENET chapter: MRBR, RACC, TACC.
 */
bool ImxrtEnetLendingDevice::initialise(const MacAddress& mac) noexcept {
    if (link_.started()) {
        link_.set_mac_address(mac);
        return true;
    }

    if (rx_filled_ != rx_count_) {
        return false;
    }

    enet_buffer_config_t buffers{};
    buffers.rxBdNumber = rx_count_;
    buffers.txBdNumber = tx_count_;
    buffers.rxBuffSizeAlign = static_cast<uint16_t>(kLentBufferBytes);
    buffers.txBuffSizeAlign = static_cast<uint16_t>(kLentBufferBytes);
    buffers.rxBdStartAddrAlign = rx_ring_;
    buffers.txBdStartAddrAlign = tx_ring_;

    ImxrtEnetMode mode;
    mode.interrupts = kENET_RxFrameInterrupt | kENET_TxFrameInterrupt;
    mode.rx_accelerators = kENET_RxAccelisShift16Enabled;
    mode.tx_accelerators = kENET_TxAccelIsShift16Enabled;
    return link_.start(mac, buffers, mode) && link_.start_receiving();
}

/**
 * @brief Arms the receive slot after the last lent one with @p data.
 *
 * @param[in] data  Frame pointer of the buffer; its region starts
 *   kLentFrameOffset octets earlier.
 * @param[in] owner The consumer's handle for the buffer.
 * @return False when the ring is full or the buffer is misaligned.
 *
 * Verification:
 * - Correct means: with the ring full and the consumer's pool empty, every
 *   taken buffer lent straight back keeps all rx_count_ slots armed, and
 *   frames are received again once the pool recovers.
 * - Covered by: a ping flood against the MIMXRT1020-EVK with the UDP echo
 *   probe running (no automated test).
 */
bool ImxrtEnetLendingDevice::lend_receive_buffer(uint8_t* data, void* owner) noexcept {
    if (!aligned(data) || rx_filled_ == rx_count_) {
        return false;
    }

    const uint16_t slot = static_cast<uint16_t>((rx_head_ + rx_filled_) % rx_count_);
    rx_owners_[slot] = owner;
    volatile enet_rx_bd_struct_t& descriptor = rx_ring_[slot];
    descriptor.buffer = static_cast<uint32_t>(reinterpret_cast<uintptr_t>(data) - kLentFrameOffset);
    descriptor.length = 0u;

    // The address must be visible to the DMA before the slot is handed over.
    __DMB();
    descriptor.control =
        static_cast<uint16_t>(ENET_BUFFDESCRIPTOR_RX_EMPTY_MASK
                              | (slot == rx_count_ - 1u ? ENET_BUFFDESCRIPTOR_RX_WRAP_MASK : 0u));
    ++rx_filled_;

    if (link_.started()) {
        __DSB();
        ENET->RDAR = ENET_RDAR_RDAR_MASK;
    }

    return true;
}

/**
 * @brief Takes the oldest completed receive slot's frame; errored frames go back to the ring.
 *
 * A frame the MAC flagged (CRC, overrun, length, truncation), or one that
 * spans several buffers because it outgrew MRBR, is not returned: each of
 * its buffers is lent again at the tail, and the search moves on.
 *
 * @param[out] frame The frame and its buffer's owner, on a true return.
 * @return False when the oldest slot is still the DMA's, or the ring is empty.
 */
bool ImxrtEnetLendingDevice::take_received_frame(LentFrame& frame) noexcept {
    while (rx_filled_ > 0u) {
        volatile enet_rx_bd_struct_t& descriptor = rx_ring_[rx_head_];
        const uint16_t                control = descriptor.control;
        if ((control & ENET_BUFFDESCRIPTOR_RX_EMPTY_MASK) != 0u) {
            return false;
        }

        // The DMA writes the frame before the status; read it in that order.
        __DMB();
        uint8_t* const data = reinterpret_cast<uint8_t*>(descriptor.buffer) + kLentFrameOffset;
        void* const    owner = rx_owners_[rx_head_];
        const uint16_t length = descriptor.length;
        rx_owners_[rx_head_] = nullptr;
        rx_head_ = static_cast<uint16_t>((rx_head_ + 1u) % rx_count_);
        --rx_filled_;

        const bool last = (control & ENET_BUFFDESCRIPTOR_RX_LAST_MASK) != 0u;
        const bool good = last && !rx_discarding_
                          && (control & ENET_BUFFDESCRIPTOR_RX_ERR_MASK) == 0u
                          && length > kShiftPad;
        rx_discarding_ = !last;
        if (!good) {
            (void)lend_receive_buffer(data, owner);
            continue;
        }

        frame.data = data;
        frame.length = length - kShiftPad;
        frame.owner = owner;
        return true;
    }

    return false;
}

/**
 * @brief Points the next transmit descriptor at the lent frame and starts the DMA.
 *
 * @param[in] frame 1..max_frame_size() octets in an aligned lent buffer.
 * @return False before initialise(), for a bad frame, or when every
 *   transmit descriptor is still queued.
 *
 * Verification:
 * - Correct means: the wire carries exactly frame.length octets from
 *   frame.data plus the CRC, and the buffer comes back from
 *   take_sent_frame() only after the ENET cleared the ready bit.
 * - Covered by: tools/udp_echo_probe.py against the MIMXRT1020-EVK (each
 *   echo is compared byte for byte, at sizes 1 to 1472 octets).
 */
bool ImxrtEnetLendingDevice::send_lent_frame(const LentFrame& frame) noexcept {
    if (!link_.started() || !aligned(frame.data) || frame.length == 0u
        || frame.length > max_frame_size()) {
        return false;
    }

    pf::LockGuard guard(tx_mutex_);
    if (tx_used_ == tx_count_) {
        return false;
    }

    const uint16_t slot = tx_next_;
    tx_owners_[slot] = frame.owner;
    volatile enet_tx_bd_struct_t& descriptor = tx_ring_[slot];
    descriptor.buffer =
        static_cast<uint32_t>(reinterpret_cast<uintptr_t>(frame.data) - kLentFrameOffset);
    descriptor.length = static_cast<uint16_t>(frame.length + kShiftPad);

    __DMB();
    descriptor.control =
        static_cast<uint16_t>(ENET_BUFFDESCRIPTOR_TX_READY_MASK | ENET_BUFFDESCRIPTOR_TX_LAST_MASK
                              | ENET_BUFFDESCRIPTOR_TX_TRANMITCRC_MASK
                              | (slot == tx_count_ - 1u ? ENET_BUFFDESCRIPTOR_TX_WRAP_MASK : 0u));
    tx_next_ = static_cast<uint16_t>((slot + 1u) % tx_count_);
    ++tx_used_;

    __DSB();
    ENET->TDAR = ENET_TDAR_TDAR_MASK;
    return true;
}

bool ImxrtEnetLendingDevice::take_sent_frame(void*& owner) noexcept {
    pf::LockGuard guard(tx_mutex_);
    if (tx_used_ == 0u || (tx_ring_[tx_done_].control & ENET_BUFFDESCRIPTOR_TX_READY_MASK) != 0u) {
        return false;
    }

    owner = tx_owners_[tx_done_];
    tx_owners_[tx_done_] = nullptr;
    tx_done_ = static_cast<uint16_t>((tx_done_ + 1u) % tx_count_);
    --tx_used_;
    return true;
}

}  // namespace rosette::net::imxrt
