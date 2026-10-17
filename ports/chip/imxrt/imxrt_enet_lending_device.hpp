#pragma once

/**
 * @file ports/chip/imxrt/imxrt_enet_lending_device.hpp
 * @brief The i.MX RT10xx ENET MAC, with its RMII PHY, as a rosette::net::LendingMacDevice.
 *
 * The ENET's DMA follows rings of eight-octet buffer descriptors, each
 * pointing at one frame buffer. This device fills those pointers with
 * buffers the IP stack lends it (rosette/net/lending_mac.hpp), so frames go
 * from the wire into the stack's network buffers and from them back to the
 * wire without a copy, and the device owns no frame storage: its board-sized
 * ImxrtEnetLendingRings is the descriptors and one owner handle per
 * descriptor, about 12 octets per slot. ImxrtEnetMacDevice
 * (imxrt_enet_mac_device.hpp) is the same ENET with its own buffers.
 *
 * The descriptors are driven here rather than through fsl_enet's zero-copy
 * calls, because those allocate a replacement buffer inside the receive call
 * and reclaim transmit buffers in the interrupt, and an IP stack's pool
 * (FreeRTOS+TCP's) is not usable from an interrupt. fsl_enet still brings
 * the MAC up, through ImxrtEnetLink, which also owns the PHY, the multicast
 * filter, and the interrupt; the transmit-done interrupt wakes the consumer
 * so it can collect sent buffers promptly.
 *
 * Buffer layout: both directions run with the ENET's SHIFT16 accelerators,
 * so each frame starts two octets into a 64-octet-aligned region and the IP
 * header after the 14-octet Ethernet header is word-aligned, as FreeRTOS+TCP
 * and lwIP require. Lent buffers must be in RAM the DMA reaches and the CPU
 * does not cache (the DTCM on these parts).
 *
 * References: i.MX RT1020 Reference Manual, ENET chapter: "Legacy buffer
 * descriptors", RDAR / TDAR, MRBR, RACC[SHIFT16], TACC[SHIFT16]; MCUXpresso
 * SDK fsl_enet.h 2.9.1 (descriptor bit masks).
 */

#include <cstddef>
#include <cstdint>

#include "fsl_enet.h"
#include "imxrt_enet_link.hpp"
#include "rosette/net/lending_mac.hpp"
#include "rosette/platform/platform.hpp"

namespace rosette::net::imxrt {

/**
 * @brief The descriptor rings the ENET follows, and the owner of the buffer behind each
 * descriptor.
 *
 * Sized by the board and placed in DMA-reachable, non-cached RAM (.bss, in
 * the DTCM). The receive ring holds RxCount lent buffers at all times, so
 * RxCount frames can arrive while the receive task is busy; the transmit
 * ring holds at most TxCount frames the wire has not taken yet.
 *
 * @tparam RxCount Receive descriptors; at least 2.
 * @tparam TxCount Transmit descriptors; at least 1.
 */
template <std::size_t RxCount, std::size_t TxCount>
struct ImxrtEnetLendingRings {
    static_assert(RxCount >= 2u && TxCount >= 1u, "ImxrtEnetLendingRings: too few descriptors");

    alignas(ENET_BUFF_ALIGNMENT) enet_rx_bd_struct_t rx_descriptors[RxCount];
    alignas(ENET_BUFF_ALIGNMENT) enet_tx_bd_struct_t tx_descriptors[TxCount];
    void* rx_owners[RxCount];
    void* tx_owners[TxCount];
};

/**
 * @brief The ENET and its PHY behind the net::LendingMacDevice contract.
 *
 * One instance drives the part's one ENET for the program's lifetime,
 * normally owned by the board's FreeRTOS+TCP interface
 * (ports/net/freertos_tcp/lending_interface.hpp). It is neither copied nor
 * moved, because fsl_enet's handle and the interrupt callback hold its
 * address.
 *
 * Concurrency: the receive ring is touched only by the consumer's receive
 * task, and before that task exists by the initial lending; the transmit
 * calls come from the IP task and the receive task and share one mutex.
 *
 * Example:
 * @code{.cpp}
 * imxrt::ImxrtEnetLendingRings<4, 4> g_rings;  // .bss: DTCM, DMA-reachable and not cached
 * freertos::LendingNetworkInterface<imxrt::ImxrtEnetLendingDevice> g_enet0{
 *     MacAddress{}, g_rings, imxrt::ImxrtEnetConfig{2u, 5u, &configure_ksz8081}};
 * NetworkInterface_t g_descriptor;
 * g_enet0.add_to_freertos(g_descriptor, "enet0");  // lending and initialise() run in the IP task
 * @endcode
 * Full program: boards/mimxrt1020-evk/enet_lending_interface.cpp.
 */
class ImxrtEnetLendingDevice final : public net::LendingMacDevice {
public:
    /// Octets SHIFT16 places in front of each frame.
    static constexpr std::size_t kLentFrameOffset = 2u;
    /// Alignment of the region each lent buffer starts with (the ENET's DMA rule).
    static constexpr std::size_t kLentBufferAlignment = ENET_BUFF_ALIGNMENT;
    /// Receive buffer size, MRBR: the SHIFT16 pad and the longest frame, rounded up to 16 octets.
    static constexpr std::size_t kLentBufferBytes =
        SDK_SIZEALIGN(kLentFrameOffset + ENET_FRAME_MAX_FRAMELEN, 16u);

    /**
     * @brief Records the rings and settings; touches no hardware until initialise().
     *
     * @param[in] rings  The board's rings, in DMA-reachable non-cached RAM;
     *   they outlive the device.
     * @param[in] config PHY address, interrupt priority, PHY set-up hook.
     */
    template <std::size_t RxCount, std::size_t TxCount>
    ImxrtEnetLendingDevice(ImxrtEnetLendingRings<RxCount, TxCount>& rings,
                           const ImxrtEnetConfig&                   config) noexcept
        : rx_ring_(rings.rx_descriptors),
          tx_ring_(rings.tx_descriptors),
          rx_owners_(rings.rx_owners),
          tx_owners_(rings.tx_owners),
          rx_count_(static_cast<uint16_t>(RxCount)),
          tx_count_(static_cast<uint16_t>(TxCount)),
          link_(config) {}

    ImxrtEnetLendingDevice(const ImxrtEnetLendingDevice&) = delete;
    ImxrtEnetLendingDevice& operator=(const ImxrtEnetLendingDevice&) = delete;

    bool        initialise(const MacAddress& mac) noexcept override;
    std::size_t max_frame_size() const noexcept override { return ENET_FRAME_MAX_FRAMELEN - 4u; }

    std::size_t lent_frame_offset() const noexcept override { return kLentFrameOffset; }
    std::size_t lent_buffer_alignment() const noexcept override { return kLentBufferAlignment; }
    std::size_t lent_buffer_bytes() const noexcept override { return kLentBufferBytes; }
    bool        lend_receive_buffer(uint8_t* data, void* owner) noexcept override;
    bool        take_received_frame(LentFrame& frame) noexcept override;
    bool        send_lent_frame(const LentFrame& frame) noexcept override;
    bool        take_sent_frame(void*& owner) noexcept override;

    void set_mac_address(const MacAddress& mac) noexcept override { link_.set_mac_address(mac); }
    PhyStatus phy_status() noexcept override { return link_.phy_status(); }
    bool      set_receive_event(rosette::platform::Event* event) noexcept override {
        return link_.set_receive_event(event);
    }

    void add_multicast(const MacAddress& group) noexcept override { link_.add_multicast(group); }
    void remove_multicast(const MacAddress& group) noexcept override {
        link_.remove_multicast(group);
    }

    /// @brief The ENET's control half, for the board's PHY set-up and diagnostics.
    ImxrtEnetLink& link() noexcept { return link_; }

private:
    bool aligned(const uint8_t* data) const noexcept;

    volatile enet_rx_bd_struct_t* const rx_ring_;
    volatile enet_tx_bd_struct_t* const tx_ring_;
    void** const                        rx_owners_;
    void** const                        tx_owners_;
    const uint16_t                      rx_count_;
    const uint16_t                      tx_count_;
    uint16_t                            rx_head_ = 0u;    ///< Oldest lent receive slot.
    uint16_t                            rx_filled_ = 0u;  ///< Receive slots holding a buffer.
    bool                                rx_discarding_ = false;  ///< Inside a multi-buffer frame.
    uint16_t                            tx_next_ = 0u;           ///< Next transmit slot to fill.
    uint16_t                            tx_done_ = 0u;           ///< Oldest queued frame.
    uint16_t                            tx_used_ = 0u;           ///< Frames queued.
    ImxrtEnetLink                       link_;
    rosette::platform::Mutex            tx_mutex_;
};

}  // namespace rosette::net::imxrt
