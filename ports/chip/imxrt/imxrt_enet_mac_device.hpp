#pragma once

/**
 * @file ports/chip/imxrt/imxrt_enet_mac_device.hpp
 * @brief The i.MX RT10xx ENET MAC, with its RMII PHY, as a copying rosette::net::MacDevice.
 *
 * The ENET moves frames between the RMII pins and buffer-descriptor rings in
 * RAM by DMA. This device drives it through the MCUXpresso SDK's fsl_enet
 * driver in copy mode: received frames land in the device's own RX buffers
 * and receive_frame() copies one out; send_frame() copies a frame into a TX
 * buffer and starts the DMA. The rings and buffers are a board-sized
 * ImxrtEnetRings object the board places in DMA-reachable, non-cached RAM
 * (DTCM on these parts, where the SDK's linker scripts put .bss), so no
 * cache maintenance is needed. ImxrtEnetZeroCopyDevice
 * (imxrt_enet_zero_copy_device.hpp) is the same ENET without the buffers.
 *
 * Bring-up, the PHY, the multicast filter, and the receive interrupt are
 * ImxrtEnetLink's (imxrt_enet_link.hpp); this class adds the copying data
 * path and forwards the rest.
 *
 * References: i.MX RT1020 Reference Manual, ENET chapter; MCUXpresso SDK
 * fsl_enet.h 2.9.1 ENET_SendFrame(), ENET_ReadFrame().
 */

#include <cstddef>
#include <cstdint>

#include "fsl_enet.h"
#include "imxrt_enet_link.hpp"
#include "rosette/net/mac.hpp"
#include "rosette/platform/platform.hpp"

namespace rosette::net::imxrt {

/// Bytes of one RX or TX buffer: a VLAN-tagged frame with its FCS, rounded up to
/// ENET_BUFF_ALIGNMENT.
inline constexpr std::size_t kEnetBufferBytes =
    SDK_SIZEALIGN(ENET_FRAME_MAX_FRAMELEN + 4u, ENET_BUFF_ALIGNMENT);

/**
 * @brief The descriptor rings and frame buffers the ENET reads and writes by DMA.
 *
 * Sized by the board: each RX buffer holds one received frame until the
 * receive task copies it out, and each TX buffer one frame until the wire
 * has taken it. At 100 Mbit/s a full-size frame takes 120 µs on the wire.
 *
 * @tparam RxCount Receive descriptors and buffers; at least 2.
 * @tparam TxCount Transmit descriptors and buffers; at least 1.
 */
template <std::size_t RxCount, std::size_t TxCount>
struct ImxrtEnetRings {
    static_assert(RxCount >= 2u && TxCount >= 1u, "ImxrtEnetRings: too few buffers");

    alignas(ENET_BUFF_ALIGNMENT) enet_rx_bd_struct_t rx_descriptors[RxCount];
    alignas(ENET_BUFF_ALIGNMENT) enet_tx_bd_struct_t tx_descriptors[TxCount];
    alignas(ENET_BUFF_ALIGNMENT) uint8_t rx_buffers[RxCount][kEnetBufferBytes];
    alignas(ENET_BUFF_ALIGNMENT) uint8_t tx_buffers[TxCount][kEnetBufferBytes];

    /// @brief fsl_enet's description of these rings.
    enet_buffer_config_t buffer_config() noexcept {
        enet_buffer_config_t config{};
        config.rxBdNumber = static_cast<uint16_t>(RxCount);
        config.txBdNumber = static_cast<uint16_t>(TxCount);
        config.rxBuffSizeAlign = static_cast<uint16_t>(kEnetBufferBytes);
        config.txBuffSizeAlign = static_cast<uint16_t>(kEnetBufferBytes);
        config.rxBdStartAddrAlign = rx_descriptors;
        config.txBdStartAddrAlign = tx_descriptors;
        config.rxBufferAlign = &rx_buffers[0][0];
        config.txBufferAlign = &tx_buffers[0][0];
        config.rxMaintainEnable = false;
        config.txMaintainEnable = false;
        config.txFrameInfo = nullptr;
        return config;
    }
};

/**
 * @brief The ENET and its PHY behind the net::MacDevice contract.
 *
 * One instance drives the part's one ENET for the program's lifetime,
 * normally owned by the board's FreeRTOS+TCP interface
 * (ports/net/freertos_tcp/interface.hpp). It is neither copied nor moved,
 * because fsl_enet's handle and the interrupt callback hold its address.
 *
 * Concurrency: the receive calls run on the consumer's receive task and
 * send_frame() on the IP task; fsl_enet keeps the two rings independent.
 * Transmits are serialised by one mutex; ImxrtEnetLink serialises the PHY
 * and the multicast filter.
 *
 * Example:
 * @code{.cpp}
 * imxrt::ImxrtEnetRings<4, 2> g_rings;  // .bss: DTCM, DMA-reachable and not cached
 * freertos::NetworkInterface<imxrt::ImxrtEnetMacDevice> g_enet0{
 *     MacAddress{}, g_rings, imxrt::ImxrtEnetConfig{2u, 5u, &configure_ksz8081}};
 * NetworkInterface_t g_descriptor;
 * g_enet0.add_to_freertos(g_descriptor, "enet0");  // initialise() runs in the IP task
 * @endcode
 * Full program: boards/mimxrt1020-evk/enet_copying_interface.cpp.
 */
class ImxrtEnetMacDevice final : public net::MacDevice {
public:
    /**
     * @brief Records the rings and settings; touches no hardware until initialise().
     *
     * @param[in] rings  The board's rings, in DMA-reachable non-cached RAM;
     *   they outlive the device.
     * @param[in] config PHY address, interrupt priority, PHY set-up hook.
     */
    template <std::size_t RxCount, std::size_t TxCount>
    ImxrtEnetMacDevice(ImxrtEnetRings<RxCount, TxCount>& rings,
                       const ImxrtEnetConfig&            config) noexcept
        : buffers_(rings.buffer_config()), link_(config) {}

    ImxrtEnetMacDevice(const ImxrtEnetMacDevice&) = delete;
    ImxrtEnetMacDevice& operator=(const ImxrtEnetMacDevice&) = delete;

    bool        initialise(const MacAddress& mac) noexcept override;
    std::size_t max_frame_size() const noexcept override { return ENET_FRAME_MAX_FRAMELEN - 4u; }
    bool        send_frame(const uint8_t* data, std::size_t length) noexcept override;
    std::size_t receive_frame(uint8_t* buffer, std::size_t capacity) noexcept override;
    std::size_t peek_frame_size() noexcept override;

    void set_mac_address(const MacAddress& mac) noexcept override { link_.set_mac_address(mac); }
    PhyStatus phy_status() noexcept override { return link_.phy_status(); }
    bool      set_receive_event(rosette::platform::Event* event) noexcept override {
        return link_.set_receive_event(event);
    }

    void add_multicast(const MacAddress& group) noexcept override { link_.add_multicast(group); }
    void remove_multicast(const MacAddress& group) noexcept override {
        link_.remove_multicast(group);
    }

    /// @brief The ENET's control half, for diagnostics.
    ImxrtEnetLink& link() noexcept { return link_; }

private:
    enet_buffer_config_t     buffers_;
    ImxrtEnetLink            link_;
    rosette::platform::Mutex tx_mutex_;
};

}  // namespace rosette::net::imxrt
