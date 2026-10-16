#pragma once

/**
 * @file ports/chip/imxrt/imxrt_enet_mac_device.hpp
 * @brief The i.MX RT10xx ENET MAC, with its RMII PHY, as a rosette::net::MacDevice.
 *
 * The ENET moves frames between the RMII pins and buffer-descriptor rings in
 * RAM by DMA. This device drives it through the MCUXpresso SDK's fsl_enet
 * driver in copy mode: received frames land in the device's own RX buffers
 * and receive_frame() copies one out; send_frame() copies a frame into a TX
 * buffer and starts the DMA. The rings and buffers are a board-sized
 * ImxrtEnetRings object the board places in DMA-reachable, non-cached RAM
 * (DTCM on these parts, where the SDK's linker scripts put .bss), so no
 * cache maintenance is needed.
 *
 * Receive notification is the ENET's own interrupt: fsl_enet's handler runs
 * the device's callback for each received frame, which signals the
 * consumer's platform::Event. The PHY is reached over the ENET's MDIO bus
 * with the IEEE 802.3 clause 22 registers every PHY implements; anything
 * vendor-specific (the KSZ8081's 50 MHz RMII clock mode) is the board's, run
 * from ImxrtEnetConfig::configure_phy after the PHY's software reset.
 *
 * What the board does before initialise(): mux the RMII and MDIO pads,
 * supply the RMII reference clock (on the RT1020-EVK, the ENET PLL's 50 MHz
 * output driven to the PHY), and release the PHY from hardware reset.
 *
 * References: i.MX RT1020 Reference Manual, ENET chapter; MCUXpresso SDK
 * fsl_enet.h 2.9.1; IEEE 802.3-2018 §22.2.4 (MII management registers) and
 * Annex 28B.3 (priority resolution).
 */

#include <array>
#include <cstddef>
#include <cstdint>

#include "fsl_enet.h"
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

class ImxrtEnetMacDevice;

/// The board's settings for the ENET and its PHY.
struct ImxrtEnetConfig {
    /// The PHY's MDIO address (its PHYAD strapping).
    uint8_t phy_address = 0u;
    /// NVIC priority of the ENET interrupt. Under an RTOS it must allow the
    /// interrupt to signal an event (FreeRTOS: numerically at least
    /// configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY).
    uint8_t irq_priority = 0u;
    /// Vendor-specific PHY set-up, run after the PHY's software reset and
    /// before auto-negotiation starts; null when the PHY needs none. Returns
    /// false to fail initialise().
    bool (*configure_phy)(ImxrtEnetMacDevice& device) = nullptr;
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
 * Transmits are serialised by one mutex, and MDIO transactions and the
 * multicast filter (phy_status(), the board's configure_phy, add_multicast())
 * by another, since each is reachable from more than one task. The interrupt
 * only signals the receive event.
 *
 * Example:
 * @code{.cpp}
 * imxrt::ImxrtEnetRings<4, 2> g_rings;  // .bss: DTCM, DMA-reachable and not cached
 * freertos::NetworkInterface<imxrt::ImxrtEnetMacDevice> g_enet0{
 *     MacAddress{}, g_rings, imxrt::ImxrtEnetConfig{2u, 5u, &configure_ksz8081}};
 * NetworkInterface_t g_descriptor;
 * g_enet0.add_to_freertos(g_descriptor, "enet0");  // initialise() runs in the IP task
 * @endcode
 * Full program: boards/mimxrt1020-evk/enet_interface.cpp.
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
        : buffers_(rings.buffer_config()), config_(config) {}

    ImxrtEnetMacDevice(const ImxrtEnetMacDevice&) = delete;
    ImxrtEnetMacDevice& operator=(const ImxrtEnetMacDevice&) = delete;

    bool        initialise(const MacAddress& mac) noexcept override;
    void        set_mac_address(const MacAddress& mac) noexcept override;
    std::size_t max_frame_size() const noexcept override { return ENET_FRAME_MAX_FRAMELEN - 4u; }
    bool        send_frame(const uint8_t* data, std::size_t length) noexcept override;
    std::size_t receive_frame(uint8_t* buffer, std::size_t capacity) noexcept override;
    std::size_t peek_frame_size() noexcept override;
    PhyStatus   phy_status() noexcept override;
    bool        set_receive_event(rosette::platform::Event* event) noexcept override;
    void        add_multicast(const MacAddress& group) noexcept override;
    void        remove_multicast(const MacAddress& group) noexcept override;

    /**
     * @brief Reads one clause 22 register of the PHY.
     *
     * For the board's configure_phy hook and diagnostics; valid once
     * initialise() has started the ENET clock.
     *
     * @param[in]  reg   Register number, 0..31.
     * @param[out] value The register's contents; untouched on failure.
     * @return False when the MDIO transaction timed out.
     */
    bool phy_read(uint8_t reg, uint16_t& value) noexcept;

    /// @brief Writes one clause 22 register of the PHY; false when the MDIO transaction timed out.
    bool phy_write(uint8_t reg, uint16_t value) noexcept;

private:
    static void on_enet_event(ENET_Type* base, enet_handle_t* handle, enet_event_t event,
                              enet_frame_info_t* frame_info, void* user_data);

    bool start_enet(const MacAddress& mac) noexcept;
    bool start_phy() noexcept;
    void program_multicast_filter() noexcept;

    enet_handle_t            handle_{};
    enet_buffer_config_t     buffers_;
    ImxrtEnetConfig          config_;
    rosette::platform::Mutex tx_mutex_;
    rosette::platform::Mutex control_mutex_;
    rosette::platform::Event* volatile rx_event_ = nullptr;
    std::array<uint8_t, 64> multicast_refs_{};
    PhyStatus               programmed_{};
    bool                    enet_clocked_ = false;  ///< Registers are accessible.
    bool                    started_ = false;       ///< The ENET and the PHY are up.
};

}  // namespace rosette::net::imxrt
