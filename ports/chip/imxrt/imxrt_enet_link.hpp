#pragma once

/**
 * @file ports/chip/imxrt/imxrt_enet_link.hpp
 * @brief The i.MX RT10xx ENET's control half: bring-up, the RMII PHY, the multicast filter, the
 * interrupt.
 *
 * Two devices drive the ENET, one per data path, and both are built on this
 * class: ImxrtEnetMacDevice copies frames through buffers it owns
 * (imxrt_enet_mac_device.hpp), and ImxrtEnetZeroCopyDevice moves them in the
 * IP stack's own buffers (imxrt_enet_zero_copy_device.hpp). Everything that
 * does not touch a frame is here, so a board can build either device and
 * measure one against the other: the MCUXpresso SDK's fsl_enet brings the
 * MAC up, the PHY is reached over the ENET's MDIO bus with the IEEE 802.3
 * clause 22 registers every PHY implements, and the GAUR/GALR hash filter is
 * kept from per-bucket reference counts. Anything vendor-specific in the PHY
 * (the KSZ8081's 50 MHz RMII clock mode) is the board's, run from
 * ImxrtEnetConfig::configure_phy after the PHY's software reset.
 *
 * The ENET interrupt only signals the consumer's platform::Event: fsl_enet's
 * handler calls back here for each frame received and, when the device
 * enabled it, each frame sent.
 *
 * What the board does before the first start(): mux the RMII and MDIO pads,
 * supply the RMII reference clock (on the RT1020-EVK, the ENET PLL's 50 MHz
 * output driven to the PHY), and release the PHY from hardware reset.
 *
 * References: i.MX RT1020 Reference Manual, ENET chapter; MCUXpresso SDK
 * fsl_enet.h 2.9.1; IEEE 802.3-2018 §22.2.4 (MII management registers) and
 * Annex 28B.3 (priority resolution).
 */

#include <array>
#include <cstdint>

#include "fsl_enet.h"
#include "rosette/net/mac.hpp"
#include "rosette/platform/platform.hpp"

namespace rosette::net::imxrt {

class ImxrtEnetLink;

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
    /// false to fail the device's initialise().
    bool (*configure_phy)(ImxrtEnetLink& link) = nullptr;
};

/// What a device asks of the ENET beyond the board's settings.
struct ImxrtEnetMode {
    /// fsl_enet interrupt sources (enet_interrupt_enable_t); each wakes the consumer.
    uint32_t interrupts = kENET_RxFrameInterrupt;
    /// RACC accelerators (enet_rx_accelerator_t), e.g. SHIFT16.
    uint8_t  rx_accelerators = 0u;
    /// TACC accelerators (enet_tx_accelerator_t), e.g. SHIFT16.
    uint8_t  tx_accelerators = 0u;
};

/**
 * @brief The ENET's MAC and PHY control, shared by the copying and the zero-copy device.
 *
 * Owned by value by one device for the program's lifetime; neither copied
 * nor moved, because fsl_enet's handle and the interrupt callback hold its
 * address. Bring-up is two calls so a device can prepare its rings between
 * them: start() configures the MAC, and start_receiving() lets the receive
 * DMA run and brings the PHY up.
 *
 * Concurrency: MDIO transactions and the multicast filter (phy_status(), the
 * board's configure_phy, add_multicast()) are reachable from more than one
 * task, so one mutex serialises them. The interrupt touches nothing but the
 * receive event.
 */
class ImxrtEnetLink {
public:
    /// @brief Records the board's settings; touches no hardware until start().
    explicit ImxrtEnetLink(const ImxrtEnetConfig& config) noexcept : config_(config) {}

    ImxrtEnetLink(const ImxrtEnetLink&) = delete;
    ImxrtEnetLink& operator=(const ImxrtEnetLink&) = delete;

    /**
     * @brief Ungates and configures the ENET over @p buffers; reception stays off.
     *
     * @param[in] mac     Station address for the MAC's address filter.
     * @param[in] buffers The device's rings, in fsl_enet's terms; null data
     *   buffers leave the descriptors to the device.
     * @param[in] mode    Interrupts and accelerators the device needs.
     * @return False when fsl_enet rejects the configuration.
     */
    bool start(const MacAddress& mac, const enet_buffer_config_t& buffers,
               const ImxrtEnetMode& mode) noexcept;

    /**
     * @brief Lets the receive DMA run, then resets the PHY and starts auto-negotiation.
     *
     * @return False when the PHY does not answer on MDIO or the board's
     *   configure_phy fails; the ENET is shut down again so the next start()
     *   begins from reset.
     */
    bool start_receiving() noexcept;

    /// @brief True once start_receiving() has succeeded: the ENET and the PHY are up.
    bool started() const noexcept { return started_; }

    /// @brief Reprograms the station address; a no-op before started().
    void set_mac_address(const MacAddress& mac) noexcept;

    /// @brief MacLink::phy_status(): the PHY's link, with the MAC moved to the negotiated mode.
    PhyStatus phy_status() noexcept;

    /// @brief MacLink::set_receive_event(): the event the ENET interrupt signals.
    bool set_receive_event(rosette::platform::Event* event) noexcept;

    /// @brief MacLink::add_multicast(): opens @p group's hash bucket.
    void add_multicast(const MacAddress& group) noexcept;

    /// @brief MacLink::remove_multicast(): closes the bucket when its last group leaves.
    void remove_multicast(const MacAddress& group) noexcept;

    /**
     * @brief Reads one clause 22 register of the PHY.
     *
     * For the board's configure_phy hook and diagnostics; valid once start()
     * has ungated the ENET clock.
     *
     * @param[in]  reg   Register number, 0..31.
     * @param[out] value The register's contents; untouched on failure.
     * @return False when the MDIO transaction timed out.
     */
    bool phy_read(uint8_t reg, uint16_t& value) noexcept;

    /// @brief Writes one clause 22 register of the PHY; false when the MDIO transaction timed out.
    bool phy_write(uint8_t reg, uint16_t value) noexcept;

    /// @brief fsl_enet's handle, for a device's frame calls.
    enet_handle_t& handle() noexcept { return handle_; }

private:
    static void on_enet_event(ENET_Type* base, enet_handle_t* handle, enet_event_t event,
                              enet_frame_info_t* frame_info, void* user_data);

    bool start_phy() noexcept;
    void program_multicast_filter() noexcept;

    enet_handle_t            handle_{};
    ImxrtEnetConfig          config_;
    rosette::platform::Mutex control_mutex_;
    rosette::platform::Event* volatile rx_event_ = nullptr;
    std::array<uint8_t, 64> multicast_refs_{};
    PhyStatus               programmed_{};
    bool                    enet_clocked_ = false;  ///< Registers are accessible.
    bool                    started_ = false;       ///< The ENET and the PHY are up.
};

}  // namespace rosette::net::imxrt
