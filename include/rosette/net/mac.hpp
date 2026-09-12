#pragma once

/**
 * @file mac.hpp
 * @brief Hardware-neutral contract for an Ethernet MAC / link device.
 *
 * rosette::net::MacDevice is the abstract interface that the FreeRTOS+TCP
 * network interface (rosette::net::freertos::NetworkInterface) and any other
 * link-layer consumer program against. A concrete driver -- today the WIZnet
 * MACRAW device -- derives from it and implements the data-plane and PHY
 * status operations against real hardware.
 *
 * The interface mirrors rosette::net::AddressProvider in style: a small set
 * of pure virtual operations, no templates, no compile-time capability
 * probing. Consumers hold a MacDevice reference and call it directly; the
 * vtable indirection is negligible next to the SPI burst that backs every
 * frame operation.
 */

#include <array>
#include <cstddef>
#include <cstdint>

namespace rosette::net {

/// Six-octet IEEE 802 hardware address, transmitted most-significant byte first.
using MacAddress = std::array<uint8_t, 6>;

enum class PhySpeed : uint8_t {
    unknown = 0u,
    mbps10,
    mbps100,
    mbps1000,
};

enum class PhyDuplex : uint8_t {
    unknown = 0u,
    half,
    full,
};

/**
 * @brief Snapshot of the PHY's negotiated link state.
 *
 * Implementations that only track carrier presence populate @c link_up and
 * leave the negotiated fields at their unknown defaults.
 */
struct PhyStatus {
    bool link_up = false;
    bool auto_negotiation_complete = false;
    PhySpeed speed = PhySpeed::unknown;
    PhyDuplex duplex = PhyDuplex::unknown;
};

/**
 * @brief Abstract Ethernet MAC device: framed transmit/receive plus link state.
 *
 * A MacDevice sits directly above the register interface of an Ethernet
 * controller and exposes whole-frame send and receive to the IP stack. It
 * owns the controller's data path: callers never touch chip registers, they
 * hand down frames and query link state.
 *
 * Lifetime and threading: a MacDevice is constructed once and lives for the
 * duration of the network stack. receive_frame() and peek_frame_size() run
 * on the network interface's RX task; send_frame() runs on the IP task and
 * the IGMP output path. Implementations must make those two call sites safe
 * to run concurrently, either lock-free (independent controller sockets) or
 * behind the bus mutex.
 */
class MacDevice {
public:
    virtual ~MacDevice() = default;

    /**
     * @brief Bring the controller out of reset and program @p mac.
     *
     * Called once, before any frame traffic, from the network interface's
     * initialise path. Implementations verify the chip, place it in the
     * mode that presents raw Ethernet frames, and latch the station address.
     *
     * @param[in] mac Station hardware address to program.
     * @return True once the device is ready to carry frames; false on any
     *         probe or configuration failure, leaving the interface down.
     */
    virtual bool initialise(const MacAddress& mac) noexcept = 0;

    /**
     * @brief Update the station hardware address on an initialised controller.
     *
     * Invoked when FreeRTOS+TCP resolves its endpoint MAC after the device
     * is already running. Safe to call at any point after initialise().
     *
     * @param[in] mac New station hardware address.
     */
    virtual void set_mac_address(const MacAddress& mac) noexcept = 0;

    /**
     * @brief Largest frame the controller can move, in bytes.
     *
     * Measured as the buffer handed to send_frame(): the Ethernet header
     * plus payload, without the hardware-appended FCS. The network interface
     * rejects oversized transmit descriptors before calling send_frame().
     *
     * @return Maximum frame size in bytes; 0 disables the length check.
     */
    virtual std::size_t max_frame_size() const noexcept = 0;

    /**
     * @brief Transmit one complete Ethernet frame.
     *
     * @param[in] data   Frame bytes, starting at the destination MAC.
     * @param[in] length Frame length in bytes; the caller guarantees it is
     *                   non-zero and within max_frame_size().
     * @return True if the frame was handed to the controller for
     *         transmission; false on a bus error or an exhausted TX buffer.
     */
    virtual bool send_frame(const uint8_t* data, std::size_t length) noexcept = 0;

    /**
     * @brief Copy the frame at the head of the RX queue into @p buffer.
     *
     * Call after peek_frame_size() reports a non-zero size. Consumes exactly
     * one frame from the controller's receive queue.
     *
     * @param[out] buffer   Destination for the frame bytes.
     * @param[in]  capacity Size of @p buffer; at least the most recent
     *                      peek_frame_size() result.
     * @return Number of bytes written to @p buffer, or 0 if no frame was
     *         available or it did not fit.
     */
    virtual std::size_t receive_frame(uint8_t* buffer, std::size_t capacity) noexcept = 0;

    /**
     * @brief Length of the head-of-queue received frame without consuming it.
     *
     * Drives the RX task loop: a non-zero result means receive_frame() has a
     * frame to hand over; zero means the task blocks until the next RX
     * notification.
     *
     * @return Byte length of the next frame, or 0 when the RX queue is empty.
     */
    virtual std::size_t peek_frame_size() noexcept = 0;

    /**
     * @brief Current PHY link state.
     *
     * @return A PhyStatus snapshot for the controller's link.
     */
    virtual PhyStatus phy_status() noexcept = 0;

protected:
    MacDevice() = default;
    MacDevice(const MacDevice&) = default;
    MacDevice(MacDevice&&) = default;
    MacDevice& operator=(const MacDevice&) = default;
    MacDevice& operator=(MacDevice&&) = default;
};

}  // namespace rosette::net
