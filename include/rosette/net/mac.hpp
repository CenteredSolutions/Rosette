// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

#pragma once

/**
 * @file mac.hpp
 * @brief Hardware-neutral contract for an Ethernet MAC / link device.
 *
 * rosette::net::MacDevice is the contract between a link-layer device and a
 * software IP stack that runs on the microcontroller; MacLink is its control
 * half, which a zero-copy device (rosette/net/zero_copy_mac.hpp) shares. Three
 * roles meet here:
 *
 *  - this header defines the contract and nothing else; it names no RTOS,
 *    no IP stack, and no chip;
 *  - a MAC provider implements it against hardware, for example the WIZnet
 *    chip in MACRAW mode (ports/net/wiznet/macraw_device.hpp) or the i.MX RT
 *    ENET (ports/chip/imxrt/imxrt_enet_mac_device.hpp);
 *  - an IP-stack port consumes it, for example the FreeRTOS+TCP network
 *    interface (ports/net/freertos_tcp/interface.hpp), which is
 *    a template over the provider type and owns the provider by value.
 *
 * A board composes the two, e.g.
 * `freertos::NetworkInterface<wiznet::WiznetMacrawDevice<Chip, Bus>>`, and
 * wires a SPI controller's interrupt pin to the consumer; an on-chip MAC
 * signals the consumer from its own interrupt (set_receive_event()). Neither
 * the provider nor the consumer knows the other's concrete type.
 *
 * The interface mirrors rosette::net::AddressProvider in style: a small set
 * of pure virtual operations and no compile-time capability probing. A
 * consumer that holds a `final` provider by concrete type calls it without
 * vtable dispatch; one that holds a MacDevice reference pays one indirect call,
 * negligible next to the bus transfer behind every frame operation.
 */

#include <array>
#include <cstddef>
#include <cstdint>

namespace rosette::platform {
class Event;
}  // namespace rosette::platform

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
 * @brief The control half of an Ethernet controller: bring-up, address, PHY, wake-up, multicast.
 *
 * Every link-layer device a software IP stack drives has these operations,
 * whatever its data path. MacDevice adds whole-frame copying on top
 * (send_frame(), receive_frame()); ZeroCopyMacDevice (rosette/net/zero_copy_mac.hpp)
 * adds buffer exchange, for a DMA controller that moves frames straight into
 * and out of the IP stack's own buffers. A consumer is written against one of
 * the two, never against MacLink alone.
 *
 * Receive notification: a consumer drains the receive queue until it is
 * empty, then sleeps on a platform::Event until a frame arrives (or a poll
 * interval passes). Two kinds of provider signal it. An on-chip MAC with its
 * own interrupt signals the event from that interrupt (set_receive_event()).
 * A SPI controller cannot: its interrupt line is a GPIO, a board fact, so the
 * board attaches the line to the consumer instead, and the provider must
 * leave the line so that any frame queued after the consumer's last receive
 * produces a fresh falling edge.
 *
 * Multicast: a MAC that filters multicast frames in hardware passes only the
 * groups the consumer has asked for (add_multicast()); one that passes every
 * multicast frame treats those calls as no-ops.
 *
 * Lifetime and threading: a device is constructed once and lives for the
 * duration of the network stack. The receive operations run on the network
 * interface's RX task; the transmit operations run on the IP task and the
 * IGMP output path; phy_status() and the multicast calls run on either.
 * Implementations must make those call sites safe to run concurrently,
 * either lock-free (independent controller sockets or rings) or behind a
 * mutex.
 */
class MacLink {
public:
    virtual ~MacLink() = default;

    /**
     * @brief Bring the controller out of reset and program @p mac.
     *
     * Called before any frame traffic from the consumer's initialise path,
     * and again whenever the IP stack re-initialises its interface (after a
     * link-down, or after an earlier call failed). The first successful call
     * verifies the chip, places it in the mode that presents raw Ethernet
     * frames, and latches the station address; a later call only latches
     * the address, so frames already queued survive.
     *
     * @param[in] mac Station hardware address to program.
     * @return True once the device is ready to carry frames; false on any
     *         probe or configuration failure, leaving the interface down.
     */
    virtual bool initialise(const MacAddress& mac) noexcept = 0;

    /**
     * @brief Update the station hardware address on an initialised controller.
     *
     * Invoked when the IP stack settles its endpoint MAC after the device
     * is already running. Safe to call at any point after initialise().
     *
     * @param[in] mac New station hardware address.
     */
    virtual void set_mac_address(const MacAddress& mac) noexcept = 0;

    /**
     * @brief Largest frame the controller can move, in bytes.
     *
     * Measured from the destination MAC: the Ethernet header plus payload,
     * without the hardware-appended FCS. The network interface rejects
     * longer frames before handing them to the device.
     *
     * @return Maximum frame size in bytes; 0 disables the length check.
     */
    virtual std::size_t max_frame_size() const noexcept = 0;

    /**
     * @brief Current PHY link state, with the MAC following what the PHY negotiated.
     *
     * The consumer calls it on every idle wake of its receive task and
     * whenever the IP stack asks for the link. A MAC whose speed and duplex
     * are programmed separately from the PHY's (an on-chip MAC over RMII)
     * reprograms them here when the negotiated values change, so frames keep
     * flowing after the link partner renegotiates.
     *
     * @return A PhyStatus snapshot for the controller's link.
     */
    virtual PhyStatus phy_status() noexcept = 0;

    /**
     * @brief Hands the device the event to signal when a frame arrives.
     *
     * Called once by the consumer before its receive task starts. A device
     * with its own receive interrupt signals @p event from it
     * (platform::Event::signal_from_isr()) for every frame queued after the
     * consumer last found the queue empty.
     *
     * @param[in] event The consumer's receive event; it outlives the device.
     * @return True when the device signals @p event itself; false when it has
     *   no interrupt of its own, in which case the board attaches the
     *   controller's interrupt line to the consumer, or the consumer polls.
     */
    virtual bool set_receive_event(rosette::platform::Event* event) noexcept = 0;

    /**
     * @brief Accepts frames addressed to one multicast MAC address.
     *
     * The consumer calls it once per group join (IPv4 01:00:5e:..., IPv6
     * 33:33:...), and remove_multicast() once per leave. A hardware hash
     * filter can map two groups to one bucket, so the device counts requests
     * per bucket and closes a bucket only when its last group leaves.
     *
     * @param[in] group A multicast address (I/G bit set).
     */
    virtual void add_multicast(const MacAddress& group) noexcept = 0;

    /**
     * @brief Withdraws one add_multicast() request for @p group.
     *
     * @param[in] group An address passed to add_multicast() earlier.
     */
    virtual void remove_multicast(const MacAddress& group) noexcept = 0;

protected:
    MacLink() = default;
    MacLink(const MacLink&) = default;
    MacLink(MacLink&&) = default;
    MacLink& operator=(const MacLink&) = default;
    MacLink& operator=(MacLink&&) = default;
};

/**
 * @brief An Ethernet MAC whose frames are copied in and out: framed transmit/receive plus link
 * state.
 *
 * A MacDevice sits directly above the register interface of an Ethernet
 * controller and exposes whole-frame send and receive to the IP stack. It
 * owns the controller's data path and its frame storage (on-chip SRAM for a
 * SPI controller, driver-owned DMA buffers for an on-chip MAC): callers hand
 * down frames to copy and receive copies of frames.
 *
 * A consumer drains the receive queue with peek_frame_size() and
 * receive_frame() until it is empty, then sleeps as MacLink describes.
 */
class MacDevice : public MacLink {
public:
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
     * Drives the consumer's receive loop: a non-zero result is the size of
     * the buffer the consumer allocates and hands to receive_frame(); zero
     * means the consumer sleeps until the next receive notification. The
     * result is the head frame's own length, never the total of several
     * queued frames, because a consumer with fixed-size network buffers
     * cannot allocate more than max_frame_size().
     *
     * @return Byte length of the next frame, at most max_frame_size() when
     *         that is non-zero, or 0 when the RX queue is empty.
     */
    virtual std::size_t peek_frame_size() noexcept = 0;

protected:
    MacDevice() = default;
    MacDevice(const MacDevice&) = default;
    MacDevice(MacDevice&&) = default;
    MacDevice& operator=(const MacDevice&) = default;
    MacDevice& operator=(MacDevice&&) = default;
};

}  // namespace rosette::net
