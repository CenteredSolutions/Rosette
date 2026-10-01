#pragma once

/**
 * @file mac.hpp
 * @brief Hardware-neutral contract for an Ethernet MAC / link device.
 *
 * rosette::net::MacDevice is the contract between a link-layer device and a
 * software IP stack that runs on the microcontroller. Three roles meet here:
 *
 *  - this header defines the contract and nothing else; it names no RTOS,
 *    no IP stack, and no chip;
 *  - a MAC provider implements it against hardware, for example the WIZnet
 *    chip in MACRAW mode (ports/net/wiznet/macraw_device.hpp);
 *  - an IP-stack port consumes it, for example the FreeRTOS+TCP network
 *    interface (ports/net/freertos_tcp/freertos_tcp_interface.hpp), which is
 *    a template over the provider type and owns the provider by value.
 *
 * A board composes the two, e.g.
 * `freertos::NetworkInterface<wiznet::WiznetMacrawDevice<Chip, Bus>>`, and
 * wires the controller's interrupt pin to the consumer. Neither the provider
 * nor the consumer knows the other's concrete type.
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
 * Receive notification: a consumer drains the receive queue with
 * peek_frame_size() and receive_frame() until it is empty, then sleeps until
 * the controller's interrupt line falls (or a poll interval passes, on a
 * board with no line). A provider whose controller has an interrupt line
 * must leave it so that any frame queued after receive_frame() returns
 * produces a fresh falling edge. Which GPIO carries that line is a board
 * fact, so the provider never attaches it.
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
