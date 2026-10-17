#pragma once

/**
 * @file lending_mac.hpp
 * @brief Hardware-neutral contract for an Ethernet MAC that moves frames in the IP stack's own
 * buffers.
 *
 * A MacDevice (rosette/net/mac.hpp) copies: received frames land in the
 * device's storage and are copied into a network buffer, and transmitted
 * frames are copied out of one. A DMA controller can do without that
 * storage. A LendingMacDevice is lent the IP stack's buffers: the receive
 * ring holds empty network buffers that the DMA fills, and a transmit
 * descriptor points at the network buffer the stack built the frame in. The
 * device keeps no frame storage of its own; on a microcontroller with a few
 * hundred KiB of RAM that is the point, since the stack's pool already has
 * room for every frame in flight.
 *
 * Ownership: the consumer owns every buffer. The device holds one between
 * being lent it and handing it back, and identifies it to the consumer by an
 * opaque owner handle the consumer chose (a FreeRTOS+TCP
 * NetworkBufferDescriptor_t*, an lwIP pbuf*). Each lent buffer comes back
 * exactly once: a receive buffer from take_received_frame() with the frame
 * the DMA wrote into it, a transmit buffer from take_sent_frame() once the
 * wire has taken it.
 *
 * The consumer keeps the receive ring full: it lends a fresh buffer for each
 * frame it takes, and when its pool has none it gives the taken buffer back
 * unread, dropping that frame, so the ring never runs empty and reception
 * resumes as soon as the pool recovers.
 *
 * Layout: a DMA engine places frames at an aligned address, and an IP stack
 * wants its IP header word-aligned, 14 octets into the frame. A device that
 * reconciles the two by writing lent_frame_offset() octets of padding first
 * (the i.MX RT ENET's SHIFT16) says so, and every buffer handed across this
 * contract is described by its frame pointer, with the padding in front of
 * it.
 */

#include <cstddef>
#include <cstdint>

#include "rosette/net/mac.hpp"

namespace rosette::net {

/// One frame in a lent buffer.
struct LentFrame {
    /// First octet of the Ethernet header (the destination MAC).
    uint8_t*    data = nullptr;
    /// Octets of frame from @c data, without the FCS.
    std::size_t length = 0u;
    /// The consumer's handle for the buffer, as it was lent.
    void*       owner = nullptr;
};

/**
 * @brief An Ethernet MAC that receives into and transmits from buffers the consumer lends it.
 *
 * Layout rules every lent buffer meets, receive or transmit: the octets from
 * `data - lent_frame_offset()` start at a multiple of lent_buffer_alignment(),
 * and the consumer leaves them alone while the device holds the buffer. A
 * receive buffer additionally provides lent_buffer_bytes() writable octets
 * from that address. Each buffer must be reachable by the device's DMA and
 * kept coherent with it (uncached memory, on a part with a data cache).
 *
 * Threading follows MacLink: the receive calls run on the consumer's receive
 * task, send_lent_frame() on the IP task, and take_sent_frame() on either,
 * so the implementation serialises the two transmit calls.
 *
 * Example:
 * @code{.cpp}
 * LentFrame frame;  // one pass of a FreeRTOS+TCP receive task
 * if (device.take_received_frame(frame)) {
 *     auto* fresh = pxGetNetworkBufferWithDescriptor(ipTOTAL_ETHERNET_FRAME_SIZE, 0);
 *     if (fresh == nullptr) {
 *         device.lend_receive_buffer(frame.data, frame.owner);  // drop it; the ring stays full
 *     } else {
 *         device.lend_receive_buffer(fresh->pucEthernetBuffer, fresh);
 *         auto* received = static_cast<NetworkBufferDescriptor_t*>(frame.owner);
 *         received->xDataLength = frame.length;  // then post it to the IP task
 *     }
 * }
 * @endcode
 * Full program: ports/net/freertos_tcp/lending_interface.hpp with
 * boards/mimxrt1020-evk/enet_lending_interface.cpp.
 */
class LendingMacDevice : public MacLink {
public:
    /// @brief Octets the device writes in front of each frame; the frame pointer is this far into
    /// the aligned region.
    virtual std::size_t lent_frame_offset() const noexcept = 0;

    /// @brief Alignment, in octets, of `data - lent_frame_offset()` for every lent buffer.
    virtual std::size_t lent_buffer_alignment() const noexcept = 0;

    /// @brief Writable octets a receive buffer provides, counted from `data -
    /// lent_frame_offset()`.
    virtual std::size_t lent_buffer_bytes() const noexcept = 0;

    /**
     * @brief Adds one empty buffer to the tail of the receive ring.
     *
     * The consumer lends buffers until the ring is full before the first
     * initialise(), and one for every frame it takes afterwards. A buffer
     * lent before initialise() waits for reception to start.
     *
     * @param[in] data  Where the first octet of a received frame goes; meets
     *   the layout rules above.
     * @param[in] owner The consumer's handle, returned with the frame.
     * @return True when the ring took the buffer; false when the ring is full
     *   or the buffer breaks the layout rules, in which case the consumer
     *   still owns it.
     */
    virtual bool lend_receive_buffer(uint8_t* data, void* owner) noexcept = 0;

    /**
     * @brief Takes the frame at the head of the receive ring, with its buffer.
     *
     * Frames the MAC flagged as errored are not returned: their buffers go
     * back to the tail of the ring and the search continues.
     *
     * @param[out] frame The frame, in the buffer lent with @c frame.owner;
     *   written only on a true return. The consumer owns the buffer again.
     * @return True for one good frame; false when no complete frame is
     *   waiting.
     */
    virtual bool take_received_frame(LentFrame& frame) noexcept = 0;

    /**
     * @brief Queues the frame in a lent buffer for transmission, without copying it.
     *
     * @param[in] frame A frame of 1..max_frame_size() octets in a buffer that
     *   meets the layout rules; the device holds the buffer until
     *   take_sent_frame() returns @c frame.owner.
     * @return True when the frame is queued; false when the transmit ring is
     *   full, the device is not initialised, or the buffer breaks the layout
     *   rules, in which case the consumer still owns it.
     */
    virtual bool send_lent_frame(const LentFrame& frame) noexcept = 0;

    /**
     * @brief Hands back the buffer of the oldest frame the wire has taken.
     *
     * @param[out] owner The handle passed with that frame to
     *   send_lent_frame(); written only on a true return.
     * @return True for one finished frame, in send order; false when every
     *   queued frame is still waiting to go.
     */
    virtual bool take_sent_frame(void*& owner) noexcept = 0;

protected:
    LendingMacDevice() = default;
    LendingMacDevice(const LendingMacDevice&) = default;
    LendingMacDevice(LendingMacDevice&&) = default;
    LendingMacDevice& operator=(const LendingMacDevice&) = default;
    LendingMacDevice& operator=(LendingMacDevice&&) = default;
};

}  // namespace rosette::net
