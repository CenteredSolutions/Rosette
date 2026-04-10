/**
 * @file rtps/transport.hpp
 * @brief UDP transport abstraction and WizNET W5500 backend.
 *
 * The RTPS spec defines an abstract transport model in §8.7.1
 * (OMG formal/2022-04-01).  The Platform-Specific Model (PSM) for
 * UDP/IP is defined in §9.  This file provides:
 *
 *   ITransport  — the abstract interface used throughout the stack
 *                 (declared in stateful_writer.hpp; restated here with
 *                 receive support added).
 *
 *   WiznetUdpTransport — concrete backend for the WizNET W5500 with
 *                        offloaded UDP (hardware TCP/IP stack on the W5500
 *                        handles framing; the MCU calls socket send/recv).
 *
 * UDP/IPv4 port assignment follows §9.6.1, Table 9.35, p. 254.
 * The transport is NOT responsible for port assignment; the
 * RTPSParticipant computes well-known ports and passes locators.
 *
 * Receive path
 * ------------
 * The W5500 stores received datagrams in its internal RX ring buffer.
 * WiznetUdpTransport::poll() drains up to one datagram per call into
 * a caller-supplied buffer and returns the number of bytes received
 * (0 = nothing available, UINT32_MAX = error).  The caller
 * (RTPSParticipant::parse_received()) does the actual parsing.
 *
 * Multicast
 * ---------
 * §9.3.1.2, p. 186: "UDP multicast is the preferred discovery transport."
 * The W5500 supports multicast send; receive on the SPDP multicast group
 * requires a join, performed in open().
 *
 * Spec references (all OMG formal/2022-04-01)
 * -------------------------------------------
 *   Transport model           — §8.7.1,  p. 192
 *   UDP/IPv4 PSM              — §9.3,    p. 183
 *   Well-known locators       — §9.6.1,  Table 9.35, p. 254
 */

#pragma once

#include "buf_pool.hpp"
#include "types.hpp"

namespace rtps {

class RTPSParticipant;

// ============================================================
// ITransport — thin virtual transport interface.
// Virtual because the backend (WizNET, loopback) varies by deployment.
// Called at most once per DATA send per reader.
// ============================================================

struct ITransport {
    virtual ~ITransport() = default;
    virtual bool send_unicast(const BufRef& buf,
                              const Locator& dest) noexcept = 0;
    virtual bool send_multicast(const BufRef& buf,
                                const Locator& dest) noexcept = 0;
};

// ============================================================
// ITransportRuntime
//
// Companion runtime/lifecycle interface for transports that need
// platform-specific startup and receive polling.
//
// This lives alongside ITransport because socket bring-up and RX
// polling are RTPS transport concerns, not ROS concerns.
// ============================================================

struct ITransportRuntime {
    virtual ~ITransportRuntime() = default;
    virtual bool start() noexcept = 0;
    virtual void spin_some(Octet*            rx_buffer,
                           uint32_t          rx_buffer_size,
                           RTPSParticipant&  participant) noexcept = 0;
};

// ============================================================
// WiznetUdpTransport
//
// Thin wrapper around the WizNET W5500 socket API.
// Uses two sockets:
//   sock_meta_  — metatraffic (SPDP / SEDP / HEARTBEAT / ACKNACK)
//                 bound to the participant's metatraffic unicast port
//                 and joined to the SPDP multicast group.
//   sock_user_  — user data (DATA / DATA_FRAG)
//                 bound to the participant's default unicast port.
//
// The W5500 uses socket numbers 0–7; two sockets are allocated here.
// Adjust RTPS_WIZNET_META_SOCKET and RTPS_WIZNET_USER_SOCKET if the
// application uses other W5500 sockets for non-RTPS purposes.
// ============================================================

#ifndef RTPS_WIZNET_META_SOCKET
#  define RTPS_WIZNET_META_SOCKET 0
#endif
#ifndef RTPS_WIZNET_USER_SOCKET
#  define RTPS_WIZNET_USER_SOCKET 1
#endif

class WiznetUdpTransport : public ITransport {
public:

    // ----------------------------------------------------------
    // Constructor
    //
    // @param meta_port      Metatraffic unicast port (SPDP/SEDP/control).
    // @param user_port      User-data unicast port.
    // @param multicast_addr IPv4 multicast group to join for SPDP RX
    //                       (network byte order, e.g. 0xEFFF0001 for
    //                       239.255.0.1).
    // ----------------------------------------------------------
    WiznetUdpTransport(uint16_t meta_port,
                       uint16_t user_port,
                       uint32_t multicast_addr) noexcept
        : meta_port_(meta_port)
        , user_port_(user_port)
        , multicast_addr_(multicast_addr)
    {}

    // ----------------------------------------------------------
    // open()
    //
    // Open both sockets and join the SPDP multicast group.
    // Must be called once before send/recv.
    //
    // §9.3.1.2, p. 186: join the SPDP multicast group on the
    // metatraffic socket.
    //
    // @returns true on success.
    // ----------------------------------------------------------
    bool open() noexcept;

    // ----------------------------------------------------------
    // close()
    //
    // Close both sockets and leave the multicast group.
    // ----------------------------------------------------------
    void close() noexcept;

    // ----------------------------------------------------------
    // poll_meta() / poll_user()
    //
    // Check for a received datagram on the metatraffic or user-data
    // socket.  Copies at most `buf_size` bytes into `buf`.
    //
    // @param buf       Destination buffer.
    // @param buf_size  Buffer capacity in bytes.
    // @param src_addr  Set to the sender's IPv4 address (network order).
    // @param src_port  Set to the sender's UDP source port.
    // @returns number of bytes received; 0 = nothing available.
    // ----------------------------------------------------------
    uint32_t poll_meta(Octet*    buf,
                       uint32_t  buf_size,
                       uint32_t& src_addr,
                       uint16_t& src_port) noexcept;

    uint32_t poll_user(Octet*    buf,
                       uint32_t  buf_size,
                       uint32_t& src_addr,
                       uint16_t& src_port) noexcept;

    // ----------------------------------------------------------
    // ITransport::send_unicast()
    //
    // Send buf to the unicast locator.
    // Selects the metatraffic socket for ports in the metatraffic
    // range; user socket otherwise.
    //
    // §9.3.1.1, p. 184: UDP unicast send.
    // ----------------------------------------------------------
    bool send_unicast(const BufRef& buf,
                      const Locator& dest) noexcept override;

    // ----------------------------------------------------------
    // ITransport::send_multicast()
    //
    // Send buf to the multicast locator via the metatraffic socket.
    //
    // §9.3.1.2, p. 186: UDP multicast send.
    // ----------------------------------------------------------
    bool send_multicast(const BufRef& buf,
                        const Locator& dest) noexcept override;

private:

    uint16_t meta_port_;
    uint16_t user_port_;
    uint32_t multicast_addr_;   ///< Network byte order.

    uint8_t meta_sock_ = -1;
    uint8_t user_sock_ = -1;

    bool meta_open_ = false;
    bool user_open_ = false;

    /// Determine which socket to use based on the destination port.
    /// Metatraffic ports follow the §9.6.1 formula and tend to be
    /// even-numbered in the 74xx range; user-data ports are odd.
    /// We classify by comparing against the configured user_port_.
    uint8_t socket_for(const Locator& dest) const noexcept {
        const uint16_t port = static_cast<uint16_t>(dest.port);
        return (port == user_port_) ? RTPS_WIZNET_USER_SOCKET
                                    : RTPS_WIZNET_META_SOCKET;
    }
};

class WiznetTransportRuntime : public ITransportRuntime {
public:
    explicit WiznetTransportRuntime(WiznetUdpTransport& transport) noexcept
        : transport_(transport) {}

    bool start() noexcept override;
    void spin_some(Octet*           rx_buffer,
                   uint32_t         rx_buffer_size,
                   RTPSParticipant& participant) noexcept override;

private:
    WiznetUdpTransport& transport_;
};

// ============================================================
// LoopbackTransport
//
// In-process loopback for unit testing: send() copies into an
// internal ring buffer; poll() returns the next queued datagram.
// Not used in production; compiled out by #ifdef RTPS_TEST_BUILD.
// ============================================================

#ifdef RTPS_TEST_BUILD

class LoopbackTransport : public ITransport {
public:

    static constexpr uint32_t kSlots    = 8;
    static constexpr uint32_t kSlotSize = RTPS_BUF_BLOCK_SIZE;

    bool send_unicast(const BufRef& buf, const Locator&) noexcept override {
        return enqueue(buf);
    }
    bool send_multicast(const BufRef& buf, const Locator&) noexcept override {
        return enqueue(buf);
    }

    /// @returns bytes dequeued; 0 if empty.
    uint32_t poll(Octet* dst, uint32_t cap) noexcept {
        if (read_ == write_) return 0;
        const uint32_t len = slot_lens_[read_ % kSlots];
        if (len > cap) return 0;
        memcpy(dst, slots_[read_ % kSlots], len);
        ++read_;
        return len;
    }

    bool empty() const noexcept { return read_ == write_; }

private:
    Octet    slots_[kSlots][kSlotSize];
    uint32_t slot_lens_[kSlots] = {};
    uint32_t read_  = 0;
    uint32_t write_ = 0;

    bool enqueue(const BufRef& buf) noexcept {
        if (write_ - read_ >= kSlots) return false;
        const uint32_t n = buf.size < kSlotSize ? buf.size : kSlotSize;
        memcpy(slots_[write_ % kSlots], buf.data, n);
        slot_lens_[write_ % kSlots] = n;
        ++write_;
        return true;
    }
};

#endif // RTPS_TEST_BUILD

} // namespace rtps
