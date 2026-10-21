/**
 * @file rtps/transport.hpp
 * @brief RTPS UDP transport abstraction backed by the shared socket module.
 *
 * The RTPS stack only needs a small subset of UDP/IP behavior: open the
 * metatraffic, SPDP-multicast, and user-data sockets, send datagrams, and
 * poll for received datagrams. This header keeps the RTPS-facing API
 * stable while routing all socket operations through the backend-agnostic
 * socket layer in `net/socket.hpp`.
 */

#pragma once

#include <cstring>
#include <cstdint>
#include <memory>

#include "buf_pool.hpp"
#include "types.hpp"
#include "rosette/net/socket.hpp"

namespace rosette::rtps {

class RTPSParticipant;

struct ITransport {
    virtual ~ITransport() = default;
    virtual bool send_unicast(const BufRef& buf,
                              const Locator& dest) noexcept = 0;
    virtual bool send_multicast(const BufRef& buf,
                                const Locator& dest) noexcept = 0;
    // Largest RTPS message (one UDP datagram payload) this transport may
    // send. Writers split larger samples into DATA_FRAG submessages so no
    // datagram exceeds it, which keeps traffic free of IP fragmentation on
    // links and stacks that cannot reassemble (WIZnet hardware UDP,
    // FreeRTOS+TCP). 65507 is the IPv4 UDP maximum.
    virtual uint32_t max_datagram_size() const noexcept { return 65507u; }

    // Local ports used for SPDP/SEDP/user-data locator advertisement.
    // Return 0 to indicate "unknown" and let callers apply defaults.
    virtual uint16_t local_metatraffic_unicast_port() const noexcept { return 0u; }
    virtual uint16_t local_default_unicast_port() const noexcept { return 0u; }
};

/// Largest IPv4 UDP payload (65535 - 20 IP - 8 UDP); a transport limited
/// below this sits on a link that cannot deliver IP-fragmented datagrams.
constexpr uint32_t kMaxUdpDatagram = 65507u;

/**
 * @brief True when a reliable reader must request repairs one change at a time.
 *
 * A writer answers an ACKNACK by packing every requested change into as few
 * datagrams as its own message size allows (Fast DDS: 65500 octets). On a
 * link that cannot carry datagrams above `transport.max_datagram_size()`,
 * such a repair datagram is lost again on every retry, so readers there
 * request only the lowest missing change and ask for the next as soon as it
 * arrives; each repair is then a single DATA.
 *
 * @param[in] transport Transport the reader sends and receives on.
 * @return `true` when the transport's datagram limit is below kMaxUdpDatagram.
 *
 * Verification:
 * - Correct means: stock Fast DDS (default message size) completes discovery
 *   with a participant whose link drops datagrams above 1472 octets.
 * - Covered by: TC-SRR-8.4.7-003, TC-SEDP-8.5.4-010,
 *   tests/ros2/run_rmw_frag_loopback.sh (stock peer).
 */
inline bool repairs_must_fit_datagram(const ITransport& transport) noexcept {
    return transport.max_datagram_size() < kMaxUdpDatagram;
}

struct ITransportRuntime {
    virtual ~ITransportRuntime() = default;
    virtual bool start() noexcept = 0;
    virtual void spin_some(Octet*           rx_buffer,
                           uint32_t         rx_buffer_size,
                           RTPSParticipant& participant) noexcept = 0;
};

/**
 * @brief UDP transport over backend-provided datagram sockets.
 *
 * DDSI-RTPS assigns each participant three receive ports (§9.6.1.1):
 * - the SPDP multicast port `PB + DG*domainId + d0`, shared by every
 *   participant in the domain;
 * - its metatraffic unicast port `PB + DG*domainId + d1 + PG*participantId`
 *   (SEDP and unicast SPDP);
 * - its user-data unicast port `PB + DG*domainId + d3 + PG*participantId`.
 *
 * `spdp_socket_` binds the shared multicast port and holds the group
 * membership; the backend must allow several processes to bind it at once
 * (the POSIX provider sets SO_REUSEADDR and SO_REUSEPORT). `meta_socket_`
 * and `user_socket_` bind the participant's unicast ports and are also used
 * to send. When the multicast port equals `meta_port` (a single-participant
 * board that listens for SPDP on its metatraffic socket), no separate
 * multicast socket is opened and the metatraffic socket joins the group.
 *
 * The concrete backend is selected by the supplied `SocketProvider`, so the
 * RTPS code does not know whether packets travel through POSIX sockets,
 * FreeRTOS+TCP, or a WIZnet offload path.
 */
/**
 * @brief Datagram counters for verifying fragmentation behavior.
 *
 * A link without IP fragmentation must never see a datagram above the
 * configured maximum in either direction, and a receive buffer sized to that
 * maximum must never truncate. These counters make that checkable on the
 * host and on hardware.
 */
struct TransportStats {
    uint32_t tx_datagrams = 0;
    uint32_t rx_datagrams = 0;
    uint32_t max_tx_datagram = 0;   ///< Largest datagram sent, in octets.
    uint32_t max_rx_datagram = 0;   ///< Largest datagram received (real size).
    uint32_t oversize_tx = 0;       ///< Datagrams sent above max_datagram_size().
    uint32_t oversize_rx = 0;       ///< Datagrams received above max_datagram_size()
                                    ///< (a peer not capped to this link's limit).
    uint32_t truncated_rx = 0;      ///< Datagrams larger than the receive buffer.
    uint32_t dropped_rx = 0;        ///< Oversize datagrams discarded by strict-link mode.
};

class SocketUdpTransport : public ITransport {
public:
    /**
     * @brief Creates an IPv4 transport; no socket is opened until open().
     *
     * @param[in] provider       Socket backend; must outlive the transport.
     * @param[in] meta_port      Metatraffic unicast port to bind.
     * @param[in] user_port      User-data unicast port to bind.
     * @param[in] multicast_addr SPDP multicast group, host-order IPv4
     *   (0xEFFF0001 for 239.255.0.1).
     * @param[in] multicast_port SPDP multicast port for the domain.
     */
    SocketUdpTransport(rosette::net::SocketProvider& provider,
                       uint16_t meta_port,
                       uint16_t user_port,
                       uint32_t multicast_addr,
                       uint16_t multicast_port) noexcept;

    /**
     * @brief Creates a transport whose SPDP group and port come from a locator.
     *
     * @param[in] provider          Socket backend; must outlive the transport.
     * @param[in] meta_port         Metatraffic unicast port to bind.
     * @param[in] user_port         User-data unicast port to bind.
     * @param[in] multicast_locator SPDP multicast group and port; its kind
     *   (UDPv4 or UDPv6) selects the socket domain.
     */
    SocketUdpTransport(rosette::net::SocketProvider& provider,
                       uint16_t meta_port,
                       uint16_t user_port,
                       const Locator& multicast_locator) noexcept;
    ~SocketUdpTransport() noexcept;

    /**
     * @brief Opens and binds every socket and joins the SPDP group.
     *
     * @return `true` when all sockets are ready; `false` (with everything
     *   closed again) if any socket fails to open, bind, or join. A `false`
     *   from a busy unicast port lets the caller retry with another
     *   participant ID.
     *
     * Verification:
     * - Correct means: a participant receives SPDP multicast sent to its
     *   domain's port from other processes on the same host, and two
     *   transports in one domain can be open at once.
     * - Covered by: tests/ros2/run_rmw_interop_loopback.sh (discovery
     *   against rmw_fastrtps_cpp).
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §9.6.1.1 (default port mapping)
     * - OMG DDSI-RTPS 2.5 §8.5.3 (SPDP)
     */
    bool open() noexcept;
    void close() noexcept;

    /// @brief Receives one metatraffic datagram (unicast first, then SPDP multicast).
    uint32_t poll_meta(Octet*    buf,
                       uint32_t  buf_size,
                       uint32_t& src_addr,
                       uint16_t& src_port) noexcept;

    /// @brief Receives one user-data datagram.
    uint32_t poll_user(Octet*    buf,
                       uint32_t  buf_size,
                       uint32_t& src_addr,
                       uint16_t& src_port) noexcept;

    /**
     * @brief Sleeps until a datagram is readable on any of this transport's
     * sockets, `wake` is signaled, or `timeout_ms` passes.
     *
     * Delegates to SocketProvider::wait_readable(); callers drain with
     * poll_meta()/poll_user() afterwards whatever the reason for waking.
     *
     * @param[in] wake       Event that ends the wait early.
     * @param[in] timeout_ms Longest wait; platform::kWaitForever blocks.
     */
    void wait_readable(rosette::platform::Event& wake, uint32_t timeout_ms) noexcept {
        rosette::net::Socket* const sockets[3] = {
            meta_socket_.get(), spdp_socket_.get(), user_socket_.get()};
        provider_.wait_readable(sockets, 3u, wake, timeout_ms);
    }

    bool send_unicast(const BufRef& buf,
                      const Locator& dest) noexcept override;
    bool send_multicast(const BufRef& buf,
                        const Locator& dest) noexcept override;
    uint16_t local_metatraffic_unicast_port() const noexcept override { return meta_port_; }
    uint16_t local_default_unicast_port() const noexcept override { return user_port_; }

    /// @brief Largest datagram writers may send; see ITransport.
    uint32_t max_datagram_size() const noexcept override { return max_datagram_; }

    /**
     * @brief Sets the datagram limit, typically the link MTU minus IP and
     * UDP headers (1472 for Ethernet) on a stack without IP fragmentation.
     *
     * @param[in] octets Largest UDP payload to send; values above 65507 and
     *   below 512 are clamped into that range.
     */
    void set_max_datagram_size(uint32_t octets) noexcept {
        max_datagram_ = octets > 65507u ? 65507u : (octets < 512u ? 512u : octets);
    }

    /**
     * @brief Makes the host behave like a link that cannot carry datagrams
     * above max_datagram_size().
     *
     * A WIZnet hardware-UDP socket, or any stack without IP reassembly, never
     * delivers a datagram whose IP packet was fragmented on the wire: it is
     * lost without trace. With strict-link mode on, receive discards every
     * datagram longer than max_datagram_size() (counting it in
     * TransportStats::dropped_rx) so a host run against an unmodified peer
     * faces exactly the losses such a board would, and the recovery paths
     * (NACK of the lost sequence numbers, one-change repairs) are exercised
     * with real stock-middleware traffic.
     *
     * @param[in] enabled `true` to discard oversize datagrams on receive.
     *
     * Verification:
     * - Correct means: no datagram above the limit reaches the participant;
     *   discovery and every sample the peer can send within the limit still
     *   arrive from stock Fast DDS with its default (65500) message size.
     * - Covered by: tests/ros2/run_rmw_frag_loopback.sh (stock peer mode).
     */
    void set_strict_link(bool enabled) noexcept { strict_link_ = enabled; }

    /// @brief Datagram counters since open().
    const TransportStats& stats() const noexcept { return stats_; }

private:
    rosette::net::Socket* socket_for(const Locator& dest) const noexcept;
    uint32_t receive_from(rosette::net::Socket* socket,
                                 Octet*    buf,
                                 uint32_t  buf_size,
                                 uint32_t& src_addr,
                                 uint16_t& src_port) noexcept;
    static rosette::net::SocketAddress locator_to_socket_address(
        const Locator& dest) noexcept;
    static rosette::net::SocketMulticastMembership locator_to_multicast_membership(
        const Locator& dest) noexcept;
    rosette::net::SocketAddress any_bind_address(uint16_t port) const noexcept;
    /**
     * @brief Opens a socket, joins the multicast group when asked, and binds `port`.
     *
     * @param[in] port          Local port to bind.
     * @param[in] join_group    Join the SPDP multicast group first.
     * @param[in] receive_bytes Receive buffering to ask for before bind()
     *   (Socket::set_receive_buffer_size()); 0 asks for none, which leaves a
     *   WIZnet socket on the smallest buffer. The user-traffic socket asks for
     *   a whole fragmented sample (RTPS_USER_SOCKET_RECEIVE_BUFFER), the
     *   metatraffic socket for an SEDP burst (RTPS_META_SOCKET_RECEIVE_BUFFER).
     * @return The bound socket, or null when any step fails.
     */
    std::unique_ptr<rosette::net::Socket> open_bound(uint16_t port, bool join_group,
                                                     uint32_t receive_bytes) noexcept;

    rosette::net::SocketProvider& provider_;
    uint16_t meta_port_;
    uint16_t user_port_;
    Locator multicast_locator_{};
    std::unique_ptr<rosette::net::Socket> meta_socket_;
    std::unique_ptr<rosette::net::Socket> spdp_socket_;
    std::unique_ptr<rosette::net::Socket> user_socket_;
    bool meta_open_ = false;
    bool user_open_ = false;
    uint32_t max_datagram_ = 65507u;
    bool strict_link_ = false;
    TransportStats stats_{};
};

class SocketTransportRuntime : public ITransportRuntime {
public:
    explicit SocketTransportRuntime(SocketUdpTransport& transport) noexcept
        : transport_(transport) {}

    bool start() noexcept override;
    void spin_some(Octet*           rx_buffer,
                   uint32_t         rx_buffer_size,
                   RTPSParticipant& participant) noexcept override;

private:
    SocketUdpTransport& transport_;
};

#ifdef RTPS_TEST_BUILD

class LoopbackTransport : public ITransport {
public:
    static constexpr uint32_t kSlots = 8;
    static constexpr uint32_t kSlotSize = RTPS_BUF_BLOCK_SIZE;

    bool send_unicast(const BufRef& buf, const Locator&) noexcept override
    {
        return enqueue(buf);
    }

    bool send_multicast(const BufRef& buf, const Locator&) noexcept override
    {
        return enqueue(buf);
    }

    uint32_t poll(Octet* dst, uint32_t cap) noexcept
    {
        if (read_ == write_) {
            return 0;
        }

        const uint32_t len = slot_lens_[read_ % kSlots];
        if (len > cap) {
            return 0;
        }

        memcpy(dst, slots_[read_ % kSlots], len);
        ++read_;
        return len;
    }

    bool empty() const noexcept { return read_ == write_; }

private:
    bool enqueue(const BufRef& buf) noexcept
    {
        if (write_ - read_ >= kSlots) {
            return false;
        }

        const uint32_t n = buf.size < kSlotSize ? buf.size : kSlotSize;
        memcpy(slots_[write_ % kSlots], buf.data, n);
        slot_lens_[write_ % kSlots] = n;
        ++write_;
        return true;
    }

    Octet slots_[kSlots][kSlotSize];
    uint32_t slot_lens_[kSlots] = {};
    uint32_t read_ = 0;
    uint32_t write_ = 0;
};

#endif

}  // namespace rosette::rtps
