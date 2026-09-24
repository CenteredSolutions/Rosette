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

namespace rtps {

class RTPSParticipant;

struct ITransport {
    virtual ~ITransport() = default;
    virtual bool send_unicast(const BufRef& buf,
                              const Locator& dest) noexcept = 0;
    virtual bool send_multicast(const BufRef& buf,
                                const Locator& dest) noexcept = 0;
    // Local ports used for SPDP/SEDP/user-data locator advertisement.
    // Return 0 to indicate "unknown" and let callers apply defaults.
    virtual uint16_t local_metatraffic_unicast_port() const noexcept { return 0u; }
    virtual uint16_t local_default_unicast_port() const noexcept { return 0u; }
};

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

private:
    rosette::net::Socket* socket_for(const Locator& dest) const noexcept;
    static uint32_t receive_from(rosette::net::Socket* socket,
                                 Octet*    buf,
                                 uint32_t  buf_size,
                                 uint32_t& src_addr,
                                 uint16_t& src_port) noexcept;
    static rosette::net::SocketAddress locator_to_socket_address(
        const Locator& dest) noexcept;
    static rosette::net::SocketMulticastMembership locator_to_multicast_membership(
        const Locator& dest) noexcept;
    rosette::net::SocketAddress any_bind_address(uint16_t port) const noexcept;
    std::unique_ptr<rosette::net::Socket> open_bound(uint16_t port, bool join_group) noexcept;

    rosette::net::SocketProvider& provider_;
    uint16_t meta_port_;
    uint16_t user_port_;
    Locator multicast_locator_{};
    std::unique_ptr<rosette::net::Socket> meta_socket_;
    std::unique_ptr<rosette::net::Socket> spdp_socket_;
    std::unique_ptr<rosette::net::Socket> user_socket_;
    bool meta_open_ = false;
    bool user_open_ = false;
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

}  // namespace rtps
