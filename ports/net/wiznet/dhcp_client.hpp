#pragma once

/**
 * @file ports/wiznet/dhcp_client.hpp
 * @brief A DHCPv4 client that advances one step per `poll()` and never blocks (RFC 2131).
 *
 * `DhcpClient` runs the RFC 2131 client state machine (§4.4, Figure 5) over
 * any `SocketProvider`: INIT → SELECTING → REQUESTING → BOUND, then RENEWING at
 * T1 with a unicast REQUEST to the leasing server, REBINDING at T2 with a
 * broadcast one, and back to INIT when the lease expires or a server NAKs.
 * It knows nothing about the interface: it never writes an address, and it
 * reports what changed as a `DhcpEvent` so its owner (`DhcpAddressProvider`)
 * decides whether and when the lease reaches the hardware. That separation is
 * what lets a board keep an IPv4 link-local address while DHCP is retried in
 * the background, and keep its leased address through a renewal.
 *
 * Only the hardware-socket chips need it: they offload UDP but not DHCP. A
 * board on an IP stack that runs its own DHCP (FreeRTOS+TCP, lwIP) reports
 * that stack's addressing through an observer provider instead
 * (`net/freertos/address_provider.hpp`). The client depends only on the
 * `SocketProvider` interface, not on a chip template, so it is compiled once
 * and tested on the host against a scripted socket.
 *
 * Every wait is a deadline checked against the `now_ms` the caller passes in;
 * the socket is read with a zero receive timeout. A `poll()` therefore costs
 * one non-blocking receive plus, when a timer is due, one send, however long
 * the server takes to answer.
 *
 * The client holds a socket only while it is waiting for a reply: an
 * exchange window opens with the first transmission and closes when a reply
 * settles it or `exchange_timeout_ms` passes. A bound client, and one waiting
 * to retry, holds none, which matters on a chip with four hardware sockets.
 * Inside a window a message is retransmitted after 2 s, then 4 s, 8 s, ...
 * (RFC 2131 §4.1 doubles from 4 s; the first retry comes sooner because a
 * switch port that has just come up often drops the first broadcast). Between
 * RENEWING and REBINDING attempts the client waits half the time remaining
 * to T2 (or to expiry), and at least 60 s, as RFC 2131 §4.4.5 asks.
 *
 * Lease times are tracked in 32-bit milliseconds, so every timer is capped
 * at kMaxTimerS (about 24 days); a longer lease is renewed on that schedule.
 * An infinite lease (RFC 2131 §3.3) is never renewed.
 */

#include <array>
#include <cstdint>
#include <memory>

#include "dhcp_wire.hpp"
#include "rosette/net/socket.hpp"

namespace rosette::net::wiznet {

/// Protocol settings for one `DhcpClient`.
struct DhcpClientConfig {
    uint16_t client_port = 68u;
    uint16_t server_port = 67u;
    /// How long one exchange window waits for a reply before giving up.
    uint32_t exchange_timeout_ms = 10000u;
    /// Wait after a failed INIT exchange before the next DISCOVER.
    uint32_t retry_delay_ms = 15000u;
    /// Host name option (RFC 2132 §3.14), or null; must outlive the client.
    const char* host_name = nullptr;
};

/// The RFC 2131 client state (Figure 5), plus `idle` and the wait before a retried INIT.
enum class DhcpClientState : uint8_t {
    idle = 0u,   ///< Not started, or stopped.
    selecting,   ///< DISCOVER sent; waiting for an OFFER.
    requesting,  ///< REQUEST sent for an offer; waiting for the ACK.
    bound,       ///< Holding a lease; no socket open.
    renewing,    ///< Past T1; unicast REQUESTs to the leasing server.
    rebinding,   ///< Past T2; broadcast REQUESTs to any server.
    init_wait,   ///< An INIT exchange failed or a lease was declined; waiting to retry.
};

/// What one `poll()` changed; the owner acts on it.
enum class DhcpEvent : uint8_t {
    none = 0u,
    bound,    ///< A lease with a new address, mask, or router: apply it.
    renewed,  ///< The held lease was extended unchanged: nothing to apply.
    failed,   ///< An INIT exchange ended without a lease.
    lost,     ///< The held lease expired or was NAKed: remove the address.
};

class DhcpClient {
public:
    /// Timers longer than this many seconds are capped, so they fit signed 32-bit milliseconds.
    static constexpr uint32_t kMaxTimerS = 0x7FFFFFFFu / 1000u;

    /**
     * @brief Creates an idle client.
     *
     * @param[in] sockets  Opens the client's UDP socket for each exchange window;
     *                     must outlive the client.
     * @param[in] mac      The interface MAC, sent as `chaddr` and client identifier.
     * @param[in] config   Ports, timing, and host name.
     */
    DhcpClient(SocketProvider& sockets, const std::array<uint8_t, 6>& mac,
               const DhcpClientConfig& config = {}) noexcept;

    /// @brief Enters INIT; the next `poll()` sends the first DISCOVER.
    void start() noexcept;

    /// @brief Closes any open socket and forgets the lease; the client is idle.
    void stop() noexcept;

    /**
     * @brief Advances the state machine by one step without waiting.
     *
     * Drains replies from the open socket, then acts on whichever timer is
     * due: a retransmission, the end of an exchange window, T1, T2, lease
     * expiry, or the end of an INIT retry wait.
     *
     * @param[in] now_ms  Monotonic milliseconds; successive calls must not go backwards.
     * @return The one change this step made, or `DhcpEvent::none`.
     *
     * Verification:
     * - Correct means: the call returns after at most one send and a
     *   non-blocking receive loop; an ACK in SELECTING/REQUESTING yields
     *   `bound`, an unchanged ACK in RENEWING/REBINDING yields `renewed`, a
     *   window without a lease yields `failed`, and expiry or a NAK of a held
     *   lease yields `lost`.
     * - Covered by: tests/host/dhcp_client_test.cpp (TC-NET-DHCP-001 to TC-NET-DHCP-007).
     *
     * References: RFC 2131 §3.1, §4.1, §4.4.1, §4.4.5, Figure 5.
     */
    DhcpEvent poll(uint32_t now_ms) noexcept;

    /**
     * @brief Declines the held address after a duplicate was detected on it.
     *
     * Broadcasts one DHCPDECLINE naming the address and its server, forgets
     * the lease, and waits at least 10 s before a fresh DISCOVER with a new
     * transaction ID, so the server may offer a different address.
     *
     * @param[in] now_ms  Monotonic milliseconds.
     *
     * Verification:
     * - Correct means: one DECLINE carrying option 50 = the held address and
     *   option 54 = its server, ciaddr zero; no DISCOVER for 10 s.
     * - Covered by: tests/host/dhcp_client_test.cpp (TC-NET-DHCP-005).
     *
     * References: RFC 2131 §3.1 step 5, §4.4.1, Table 5.
     */
    void decline(uint32_t now_ms) noexcept;

    DhcpClientState state() const noexcept { return state_; }

    /// @brief Whether a lease is held (BOUND, RENEWING, or REBINDING).
    bool has_lease() const noexcept {
        return state_ == DhcpClientState::bound || state_ == DhcpClientState::renewing
               || state_ == DhcpClientState::rebinding;
    }

    /// @brief The held lease; meaningful only while `has_lease()`.
    const DhcpLease& lease() const noexcept { return lease_; }

    /// @brief Whether an exchange window currently holds a socket.
    bool socket_open() const noexcept { return socket_ != nullptr; }

private:
    bool open_window(uint32_t now_ms) noexcept;
    void close_window() noexcept;
    void transmit(uint32_t now_ms) noexcept;
    DhcpEvent receive(uint32_t now_ms) noexcept;
    DhcpEvent on_reply(const DhcpServerMessage& reply, uint32_t now_ms) noexcept;
    DhcpEvent on_timers(uint32_t now_ms) noexcept;
    DhcpEvent begin_init(uint32_t now_ms) noexcept;
    DhcpEvent lose_lease(uint32_t now_ms) noexcept;
    void enter_bound(const DhcpLease& lease, uint32_t now_ms) noexcept;
    void schedule_attempt(uint32_t until_ms, uint32_t now_ms) noexcept;
    uint32_t next_xid(uint32_t now_ms) noexcept;

    SocketProvider& sockets_;
    std::array<uint8_t, 6> mac_;
    DhcpClientConfig config_;

    std::unique_ptr<Socket> socket_;
    DhcpClientState state_ = DhcpClientState::idle;
    DhcpLease lease_{};
    DhcpLease offer_{};
    uint32_t xid_ = 0u;
    uint32_t exchanges_ = 0u;
    uint32_t window_end_ms_ = 0u;  ///< End of the open exchange window.
    uint32_t retransmit_ms_ = 0u;  ///< Next retransmission inside the window.
    uint32_t retransmit_gap_ms_ = 0u;
    uint32_t attempt_ms_ = 0u;    ///< Next window: INIT retry, or RENEWING / REBINDING attempt.
    uint32_t requested_ms_ = 0u;  ///< When the REQUEST that earned the lease was first sent.
    uint32_t t1_ms_ = 0u;         ///< Absolute times; `timed_` false for an infinite lease.
    uint32_t t2_ms_ = 0u;
    uint32_t expiry_ms_ = 0u;
    bool timed_ = false;
    bool pending_start_ = false;
};

}  // namespace rosette::net::wiznet
