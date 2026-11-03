// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

#pragma once

/**
 * @file ports/net/wiznet/dhcp_wire.hpp
 * @brief DHCPv4 message encoding and decoding for a client (RFC 2131, RFC 2132).
 *
 * The DHCP client (`dhcp_client.hpp`) sends four messages and reads three.
 * This module is the byte layer between it and a UDP socket: it fills a
 * BOOTP header and the options RFC 2131 Table 5 requires for each client
 * message, and it checks a server reply against the client's transaction ID
 * and hardware address before extracting the lease it carries. It holds no
 * state and touches no socket, so both directions are tested byte for byte
 * on the host.
 *
 * The functions are defined in src/net/wiznet/dhcp_wire.cpp.
 */

#include <array>
#include <cstddef>
#include <cstdint>

namespace rosette::net::wiznet {

/// Largest DHCP message a client must accept: 576-byte IP datagram less IP and UDP headers.
inline constexpr std::size_t kDhcpMaxMessageBytes = 548u;

/// Lease time meaning "never expires" (RFC 2131 §3.3).
inline constexpr uint32_t kDhcpInfiniteLease = 0xFFFFFFFFu;

/// DHCP message type, option 53 (RFC 2132 §9.6).
enum class DhcpMessageType : uint8_t {
    none = 0u,
    discover = 1u,
    offer = 2u,
    request = 3u,
    decline = 4u,
    ack = 5u,
    nak = 6u,
    release = 7u,
};

/// The IPv4 configuration a server grants, as decoded from an OFFER or ACK.
struct DhcpLease {
    std::array<uint8_t, 4> ip{};                  ///< yiaddr.
    std::array<uint8_t, 4> subnet{};              ///< Option 1.
    std::array<uint8_t, 4> gateway{};             ///< First router of option 3.
    std::array<uint8_t, 4> server_id{};           ///< Option 54.
    std::array<std::array<uint8_t, 4>, 3> dns{};  ///< Option 6.
    std::array<std::array<uint8_t, 4>, 3> ntp{};  ///< Option 42.
    uint8_t dns_count = 0u;
    uint8_t ntp_count = 0u;
    uint32_t lease_s = 0u;  ///< Option 51, seconds; kDhcpInfiniteLease for no expiry.
    uint32_t t1_s = 0u;     ///< Option 58, or 0.5 × lease_s.
    uint32_t t2_s = 0u;     ///< Option 59, or 0.875 × lease_s.

    /// Whether the address, mask, and router match @p other (the parts an interface holds).
    bool same_config(const DhcpLease& other) const noexcept {
        return ip == other.ip && subnet == other.subnet && gateway == other.gateway;
    }
};

/// The fields of one client message; which are sent depends on `type` and `ciaddr`.
struct DhcpClientMessage {
    DhcpMessageType type = DhcpMessageType::discover;
    uint32_t xid = 0u;
    std::array<uint8_t, 6> mac{};
    std::array<uint8_t, 4> ciaddr{};        ///< Held address (RENEWING / REBINDING), else zero.
    std::array<uint8_t, 4> requested_ip{};  ///< Option 50 when nonzero.
    std::array<uint8_t, 4> server_id{};     ///< Option 54 when nonzero.
    const char* host_name = nullptr;        ///< Option 12 when non-null; at most 32 bytes sent.
};

/// A server reply that matched the client's transaction.
struct DhcpServerMessage {
    DhcpMessageType type = DhcpMessageType::none;
    DhcpLease lease{};
};

/**
 * @brief Encodes one client message into @p out.
 *
 * Writes the BOOTP request header with the client hardware address in
 * `chaddr` and the magic cookie, then the message type, the client
 * identifier (hardware type 1 and the MAC, RFC 2132 §9.14), the requested
 * address and server identifier when given, the parameter request list, and
 * the host name. The BROADCAST flag is set exactly when `ciaddr` is zero: a
 * client without a configured address cannot receive a unicast reply to the
 * address it is being offered, and one that holds `ciaddr` can.
 *
 * @param[in]  message  What to send; `mac` must be the interface's MAC.
 * @param[out] out      Buffer of at least kDhcpMaxMessageBytes bytes.
 * @return The number of bytes written.
 *
 * Verification:
 * - Correct means: every field and option RFC 2131 Table 5 lists as MUST for
 *   the message type and client state is present, and every MUST NOT is
 *   absent; the flag word is 0x8000 without ciaddr and 0 with it.
 * - Covered by: tests/host/dhcp_client_test.cpp (TC-NET-DHCP-001, TC-NET-DHCP-006).
 *
 * References: RFC 2131 §2, §4.1, Table 5; RFC 2132 §3.14, §9.6, §9.8, §9.14.
 */
std::size_t encode_dhcp_message(const DhcpClientMessage& message,
                                std::array<uint8_t, kDhcpMaxMessageBytes>& out) noexcept;

/**
 * @brief Decodes a server reply addressed to this client's transaction.
 *
 * Rejects anything that is not a BOOTREPLY for Ethernet with the expected
 * transaction ID, the client's MAC in `chaddr`, the magic cookie, and a
 * message type. For an ACK, a missing lease time becomes one day and missing
 * T1 / T2 become 0.5 and 0.875 of the lease (RFC 2131 §4.4.5).
 *
 * @param[in]  data    Received datagram; may be truncated to the buffer size.
 * @param[in]  length  Bytes available at @p data.
 * @param[in]  xid     Transaction ID of the client's current exchange.
 * @param[in]  mac     The client's MAC.
 * @param[out] out     The message type and lease; written only on success.
 * @return `true` when @p data is a reply to this transaction.
 *
 * Verification:
 * - Correct means: a reply with any other xid or chaddr is rejected, and the
 *   options of an OFFER / ACK round-trip into the lease fields.
 * - Covered by: tests/host/dhcp_client_test.cpp (TC-NET-DHCP-001).
 *
 * References: RFC 2131 §2, §3.1, §4.4.5; RFC 2132 §3.3, §3.5, §3.8, §8.3, §9.2, §9.7, §9.11, §9.12.
 */
bool decode_dhcp_message(const uint8_t* data, std::size_t length, uint32_t xid,
                         const std::array<uint8_t, 6>& mac, DhcpServerMessage& out) noexcept;

}  // namespace rosette::net::wiznet
