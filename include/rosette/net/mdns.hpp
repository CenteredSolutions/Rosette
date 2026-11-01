// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

#pragma once

/**
 * @file net/mdns.hpp
 * @brief One-shot multicast-DNS resolver for finding a host or a DNS-SD service.
 *
 * Design goal
 * -----------
 * Discover a peer on the local link without any central DNS and without the
 * peer having to know its own routable address. The resolver asks a single
 * question over multicast and reads the reply, reusing Rosette's socket
 * abstraction so the same code runs on POSIX hosts and on the MCU UDP stack.
 *
 * Why this is preferable to protocol-level agent discovery
 * -------------------------------------------------------
 * A service that reports the transport locators it was configured with will,
 * inside a container, report a bridge or loopback address that no other host
 * can reach. mDNS sidesteps that: the address that matters is the source
 * address of the reply packet, which is by construction routable from the
 * querier. `choose_agent_address()` trusts the packet source first and falls
 * back to an advertised A record only when the source itself is non-routable.
 *
 * Scope
 * -----
 * - IPv4 only, matching the rest of Rosette's locator code.
 * - One-shot queries with the RFC 6762 unicast-response bit set; replies come
 *   back to the querier's ephemeral port. The resolver does not join
 *   224.0.0.251 or listen on port 5353, so it will not see responders that
 *   only ever answer by multicast.
 *
 * References
 * ----------
 * - RFC 6762: Multicast DNS. Group 224.0.0.251, port 5353; sec. 5.4 one-shot
 *   queries and the unicast-response bit; sec. 6 response contents.
 * - RFC 6763: DNS-Based Service Discovery. An instance resolves through PTR
 *   then SRV (port + host) then A (address).
 */

#include "rosette/net/dns_wire.hpp"
#include "rosette/net/inet.h"
#include "rosette/net/socket.hpp"

#include <stddef.h>
#include <stdint.h>

#include <array>
#include <cstring>

namespace rosette::net {

/// mDNS link-local IPv4 group and port (RFC 6762 sec. 3).
inline constexpr std::array<uint8_t, 4> kMdnsGroupV4 = {224u, 0u, 0u, 251u};
inline constexpr uint16_t kMdnsPort = 5353u;

struct MdnsConfig {
    /// Per-attempt wait for replies to a query.
    uint32_t receive_timeout_ms = 1000u;
    /// Number of times the query is re-sent before giving up.
    uint8_t attempts = 3u;
};

namespace mdns_detail {

/// Reject the unspecified, loopback, link-local, and multicast/reserved ranges.
inline bool is_routable_v4(const std::array<uint8_t, 4>& a) noexcept
{
    if ((a[0] == 0u) || (a[0] == 127u) || (a[0] >= 224u)) {
        return false;
    }
    if ((a[0] == 169u) && (a[1] == 254u)) {
        return false;
    }
    return true;
}

/**
 * @brief Apply the source-first address policy.
 *
 * @param[in]  source          Reply source address.
 * @param[in]  have_advertised Whether @p advertised holds a usable A record.
 * @param[in]  advertised      Advertised A record, if any.
 * @param[out] out             Chosen address.
 * @return True when a routable address was selected.
 */
inline bool choose_agent_address(const std::array<uint8_t, 4>& source,
                                 bool have_advertised,
                                 const std::array<uint8_t, 4>& advertised,
                                 std::array<uint8_t, 4>& out) noexcept
{
    if (is_routable_v4(source)) {
        out = source;
        return true;
    }
    if (have_advertised && is_routable_v4(advertised)) {
        out = advertised;
        return true;
    }
    return false;
}

/**
 * @brief Normalise a caller name into an mDNS question name.
 *
 * A name with no dot gets ".local" appended; anything already qualified is
 * copied verbatim.
 */
inline bool to_mdns_name(const char* name,
                         char* out,
                         size_t out_capacity) noexcept
{
    if ((name == nullptr) || (name[0] == '\0')) {
        return false;
    }
    const size_t length = std::strlen(name);
    const bool qualified = (std::strchr(name, '.') != nullptr);
    const size_t suffix = qualified ? 0u : 6u;  // ".local"
    if ((length + suffix + 1u) > out_capacity) {
        return false;
    }
    std::memcpy(out, name, length);
    if (!qualified) {
        std::memcpy(out + length, ".local", 6u);
    }
    out[length + suffix] = '\0';
    return true;
}

/// The QR bit in the header flags marks a message as a response (RFC 1035).
inline constexpr uint16_t kFlagResponse = 0x8000u;

/**
 * @brief True when @p datagram is a response that answers a question for @p qname.
 *
 * mDNS is a shared bus: the socket sees every responder's announcements and
 * every other querier's replies. A datagram counts as ours only when it is a
 * response, carries at least one answer, and names @p qname as a record owner
 * (the service type for a PTR browse, the host name for an A query).
 */
inline bool response_answers(const uint8_t* datagram,
                             size_t length,
                             const char* qname) noexcept
{
    dns::Reader reader(datagram, length);
    dns::Header header{};
    if (!reader.read_header(header) ||
        ((header.flags & kFlagResponse) == 0u) ||
        (header.answer_count == 0u) ||
        !reader.skip_questions(header.question_count)) {
        return false;
    }
    for (uint16_t i = 0u; i < header.record_count(); ++i) {
        dns::Record record{};
        if (!reader.read_record(record)) {
            return false;
        }
        if (dns::names_equal(record.owner, qname)) {
            return true;
        }
    }
    return false;
}

/// Find the first A record anywhere in the reply's record sections.
inline bool find_first_a(const uint8_t* datagram,
                         size_t length,
                         std::array<uint8_t, 4>& out) noexcept
{
    dns::Reader reader(datagram, length);
    dns::Header header{};
    if (!reader.read_header(header) ||
        !reader.skip_questions(header.question_count)) {
        return false;
    }
    for (uint16_t i = 0u; i < header.record_count(); ++i) {
        dns::Record record{};
        if (!reader.read_record(record)) {
            return false;
        }
        if (reader.read_a(record, out)) {
            return true;
        }
    }
    return false;
}

/// Find the first SRV record anywhere in the reply's record sections.
inline bool find_first_srv(const uint8_t* datagram,
                           size_t length,
                           dns::Srv& out) noexcept
{
    dns::Reader reader(datagram, length);
    dns::Header header{};
    if (!reader.read_header(header) ||
        !reader.skip_questions(header.question_count)) {
        return false;
    }
    for (uint16_t i = 0u; i < header.record_count(); ++i) {
        dns::Record record{};
        if (!reader.read_record(record)) {
            return false;
        }
        if (reader.read_srv(record, out)) {
            return true;
        }
    }
    return false;
}

}  // namespace mdns_detail

/**
 * @brief Sends mDNS / DNS-SD queries and reports the first usable answer.
 *
 * The class is only the socket exchange; every parsing and policy decision
 * lives in `mdns_detail` as a plain function so it can be read and tested on
 * its own.
 */
class MdnsResolver {
public:
    MdnsResolver(SocketProvider& socket_provider,
                 const MdnsConfig& config = {}) noexcept
        : socket_provider_(socket_provider)
        , config_(config)
    {}

    /**
     * @brief Resolve a host name to an IPv4 address (discovery "by name").
     *
     * Asks for the A record of @p host_name, appending ".local" when the name
     * has no dot. The address returned is the reply's source address when that
     * is routable, otherwise the first routable A record in the reply.
     *
     * @param[in]  host_name    Bare label ("my-agent") or mDNS name
     *                          ("my-agent.local").
     * @param[out] out_address  Resolved IPv4 address in network order.
     * @return True when a routable address was found.
     */
    bool resolve_host(const char* host_name,
                      std::array<uint8_t, 4>& out_address) noexcept
    {
        char qname[dns::kMaxNameLength] = {};
        if (!mdns_detail::to_mdns_name(host_name, qname, sizeof(qname))) {
            return false;
        }

        uint8_t reply[kReplyCapacity] = {};
        size_t reply_length = 0u;
        std::array<uint8_t, 4> source{};
        if (!exchange(qname, dns::kTypeA, reply, sizeof(reply), reply_length,
                      source)) {
            return false;
        }

        std::array<uint8_t, 4> advertised{};
        const bool have_advertised =
            mdns_detail::find_first_a(reply, reply_length, advertised);
        return mdns_detail::choose_agent_address(source, have_advertised,
                                                 advertised, out_address);
    }

    /**
     * @brief Resolve a DNS-SD service type (discovery "by advertisement").
     *
     * Browses @p service_type (for example "_microrosagent._udp.local") and
     * returns the first advertised instance. The port comes from the SRV
     * record; the address follows the same source-first rule as resolve_host().
     *
     * @param[in]  service_type DNS-SD service type, ending in ".local".
     * @param[out] out_address  Resolved IPv4 address in network order.
     * @param[out] out_port     Advertised UDP port, or 0 when the reply carried
     *                          no SRV record.
     * @return True when a routable address was found.
     */
    bool resolve_service(const char* service_type,
                         std::array<uint8_t, 4>& out_address,
                         uint16_t& out_port) noexcept
    {
        out_port = 0u;

        char qname[dns::kMaxNameLength] = {};
        if (!mdns_detail::to_mdns_name(service_type, qname, sizeof(qname))) {
            return false;
        }

        uint8_t reply[kReplyCapacity] = {};
        size_t reply_length = 0u;
        std::array<uint8_t, 4> source{};
        if (!exchange(qname, dns::kTypePtr, reply, sizeof(reply), reply_length,
                      source)) {
            return false;
        }

        dns::Srv srv{};
        if (mdns_detail::find_first_srv(reply, reply_length, srv)) {
            out_port = srv.port;
        }

        std::array<uint8_t, 4> advertised{};
        const bool have_advertised =
            mdns_detail::find_first_a(reply, reply_length, advertised);
        return mdns_detail::choose_agent_address(source, have_advertised,
                                                 advertised, out_address);
    }

private:
    /// Query fits in well under 128 bytes; replies from Avahi are a few hundred.
    static constexpr size_t kQueryCapacity = 128u;
    static constexpr size_t kReplyCapacity = 512u;
    /// Upper bound on unrelated datagrams read while waiting for our answer.
    static constexpr unsigned kMaxDatagramsPerAttempt = 16u;

    /**
     * @brief Send one query and return the first reply that carries answers.
     *
     * Re-sends up to `config_.attempts` times. Within an attempt it keeps
     * reading datagrams until one parses as a response with at least one answer
     * record, or the receive times out.
     *
     * @param[in]  qname          Question name to encode.
     * @param[in]  qtype          Question TYPE (A or PTR).
     * @param[out] reply          Buffer for the raw reply datagram.
     * @param[in]  reply_capacity Size of @p reply.
     * @param[out] reply_length   Bytes written to @p reply.
     * @param[out] source         Source IPv4 address of the reply.
     * @return True when a reply with answers was captured.
     */
    bool exchange(const char* qname,
                  uint16_t qtype,
                  uint8_t* reply,
                  size_t reply_capacity,
                  size_t& reply_length,
                  std::array<uint8_t, 4>& source) noexcept
    {
        uint8_t query[kQueryCapacity] = {};
        const size_t query_length =
            dns::QueryBuilder(query, sizeof(query))
                .build(qname, qtype, /*unicast_response=*/true);
        if (query_length == 0u) {
            return false;
        }

        auto socket = socket_provider_.open(SocketDomain::ipv4,
                                            SocketType::datagram,
                                            SocketProtocol::udp);
        if (!socket) {
            return false;
        }

        // Bind an ephemeral port and set the unicast-response bit in the query
        // so the responder answers straight back to this socket (RFC 6762
        // sec. 5.4). This keeps the resolver a plain client: it never has to
        // own port 5353 or contend with a host mDNS daemon for it. The
        // trade-off is that a responder which ignores the bit and only answers
        // by multicast will not be seen.
        if (socket->bind(make_socket_address_v4({0u, 0u, 0u, 0u}, 0u)) != 0) {
            return false;
        }
        if (socket->set_receive_timeout(config_.receive_timeout_ms) != 0) {
            return false;
        }

        const SocketAddress group =
            make_socket_address_v4(kMdnsGroupV4, kMdnsPort);
        const uint8_t attempts = (config_.attempts == 0u) ? 1u : config_.attempts;

        for (uint8_t attempt = 0u; attempt < attempts; ++attempt) {
            if (socket->send_to(query, query_length, group) !=
                static_cast<std::ptrdiff_t>(query_length)) {
                continue;
            }

            // Read datagrams until one answers our question or the wait ends.
            // The cap keeps a chatty network from stalling the attempt loop.
            for (unsigned seen = 0u; seen < kMaxDatagramsPerAttempt; ++seen) {
                SocketAddress remote{};
                const std::ptrdiff_t received =
                    socket->recv_from(reply, reply_capacity, &remote);
                if (received <= 0) {
                    break;  // timed out: fall through to the next attempt
                }
                if (!remote.is_ipv4() ||
                    !mdns_detail::response_answers(
                        reply, static_cast<size_t>(received), qname)) {
                    continue;
                }
                reply_length = static_cast<size_t>(received);
                source = remote.ipv4();
                return true;
            }
        }

        return false;
    }

    SocketProvider& socket_provider_;
    MdnsConfig config_{};
};

}  // namespace rosette::net
