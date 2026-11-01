// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

#pragma once

/**
 * @file net/address_candidate.hpp
 * @brief The self-assigned addresses a board tries, in order, for each address kind.
 *
 * A board that picks its own address (RFC 3927 IPv4 link-local, the IPv6
 * link-local address, a SLAAC global address) needs a candidate that is
 * stable across reboots, so the board comes back on the address its peers
 * already know, and a next candidate for when duplicate detection finds the
 * current one taken. Both needs are met by one rule: the candidate is a
 * deterministic function of the board MAC, the prefix (for IPv6), and a
 * conflict counter that starts at 0 and rises by one per conflict. The
 * providers keep only the counter; the addresses are recomputed from it.
 *
 * The function is 64-bit FNV-1a (`support/fnv1a.hpp`) over a tag byte that
 * separates the address kinds, the counter, and the MAC (and prefix). RFC 7217
 * asks for a keyed pseudo-random function with a secret key, so that addresses
 * do not identify a device across networks; Rosette boards sit on a controlled
 * LAN where that privacy goal does not apply, and the only requirement left is
 * that candidates spread evenly and never repeat for one board. IPv6 counter 0
 * is the modified EUI-64 identifier rather than a hash, so a board's
 * first-choice address can be worked out from the MAC printed on its label.
 *
 * Counters are not persisted: each `begin()` of a provider restarts at 0.
 *
 * The generators are defined in src/net/address_candidate.cpp, so each
 * firmware image carries one copy however many providers call them.
 */

#include <array>
#include <cstddef>
#include <cstdint>

namespace rosette::net {

/// IDGEN_RETRIES (RFC 7217 §6): conflicts after which an IPv6 provider gives up an address.
inline constexpr uint8_t kIdgenRetries = 3u;

/**
 * @brief The modified EUI-64 interface identifier for a 48-bit MAC.
 *
 * Inserts ff:fe between the OUI and the NIC-specific half and inverts the
 * universal/local bit, as RFC 4291 Appendix A specifies.
 *
 * @param[in] mac Board MAC address.
 * @return The eight identifier octets, network byte order.
 *
 * Verification:
 * - Correct means: 00:1c:42:14:87:37 yields 02:1c:42:ff:fe:14:87:37 (the
 *   identifier Linux derives for the same MAC).
 * - Covered by: tests/host/wiznet_slaac_provider_test.cpp (TC-NET-SLAAC-001).
 *
 * References: RFC 4291 §2.5.1 and Appendix A.
 */
inline std::array<uint8_t, 8> modified_eui64(const std::array<uint8_t, 6>& mac) noexcept {
    return {
        static_cast<uint8_t>(mac[0] ^ 0x02u), mac[1], mac[2], 0xFFu, 0xFEu, mac[3], mac[4], mac[5]};
}

/**
 * @brief Whether an IPv6 interface identifier is reserved and must not be self-assigned.
 *
 * RFC 7217 §5 requires a generated identifier that falls in a reserved range
 * to be discarded. The reserved ranges are the subnet-router anycast
 * identifier (all zero), the reserved subnet anycast identifiers, and the
 * IANA-reserved 0200:5eff:fe00:0000 through 0200:5eff:feff:ffff block, which
 * covers the Proxy Mobile IPv6 identifier.
 *
 * @param[in] iid Interface identifier, network byte order.
 * @return `true` when @p iid is reserved.
 *
 * Verification:
 * - Correct means: exactly the ranges of the RFC 5453 §3 registry are
 *   rejected.
 * - Covered by: tests/host/address_candidate_test.cpp (TC-NET-CAND-003).
 *
 * References: RFC 5453 §3; RFC 2526 §2; RFC 4291 §2.6.1; RFC 7217 §5.
 */
inline bool is_reserved_interface_id(const std::array<uint8_t, 8>& iid) noexcept {
    const bool all_zero = iid == std::array<uint8_t, 8>{};
    const bool subnet_anycast = iid[0] == 0xFDu && iid[1] == 0xFFu && iid[2] == 0xFFu
                                && iid[3] == 0xFFu && iid[4] == 0xFFu && iid[5] == 0xFFu
                                && iid[6] == 0xFFu && iid[7] >= 0x80u;
    const bool iana_block =
        iid[0] == 0x02u && iid[1] == 0x00u && iid[2] == 0x5Eu && iid[3] == 0xFFu && iid[4] == 0xFEu;
    return all_zero || subnet_anycast || iana_block;
}

/**
 * @brief The self-assigned IPv6 address to try after @p conflicts duplicates under @p prefix.
 *
 * Used for both the link-local address (prefix fe80::/64) and each SLAAC
 * global address. Counter 0 yields the modified EUI-64 identifier. Every
 * other counter, and counter 0 when the EUI-64 identifier is reserved, yields
 * the first non-reserved FNV-1a hash of (tag, counter, salt, prefix, MAC) over
 * salts 0, 1, 2, ..., which plays the part of RFC 7217's function F with the
 * counter as DAD_Counter. Inputs that differ only in the counter byte hash to
 * different identifiers, and the bytes after the counter spread the change
 * across the whole identifier, so a board that lost its address to a conflict
 * moves somewhere unrelated in the /64.
 *
 * @param[in] prefix    Address whose first eight octets are the /64 prefix;
 *                      the rest is ignored.
 * @param[in] mac       Board MAC address (RFC 7217 Net_Iface).
 * @param[in] conflicts Duplicates seen for this prefix since the provider began.
 * @return The prefix followed by the chosen interface identifier.
 *
 * Verification:
 * - Correct means: the result is a pure function of its arguments; counter 0
 *   is prefix + EUI-64; counters 0..IDGEN_RETRIES are pairwise distinct; no
 *   result carries a reserved identifier.
 * - Covered by: tests/host/address_candidate_test.cpp (TC-NET-CAND-001,
 *   TC-NET-CAND-003).
 *
 * References: RFC 7217 §5 (F and DAD_Counter), §6 (IDGEN_RETRIES);
 * RFC 4862 §5.3 and §5.5.3(d).
 */
std::array<uint8_t, 16> ipv6_address_candidate(const std::array<uint8_t, 16>& prefix,
                                               const std::array<uint8_t, 6>& mac,
                                               uint8_t conflicts) noexcept;

/**
 * @brief The RFC 3927 IPv4 link-local address to try after @p conflicts duplicates.
 *
 * RFC 3927 §2.1 asks for a pseudo-random address in 169.254.1.0 through
 * 169.254.254.255, seeded from the MAC so a board picks the same sequence on
 * every boot. The address is the FNV-1a hash of (tag, counter, MAC) folded
 * into that range.
 *
 * @param[in] mac       Board MAC address.
 * @param[in] conflicts Conflicts seen since the provider began, including those
 *                      before a rate-limit pause.
 * @return Candidate address, network byte order.
 *
 * Verification:
 * - Correct means: every result lies in 169.254.1.0-169.254.254.255, the
 *   result is a pure function of its arguments, and the first
 *   RFC 3927 MAX_CONFLICTS counters for a board do not repeat an address.
 * - Covered by: tests/host/address_candidate_test.cpp (TC-NET-CAND-002).
 *
 * References: RFC 3927 §2.1.
 */
std::array<uint8_t, 4> ipv4_link_local_candidate(const std::array<uint8_t, 6>& mac,
                                                 uint32_t conflicts) noexcept;

}  // namespace rosette::net
