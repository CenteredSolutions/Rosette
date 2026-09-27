/**
 * @file address_candidate_test.cpp
 * @brief 64-bit FNV-1a and the self-assigned address candidates built on it.
 *
 * Test cases: TC-NET-FNV-001 (published FNV-1a 64-bit vectors),
 * TC-NET-CAND-001 (IPv6 candidates: counter 0 is EUI-64, later counters are
 * distinct, repeatable, and keep the prefix), TC-NET-CAND-002 (IPv4
 * link-local candidates stay in 169.254.1.0-169.254.254.255 and do not repeat
 * within MAX_CONFLICTS), TC-NET-CAND-003 (reserved interface identifiers are
 * rejected and never produced).
 */

#include "test_util.hpp"

#include "rosette/net/address_candidate.hpp"
#include "rosette/support/fnv1a.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>
#include <set>

using namespace rosette::net;

namespace {

const std::array<uint8_t, 6> kMac{0x00, 0x1C, 0x42, 0x14, 0x87, 0x37};
const std::array<uint8_t, 16> kLinkLocal{0xFE, 0x80};
const std::array<uint8_t, 16> kPrefix{0x26, 0x03, 0x80, 0x80, 0xF8, 0x06, 0xF4, 0x50};

uint64_t fnv(const char* text) {
    return rosette::support::fnv1a64(reinterpret_cast<const uint8_t*>(text), std::strlen(text));
}

std::array<uint8_t, 8> iid_of(const std::array<uint8_t, 16>& a) {
    return {a[8], a[9], a[10], a[11], a[12], a[13], a[14], a[15]};
}

}  // namespace

int main() {
    // TC-NET-FNV-001
    check(fnv("") == 0xCBF29CE484222325ull, "FNV-1a 64: empty string is the offset basis");
    check(fnv("a") == 0xAF63DC4C8601EC8Cull, "FNV-1a 64: \"a\"");
    check(fnv("foobar") == 0x85944171F73967E8ull, "FNV-1a 64: \"foobar\"");

    // TC-NET-CAND-001
    {
        const auto first = ipv6_address_candidate(kLinkLocal, kMac, 0u);
        check(iid_of(first) == modified_eui64(kMac), "counter 0: modified EUI-64 identifier");

        std::set<std::array<uint8_t, 16>> seen;
        bool prefix_kept = true;
        for (uint8_t k = 0u; k <= kIdgenRetries; ++k) {
            for (const auto& prefix : {kLinkLocal, kPrefix}) {
                const auto a = ipv6_address_candidate(prefix, kMac, k);
                seen.insert(a);
                prefix_kept = prefix_kept && std::equal(a.begin(), a.begin() + 8, prefix.begin());
            }
        }

        check(seen.size() == 2u * (kIdgenRetries + 1u),
              "candidates 0..IDGEN_RETRIES are pairwise distinct");
        check(prefix_kept, "every candidate keeps its /64 prefix");
        check(
            ipv6_address_candidate(kPrefix, kMac, 2u) == ipv6_address_candidate(kPrefix, kMac, 2u),
            "candidates are repeatable (stable across reboots)");
        check(iid_of(ipv6_address_candidate(kPrefix, kMac, 1u))
                  != iid_of(ipv6_address_candidate(kLinkLocal, kMac, 1u)),
              "after a conflict, identifiers differ per prefix (RFC 7217)");

        std::array<uint8_t, 6> other = kMac;
        other[5] ^= 0x01u;
        check(
            ipv6_address_candidate(kPrefix, kMac, 1u) != ipv6_address_candidate(kPrefix, other, 1u),
            "a different MAC gives a different candidate");
    }

    // TC-NET-CAND-002
    {
        bool in_range = true;
        std::set<std::array<uint8_t, 4>> seen;
        for (uint32_t k = 0u; k < 10u; ++k) {
            const auto a = ipv4_link_local_candidate(kMac, k);
            in_range = in_range && a[0] == 169u && a[1] == 254u && a[2] >= 1u && a[2] <= 254u;
            seen.insert(a);
        }

        for (uint32_t k = 10u; k < 5000u; ++k) {
            const auto a = ipv4_link_local_candidate(kMac, k);
            in_range = in_range && a[0] == 169u && a[1] == 254u && a[2] >= 1u && a[2] <= 254u;
        }

        check(in_range, "every IPv4 candidate lies in 169.254.1.0-169.254.254.255");
        check(seen.size() == 10u, "the first MAX_CONFLICTS (10) candidates do not repeat");
        check(ipv4_link_local_candidate(kMac, 3u) == ipv4_link_local_candidate(kMac, 3u),
              "IPv4 candidates are repeatable");
    }

    // TC-NET-CAND-003
    {
        check(is_reserved_interface_id({}), "all-zero (subnet-router anycast) is reserved");
        check(is_reserved_interface_id({0xFD, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x80}),
              "fdff:ffff:ffff:ff80 is reserved");
        check(!is_reserved_interface_id({0xFD, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x7F}),
              "fdff:ffff:ffff:ff7f is not");
        check(is_reserved_interface_id({0x02, 0x00, 0x5E, 0xFF, 0xFE, 0x00, 0x52, 0x13}),
              "Proxy Mobile IPv6 IID is reserved");
        check(!is_reserved_interface_id(modified_eui64(kMac)),
              "an ordinary EUI-64 identifier is not");

        // A MAC in the IANA OUI 00-00-5e maps to the reserved 0200:5eff:fe block,
        // so even counter 0 must be a hash.
        const std::array<uint8_t, 6> iana_mac{0x00, 0x00, 0x5E, 0x00, 0x01, 0x01};
        const auto a = ipv6_address_candidate(kLinkLocal, iana_mac, 0u);
        check(is_reserved_interface_id(modified_eui64(iana_mac))
                  && !is_reserved_interface_id(iid_of(a)),
              "a reserved EUI-64 identifier is replaced by a hashed one");
    }

    return test_summary();
}
