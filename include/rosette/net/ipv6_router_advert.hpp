#pragma once

/**
 * @file net/ipv6_router_advert.hpp
 * @brief Decodes an ICMPv6 Router Advertisement into what SLAAC needs.
 *
 * Stateless address autoconfiguration needs three things from a router
 * advertisement: the router itself (its link-local source address and router
 * lifetime, which make it a default router), an autonomous prefix to form a
 * global address from, and that prefix's lifetimes. This header decodes them
 * from the ICMPv6 message, independent of any network stack, so a driver that
 * receives raw ICMPv6 (the WIZnet W6100 / W6300 IPRAW6 socket) and a host test
 * share one implementation.
 *
 * Options are walked in whatever order the router sends them (RFC 4861 §4.2
 * places no order on them); unknown options are skipped by their length.
 */

#include <array>
#include <cstddef>
#include <cstdint>

namespace rosette::net {

/// @brief The parts of one Router Advertisement that address configuration uses.
struct RouterAdvert {
    std::array<uint8_t, 16> router{};        ///< Sender: the router's link-local address.
    uint16_t router_lifetime_s = 0u;         ///< 0: not a default router (RFC 4861 §4.2).
    bool has_prefix = false;                 ///< An autonomous prefix was found.
    uint8_t prefix_length = 0u;              ///< Bits; always 64 when `has_prefix`.
    bool on_link = false;                    ///< L flag of the chosen prefix.
    uint32_t valid_lifetime_s = 0u;          ///< 0xFFFFFFFF is infinite.
    uint32_t preferred_lifetime_s = 0u;
    std::array<uint8_t, 16> prefix{};        ///< Host bits zero.
    uint32_t mtu = 0u;                       ///< MTU option, 0 when absent.
};

/**
 * @brief Decodes a Router Advertisement and selects the prefix SLAAC can use.
 *
 * Validates the parts of RFC 4861 §6.1.2 visible above the IP layer (type 134,
 * code 0, at least 16 octets, a link-local source, every option of nonzero
 * length inside the message), then walks the options. The first Prefix
 * Information option that RFC 4862 §5.5.3 lets a host autoconfigure from is
 * chosen: autonomous flag set, not the link-local prefix, preferred lifetime
 * not above valid lifetime, and a 64-bit length to match a 64-bit interface
 * identifier (RFC 4291 §2.5.1). The IPv6 hop limit of 255 and the checksum are
 * checked below this layer, by whatever received the message.
 *
 * @param[in]  icmp   The ICMPv6 message, starting at its Type octet.
 * @param[in]  length Octets in `icmp`.
 * @param[in]  source IPv6 source address of the packet that carried it.
 * @param[out] out    Decoded advertisement; written only on success.
 * @return `true` when `icmp` is a valid Router Advertisement (with or without a
 *   usable prefix; see `out.has_prefix`); `false` when it is not one or is
 *   malformed, in which case it must be ignored.
 *
 * Verification:
 * - Correct means: the same prefix, lifetimes, and router result whatever the
 *   option order; malformed messages are rejected rather than partially read.
 * - Covered by: tests/host/ipv6_router_advert_test.cpp (TC-NET-RA-001..006),
 *   including a captured advertisement whose prefix option precedes the
 *   source link-layer address option.
 *
 * References: RFC 4861 §4.2, §4.6.2, §4.6.4, §6.1.2; RFC 4862 §5.5.3.
 */
inline bool parse_router_advert(const uint8_t* icmp, std::size_t length,
                                const std::array<uint8_t, 16>& source, RouterAdvert& out) noexcept
{
    constexpr uint8_t kTypeRouterAdvert = 134u;
    constexpr std::size_t kHeaderOctets = 16u;
    constexpr uint8_t kOptionPrefix = 3u;
    constexpr uint8_t kOptionMtu = 5u;
    constexpr uint8_t kFlagOnLink = 0x80u;
    constexpr uint8_t kFlagAutonomous = 0x40u;

    auto u32 = [](const uint8_t* p) {
        return (static_cast<uint32_t>(p[0]) << 24) | (static_cast<uint32_t>(p[1]) << 16) |
               (static_cast<uint32_t>(p[2]) << 8) | p[3];
    };

    if (icmp == nullptr || length < kHeaderOctets) return false;
    if (icmp[0] != kTypeRouterAdvert || icmp[1] != 0u) return false;
    if (source[0] != 0xFEu || (source[1] & 0xC0u) != 0x80u) return false;  // fe80::/10

    RouterAdvert ra{};
    ra.router = source;
    ra.router_lifetime_s = static_cast<uint16_t>((icmp[6] << 8) | icmp[7]);

    std::size_t at = kHeaderOctets;
    while (at < length) {
        if (length - at < 2u) return false;
        const uint8_t type = icmp[at];
        const std::size_t octets = static_cast<std::size_t>(icmp[at + 1u]) * 8u;
        if (octets == 0u || octets > length - at) return false;
        const uint8_t* opt = icmp + at;

        if (type == kOptionPrefix && octets == 32u && !ra.has_prefix) {
            const uint8_t prefix_length = opt[2];
            const uint8_t flags = opt[3];
            const uint32_t valid = u32(opt + 4);
            const uint32_t preferred = u32(opt + 8);
            const bool link_local = opt[16] == 0xFEu && (opt[17] & 0xC0u) == 0x80u;
            if ((flags & kFlagAutonomous) != 0u && !link_local && preferred <= valid &&
                prefix_length == 64u) {
                ra.has_prefix = true;
                ra.prefix_length = prefix_length;
                ra.on_link = (flags & kFlagOnLink) != 0u;
                ra.valid_lifetime_s = valid;
                ra.preferred_lifetime_s = preferred;
                for (std::size_t i = 0; i < 8u; ++i) ra.prefix[i] = opt[16u + i];
            }
        } else if (type == kOptionMtu && octets == 8u) {
            ra.mtu = u32(opt + 4);
        }
        at += octets;
    }
    out = ra;
    return true;
}

}  // namespace rosette::net
