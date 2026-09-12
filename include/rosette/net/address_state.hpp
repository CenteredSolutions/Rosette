#pragma once

#include <array>
#include <cstdint>

namespace rosette::net {

/**
 * @brief Lifecycle state of one address — a provider's own state, and (once
 *        folded into a snapshot) the per-family state a supervisor reads.
 *
 * Higher-level firmware often needs to know whether an interface is still
 * waiting for configuration, has a usable address, or failed to acquire one.
 * The exact mechanism varies by provider: a POSIX test harness may begin with
 * a static address, a WIZnet board may run an explicit DHCP client, and a
 * future stack may observe callback-driven state from FreeRTOS+TCP or lwIP —
 * this one small enum is the common vocabulary for all of them.
 */
enum class AddressPhase : uint8_t {
    idle = 0u,    ///< Not started, or stopped.
    acquiring,    ///< Working toward an address — initial bring-up, or
                  ///< re-acquiring after losing one.
    bound,        ///< Holding a usable address.
    failed,       ///< Cannot get an address. A manager falls through to the
                  ///< next provider in its list.
};

/**
 * @brief How an address was obtained. Family-agnostic — `AddressFamily` already
 *        says v4 or v6, so `slaac` covers both the SLAAC-derived link-local and
 *        any SLAAC global; `AddressFlags::link_local` tells them apart.
 */
enum class AddressSource : uint8_t {
    manual = 0u,    ///< Statically configured / operator-supplied.
    dhcp,           ///< DHCPv4.
    zeroconf,       ///< RFC 3927 IPv4 link-local.
    slaac,          ///< IPv6 stateless address autoconfiguration (RFC 4862).
};

/**
 * @brief Address family an acquisition path or snapshot pertains to.
 */
enum class AddressFamily : uint8_t {
    ipv4 = 0u,
    ipv6,
};

/**
 * @brief Orthogonal properties of an address — independent bits rather than an
 *        ordered "quality" level, since an address can be more than one of
 *        these at once (e.g. a temporary SLAAC global address is both
 *        `routable` and `temporary`).
 */
enum class AddressFlags : uint8_t {
    none       = 0u,
    link_local = 1u << 0,    ///< Useful for on-link peers only (169.254/16, fe80::/10).
    routable   = 1u << 1,    ///< Usable beyond the local link.
    duplicate  = 1u << 2,    ///< A conflicting host was seen; re-acquiring or defending.
    temporary  = 1u << 3,    ///< RFC 8981 SLAAC privacy address — rotates, not yet generated.
    permanent  = 1u << 4,    ///< Stable for the life of the lease/configuration.
};

constexpr AddressFlags operator|(AddressFlags a, AddressFlags b) noexcept {
    return static_cast<AddressFlags>(static_cast<uint8_t>(a) | static_cast<uint8_t>(b));
}
constexpr AddressFlags operator&(AddressFlags a, AddressFlags b) noexcept {
    return static_cast<AddressFlags>(static_cast<uint8_t>(a) & static_cast<uint8_t>(b));
}
constexpr AddressFlags& operator|=(AddressFlags& a, AddressFlags b) noexcept { return a = a | b; }

/// Whether every bit in @p bit is set in @p value.
constexpr bool has_flag(AddressFlags value, AddressFlags bit) noexcept {
    return (static_cast<uint8_t>(value) & static_cast<uint8_t>(bit)) ==
           static_cast<uint8_t>(bit);
}

/// Sets or clears @p bit in @p value in place.
constexpr void set_flag(AddressFlags& value, AddressFlags bit, bool on) noexcept {
    value = on ? (value | bit)
               : static_cast<AddressFlags>(static_cast<uint8_t>(value) &
                                           ~static_cast<uint8_t>(bit));
}

/**
 * @brief Determine whether an IPv4 address is within the RFC 3927 link-local range.
 *
 * @param[in] ipv4 IPv4 address to classify.
 * @return `true` when the address is link-local; otherwise `false`.
 *
 * References:
 * - RFC 3927 section 2.1.
 */
inline bool IsIpv4LinkLocal(const std::array<uint8_t, 4>& ipv4) noexcept {
    return ipv4[0] == 169u && ipv4[1] == 254u;
}

/**
 * @brief Determine whether a 16-byte IPv6 address is link-local (`fe80::/10`).
 *
 * @param[in] ipv6 IPv6 address to classify.
 * @return `true` when the address is link-local; otherwise `false`.
 *
 * References:
 * - RFC 4291 section 2.5.6.
 */
inline bool IsIpv6LinkLocal(const std::array<uint8_t, 16>& ipv6) noexcept {
    return ipv6[0] == 0xFEu && (ipv6[1] & 0xC0u) == 0x80u;
}

/**
 * @brief Derive the `link_local` / `routable` bits from an address's own bytes.
 *
 * The other `AddressFlags` bits (`duplicate`, `temporary`, `permanent`) are
 * reported by the provider, not derived — OR them in separately.
 *
 * @return `AddressFlags::none` when @p has_address is false.
 */
inline AddressFlags classify(AddressFamily family, const std::array<uint8_t, 16>& address,
                             bool has_address) noexcept {
    if (!has_address) {
        return AddressFlags::none;
    }
    if (family == AddressFamily::ipv4) {
        const std::array<uint8_t, 4> a{address[0], address[1], address[2], address[3]};
        return IsIpv4LinkLocal(a) ? AddressFlags::link_local : AddressFlags::routable;
    }
    return IsIpv6LinkLocal(address) ? AddressFlags::link_local : AddressFlags::routable;
}

/**
 * @brief Per-family view of an interface's addressing state.
 *
 * IPv4 and IPv6 come up independently, so each family carries its own state,
 * source, and flags. `address` holds the assigned address: bytes `[0..3]` for
 * IPv4, `[0..15]` for IPv6.
 */
struct FamilyAddressSnapshot {
    AddressPhase state = AddressPhase::idle;
    AddressSource source = AddressSource::manual;
    AddressFlags flags = AddressFlags::none;
    bool has_address = false;
    std::array<uint8_t, 16> address{};
    uint32_t lease_remaining_s = 0u;

    bool link_local() const noexcept { return has_flag(flags, AddressFlags::link_local); }
    bool routable() const noexcept { return has_flag(flags, AddressFlags::routable); }
    bool duplicate() const noexcept { return has_flag(flags, AddressFlags::duplicate); }
};

inline bool operator==(const FamilyAddressSnapshot& lhs,
                       const FamilyAddressSnapshot& rhs) noexcept {
    return lhs.state == rhs.state && lhs.source == rhs.source &&
           lhs.flags == rhs.flags && lhs.has_address == rhs.has_address &&
           lhs.address == rhs.address &&
           lhs.lease_remaining_s == rhs.lease_remaining_s;
}

inline bool operator!=(const FamilyAddressSnapshot& lhs,
                       const FamilyAddressSnapshot& rhs) noexcept {
    return !(lhs == rhs);
}

/**
 * @brief Platform-neutral snapshot of interface addressing state.
 *
 * Rosette uses this snapshot as the common reporting surface for address
 * providers. A provider may actively run DHCP, observe a third-party TCP/IP
 * stack, or simply expose a static test-harness address. Higher layers should
 * supervise this snapshot rather than making assumptions about how the address
 * was obtained.
 *
 * `active` is provider lifecycle (started vs stopped); `assigned` is "at least
 * one family has a usable address". The two `FamilyAddressSnapshot` members
 * carry the per-family detail.
 */
struct AddressSnapshot {
    bool active = false;
    bool assigned = false;
    FamilyAddressSnapshot v4{};
    FamilyAddressSnapshot v6{};
    std::array<std::array<uint8_t, 4>, 2> ntp_servers_v4{};
    uint8_t ntp_server_count = 0u;
};

/**
 * @brief Compare two address snapshots for supervisor-visible changes.
 *
 * Address providers typically update state from polling loops or callback-fed
 * observers. Equality lets higher layers decide whether a step materially
 * changed readiness, address flags, or NTP server information.
 *
 * @param[in] lhs First snapshot to compare.
 * @param[in] rhs Second snapshot to compare.
 * @return `true` when both snapshots describe the same supervisor-visible
 *         address state.
 */
inline bool operator==(const AddressSnapshot& lhs,
                       const AddressSnapshot& rhs) noexcept {
    return lhs.active == rhs.active && lhs.assigned == rhs.assigned &&
           lhs.v4 == rhs.v4 && lhs.v6 == rhs.v6 &&
           lhs.ntp_servers_v4 == rhs.ntp_servers_v4 &&
           lhs.ntp_server_count == rhs.ntp_server_count;
}

/**
 * @brief Compare two address snapshots for supervisor-visible changes.
 *
 * @param[in] lhs First snapshot to compare.
 * @param[in] rhs Second snapshot to compare.
 * @return `true` when the snapshots differ.
 */
inline bool operator!=(const AddressSnapshot& lhs,
                       const AddressSnapshot& rhs) noexcept {
    return !(lhs == rhs);
}

/**
 * @brief Count the leading set bits of a contiguous IPv4 subnet mask.
 *
 * @param[in] mask Dotted-quad subnet mask (`255.255.255.0` → 24).
 * @return Prefix length in bits `[0..32]`.
 */
inline uint8_t Ipv4MaskToPrefix(const std::array<uint8_t, 4>& mask) noexcept {
    uint8_t bits = 0u;
    for (uint8_t octet : mask) {
        while (octet & 0x80u) {
            ++bits;
            octet = static_cast<uint8_t>(octet << 1);
        }
        if (octet != 0u) {
            break;    // non-contiguous mask — stop at the first gap
        }
    }
    return bits;
}

/**
 * @brief Expand an IPv4 prefix length into a dotted-quad subnet mask.
 *
 * @param[in] prefix_len Prefix length in bits; clamped to `[0..32]`.
 * @return Subnet mask (`24` → `255.255.255.0`).
 */
inline std::array<uint8_t, 4> Ipv4PrefixToMask(uint8_t prefix_len) noexcept {
    if (prefix_len > 32u) {
        prefix_len = 32u;
    }
    std::array<uint8_t, 4> mask{};
    for (uint8_t i = 0u; i < 4u; ++i) {
        if (prefix_len >= 8u) {
            mask[i] = 0xFFu;
            prefix_len = static_cast<uint8_t>(prefix_len - 8u);
        } else if (prefix_len > 0u) {
            mask[i] = static_cast<uint8_t>(0xFFu << (8u - prefix_len));
            prefix_len = 0u;
        }
    }
    return mask;
}

}    // namespace rosette::net
