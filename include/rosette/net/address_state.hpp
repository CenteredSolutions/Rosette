// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <array>
#include <cstddef>
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
 * @brief Orthogonal properties of an address, as independent bits.
 *
 * `link_local` and `routable` are derived from the address bytes when the
 * address enters an `AddressList`. `deprecated` is set by the provider when an
 * IPv6 address's preferred lifetime has ended. `duplicate` is a family-level
 * event bit: a provider sets it on its lease for the one `poll()` that observed
 * a conflict.
 */
enum class AddressFlags : uint8_t {
    none = 0u,
    link_local = 1u << 0,  ///< Useful for on-link peers only (169.254/16, fe80::/10).
    routable = 1u << 1,    ///< Usable beyond the local link.
    duplicate = 1u << 2,   ///< A conflicting host was seen; re-acquiring or defending.
    deprecated = 1u << 3,  ///< Preferred lifetime over; valid, but not for new communication.
};

constexpr AddressFlags operator|(AddressFlags a, AddressFlags b) noexcept {
    return static_cast<AddressFlags>(static_cast<uint8_t>(a) | static_cast<uint8_t>(b));
}

constexpr AddressFlags operator&(AddressFlags a, AddressFlags b) noexcept {
    return static_cast<AddressFlags>(static_cast<uint8_t>(a) & static_cast<uint8_t>(b));
}

constexpr AddressFlags& operator|=(AddressFlags& a, AddressFlags b) noexcept {
    return a = a | b;
}

/// Whether every bit in @p bit is set in @p value.
constexpr bool has_flag(AddressFlags value, AddressFlags bit) noexcept {
    return (static_cast<uint8_t>(value) & static_cast<uint8_t>(bit)) == static_cast<uint8_t>(bit);
}

/// Sets or clears @p bit in @p value in place.
constexpr void set_flag(AddressFlags& value, AddressFlags bit, bool on) noexcept {
    value =
        on ? (value | bit)
           : static_cast<AddressFlags>(static_cast<uint8_t>(value) & ~static_cast<uint8_t>(bit));
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
 * @param[in] family  Address family of @p address.
 * @param[in] address Bytes `[0..3]` for IPv4, `[0..15]` for IPv6.
 * @param[in] has_address `false` for "no address".
 * @return `AddressFlags::none` when @p has_address is false; otherwise exactly
 *         one of `link_local` or `routable`.
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

/// Lifetime value meaning "never expires" (RFC 4861 §4.6.2 uses all ones).
inline constexpr uint32_t kInfiniteLifetime = 0xFFFFFFFFu;

/**
 * @brief One address an interface holds, with its prefix and lifetimes.
 *
 * Lifetimes are the values the address was granted or last refreshed with
 * (DHCP lease time, router-advertised lifetimes), not a countdown, so an entry
 * changes only when the provider refreshes it. Link-local, static, and IPv4
 * link-local addresses carry `kInfiniteLifetime`.
 */
struct AddressEntry {
    std::array<uint8_t, 16> address{};  ///< `[0..3]` for IPv4, `[0..15]` for IPv6.
    uint8_t prefix_len = 0u;
    AddressFlags flags = AddressFlags::none;
    uint32_t valid_lifetime_s = 0u;
    uint32_t preferred_lifetime_s = 0u;

    bool link_local() const noexcept { return has_flag(flags, AddressFlags::link_local); }

    bool routable() const noexcept { return has_flag(flags, AddressFlags::routable); }

    bool deprecated() const noexcept { return has_flag(flags, AddressFlags::deprecated); }
};

inline bool operator==(const AddressEntry& lhs, const AddressEntry& rhs) noexcept {
    return lhs.address == rhs.address && lhs.prefix_len == rhs.prefix_len && lhs.flags == rhs.flags
           && lhs.valid_lifetime_s == rhs.valid_lifetime_s
           && lhs.preferred_lifetime_s == rhs.preferred_lifetime_s;
}

inline bool operator!=(const AddressEntry& lhs, const AddressEntry& rhs) noexcept {
    return !(lhs == rhs);
}

/**
 * @brief Most addresses one family holds at once.
 *
 * The W6100 / W6300 hold one IPv6 link-local address (LLAR) and one global
 * address (GUAR); an IPv4 interface holds one address. A backend with more
 * address slots raises this.
 */
inline constexpr std::size_t kMaxFamilyAddresses = 2u;

/**
 * @brief The addresses one family holds, as a bounded value type.
 *
 * Leases and snapshots are copied and compared by value, so the list lives
 * inline rather than behind a pointer. Slots past `size()` are always zero,
 * which lets equality compare the whole array and makes `primary()` of an
 * empty list an all-zero entry.
 */
class AddressList {
public:
    /**
     * @brief Appends an address, deriving its link-local / routable bits.
     *
     * @param[in] family       Family of @p address; selects the classifier.
     * @param[in] address      Bytes `[0..3]` for IPv4, `[0..15]` for IPv6.
     * @param[in] prefix_len   On-link prefix length in bits.
     * @param[in] valid_s      Valid lifetime in seconds, or `kInfiniteLifetime`.
     * @param[in] preferred_s  Preferred lifetime in seconds, or `kInfiniteLifetime`.
     * @param[in] deprecated   Whether the preferred lifetime has already ended.
     * @return `false`, leaving the list unchanged, when it is full.
     */
    bool add(AddressFamily family, const std::array<uint8_t, 16>& address, uint8_t prefix_len,
             uint32_t valid_s = kInfiniteLifetime, uint32_t preferred_s = kInfiniteLifetime,
             bool deprecated = false) noexcept;

    /// Removes every address.
    void clear() noexcept {
        entries_ = {};
        count_ = 0u;
    }

    std::size_t size() const noexcept { return count_; }

    bool empty() const noexcept { return count_ == 0u; }

    const AddressEntry& operator[](std::size_t i) const noexcept { return entries_[i]; }

    const AddressEntry* begin() const noexcept { return entries_.data(); }

    const AddressEntry* end() const noexcept { return entries_.data() + count_; }

    /**
     * @brief The address to report when one address must stand for the family.
     *
     * The primary address is what `board_ipv4_address()`, the platform
     * runtime, and log lines show. The ranking follows RFC 6724 source
     * address selection for a global-scope destination, the case a DDS
     * participant announcing itself serves: an address of adequate scope
     * (routable) ranks above a link-local one (rule 2), then a preferred
     * address above a deprecated one (rule 3), then list order. Tentative and
     * duplicate addresses never enter the list, so rule 1's exclusions are
     * already applied.
     *
     * @return The best entry; the all-zero entry when the list is empty.
     *
     * Verification:
     * - Correct means: a routable address always beats a link-local one, a
     *   preferred one beats a deprecated one of the same scope, and ties keep
     *   list order.
     * - Covered by: tests/host/address_snapshot_test.cpp (TC-NET-6724-001).
     *
     * References: RFC 6724 §5 (rules 2 and 3); RFC 4862 §5.5.4.
     */
    const AddressEntry& primary() const noexcept;

    friend bool operator==(const AddressList& lhs, const AddressList& rhs) noexcept {
        return lhs.count_ == rhs.count_ && lhs.entries_ == rhs.entries_;
    }

    friend bool operator!=(const AddressList& lhs, const AddressList& rhs) noexcept {
        return !(lhs == rhs);
    }

private:
    std::array<AddressEntry, kMaxFamilyAddresses> entries_{};
    uint8_t count_ = 0u;
};

/**
 * @brief Per-family view of an interface's addressing state.
 *
 * IPv4 and IPv6 come up independently, so each family carries its own state,
 * source, and address list. `flags` holds the family-level `duplicate` event;
 * per-address properties live on each `AddressEntry`.
 */
struct FamilyAddressSnapshot {
    AddressPhase state = AddressPhase::idle;
    AddressSource source = AddressSource::manual;
    AddressFlags flags = AddressFlags::none;
    AddressList addresses{};

    bool has_address() const noexcept { return !addresses.empty(); }

    const AddressEntry& primary() const noexcept { return addresses.primary(); }

    const std::array<uint8_t, 16>& address() const noexcept { return primary().address; }

    bool link_local() const noexcept { return has_address() && primary().link_local(); }

    bool routable() const noexcept { return has_address() && primary().routable(); }
    bool duplicate() const noexcept { return has_flag(flags, AddressFlags::duplicate); }
};

inline bool operator==(const FamilyAddressSnapshot& lhs,
                       const FamilyAddressSnapshot& rhs) noexcept {
    return lhs.state == rhs.state && lhs.source == rhs.source && lhs.flags == rhs.flags
           && lhs.addresses == rhs.addresses;
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
 * carry the per-family detail, including every address each family holds.
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
 * changed readiness, addresses, flags, or NTP server information.
 *
 * @param[in] lhs First snapshot to compare.
 * @param[in] rhs Second snapshot to compare.
 * @return `true` when both snapshots describe the same supervisor-visible
 *         address state.
 */
inline bool operator==(const AddressSnapshot& lhs, const AddressSnapshot& rhs) noexcept {
    return lhs.active == rhs.active && lhs.assigned == rhs.assigned && lhs.v4 == rhs.v4
           && lhs.v6 == rhs.v6 && lhs.ntp_servers_v4 == rhs.ntp_servers_v4
           && lhs.ntp_server_count == rhs.ntp_server_count;
}

/**
 * @brief Compare two address snapshots for supervisor-visible changes.
 *
 * @param[in] lhs First snapshot to compare.
 * @param[in] rhs Second snapshot to compare.
 * @return `true` when the snapshots differ.
 */
inline bool operator!=(const AddressSnapshot& lhs, const AddressSnapshot& rhs) noexcept {
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
