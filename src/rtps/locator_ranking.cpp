// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file rtps/locator_ranking.cpp
 * @brief LocalSubnets: ranks remote locators by reachability (RFC 6724 §6).
 */

#include "locator_ranking.hpp"

namespace rosette::rtps {

namespace {

/// Address octets of a UDP locator: the last 4 for IPv4 (§9.3.2), all 16 for IPv6.
const Octet* address_octets(const Locator& l) noexcept {
    return l.is_udpv4() ? l.ipv4_address() : l.ipv6_address();
}

/// Address length in bits of a UDP locator's family.
uint8_t address_bits(const Locator& l) noexcept {
    return l.is_udpv4() ? 32u : 128u;
}

bool same_family(const Locator& a, const Locator& b) noexcept {
    return a.kind == b.kind;
}

bool same_address(const Locator& a, const Locator& b) noexcept {
    return same_family(a, b)
           && memcmp(address_octets(a), address_octets(b), address_bits(a) / 8u) == 0;
}

/// True when the first `bits` bits of two addresses of one family are equal.
bool prefix_matches(const Octet* a, const Octet* b, uint8_t bits) noexcept {
    const uint8_t whole = bits / 8u;
    if (memcmp(a, b, whole) != 0) {
        return false;
    }

    const uint8_t rest = bits % 8u;
    if (rest == 0u) {
        return true;
    }

    const Octet mask = static_cast<Octet>(0xFFu << (8u - rest));
    return (a[whole] & mask) == (b[whole] & mask);
}

bool is_loopback(const Locator& l) noexcept {
    const Octet* a = address_octets(l);
    if (l.is_udpv4()) {
        return a[0] == 127u;  // 127/8, RFC 1122 §3.2.1.3
    }

    static constexpr Octet kLoopback6[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1};
    return memcmp(a, kLoopback6, 16u) == 0;  // ::1, RFC 4291 §2.5.3
}

bool is_unspecified(const Locator& l) noexcept {
    static constexpr Octet kZero[16] = {};
    return memcmp(address_octets(l), kZero, address_bits(l) / 8u) == 0;
}

bool is_multicast(const Locator& l) noexcept {
    const Octet* a = address_octets(l);
    return l.is_udpv4() ? (a[0] & 0xF0u) == 0xE0u  // 224/4, RFC 5771
                        : a[0] == 0xFFu;           // ff00::/8, RFC 4291 §2.7
}

bool is_link_local(const Locator& l) noexcept {
    const Octet* a = address_octets(l);
    return l.is_udpv4() ? (a[0] == 169u && a[1] == 254u)               // RFC 3927
                        : (a[0] == 0xFEu && (a[1] & 0xC0u) == 0x80u);  // fe80::/10
}

}  // namespace

bool LocalSubnets::add(const Locator& address, uint8_t prefix_len) noexcept {
    if (!address.is_udpv4() && !address.is_udpv6()) {
        return false;
    }

    const uint8_t bits = address_bits(address);
    const uint8_t clamped = prefix_len < bits ? prefix_len : bits;
    for (uint8_t i = 0; i < count_; ++i) {
        if (same_address(entries_[i].address, address)) {
            entries_[i].prefix_len = clamped;
            return true;
        }
    }

    if (count_ >= RTPS_MAX_LOCATORS_PER_PARTICIPANT) {
        return false;
    }

    entries_[count_] = Entry{address, clamped};
    entries_[count_].address.port = 0u;
    ++count_;
    return true;
}

bool LocalSubnets::remove(const Locator& address) noexcept {
    for (uint8_t i = 0; i < count_; ++i) {
        if (!same_address(entries_[i].address, address)) {
            continue;
        }

        // Shifted down rather than swapped, so the remaining addresses keep
        // the order they were added in.
        for (uint8_t k = i; (k + 1u) < count_; ++k) {
            entries_[k] = entries_[k + 1u];
        }

        --count_;
        entries_[count_] = Entry{};
        return true;
    }

    return false;
}

bool LocalSubnets::holds_family(const Locator& remote) const noexcept {
    for (uint8_t i = 0; i < count_; ++i) {
        if (same_family(entries_[i].address, remote)) {
            return true;
        }
    }

    return false;
}

bool LocalSubnets::on_link(const Locator& remote) const noexcept {
    for (uint8_t i = 0; i < count_; ++i) {
        const Entry& e = entries_[i];
        if (!same_family(e.address, remote) || e.prefix_len == 0u) {
            continue;
        }

        if (prefix_matches(address_octets(e.address), address_octets(remote), e.prefix_len)) {
            return true;
        }
    }

    return false;
}

LocatorReach LocalSubnets::rank(const Locator& remote) const noexcept {
    if ((!remote.is_udpv4() && !remote.is_udpv6()) || is_unspecified(remote)
        || is_multicast(remote)) {
        return LocatorReach::unusable;
    }

    if (is_loopback(remote)) {
        // A loopback address names the announcing host, so it reaches that
        // peer only from the same host, and this participant is on that host
        // only when it holds a loopback address itself.
        for (uint8_t i = 0; i < count_; ++i) {
            if (same_family(entries_[i].address, remote) && is_loopback(entries_[i].address)) {
                return LocatorReach::on_link;
            }
        }

        return LocatorReach::unusable;
    }

    if (on_link(remote)) {
        return LocatorReach::on_link;
    }

    if (!holds_family(remote)) {
        return LocatorReach::unlikely;
    }

    if (is_link_local(remote)) {
        return remote.is_udpv6() ? LocatorReach::link_local : LocatorReach::unlikely;
    }

    return LocatorReach::routed;
}

uint8_t LocalSubnets::select(Locator* out, uint8_t cap, const Locator* src,
                             uint8_t n) const noexcept {
    if ((out == nullptr) || (src == nullptr)) {
        return 0u;
    }

    // One pass per class keeps the result stable within a class without a
    // scratch buffer; lists are a handful of locators long.
    uint8_t kept = 0u;
    for (uint8_t cls = 0u; cls < static_cast<uint8_t>(LocatorReach::unusable); ++cls) {
        for (uint8_t i = 0u; (i < n) && (kept < cap); ++i) {
            if (static_cast<uint8_t>(rank(src[i])) == cls) {
                out[kept++] = src[i];
            }
        }
    }

    return kept;
}

bool LocalSubnets::offer(Locator* list, uint8_t& count, uint8_t cap,
                         const Locator& candidate) const noexcept {
    const LocatorReach reach = rank(candidate);
    if ((list == nullptr) || (cap == 0u) || (reach == LocatorReach::unusable)) {
        return false;
    }

    uint8_t pos = 0u;
    while ((pos < count) && (rank(list[pos]) <= reach)) {
        ++pos;
    }

    if (pos >= cap) {
        return false;
    }

    // Shift the worse-ranked tail down one place; a full list loses its last.
    const uint8_t last = count < cap ? count : static_cast<uint8_t>(cap - 1u);
    for (uint8_t k = last; k > pos; --k) {
        list[k] = list[k - 1u];
    }

    list[pos] = candidate;
    if (count < cap) {
        ++count;
    }

    return true;
}

}  // namespace rosette::rtps
