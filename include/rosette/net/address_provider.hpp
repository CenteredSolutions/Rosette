#pragma once

/**
 * @file net/address_provider.hpp
 * @brief The one contract every address source implements, plus the small value
 *        type it reports.
 *
 * An `AddressProvider` is a single way to obtain one interface address for one
 * family — a DHCP client, a static address, RFC 3927 IPv4 link-local, IPv6
 * SLAAC. The whole contract is three methods:
 *
 *   - `begin()` / `stop()` — lifecycle, both non-blocking;
 *   - `poll(now_ms)` — a cooperative step called every superloop iteration for
 *     the life of the provider, with the loop's millisecond timestamp. Before an
 *     address is in hand it drives acquisition; afterwards it keeps the address
 *     valid (DHCP renewal) and watches for a duplicate. It returns the
 *     provider's current `AddressLease`.
 *
 * One `AddressManager` drives an ordered list of providers for a single family
 * (first to bind wins); `NetworkAddress` combines a v4 and an optional v6
 * manager and supervises the physical link.
 *
 * The provider is handed a `NetInterface` (`net/interface.hpp`) — the one
 * backend hook: it writes the chosen address onto the hardware, reports
 * duplicate-address signals, and reports carrier state.
 */

#include <array>
#include <cstdint>

#include "address_state.hpp"

namespace rosette::net {

/**
 * @brief One provider's current address contribution.
 *
 * A lease carries every address the provider holds for its family (the
 * SLAAC provider holds a link-local and a global address; the IPv4 providers
 * hold one), plus the family's routing and NTP information. `NetInterface::apply()`
 * programs the whole lease at once, so the hardware always matches it.
 *
 * `flags`' `duplicate` bit latches true for the single `poll()` that first
 * observes one; the provider then recovers on its own (DHCPDECLINE +
 * re-DISCOVER, the next address candidate) and, if it had to release the
 * address to do so, moves back to `AddressPhase::acquiring`.
 */
struct AddressLease {
    AddressFamily family = AddressFamily::ipv4;
    AddressSource source = AddressSource::manual;
    AddressPhase state = AddressPhase::idle;
    AddressFlags flags = AddressFlags::none;  ///< Family-level event bits (`duplicate`).
    AddressList addresses{};
    std::array<uint8_t, 4> gateway_v4{};
    std::array<uint8_t, 16> gateway_v6{};  ///< Default router (link-local), zero when none.
    std::array<std::array<uint8_t, 4>, 2> ntp_v4{};
    uint8_t ntp_count = 0u;

    /// A cleared lease that keeps only the family and source (what a provider reports while idle).
    AddressLease empty() const noexcept {
        AddressLease l{};
        l.family = family;
        l.source = source;
        return l;
    }

    /// Appends @p address to this lease's family list; see `AddressList::add()`.
    bool add(const std::array<uint8_t, 16>& address, uint8_t prefix_len,
             uint32_t valid_s = kInfiniteLifetime, uint32_t preferred_s = kInfiniteLifetime,
             bool deprecated = false) noexcept {
        return addresses.add(family, address, prefix_len, valid_s, preferred_s, deprecated);
    }

    bool has_address() const noexcept { return !addresses.empty(); }

    const AddressEntry& primary() const noexcept { return addresses.primary(); }

    const std::array<uint8_t, 16>& address() const noexcept { return primary().address; }

    bool link_local() const noexcept { return has_address() && primary().link_local(); }

    bool routable() const noexcept { return has_address() && primary().routable(); }
    bool duplicate() const noexcept { return has_flag(flags, AddressFlags::duplicate); }

    /// Fold this lease into the matching family slot of a combined snapshot.
    void fill_family(AddressSnapshot& s) const noexcept {
        FamilyAddressSnapshot& f = (family == AddressFamily::ipv4) ? s.v4 : s.v6;
        f.state = state;
        f.source = source;
        f.flags = flags;
        f.addresses = addresses;
        if (family == AddressFamily::ipv4 && ntp_count > 0u) {
            const uint8_t n = ntp_count > 2u ? 2u : ntp_count;
            s.ntp_server_count = n;
            for (uint8_t k = 0u; k < n; ++k) {
                s.ntp_servers_v4[k] = ntp_v4[k];
            }
        }
    }
};

/**
 * @brief A single source of interface addressing for one family.
 */
class AddressProvider {
public:
    virtual ~AddressProvider() = default;

    /// Begin acquisition. Non-blocking kick-off; `poll()` does the work.
    virtual void begin() noexcept = 0;

    /// Return to idle. A bound address is released.
    virtual void stop() noexcept = 0;

    /**
     * @brief One cooperative step: acquire, renew, and detect a conflict.
     *
     * @param[in] now_ms  The superloop's monotonic millisecond timestamp.
     * @return the provider's lease, valid until the next call on this provider.
     */
    virtual const AddressLease& poll(uint32_t now_ms) noexcept = 0;
};

}    // namespace rosette::net
