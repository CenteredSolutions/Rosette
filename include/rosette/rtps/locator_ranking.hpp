/**
 * @file rtps/locator_ranking.hpp
 * @brief Orders a remote participant's locators by how reachable they are
 *        from this participant's own addresses.
 *
 * A participant announces every address it holds, on every interface and in
 * whatever order its host lists them: a Fast DDS process on a workstation
 * lists its Docker bridge, its Wi-Fi and its Ethernet addresses, and the
 * first is often one no other machine can reach. This stack keeps only the
 * leading RTPS_MAX_PEER_LOCATORS (and RTPS_MAX_REMOTE_ENDPOINT_LOCATORS) of a
 * list and sends to the first, so a list kept in announced order can drop the
 * one reachable address and send to an unreachable one. The decoders hold at
 * most RTPS_MAX_LOCATORS_PER_PARTICIPANT locators of a list, so the ranking
 * applies as each locator is decoded, not to the decoded list.
 *
 * LocalSubnets holds this participant's addresses with their on-link prefix
 * lengths and ranks a remote locator by destination address selection in the
 * manner of RFC 6724 §6 (prefer a destination whose scope this host can
 * reach, then the longest matching prefix), reduced to the classes a
 * single-interface device distinguishes, best first:
 *
 *   on_link      inside the on-link prefix of one of this participant's
 *                addresses of the same family, or a loopback address while
 *                this participant itself holds one.
 *   link_local   an IPv6 link-local address (fe80::/10) while this
 *                participant holds an IPv6 address; every IPv6 interface
 *                reaches its link (RFC 4291 §2.5.6).
 *   routed       any other address of a family this participant holds,
 *                reached through its gateway.
 *   unlikely     a family this participant holds no address of, or an IPv4
 *                link-local address (169.254/16) outside its own prefix: a
 *                host with a routable address sends off-prefix traffic to
 *                its router, which never forwards link-local destinations
 *                (RFC 3927 §2.7), and the WIZnet chips do exactly that.
 *   unusable     not kept: a kind other than UDPv4/UDPv6, the unspecified
 *                address, a multicast address in a unicast list, or a
 *                loopback address of another host.
 *
 * A list is ordered by class and keeps its announced order within a class,
 * so a peer's own preference still decides between equally reachable
 * addresses and a participant with no addresses yet keeps every usable
 * locator in announced order.
 *
 * Spec references
 * ---------------
 *   Locator_t kinds              — OMG DDSI-RTPS 2.5 §8.2.5, §9.3.2
 *   Participant locator lists    — OMG DDSI-RTPS 2.5 §8.5.3.2, Table 8.73
 *   Endpoint locator lists       — OMG DDSI-RTPS 2.5 §8.5.4.2
 *   Destination address selection — RFC 6724 §6
 *   IPv4 link-local forwarding   — RFC 3927 §2.6.2, §2.7
 *   IPv6 address scopes          — RFC 4291 §2.4, §2.5.6
 */

#pragma once

#include "types.hpp"

namespace rtps {

/// Reachability class of a remote locator; lower is better (see the file comment).
enum class LocatorReach : uint8_t {
    on_link = 0,
    link_local,
    routed,
    unlikely,
    unusable,
};

/**
 * @brief This participant's own addresses, and the order they imply for remote locators.
 *
 * The participant adds an address when the network assigns one and removes
 * it when the network takes it away (RTPSParticipant::add_ipv4_address(),
 * add_ipv6_address()). The SPDP and SEDP decoders rank each announced
 * locator list with offer() as they read it, and the SPDP peer table and the
 * endpoint match table narrow those lists with select() (see the file
 * comment).
 *
 * A locator kept before an address change keeps the place it was ranked into
 * until the list is announced again: SPDP replaces a peer's record on its
 * next announcement, and an endpoint's record is replaced on its next
 * SEDP announcement.
 *
 * Example:
 * @code{.cpp}
 * rtps::LocalSubnets subnets;
 * subnets.add(rtps::make_udpv4_locator(0xC0A80105u, 0u), 24u);  // 192.168.1.5/24
 * rtps::Locator kept[RTPS_MAX_PEER_LOCATORS];
 * const uint8_t n = subnets.select(kept, RTPS_MAX_PEER_LOCATORS,
 *                                  announced.defaultUnicastLocatorList,
 *                                  announced.defaultUnicastLocatorCount);
 * // kept[0] is the peer's 192.168.1.x address when it announced one.
 * @endcode
 * Full program: tests/spec/src/test_spdp_8_5_3_locator_ranking.cpp
 */
class LocalSubnets {
public:
    /**
     * @brief Records one of this participant's addresses and its on-link prefix.
     *
     * @param[in] address    A UDPv4 or UDPv6 locator holding the address; its
     *   port is ignored.
     * @param[in] prefix_len On-link prefix length in bits: at most 32 for
     *   IPv4 and 128 for IPv6, larger values are clamped. 0 means the prefix
     *   is unknown: the address then marks its family as reachable but places
     *   no remote address on link, except an equal loopback address.
     * @return `false` when the locator is not UDPv4 or UDPv6, or when
     *   RTPS_MAX_LOCATORS_PER_PARTICIPANT addresses are already held; `true`
     *   once it is held. Adding a held address replaces its prefix length.
     *
     * Verification:
     * - Correct means: after the call, rank() places remote addresses inside
     *   the prefix on link.
     * - Covered by: TC-SPDP-8.5.3-008.
     *
     * References:
     * - RFC 6724 §6, Rule 8 (longest matching prefix); RFC 4861 §2.1 (on-link)
     */
    bool add(const Locator& address, uint8_t prefix_len) noexcept;

    /**
     * @brief Forgets one of this participant's addresses.
     *
     * @param[in] address The locator given to add(); its port is ignored.
     * @return `true` when the address was held.
     */
    bool remove(const Locator& address) noexcept;

    /// @brief Forgets every address.
    void clear() noexcept { count_ = 0u; }

    /// @brief Addresses held.
    uint8_t size() const noexcept { return count_; }

    /**
     * @brief Classifies one remote locator against this participant's addresses.
     *
     * @param[in] remote A locator a remote participant or endpoint announced.
     * @return Its LocatorReach class (see the file comment).
     *
     * Verification:
     * - Correct means: each class in the file comment holds for the address
     *   kinds it names, for IPv4 and IPv6 alike.
     * - Covered by: TC-SPDP-8.5.3-008, TC-SPDP-8.5.3-009.
     *
     * References:
     * - RFC 6724 §6; RFC 3927 §2.7; RFC 4291 §2.4, §2.5.6
     */
    LocatorReach rank(const Locator& remote) const noexcept;

    /**
     * @brief Keeps the most reachable locators of an announced list, best first.
     *
     * Replaces keeping a list's leading UDP locators: the locators are taken
     * class by class from on_link to unlikely, each class in announced order,
     * until `cap` are kept; unusable ones are never kept. Senders use the
     * first kept locator (ReaderProxy::best_locator(),
     * WriterProxy::best_locator()), so it is the most reachable one.
     *
     * @param[out] out Destination; at least `cap` locators long.
     * @param[in]  cap Most locators to keep.
     * @param[in]  src Announced locators; may be null when `n` is 0.
     * @param[in]  n   Entries in `src`.
     * @return Locators written to `out`.
     *
     * Verification:
     * - Correct means: every kept locator ranks no worse than every usable
     *   locator left out; equally ranked locators keep their announced order;
     *   no unusable locator is kept.
     * - Covered by: TC-SPDP-8.5.3-007, TC-SPDP-8.5.3-010, TC-SEDP-8.5.5.1-013.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.5.3.2, §8.5.4.2; RFC 6724 §6
     */
    uint8_t select(Locator* out, uint8_t cap, const Locator* src, uint8_t n) const noexcept;

    /**
     * @brief Offers one decoded locator to a bounded list kept most reachable first.
     *
     * The SPDP and SEDP decoders call this for each locator parameter as they
     * read it, so a list holds the `cap` most reachable locators of the whole
     * announcement rather than its first `cap`: a peer that announces its
     * shared-memory locator and four Docker bridges before its LAN address
     * still has the LAN address kept. select() later narrows a list decoded
     * this way without changing its order.
     *
     * The candidate goes behind every kept locator that ranks no worse than
     * it, so equally ranked locators keep their announced order. When the list
     * is full, the last locator is pushed off to make room, or the candidate is
     * not kept when every kept locator ranks no worse than it.
     *
     * @param[in,out] list      The list, ordered as this function leaves it;
     *   at least `cap` locators long.
     * @param[in,out] count     Entries in `list`; at most `cap`.
     * @param[in]     cap       Most locators the list keeps.
     * @param[in]     candidate The locator just decoded.
     * @return `true` when the candidate is kept, `false` when it is unusable
     *   or ranks no better than any of a full list.
     *
     * Verification:
     * - Correct means: after any sequence of offers, the list equals
     *   select() over every locator offered, in the order offered.
     * - Covered by: TC-SPDP-8.5.3-011, TC-SEDP-9.6.3.2-003.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.5.3.2, §9.6.2 (parameter lists); RFC 6724 §6
     */
    bool offer(Locator* list, uint8_t& count, uint8_t cap, const Locator& candidate) const noexcept;

private:
    struct Entry {
        Locator address{};
        uint8_t prefix_len = 0u;
    };

    /// @brief True when this participant holds an address of `remote`'s family.
    bool holds_family(const Locator& remote) const noexcept;

    /// @brief True when `remote` lies inside the on-link prefix of a held address.
    bool on_link(const Locator& remote) const noexcept;

    Entry entries_[RTPS_MAX_LOCATORS_PER_PARTICIPANT] = {};
    uint8_t count_ = 0u;
};

}  // namespace rtps
