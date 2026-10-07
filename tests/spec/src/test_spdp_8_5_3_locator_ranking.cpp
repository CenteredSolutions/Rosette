/**
 * @file test_spdp_8_5_3_locator_ranking.cpp
 * @brief Peer locator lists ranked by reachability from this participant's
 *        addresses (§8.5.3.2 locator lists; RFC 6724 §6).
 *
 * A participant announces every address of every interface. LocalSubnets
 * ranks each one against this participant's addresses and prefixes so the
 * kept, and sent-to, locator is the one this network shares with the peer
 * (see rtps/locator_ranking.hpp).
 */

#include "spdp_test_support.hpp"
#include "spec_test_common.hpp"

#include "rosette/rtps/locator_ranking.hpp"
#include "rosette/rtps/spdp.hpp"

#include <string.h>

namespace spec_test {

using spdp_support::build_announcement;

#define CHECK(COND, MSG)  \
    do {                  \
        if (!(COND)) {    \
            err = (MSG);  \
            return false; \
        }                 \
    } while (0)

namespace {

GuidPrefix make_prefix(Octet a, Octet b, Octet c) {
    GuidPrefix gp{};
    gp.value[0] = a;
    gp.value[1] = b;
    gp.value[2] = c;
    return gp;
}

Locator v6(std::initializer_list<uint8_t> head, uint8_t last, uint16_t port = 7410u) {
    uint8_t a[16] = {};
    uint8_t i = 0u;
    for (uint8_t b : head) {
        a[i++] = b;
    }

    a[15] = last;
    return make_udpv6_locator(a, port);
}

/// Delivers `ann` and returns the peer's active slot: the peer is activated
/// from its record and announces again, which fills the slot with the
/// announcement's ranked lists.
const DiscoveredParticipant* announce_active(SpdpReader& r, const GuidPrefix& gp,
                                             const std::vector<Octet>& ann, uint32_t now_ms) {
    const uint32_t len = static_cast<uint32_t>(ann.size());
    if (!r.on_data(gp, ann.data(), len, now_ms)) {
        return nullptr;
    }

    PeerEntry* peer = r.find_peer(gp);
    if (peer == nullptr || r.activate(*peer) == nullptr
        || !r.on_data(gp, ann.data(), len, now_ms)) {
        return nullptr;
    }

    return r.active_data(*peer);
}

}  // namespace

// RFC 6724 §6 reduced to the classes in rtps/locator_ranking.hpp, for IPv4:
// a prefix decides on link, an unknown prefix decides nothing, an IPv4
// link-local address outside the prefix is not routed, and loopback reaches a
// peer only from the same host. add() of a held address replaces its prefix,
// and remove() forgets it.
static bool tc_spdp_8_5_3_008(std::string& err) {
    LocalSubnets s{};
    CHECK(s.rank(make_udpv4_locator(0xC0A80164u, 7410u)) == LocatorReach::unlikely,
          "with no address held, every family is unlikely");
    CHECK(s.add(make_udpv4_locator(0xC0A80105u, 7410u), 24u), "add 192.168.1.5/24");
    CHECK(s.rank(make_udpv4_locator(0xC0A80164u, 7410u)) == LocatorReach::on_link,
          "192.168.1.100 is on link");
    CHECK(s.rank(make_udpv4_locator(0xAC110001u, 7410u)) == LocatorReach::routed,
          "172.17.0.1 is routed");
    CHECK(s.rank(make_udpv4_locator(0xA9FE0102u, 7410u)) == LocatorReach::unlikely,
          "169.254.1.2 outside the prefix is unlikely");
    CHECK(s.rank(v6({0x20u, 0x01u, 0x0Du, 0xB8u}, 1u)) == LocatorReach::unlikely,
          "an IPv6 address is unlikely with no IPv6 address held");
    CHECK(s.rank(make_udpv4_locator(0x7F000001u, 7410u)) == LocatorReach::unusable,
          "another host's loopback is unusable");
    CHECK(s.rank(make_udpv4_locator(0x00000000u, 7410u)) == LocatorReach::unusable,
          "the unspecified address is unusable");
    CHECK(s.rank(make_udpv4_locator(0xEFFF0001u, 7400u)) == LocatorReach::unusable,
          "a multicast address in a unicast list is unusable");
    Locator shm{};
    shm.kind = 16;
    CHECK(s.rank(shm) == LocatorReach::unusable, "a non-UDP kind is unusable");

    CHECK(s.add(make_udpv4_locator(0xC0A80105u, 0u), 16u), "re-add with /16");
    CHECK(s.size() == 1u, "re-adding an address does not duplicate it");
    CHECK(s.rank(make_udpv4_locator(0xC0A80264u, 7410u)) == LocatorReach::on_link,
          "192.168.2.100 is on link under the replaced /16");

    CHECK(s.add(make_udpv4_locator(0xC0A80105u, 0u), 0u), "re-add with an unknown prefix");
    CHECK(s.rank(make_udpv4_locator(0xC0A80164u, 7410u)) == LocatorReach::routed,
          "an unknown prefix places no peer on link");

    CHECK(s.add(make_udpv4_locator(0x7F000001u, 0u), 0u), "add 127.0.0.1");
    CHECK(s.rank(make_udpv4_locator(0x7F000001u, 7410u)) == LocatorReach::on_link,
          "loopback reaches a peer on this host while this participant holds loopback");
    CHECK(s.remove(make_udpv4_locator(0x7F000001u, 9999u)), "remove ignores the port");
    CHECK(!s.remove(make_udpv4_locator(0x7F000001u, 0u)), "a second remove finds nothing");
    CHECK(s.rank(make_udpv4_locator(0x7F000001u, 7410u)) == LocatorReach::unusable,
          "loopback is unusable again once removed");
    return true;
}

// The same classes for IPv6: the SLAAC prefix decides on link, fe80::/10 is
// reachable from any IPv6 interface, other prefixes are routed, and IPv4 is
// unlikely while only IPv6 is held.
static bool tc_spdp_8_5_3_009(std::string& err) {
    LocalSubnets s{};
    CHECK(s.add(v6({0x20u, 0x01u, 0x0Du, 0xB8u, 0x00u, 0x01u}, 5u), 64u), "add 2001:db8:1::5/64");
    CHECK(s.rank(v6({0x20u, 0x01u, 0x0Du, 0xB8u, 0x00u, 0x01u}, 9u)) == LocatorReach::on_link,
          "2001:db8:1::9 is on link");
    CHECK(s.rank(v6({0xFEu, 0x80u}, 9u)) == LocatorReach::link_local, "fe80::9 is link-local");
    CHECK(s.rank(v6({0x20u, 0x01u, 0x0Du, 0xB8u, 0x00u, 0x02u}, 9u)) == LocatorReach::routed,
          "2001:db8:2::9 is routed");
    CHECK(s.rank(make_udpv4_locator(0xC0A80164u, 7410u)) == LocatorReach::unlikely,
          "IPv4 is unlikely with only IPv6 held");
    CHECK(s.rank(v6({}, 1u)) == LocatorReach::unusable, "another host's ::1 is unusable");
    CHECK(s.rank(v6({}, 0u)) == LocatorReach::unusable, ":: is unusable");
    CHECK(s.rank(v6({0xFFu, 0x02u}, 1u)) == LocatorReach::unusable,
          "ff02::1 in a unicast list is unusable");
    CHECK(s.add(v6({0x20u, 0x01u, 0x0Du, 0xB8u}, 5u), 200u), "an oversized prefix is clamped");
    CHECK(s.rank(v6({0x20u, 0x01u, 0x0Du, 0xB8u}, 6u)) == LocatorReach::routed,
          "a /128 places only the address itself on link");
    return true;
}

// §8.5.3.2: a peer that lists an unreachable interface first is still kept
// at, and sent to, the address it shares with this participant. Equally
// reachable addresses keep the peer's order, and a peer's next announcement
// is ranked against this participant's addresses as they are then.
static bool tc_spdp_8_5_3_010(std::string& err) {
    LocalSubnets subnets{};
    CHECK(subnets.add(make_udpv4_locator(0xC0A80105u, 0u), 24u), "add 192.168.1.5/24");
    SpdpReader r{};
    r.set_local_subnets(subnets);
    const GuidPrefix gp = make_prefix(0x81u, 0x82u, 0x83u);

    const Locator docker = make_udpv4_locator(0xAC110001u, 7410u);  // 172.17.0.1
    const Locator wifi = make_udpv4_locator(0x0A000005u, 7410u);    // 10.0.0.5
    const Locator lan = make_udpv4_locator(0xC0A80164u, 7410u);     // 192.168.1.100
    const Locator lo = make_udpv4_locator(0x7F000001u, 7410u);
    const Locator lan_user = make_udpv4_locator(0xC0A80164u, 7411u);
    const Locator docker_user = make_udpv4_locator(0xAC110001u, 7411u);
    std::vector<Octet> ann{};
    CHECK(build_announcement(gp, {lo, docker, wifi, lan}, {docker_user, lan_user}, ann),
          "announcement");
    const DiscoveredParticipant* peer = announce_active(r, gp, ann, 0u);
    CHECK(peer != nullptr, "peer recorded and active");
    CHECK(peer->metatrafficUnicastLocatorCount == 2u, "two metatraffic locators kept");
    CHECK(peer->metatrafficUnicastLocatorList[0] == lan, "the on-link address comes first");
    CHECK(peer->metatrafficUnicastLocatorList[1] == docker,
          "the routed addresses follow in announced order, loopback is dropped");
    CHECK(peer->defaultUnicastLocatorCount == 2u && peer->defaultUnicastLocatorList[0] == lan_user
              && peer->defaultUnicastLocatorList[1] == docker_user,
          "the default list is ranked the same way");

    CHECK(subnets.remove(make_udpv4_locator(0xC0A80105u, 0u)), "the LAN address goes away");
    CHECK(subnets.add(make_udpv4_locator(0xAC110005u, 0u), 16u), "172.17.0.5/16 arrives");
    CHECK(r.on_data(gp, ann.data(), static_cast<uint32_t>(ann.size()), 10u), "refresh accepted");
    CHECK(peer->metatrafficUnicastLocatorList[0] == docker,
          "the refresh is ranked against the new address");
    return true;
}

// §8.5.3.2 / §9.6.2: a list is ranked as it is decoded, so the reachable
// address is kept even when the peer announces it after more locators than a
// list holds: here a shared-memory locator and four Docker bridges come first.
// offer() keeps what select() would keep from every locator offered.
static bool tc_spdp_8_5_3_011(std::string& err) {
    LocalSubnets subnets{};
    CHECK(subnets.add(make_udpv4_locator(0xC0A80105u, 0u), 24u), "add 192.168.1.5/24");
    SpdpReader r{};
    r.set_local_subnets(subnets);
    const GuidPrefix gp = make_prefix(0x91u, 0x92u, 0x93u);

    Locator shm{};
    shm.kind = 16;
    shm.port = 7410u;
    const Locator bridges[4] = {
        make_udpv4_locator(0xAC110001u, 7410u),  // 172.17.0.1
        make_udpv4_locator(0xAC120001u, 7410u),  // 172.18.0.1
        make_udpv4_locator(0xAC130001u, 7410u),  // 172.19.0.1
        make_udpv4_locator(0xAC140001u, 7410u),  // 172.20.0.1
    };
    const Locator lan = make_udpv4_locator(0xC0A80164u, 7410u);  // 192.168.1.100
    const Locator lan_user = make_udpv4_locator(0xC0A80164u, 7411u);
    std::vector<Octet> ann{};
    CHECK(build_announcement(gp, {shm, bridges[0], bridges[1], bridges[2], bridges[3], lan},
                             {bridges[0], bridges[1], bridges[2], bridges[3], lan_user}, ann),
          "announcement");
    const DiscoveredParticipant* peer = announce_active(r, gp, ann, 0u);
    CHECK(peer != nullptr, "peer recorded and active");
    CHECK(peer->metatrafficUnicastLocatorCount == 2u
              && peer->metatrafficUnicastLocatorList[0] == lan
              && peer->metatrafficUnicastLocatorList[1] == bridges[0],
          "the sixth metatraffic locator is kept first, then the first routed one");
    CHECK(peer->defaultUnicastLocatorCount == 2u && peer->defaultUnicastLocatorList[0] == lan_user
              && peer->defaultUnicastLocatorList[1] == bridges[0],
          "the fifth default locator is kept first, then the first routed one");

    const Locator offered[7] = {shm,
                                bridges[0],
                                lan,
                                bridges[1],
                                make_udpv4_locator(0xC0A80165u, 7410u),
                                bridges[2],
                                make_udpv4_locator(0xC0A80166u, 7410u)};
    Locator list[3] = {};
    uint8_t count = 0u;
    for (const Locator& l : offered) {
        subnets.offer(list, count, 3u, l);
    }

    Locator expect[3] = {};
    const uint8_t expect_count = subnets.select(expect, 3u, offered, 7u);
    CHECK(count == expect_count && list[0] == expect[0] && list[1] == expect[1]
              && list[2] == expect[2],
          "offering each locator keeps what select() keeps from them all");
    CHECK(list[2] == make_udpv4_locator(0xC0A80166u, 7410u),
          "a full list pushes off its worst locator for a better one");
    return true;
}

void append_spdp_tests_8_5_3_locator_ranking(std::vector<TestCase>& tests) {
    tests.push_back(
        {"TC-SPDP-8.5.3-008",
         "RFC 6724 §6 / RFC 3927 §2.7: an IPv4 remote locator is ranked on link, routed, unlikely "
         "or unusable against this participant's addresses and prefixes.",
         tc_spdp_8_5_3_008});
    tests.push_back(
        {"TC-SPDP-8.5.3-009",
         "RFC 6724 §6 / RFC 4291 §2.5.6: an IPv6 remote locator is ranked on link, link-local, "
         "routed, unlikely or unusable against this participant's addresses and prefixes.",
         tc_spdp_8_5_3_009});
    tests.push_back({"TC-SPDP-8.5.3-010",
                     "§8.5.3.2: a peer's unicast locator lists are kept most reachable first, in "
                     "announced order within a class, and re-ranked on each announcement.",
                     tc_spdp_8_5_3_010});
    tests.push_back({"TC-SPDP-8.5.3-011",
                     "§8.5.3.2 / §9.6.2: a peer's most reachable unicast locator is kept however "
                     "many locators it announces before it.",
                     tc_spdp_8_5_3_011});
}

}  // namespace spec_test
