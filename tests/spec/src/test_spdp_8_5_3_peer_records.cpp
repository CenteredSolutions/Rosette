/**
 * @file test_spdp_8_5_3_peer_records.cpp
 * @brief Known-peer records and active slots of the SPDP peer table
 *        (src/rtps/spdp_peer_records.cpp).
 *
 * Every known participant has a compact record; only an active one holds a
 * slot with its full ranked locator lists (see "Known and active peers" in
 * rtps/spdp.hpp).
 */

#include "spdp_test_support.hpp"
#include "spec_test_common.hpp"

#include "rosette/rtps/locator_ranking.hpp"
#include "rosette/rtps/spdp.hpp"

namespace spec_test {

#define CHECK(COND, MSG)  \
    do {                  \
        if (!(COND)) {    \
            err = (MSG);  \
            return false; \
        }                 \
    } while (0)

using spdp_support::build_announcement;

namespace {

/// Activates every peer SPDP reports while `activate` is set, as the
/// participant does while its announcement is in hand.
struct Activator {
    SpdpReader* reader = nullptr;
    bool activate = false;
};

void on_new_peer(const PeerEntry& peer, void* user) noexcept {
    auto* a = static_cast<Activator*>(user);
    PeerEntry* record = a->reader->find_peer(peer.guidPrefix);
    if (a->activate && record != nullptr) {
        (void)a->reader->activate(*record);
    }
}

void on_peer_removed(const PeerEntry&, void*) noexcept {}

GuidPrefix make_prefix(Octet a) {
    GuidPrefix gp{};
    gp.value[0] = a;
    gp.value[1] = 0x5Au;
    return gp;
}

bool announce(SpdpReader& r, const GuidPrefix& gp, const std::vector<Octet>& ann, uint32_t now) {
    return r.on_data(gp, ann.data(), static_cast<uint32_t>(ann.size()), now);
}

}  // namespace

// A record keeps the most reachable metatraffic locator and the port of the
// default locator at that address; a slot filled while the announcement is
// in hand has both ranked lists, and one rebuilt from the record has their
// first entries until the next announcement restores the rest in place. A
// peer whose defaults are at another address is activated only during its
// announcement; removal frees its slot.
static bool tc_spdp_8_5_3_012(std::string& err) {
    LocalSubnets subnets{};
    CHECK(subnets.add(make_udpv4_locator(0xC0A80105u, 0u), 24u), "add 192.168.1.5/24");
    Activator act{};
    SpdpReader r(on_new_peer, on_peer_removed, &act);
    act.reader = &r;
    r.set_local_subnets(subnets);

    const GuidPrefix a = make_prefix(0xA1u);
    const Locator a_meta = make_udpv4_locator(0xC0A8010Au, 7410u);  // 192.168.1.10
    const Locator a_user = make_udpv4_locator(0xC0A8010Au, 7411u);
    const Locator a_bridge = make_udpv4_locator(0xAC110001u, 7410u);  // 172.17.0.1
    const Locator a_bridge_user = make_udpv4_locator(0xAC110001u, 7411u);
    std::vector<Octet> ann_a{};
    CHECK(build_announcement(a, {a_bridge, a_meta}, {a_bridge_user, a_user}, ann_a), "ann A");
    CHECK(announce(r, a, ann_a, 0u), "A accepted");

    PeerEntry* rec_a = r.find_peer(a);
    CHECK(rec_a != nullptr && !rec_a->active() && r.active_count() == 0u,
          "a peer nobody activates stays pending");
    CHECK(rec_a->metatrafficLocator == a_meta && rec_a->defaultUnicastPort == 7411u
              && rec_a->rebuildable(),
          "the record keeps the on-link metatraffic locator and its address's default port");

    const DiscoveredParticipant* slot = r.activate(*rec_a);
    CHECK(slot != nullptr && rec_a->active() && r.active_count() == 1u,
          "a rebuildable record is activated outside its announcement");
    CHECK(slot->metatrafficUnicastLocatorCount == 1u
              && slot->metatrafficUnicastLocatorList[0] == a_meta
              && slot->defaultUnicastLocatorCount == 1u
              && slot->defaultUnicastLocatorList[0] == a_user,
          "the rebuilt slot holds the first entry of each list");

    CHECK(announce(r, a, ann_a, 10u), "A announces again");
    CHECK(r.active_data(*rec_a) == slot && slot->metatrafficUnicastLocatorCount == 2u
              && slot->metatrafficUnicastLocatorList[1] == a_bridge
              && slot->defaultUnicastLocatorCount == 2u
              && slot->defaultUnicastLocatorList[1] == a_bridge_user,
          "the next announcement restores both ranked lists in the same slot");

    r.deactivate(*rec_a);
    CHECK(!rec_a->active() && r.active_count() == 0u && r.active_data(*rec_a) == nullptr,
          "deactivate frees the slot");

    // B's default locator is at another address than its metatraffic one.
    const GuidPrefix b = make_prefix(0xB1u);
    const Locator b_meta = make_udpv4_locator(0xC0A80114u, 7410u);
    const Locator b_user = make_udpv4_locator(0xC0A80115u, 7411u);
    std::vector<Octet> ann_b{};
    CHECK(build_announcement(b, {b_meta}, {b_user}, ann_b), "ann B");
    CHECK(announce(r, b, ann_b, 20u), "B accepted");
    PeerEntry* rec_b = r.find_peer(b);
    CHECK(rec_b != nullptr && rec_b->defaultUnicastPort == 0u && !rec_b->rebuildable()
              && !r.can_activate(*rec_b) && r.activate(*rec_b) == nullptr,
          "a record that cannot rebuild the default list is not activated from the record");

    act.activate = true;
    CHECK(announce(r, b, ann_b, 30u), "B announces again");
    slot = r.active_data(*rec_b);
    CHECK(slot != nullptr && slot->defaultUnicastLocatorCount == 1u
              && slot->defaultUnicastLocatorList[0] == b_user,
          "during its announcement it is activated with the announced lists");

    const GuidPrefix c = make_prefix(0xC1u);
    std::vector<Octet> ann_c{};
    CHECK(build_announcement(c, {}, {}, ann_c), "ann C");
    CHECK(announce(r, c, ann_c, 40u), "C accepted");
    const PeerEntry* rec_c = r.find_peer(c);
    CHECK(rec_c != nullptr && !rec_c->has_metatraffic_locator() && !rec_c->rebuildable(),
          "a peer announcing no metatraffic locator is recorded without one");

    CHECK(r.remove_peer(b) && r.active_count() == 0u, "removing an active peer frees its slot");
    return true;
}

void append_spdp_tests_8_5_3_peer_records(std::vector<TestCase>& tests) {
    tests.push_back({"TC-SPDP-8.5.3-012",
                     "§8.5.3.2: a known-peer record keeps the most reachable metatraffic locator "
                     "and its address's default port; an active slot holds the ranked lists, "
                     "from the announcement in hand or rebuilt from a record that can restore "
                     "both, and is freed on deactivation or removal.",
                     tc_spdp_8_5_3_012});
}

}  // namespace spec_test
