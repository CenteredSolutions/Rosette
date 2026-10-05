/**
 * @file test_sedp_8_5_4_peer_locators.cpp
 * @brief Which of a discovered participant's locators the SEDP builtin
 *        endpoints are matched at (§8.5.4.1).
 *
 * The SPDP peer table keeps a peer's metatraffic and default unicast locators
 * and no multicast list (DiscoveredParticipant in rtps/spdp.hpp). SEDP is
 * exchanged only at the metatraffic unicast locators, so a peer that
 * announces none is recorded by SPDP but matched by neither builtin writer
 * nor builtin reader.
 */

#include "sedp_peer_driver.hpp"
#include "sedp_recorded_endpoints.hpp"
#include "spec_test_common.hpp"

#include "rosette/rtps/sedp.hpp"
#include "rosette/rtps/transport.hpp"

namespace spec_test {

#define CHECK(COND, MSG)  \
    do {                  \
        if (!(COND)) {    \
            err = (MSG);  \
            return false; \
        }                 \
    } while (0)

namespace {

struct RecordingTransport : ITransport {
    std::vector<Locator> dests{};

    bool send_unicast(const BufRef&, const Locator& dest) noexcept override {
        dests.push_back(dest);
        return true;
    }

    bool send_multicast(const BufRef&, const Locator&) noexcept override { return false; }
};

GuidPrefix make_prefix(Octet a, Octet b, Octet c) {
    GuidPrefix gp{};
    gp.value[0] = a;
    gp.value[1] = b;
    gp.value[2] = c;
    return gp;
}

}  // namespace

// §8.5.4.1: a peer with no metatraffic unicast locator is matched by neither
// builtin writer (no announcement is replayed to it) nor builtin reader (its
// HEARTBEATs go unanswered). Once it announces one, both are matched there.
static bool tc_sedp_8_5_4_018(std::string& err) {
    RecordingTransport tx{};
    const GuidPrefix local_prefix = make_prefix(0x31u, 0x32u, 0x33u);
    SpecSedpWriter writer(local_prefix, tx);
    DiscoveredWriterData wd{};
    wd.writerGuid = Guid{local_prefix, make_entity(1u, 2u, 3u, 0xC2u)};
    wd.topicName.assign("chatter");
    wd.typeName.assign("std_msgs::msg::String");
    wd.reliabilityKind = ReliabilityKind::RELIABLE;
    wd.durabilityKind = DurabilityKind::TRANSIENT_LOCAL;
    CHECK(writer.register_writer(wd), "register_writer");
    EndpointMatchTable table(nullptr, nullptr, nullptr);
    table.set_local_prefix(local_prefix);
    SedpReader reader(local_prefix, tx, table);
    tx.dests.clear();

    const SedpPeer peer{make_prefix(0x41u, 0x42u, 0x43u), make_udpv4_locator(0xC0A80164u, 7410u)};
    DiscoveredParticipant no_unicast = peer.spdp();
    no_unicast.metatrafficUnicastLocatorCount = 0u;
    writer.on_peer_discovered(no_unicast);
    reader.on_peer_discovered(no_unicast);
    CHECK(tx.dests.empty(),
          "nothing is replayed or acknowledged to a peer with no unicast locator");
    const EntityId pubs = entity_id::SEDP_BUILTIN_PUBLICATIONS_ANNOUNCER;
    const Guid announcer{peer.prefix, pubs};
    CHECK(reader.reader_for_writer(pubs)->matched_writer_lookup(announcer) == nullptr,
          "the peer's announcers are not matched");

    writer.on_peer_discovered(peer.spdp());
    reader.on_peer_discovered(peer.spdp());
    CHECK(reader.reader_for_writer(pubs)->matched_writer_lookup(announcer) != nullptr,
          "a unicast locator matches the peer's announcers");
    CHECK(!tx.dests.empty(), "and replays this participant's announcements to the peer");
    for (const Locator& d : tx.dests) {
        CHECK(d == peer.meta, "SEDP goes to the metatraffic unicast locator");
    }

    return true;
}

void append_sedp_tests_8_5_4_peer_locators(std::vector<TestCase>& tests) {
    tests.push_back({"TC-SEDP-8.5.4-018",
                     "§8.5.4.1: the SEDP builtin endpoints are matched at a peer's metatraffic "
                     "unicast locators, and not at all when it announces none.",
                     tc_sedp_8_5_4_018});
}

}  // namespace spec_test
