/**
 * @file test_participant_8_4_4_reader_reliability.cpp
 * @brief TC-PART-8.4.4-001: a discovered reader's announced reliability reaches the writer.
 *
 * A writer's behavior toward each matched reader depends on the
 * reliabilityLevel of both (§8.4.4), and a remote reader's comes from its SEDP
 * announcement. The participant hands each match to the writer
 * (RTPSParticipant::on_endpoint_match()), so this checks the path end to end:
 * a BEST_EFFORT and a RELIABLE reader of one RELIABLE writer, discovered
 * through the match table, are served each by its own protocol.
 */

#include "participant_test_support.hpp"

namespace spec_test {

extern uint32_t g_participant_test_now_ms;

#define CHECK(COND, MSG)  \
    do {                  \
        if (!(COND)) {    \
            err = (MSG);  \
            return false; \
        }                 \
    } while (0)

namespace {

using namespace participant_support;

constexpr Octet kPayload[4] = {0x00u, 0x01u, 0x00u, 0x00u};

/// A reader of `rt/state` announced by `prefix`, with the given reliability.
DiscoveredReaderData state_reader(const GuidPrefix& prefix, Octet key, ReliabilityKind kind) {
    DiscoveredReaderData rd{};
    rd.readerGuid = Guid{prefix, make_entity(0x31u, 0x32u, key, 0x07u)};
    rd.topicName.assign("rt/state");
    rd.typeName.assign("pkg::msg::dds_::State_");
    rd.reliabilityKind = kind;
    rd.durabilityKind = DurabilityKind::VOLATILE;
    rd.unicastLocatorCount = 1u;
    rd.unicastLocatorList[0] = make_udpv4_locator(0xC0A80150u | key, 7411u);
    return rd;
}

}  // namespace

static bool tc_part_8_4_4_001(std::string& err) {
    CaptureTransport tx{};
    RTPSParticipant  p(0u, 1u, tx);
    CHECK(p.add_ipv4_address(0xC0A8010Au), "local address setup should succeed");

    WriterConfig cfg{};
    (void)cfg.topicName.assign("rt/state");
    (void)cfg.typeName.assign("pkg::msg::dds_::State_");
    StatefulWriter* w = p.create_writer<2>(cfg);
    CHECK(w != nullptr && w->reliability() == ReliabilityKind::RELIABLE,
          "a RELIABLE state writer is created");

    const GuidPrefix   peer = make_prefix(0xD1u, 0xD2u, 0xD3u);
    std::vector<Octet> spdp{};
    const Locator      meta = make_udpv4_locator(0xC0A80150u, 7410u);
    const Locator      user = make_udpv4_locator(0xC0A80150u, 7411u);
    CHECK(build_spdp_payload(peer, 30u, meta, spdp, user), "SPDP payload builds");
    g_participant_test_now_ms = 100u;
    CHECK(p.spdp_reader().on_data(peer, spdp.data(), static_cast<uint32_t>(spdp.size()), 100u),
          "the peer is discovered");

    const DiscoveredReaderData echo = state_reader(peer, 0x01u, ReliabilityKind::BEST_EFFORT);
    p.match_table().add_reader(echo);
    const ReaderProxy* echo_proxy = w->matched_reader_lookup(echo.readerGuid);
    CHECK(echo_proxy != nullptr, "the BEST_EFFORT reader matches the RELIABLE writer");
    CHECK(!echo_proxy->isReliable, "its proxy records it as BEST_EFFORT");

    for (int i = 0; i < 5; ++i) {
        CHECK(w->write_async(ChangeKind::ALIVE, kPayload, sizeof(kPayload)), "write");
    }

    CHECK(w->evicted_unacknowledged() == 0u && p.evicted_unacknowledged() == 0u,
          "a best-effort reader that never acknowledges loses no counted sample");

    const DiscoveredReaderData client = state_reader(peer, 0x02u, ReliabilityKind::RELIABLE);
    p.match_table().add_reader(client);
    const ReaderProxy* client_proxy = w->matched_reader_lookup(client.readerGuid);
    CHECK(client_proxy != nullptr && client_proxy->isReliable,
          "a RELIABLE reader of the same writer is served reliably");

    for (int i = 0; i < 4; ++i) {
        CHECK(w->write_async(ChangeKind::ALIVE, kPayload, sizeof(kPayload)), "write");
    }

    CHECK(w->evicted_unacknowledged() == 2u,
          "changes evicted before the reliable reader acknowledged them count");
    return true;
}

void append_participant_reader_reliability_tests(std::vector<TestCase>& tests) {
    tests.push_back({"TC-PART-8.4.4-001",
                     "§8.4.4: a discovered reader's announced reliability reaches the writer it "
                     "matches, so a RELIABLE writer serves a BEST_EFFORT reader best-effort and a "
                     "RELIABLE one reliably.",
                     tc_part_8_4_4_001});
}

}  // namespace spec_test
