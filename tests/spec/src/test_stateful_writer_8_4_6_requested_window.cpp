/**
 * @file test_stateful_writer_8_4_6_requested_window.cpp
 * @brief ReaderProxy: requested changes and per-reader storage (§8.4.1.1, §8.4.6).
 *
 * A ReaderProxy records a reader's REQUESTED changes in a 64-change window
 * and the rest of changes_for_reader as sequence-number bounds, so its size
 * does not depend on the writer's history depth. These cases check that the
 * window repairs the lowest requests first, bounds a round at 64 changes,
 * keeps the lowest requests when a lower one arrives, and forgets requests
 * the reader acknowledges; and that a matched-reader slot costs exactly one
 * ReaderProxy at any depth.
 */

#include "spec_test_common.hpp"

#include "rosette/rtps/stateful_writer.hpp"

namespace spec_test {

#define CHECK(COND, MSG) do { if (!(COND)) { err = (MSG); return false; } } while (0)

namespace {

Guid make_guid(Octet p0, Octet p1, Octet p2, const EntityId& eid) {
    Guid g{};
    g.prefix.value[0] = p0;
    g.prefix.value[1] = p1;
    g.prefix.value[2] = p2;
    g.entityId = eid;
    return g;
}

/// A set naming every sequence number in [first, last].
SequenceNumberSet run(int64_t first, int64_t last) {
    SequenceNumberSet s{};
    for (int64_t sn = first; sn <= last; ++sn) (void)s.add(SequenceNumber::from_int64(sn));
    return s;
}

/// Takes every REQUESTED change, lowest first, as a repair round does.
std::vector<int64_t> drain_requests(ReaderProxy& proxy) {
    std::vector<int64_t> out{};
    for (SequenceNumber sn = proxy.next_requested_change(); sn != SEQUENCENUMBER_UNKNOWN;
         sn = proxy.next_requested_change()) {
        out.push_back(sn.to_int64());
        proxy.mark_underway(sn);
    }
    return out;
}

}  // namespace

// §8.4.6 requested_changes_set / next_requested_change: requests are
// repaired lowest first, at most kRequestedWindow per round; a lower request
// rebases the window keeping the lowest ones; acknowledged requests are
// forgotten; a request at or below highestAckedSN is ignored.
static bool tc_swr_8_4_6_001(std::string& err) {
    ReaderProxy proxy{};
    const Guid reader = make_guid(0x51u, 0x52u, 0x53u, make_entity(1u, 2u, 3u, 0x07u));
    const Locator loc = make_udpv4_locator(0xC0A80190u, 7411u);
    proxy.init(reader, &loc, 1u, false, SequenceNumber::from_int64(1));

    proxy.requested_changes_set(run(1, 100), 10u);
    CHECK(proxy.nackPending && proxy.nackPendingSinceMs == 10u, "a request arms nackResponseDelay");
    std::vector<int64_t> round = drain_requests(proxy);
    CHECK(round.size() == ReaderProxy::kRequestedWindow && round.front() == 1 && round.back() == 64,
          "the first round repairs exactly the lowest 64 requested changes, in order");
    proxy.requested_changes_set(run(65, 100), 20u);
    round = drain_requests(proxy);
    CHECK(round.size() == 36u && round.front() == 65 && round.back() == 100,
          "the reader's next NACK repairs the rest");

    proxy.requested_changes_set(run(40, 50), 30u);
    proxy.requested_changes_set(run(20, 20), 30u);
    CHECK(proxy.next_requested_change().to_int64() == 20, "a lower request becomes the next repair");
    proxy.requested_changes_set(run(90, 90), 30u);
    round = drain_requests(proxy);
    CHECK(round.size() == 12u && round.front() == 20 && round[1] == 40 && round.back() == 50,
          "rebasing keeps the requests still in the window and drops one beyond it (90)");

    proxy.requested_changes_set(run(30, 35), 40u);
    proxy.acked_changes_set(SequenceNumber::from_int64(32));
    round = drain_requests(proxy);
    CHECK(round.size() == 3u && round.front() == 33, "acknowledged requests are forgotten");
    proxy.requested_changes_set(run(10, 32), 50u);
    CHECK(!proxy.has_requested_changes(), "a request at or below highestAckedSN is ignored");
    return true;
}

// §8.4.1.1: a matched-reader slot costs one ReaderProxy whatever the history
// depth, so a writer with many readers (SEDP: one per participant) grows by
// the proxy size per reader, not by depth × per-change state.
static bool tc_swr_8_4_6_002(std::string& err) {
    constexpr size_t shallow = sizeof(StatefulWriterStorage<1, 2>) - sizeof(StatefulWriterStorage<1, 1>);
    constexpr size_t deep = sizeof(StatefulWriterStorage<64, 2>) - sizeof(StatefulWriterStorage<64, 1>);
    CHECK(shallow == sizeof(ReaderProxy) && deep == sizeof(ReaderProxy),
          "each matched-reader slot costs exactly sizeof(ReaderProxy) at depth 1 and depth 64");

    // Every relevant change in the history is UNSENT after matching,
    // however deep the history.
    const Guid reader = make_guid(0x61u, 0x62u, 0x63u, make_entity(1u, 2u, 3u, 0x07u));
    ReaderProxy proxy{};
    proxy.init(reader, nullptr, 0u, false, SequenceNumber::from_int64(3));
    HistoryCacheStorage<8> history{};
    for (int64_t sn = 1; sn <= 6; ++sn) {
        CacheChange c{};
        c.sequenceNumber = SequenceNumber::from_int64(sn);
        c.kind = ChangeKind::ALIVE;
        CHECK(history.add_change(static_cast<CacheChange&&>(c)) != nullptr, "history add");
    }
    std::vector<int64_t> sent{};
    for (SequenceNumber sn = proxy.next_unsent_change(history); sn != SEQUENCENUMBER_UNKNOWN;
         sn = proxy.next_unsent_change(history)) {
        sent.push_back(sn.to_int64());
        proxy.mark_sent(sn);
    }
    CHECK((sent == std::vector<int64_t>{3, 4, 5, 6}),
          "a reader matched at SN 3 is sent 3..6 in order, and nothing before its match");
    CHECK(proxy.is_announcing(history), "sent but unacknowledged changes keep the proxy announcing");
    proxy.acked_changes_set(SequenceNumber::from_int64(6));
    CHECK(!proxy.is_announcing(history), "acknowledging the newest change makes the proxy idle");
    return true;
}

void append_stateful_writer_tests_8_4_6_requested_window(std::vector<TestCase>& tests) {
    tests.push_back({"TC-SWR-8.4.6-001",
                     "§8.4.6: REQUESTED changes are repaired lowest first, at most 64 per round; acknowledged requests are dropped.",
                     tc_swr_8_4_6_001});
    tests.push_back({"TC-SWR-8.4.6-002",
                     "§8.4.1.1: a ReaderProxy's size is independent of history depth; relevant changes are UNSENT in order after matching.",
                     tc_swr_8_4_6_002});
}

}  // namespace spec_test
