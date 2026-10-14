/**
 * @file test_stateful_writer_8_4_9_1_best_effort_reader.cpp
 * @brief A RELIABLE StatefulWriter serves a BEST_EFFORT reader by the best-effort protocol.
 *
 * DDS lets a RELIABLE DataWriter match a BEST_EFFORT DataReader (DDS 1.4
 * §2.2.3, RELIABILITY is requested/offered), and RTPS makes the writer's
 * behavior toward each matched reader depend on the reliabilityLevel of both
 * (§8.4.4). Toward a best-effort reader the writer follows the best-effort
 * StatefulWriter behavior (§8.4.9.1, Table 8.65): it pushes each change once,
 * sends no HEARTBEAT, and expects no ACKNACK. Such a reader therefore never
 * acknowledges anything, so treating it like a reliable one would hold every
 * change in the history until KEEP_LAST evicts it, count each eviction as a
 * lost sample, and send it HEARTBEATs it ignores. A `ros2 topic echo
 * --qos-reliability best_effort` of a reliable state topic is that reader.
 */

#include "spec_test_common.hpp"

#include "rosette/rtps/stateful_writer.hpp"

namespace spec_test {

extern uint32_t g_stateful_writer_test_now_ms;

#define CHECK(COND, MSG)  \
    do {                  \
        if (!(COND)) {    \
            err = (MSG);  \
            return false; \
        }                 \
    } while (0)

namespace {

const Locator kLocReliable = make_udpv4_locator(0xC0A80181u, 7411u);
const Locator kLocBestEffort = make_udpv4_locator(0xC0A80182u, 7411u);
const Octet   kPayload[4] = {1u, 2u, 3u, 4u};

/// Submessages one destination received.
struct Received {
    int data = 0;
    int heartbeats = 0;
    int gaps = 0;
};

/// Counts DATA, HEARTBEAT and GAP submessages per destination locator.
struct SortingTransport : ITransport {
    Received reliable{};
    Received best_effort{};

    bool send_unicast(const BufRef& buf, const Locator& dest) noexcept override {
        Received* to = dest == kLocReliable ? &reliable : &best_effort;
        ParseOps  ops{};
        ops.on_data = [](const ParseContext& c, const ParsedData&) noexcept {
            ++static_cast<Received*>(c.user_data)->data;
        };
        ops.on_heartbeat = [](const ParseContext& c, const ParsedHeartbeat&) noexcept {
            ++static_cast<Received*>(c.user_data)->heartbeats;
        };
        ops.on_gap = [](const ParseContext& c, const ParsedGap&) noexcept {
            ++static_cast<Received*>(c.user_data)->gaps;
        };
        (void)parse_message(buf.data, buf.size, GuidPrefix{}, ops, to);
        return true;
    }

    bool send_multicast(const BufRef&, const Locator&) noexcept override { return true; }
};

Guid make_guid(Octet p0, const EntityId& eid) {
    Guid g{};
    g.prefix.value[0] = p0;
    g.prefix.value[1] = 0x9Bu;
    g.entityId = eid;
    return g;
}

/// Delivers an ACKNACK from `reader`: everything below `base` acknowledged, `sns` requested.
void acknack(StatefulWriter& w, const Guid& reader, int64_t base,
             std::initializer_list<int64_t> sns, int32_t count) {
    ParsedAckNack ack{};
    ack.readerId = reader.entityId;
    ack.writerId = w.guid().entityId;
    ack.readerSNState.clear();
    ack.readerSNState.bitmapBase = SequenceNumber::from_int64(base);
    for (int64_t sn : sns) {
        ack.readerSNState.set(static_cast<uint32_t>(sn - base));
    }

    ack.count = count;
    ParseContext ctx{};
    ctx.senderGuidPrefix = reader.prefix;
    w.on_acknack(ctx, ack);
}

const Guid kWriter = make_guid(0x11u, make_entity(1u, 2u, 3u, 0x02u));
const Guid kReliableReader = make_guid(0x22u, make_entity(4u, 5u, 6u, 0x07u));
const Guid kBestEffortReader = make_guid(0x33u, make_entity(4u, 5u, 6u, 0x07u));

}  // namespace

// §8.4.4, §8.4.9.1: a RELIABLE VOLATILE writer whose only reader is
// BEST_EFFORT pushes each change once with no HEARTBEAT, keeps nothing,
// counts no eviction, never schedules a HEARTBEAT, and acts on no ACKNACK.
static bool tc_swr_8_4_9_1_001(std::string& err) {
    SortingTransport            tx{};
    StatefulWriterStorage<4, 1> w(kWriter, ReliabilityKind::RELIABLE, DurabilityKind::VOLATILE, tx,
                                  5u, 1000u);
    CHECK(w.matched_reader_add(kBestEffortReader, &kLocBestEffort, 1u, false,
                               ReliabilityKind::BEST_EFFORT),
          "a RELIABLE writer matches a BEST_EFFORT reader");
    CHECK(!w.matched_reader_lookup(kBestEffortReader)->isReliable,
          "the proxy records the reader as BEST_EFFORT");

    g_stateful_writer_test_now_ms = 1000u;
    for (int i = 0; i < 10; ++i) {
        CHECK(w.write_async(ChangeKind::ALIVE, kPayload, 4u), "write");
    }

    CHECK(tx.best_effort.data == 10, "each change is pushed to the reader once");
    CHECK(tx.best_effort.heartbeats == 0, "no HEARTBEAT is piggybacked for a best-effort reader");
    CHECK(w.cache().size() == 0u,
          "a change sent to its only best-effort reader leaves the history");
    CHECK(w.evicted_unacknowledged() == 0u, "nothing a best-effort reader lacks counts as lost");
    CHECK(w.acknowledged_by_all(), "a best-effort reader owes no acknowledgment");
    CHECK(w.acknowledged_by_participant(kBestEffortReader.prefix),
          "nor does its participant, so a peer of best-effort readers may be parked");
    CHECK(w.ms_until_next_timer(1000u) == UINT32_MAX, "no periodic HEARTBEAT is scheduled");

    acknack(w, kBestEffortReader, 9, {9}, 1);
    g_stateful_writer_test_now_ms = 5000u;
    w.tick(5000u);
    CHECK(tx.best_effort.data == 10 && tx.best_effort.gaps == 0 && tx.best_effort.heartbeats == 0,
          "an ACKNACK from a best-effort reader is not acted on");
    return true;
}

// §8.4.4: one RELIABLE writer serves a RELIABLE and a BEST_EFFORT reader each
// by its own protocol. Only the reliable reader holds changes, receives
// HEARTBEATs, and makes an eviction count; once it acknowledges, nothing is
// left for the best-effort one.
static bool tc_swr_8_4_9_1_002(std::string& err) {
    SortingTransport            tx{};
    StatefulWriterStorage<4, 2> w(kWriter, ReliabilityKind::RELIABLE, DurabilityKind::VOLATILE, tx,
                                  5u, 1000u);
    CHECK(
        w.matched_reader_add(kReliableReader, &kLocReliable, 1u, false, ReliabilityKind::RELIABLE),
        "match the reliable reader");
    CHECK(w.matched_reader_add(kBestEffortReader, &kLocBestEffort, 1u, false,
                               ReliabilityKind::BEST_EFFORT),
          "match the best-effort reader");

    g_stateful_writer_test_now_ms = 1000u;
    for (int i = 0; i < 6; ++i) {
        CHECK(w.write_async(ChangeKind::ALIVE, kPayload, 4u), "write");
    }

    CHECK(tx.reliable.data == 6 && tx.best_effort.data == 6, "both readers are sent every change");
    CHECK(tx.reliable.heartbeats > 0, "the reliable reader receives piggybacked HEARTBEATs");
    CHECK(tx.best_effort.heartbeats == 0, "the best-effort reader receives none");
    CHECK(w.cache().size() == 4u, "the reliable reader's unacknowledged changes stay");
    CHECK(w.evicted_unacknowledged() == 2u,
          "the two changes evicted before the reliable reader acknowledged count");
    CHECK(!w.acknowledged_by_all(), "the reliable reader still owes acknowledgments");

    const int reliable_hbs = tx.reliable.heartbeats;
    g_stateful_writer_test_now_ms = 2100u;
    w.tick(2100u);
    CHECK(tx.reliable.heartbeats == reliable_hbs + 1, "the periodic HEARTBEAT goes to it");
    CHECK(tx.best_effort.heartbeats == 0, "and not to the best-effort reader");

    acknack(w, kReliableReader, 7, {}, 1);
    CHECK(w.cache().size() == 0u, "acknowledged by the reliable reader, every change leaves");
    CHECK(w.acknowledged_by_all(), "every reader served reliably has acknowledged");
    return true;
}

// §8.4.4, §8.7.2.2: a RELIABLE TRANSIENT_LOCAL writer replays its history to a
// newly matched BEST_EFFORT reader without the HEARTBEAT a reliable reader's
// replay carries, and keeps the history for readers that match later.
static bool tc_swr_8_4_9_1_003(std::string& err) {
    SortingTransport            tx{};
    StatefulWriterStorage<4, 1> w(kWriter, ReliabilityKind::RELIABLE,
                                  DurabilityKind::TRANSIENT_LOCAL, tx, 5u, 1000u);
    g_stateful_writer_test_now_ms = 1000u;
    for (int i = 0; i < 3; ++i) {
        CHECK(w.write_async(ChangeKind::ALIVE, kPayload, 4u), "write before any reader");
    }

    CHECK(w.matched_reader_add(kBestEffortReader, &kLocBestEffort, 1u, false,
                               ReliabilityKind::BEST_EFFORT),
          "match the best-effort reader");
    CHECK(tx.best_effort.data == 3, "the history is replayed to it");
    CHECK(tx.best_effort.heartbeats == 0, "with no HEARTBEAT");
    CHECK(w.cache().size() == 3u, "the history stays for readers that match later");
    return true;
}

void append_stateful_writer_tests_8_4_9_1_best_effort_reader(std::vector<TestCase>& tests) {
    tests.push_back({"TC-SWR-8.4.9.1-001",
                     "§8.4.4/§8.4.9.1: toward a BEST_EFFORT reader a RELIABLE writer pushes "
                     "each change once, sends no HEARTBEAT, holds nothing, counts no eviction "
                     "and acts on no ACKNACK.",
                     tc_swr_8_4_9_1_001});
    tests.push_back({"TC-SWR-8.4.9.1-002",
                     "§8.4.4: a RELIABLE writer serves a RELIABLE and a BEST_EFFORT reader each "
                     "by its own protocol; only the reliable one holds changes, receives "
                     "HEARTBEATs and counts evictions.",
                     tc_swr_8_4_9_1_002});
    tests.push_back({"TC-SWR-8.4.9.1-003",
                     "§8.4.4/§8.7.2.2: a TRANSIENT_LOCAL replay to a BEST_EFFORT reader carries "
                     "no HEARTBEAT, and the history stays.",
                     tc_swr_8_4_9_1_003});
}

}  // namespace spec_test
