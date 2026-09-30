/**
 * @file test_stateful_writer_8_4_7_release_finished.cpp
 * @brief VOLATILE StatefulWriter: a change every matched reader is done with leaves the history.
 *
 * KEEP_LAST(depth) bounds a history; it does not keep it full. A VOLATILE
 * writer never sends a change to a reader that matches after it was written
 * (§8.7.2.2.1), so once every matched reader has acknowledged a change
 * (RELIABLE) or been sent it (BEST_EFFORT), nothing can use it and its
 * payload block goes back to the pool. On a board whose pool holds a few
 * large blocks this is what lets a service answer an unbounded run of large
 * replies: each reply writer otherwise pins its last depth replies. A
 * TRANSIENT_LOCAL writer keeps its history for readers that match later,
 * except that a service reply writer drops each reply once the client that
 * asked for it has it.
 */

#include "spec_test_common.hpp"

#include "rosette/rtps/gap_encoding.hpp"
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

struct CaptureTransport : ITransport {
    std::vector<std::vector<Octet>> unicast_msgs{};

    bool send_unicast(const BufRef& buf, const Locator&) noexcept override {
        unicast_msgs.emplace_back(buf.data, buf.data + buf.size);
        return true;
    }

    bool send_multicast(const BufRef&, const Locator&) noexcept override { return true; }
};

Guid make_guid(Octet p0, const EntityId& eid) {
    Guid g{};
    g.prefix.value[0] = p0;
    g.prefix.value[1] = 0x7Cu;
    g.entityId = eid;
    return g;
}

/// An ACKNACK from `reader` acknowledging everything below `base` and requesting `sns`.
ParsedAckNack acknack(const Guid& reader, const Guid& writer, int64_t base,
                      std::initializer_list<int64_t> sns, int32_t count) {
    ParsedAckNack ack{};
    ack.readerId = reader.entityId;
    ack.writerId = writer.entityId;
    ack.readerSNState.clear();
    ack.readerSNState.bitmapBase = SequenceNumber::from_int64(base);
    for (int64_t sn : sns) {
        ack.readerSNState.set(static_cast<uint32_t>(sn - base));
    }

    ack.count = count;
    return ack;
}

void deliver(StatefulWriter& w, const Guid& reader, const ParsedAckNack& ack) {
    ParseContext ctx{};
    ctx.senderGuidPrefix = reader.prefix;
    w.on_acknack(ctx, ack);
}

/// The GAPs and HEARTBEATs in every captured message, keeping the last of each.
struct Seen {
    int gaps = 0;
    int heartbeats = 0;
    ParsedGap gap{};
    ParsedHeartbeat heartbeat{};
};

Seen inspect(const CaptureTransport& tx) {
    Seen seen{};
    ParseOps ops{};
    ops.on_gap = [](const ParseContext& c, const ParsedGap& g) noexcept {
        auto* out = static_cast<Seen*>(c.user_data);
        ++out->gaps;
        out->gap = g;
    };
    ops.on_heartbeat = [](const ParseContext& c, const ParsedHeartbeat& hb) noexcept {
        auto* out = static_cast<Seen*>(c.user_data);
        ++out->heartbeats;
        out->heartbeat = hb;
    };
    for (const auto& msg : tx.unicast_msgs) {
        (void)parse_message(msg.data(), static_cast<uint32_t>(msg.size()), GuidPrefix{}, ops,
                            &seen);
    }

    return seen;
}

bool holds(const StatefulWriter& w, int64_t sn) {
    return w.cache().get_change(SequenceNumber::from_int64(sn)) != nullptr;
}

const Locator kLocA = make_udpv4_locator(0xC0A80181u, 7411u);
const Locator kLocB = make_udpv4_locator(0xC0A80182u, 7411u);
const Octet kPayload[4] = {1u, 2u, 3u, 4u};

}  // namespace

// §8.4.7.1, §8.7.2.2.1: a RELIABLE VOLATILE writer drops a change once the
// last matched reader acknowledges it, and not before; its payload block
// returns to the pool; an unmatch counts as that reader being done.
static bool tc_swr_8_4_7_003(std::string& err) {
    CaptureTransport tx{};
    const Guid writer = make_guid(0x11u, make_entity(1u, 2u, 3u, 0x02u));
    const Guid a = make_guid(0x22u, make_entity(4u, 5u, 6u, 0x07u));
    const Guid b = make_guid(0x33u, make_entity(4u, 5u, 6u, 0x07u));
    StatefulWriterStorage<8, 2> w(writer, ReliabilityKind::RELIABLE, DurabilityKind::VOLATILE, tx,
                                  5u, 1000u);
    CHECK(w.matched_reader_add(a, &kLocA, 1u, false), "match A");
    CHECK(w.matched_reader_add(b, &kLocB, 1u, false), "match B");
    for (int i = 0; i < 3; ++i) {
        CHECK(w.write_async(ChangeKind::ALIVE, kPayload, 4u), "write");
    }

    CHECK(w.cache().size() == 3u, "unacknowledged changes stay");
    const BufBlock* first_block =
        w.cache().get_change(SequenceNumber::from_int64(1))->payload.block;

    deliver(w, a, acknack(a, writer, 4, {}, 1));
    CHECK(w.cache().size() == 3u, "one reader's acknowledgement is not enough");

    deliver(w, b, acknack(b, writer, 3, {}, 1));
    CHECK(!holds(w, 1) && !holds(w, 2) && holds(w, 3),
          "changes both readers acknowledged leave; the rest stay");
    CHECK(first_block->refcount.load() == 0,
          "the released change's block is back on the free list");

    w.matched_reader_remove(b);
    CHECK(w.cache().size() == 0u, "unmatching the last holdout releases what it held");

    CHECK(w.write_async(ChangeKind::ALIVE, kPayload, 4u), "write SN 4");
    CHECK(holds(w, 4), "a change the remaining reader has not acknowledged stays");
    return true;
}

// §8.7.2.2: a TRANSIENT_LOCAL writer keeps acknowledged changes for readers
// that match later, and replays them to the next one.
static bool tc_swr_8_4_7_004(std::string& err) {
    CaptureTransport tx{};
    const Guid writer = make_guid(0x44u, make_entity(1u, 2u, 3u, 0x02u));
    const Guid a = make_guid(0x55u, make_entity(4u, 5u, 6u, 0x07u));
    const Guid late = make_guid(0x66u, make_entity(4u, 5u, 6u, 0x07u));
    StatefulWriterStorage<4, 2> w(writer, ReliabilityKind::RELIABLE,
                                  DurabilityKind::TRANSIENT_LOCAL, tx, 5u, 1000u);
    CHECK(w.matched_reader_add(a, &kLocA, 1u, false), "match");
    for (int i = 0; i < 3; ++i) {
        CHECK(w.write_async(ChangeKind::ALIVE, kPayload, 4u), "write");
    }

    deliver(w, a, acknack(a, writer, 4, {}, 1));
    w.matched_reader_remove(a);
    CHECK(w.cache().size() == 3u, "acknowledged changes stay in a TRANSIENT_LOCAL history");

    tx.unicast_msgs.clear();
    CHECK(w.matched_reader_add(late, &kLocB, 1u, false), "late match");
    CHECK(tx.unicast_msgs.size() >= 3u, "the late reader is sent the kept history");
    return true;
}

// §8.4.8.1, §8.7.2.2.1: a BEST_EFFORT VOLATILE writer drops a change once it
// has been sent to every matched reader; with no reader matched, a VOLATILE
// writer of either reliability keeps nothing, and a reader matched later is
// sent nothing written before it.
static bool tc_swr_8_4_7_005(std::string& err) {
    CaptureTransport tx{};
    const Guid writer = make_guid(0x77u, make_entity(1u, 2u, 3u, 0x03u));
    const Guid a = make_guid(0x88u, make_entity(4u, 5u, 6u, 0x04u));
    StatefulWriterStorage<4, 1> be(writer, ReliabilityKind::BEST_EFFORT, DurabilityKind::VOLATILE,
                                   tx, 5u, 1000u);
    CHECK(be.matched_reader_add(a, &kLocA, 1u, false), "match");
    CHECK(be.write_async(ChangeKind::ALIVE, kPayload, 4u), "best-effort write");
    CHECK(tx.unicast_msgs.size() == 1u, "the change is sent");
    CHECK(be.cache().size() == 0u, "and then released: best effort repairs nothing");

    StatefulWriterStorage<4, 1> rel(writer, ReliabilityKind::RELIABLE, DurabilityKind::VOLATILE, tx,
                                    5u, 1000u);
    CHECK(rel.write_async(ChangeKind::ALIVE, kPayload, 4u), "reliable write with no reader");
    CHECK(rel.cache().size() == 0u, "no reader can ever receive it, so it is released");

    tx.unicast_msgs.clear();
    CHECK(rel.matched_reader_add(a, &kLocA, 1u, false), "late match");
    rel.tick(0u);
    CHECK(tx.unicast_msgs.empty(), "the late reader is sent nothing and is not heartbeated");
    CHECK(rel.acknowledged_by_all(), "an empty history is acknowledged by all");
    return true;
}

// §8.4.9.2.12, §8.4.2.2: after a release, a NACK for the dropped change is
// answered with a GAP naming it, and a HEARTBEAT advertises the empty
// history as firstSN = lastSN + 1.
static bool tc_swr_8_4_7_006(std::string& err) {
    CaptureTransport tx{};
    const Guid writer = make_guid(0x99u, make_entity(1u, 2u, 3u, 0x02u));
    const Guid a = make_guid(0xAAu, make_entity(4u, 5u, 6u, 0x07u));
    StatefulWriterStorage<4, 1> w(writer, ReliabilityKind::RELIABLE, DurabilityKind::VOLATILE, tx,
                                  5u, 1000u);
    CHECK(w.matched_reader_add(a, &kLocA, 1u, false), "match");
    CHECK(w.write_async(ChangeKind::ALIVE, kPayload, 4u), "write SN 1");
    CHECK(w.write_async(ChangeKind::ALIVE, kPayload, 4u), "write SN 2");
    deliver(w, a, acknack(a, writer, 3, {}, 1));
    CHECK(w.cache().size() == 0u, "both changes are acknowledged and released");

    tx.unicast_msgs.clear();
    g_stateful_writer_test_now_ms = 100u;
    deliver(w, a, acknack(a, writer, 1, {1, 2}, 2));
    w.tick(200u);

    Seen seen = inspect(tx);
    CHECK(seen.gaps == 1, "one GAP answers the NACK for released changes");
    CHECK(seen.gap.gapStart == SequenceNumber::from_int64(1)
              && gap_run_end(seen.gap.gapStart, seen.gap.gapList) == SequenceNumber::from_int64(3),
          "the GAP names exactly SNs 1 and 2");

    tx.unicast_msgs.clear();
    w.send_heartbeat();
    seen = inspect(tx);
    CHECK(seen.heartbeats == 1, "the HEARTBEAT is sent");
    CHECK(seen.heartbeat.firstSN == SequenceNumber::from_int64(3)
              && seen.heartbeat.lastSN == SequenceNumber::from_int64(2),
          "the empty history is advertised as firstSN = lastSN + 1");
    return true;
}

/// Inline QoS naming the request a reply answers: the request writer `requester`, SN 1.
InlineQos reply_to(const Guid& requester) {
    InlineQos qos{};
    qos.hasRelatedSampleIdentity = true;
    qos.relatedSampleIdentity.writerGuid = requester;
    qos.relatedSampleIdentity.sequenceNumber = SequenceNumber::from_int64(1);
    return qos;
}

// DDS-RPC 1.0 §7.8.2, §8.4.15.4: a TRANSIENT_LOCAL service reply writer
// piggybacks a HEARTBEAT on every reply, and releases a reply, and its block,
// once the requester's reply reader (the reader sharing the request writer's
// GUID prefix) acknowledges it, without waiting for other clients' readers; a
// change naming no request stays.
static bool tc_swr_8_4_7_007(std::string& err) {
    CaptureTransport tx{};
    const Guid writer = make_guid(0x12u, make_entity(1u, 2u, 3u, 0x02u));
    const Guid a_reader = make_guid(0x23u, make_entity(4u, 5u, 6u, 0x07u));
    const Guid a_request_writer = make_guid(0x23u, make_entity(7u, 8u, 9u, 0x03u));
    const Guid b_reader = make_guid(0x34u, make_entity(4u, 5u, 6u, 0x07u));
    StatefulWriterStorage<8, 2> w(writer, ReliabilityKind::RELIABLE,
                                  DurabilityKind::TRANSIENT_LOCAL, tx, 5u, 1000u);
    w.release_replies_on_delivery(true);
    CHECK(w.matched_reader_add(a_reader, &kLocA, 1u, false), "match A");
    CHECK(w.matched_reader_add(b_reader, &kLocB, 1u, false), "match B");

    CHECK(w.write_async(ChangeKind::ALIVE, kPayload, 4u, reply_to(a_request_writer)), "reply to A");
    CHECK(w.write_async(ChangeKind::ALIVE, kPayload, 4u), "a change naming no request");
    const BufBlock* reply_block =
        w.cache().get_change(SequenceNumber::from_int64(1))->payload.block;

    const Seen sent = inspect(tx);
    CHECK(sent.heartbeats == 4, "every write carries a HEARTBEAT to each reader");
    CHECK(!sent.heartbeat.finalFlag, "asking for the acknowledgement at once");

    deliver(w, b_reader, acknack(b_reader, writer, 3, {}, 1));
    CHECK(holds(w, 1), "another client's acknowledgement does not release A's reply");

    deliver(w, a_reader, acknack(a_reader, writer, 3, {}, 1));
    CHECK(!holds(w, 1), "the requester's acknowledgement releases the reply");
    CHECK(reply_block->refcount.load() == 0, "and returns its block to the pool");
    CHECK(holds(w, 2), "a TRANSIENT_LOCAL change naming no request stays");
    return true;
}

// DDS-RPC 1.0 §7.8.2, §8.7.2.2: a reply whose requester has not matched yet
// stays, reaches the requester's reader when it matches, and is released
// when that reader acknowledges it.
static bool tc_swr_8_4_7_008(std::string& err) {
    CaptureTransport tx{};
    const Guid writer = make_guid(0x45u, make_entity(1u, 2u, 3u, 0x02u));
    const Guid a_reader = make_guid(0x56u, make_entity(4u, 5u, 6u, 0x07u));
    const Guid a_request_writer = make_guid(0x56u, make_entity(7u, 8u, 9u, 0x03u));
    const Guid b_reader = make_guid(0x67u, make_entity(4u, 5u, 6u, 0x07u));
    StatefulWriterStorage<4, 2> w(writer, ReliabilityKind::RELIABLE,
                                  DurabilityKind::TRANSIENT_LOCAL, tx, 5u, 1000u);
    w.release_replies_on_delivery(true);
    CHECK(w.matched_reader_add(b_reader, &kLocB, 1u, false), "match B");
    CHECK(w.write_async(ChangeKind::ALIVE, kPayload, 4u, reply_to(a_request_writer)),
          "reply to A before A's reader matched");
    deliver(w, b_reader, acknack(b_reader, writer, 2, {}, 1));
    CHECK(holds(w, 1), "a reply whose requester has not matched stays");

    tx.unicast_msgs.clear();
    CHECK(w.matched_reader_add(a_reader, &kLocA, 1u, false), "A's reader matches late");
    CHECK(!tx.unicast_msgs.empty(), "the kept reply is sent to it");
    CHECK(holds(w, 1), "and stays until A acknowledges it");

    deliver(w, a_reader, acknack(a_reader, writer, 2, {}, 1));
    CHECK(w.cache().size() == 0u, "A's acknowledgement releases it");
    return true;
}

void append_stateful_writer_tests_8_4_7_release_finished(std::vector<TestCase>& tests) {
    tests.push_back({"TC-SWR-8.4.7-003",
                     "§8.4.7.1/§8.7.2.2.1: a RELIABLE VOLATILE writer releases a change, and its "
                     "block, once every matched reader acknowledges it or unmatches.",
                     tc_swr_8_4_7_003});
    tests.push_back({"TC-SWR-8.4.7-004",
                     "§8.7.2.2: a TRANSIENT_LOCAL writer keeps acknowledged changes and replays "
                     "them to a late reader.",
                     tc_swr_8_4_7_004});
    tests.push_back({"TC-SWR-8.4.7-005",
                     "§8.4.8.1/§8.7.2.2.1: BEST_EFFORT releases after sending; with no reader, "
                     "VOLATILE keeps nothing and a late reader is sent nothing.",
                     tc_swr_8_4_7_005});
    tests.push_back({"TC-SWR-8.4.7-006",
                     "§8.4.9.2.12/§8.4.2.2: a NACK for a released change is answered with GAP; the "
                     "empty history is advertised as lastSN + 1.",
                     tc_swr_8_4_7_006});
    tests.push_back(
        {"TC-SWR-8.4.7-007",
         "DDS-RPC §7.8.2/§8.4.15.4: a service reply writer heartbeats every reply and releases it, "
         "and its block, once the requester's reader acknowledges it, not other clients'.",
         tc_swr_8_4_7_007});
    tests.push_back({"TC-SWR-8.4.7-008",
                     "DDS-RPC §7.8.2/§8.7.2.2: a reply to a requester that has not matched stays, "
                     "reaches its reader on match, and is released on its acknowledgement.",
                     tc_swr_8_4_7_008});
}

}  // namespace spec_test
