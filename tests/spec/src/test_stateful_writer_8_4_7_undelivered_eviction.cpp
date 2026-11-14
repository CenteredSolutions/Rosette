// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file test_stateful_writer_8_4_7_undelivered_eviction.cpp
 * @brief A change KEEP_LAST evicts before its readers have it is lost, and counted.
 *
 * A service reply writer (release_replies_on_delivery()) keeps each reply
 * until its requester acknowledges it, in a KEEP_LAST history. When a reply's
 * datagram is lost, the requester's ACKNACK asks for it again, and the writer
 * can resend it only if no later reply has displaced it: with depth replies
 * in flight to other clients, the next one evicts it (§8.3.3.1) and the
 * writer answers the request with a GAP (§8.4.9.2.12), so the client never
 * receives its reply. That is why rmw_rosette gives every reply writer
 * RMW_ROSETTE_REPLY_DEPTH (4) rather than depth 1, and why that depth bounds
 * the replies a service has in flight to concurrent clients.
 *
 * Every such eviction is counted (StatefulWriter::evicted_unacknowledged()),
 * on any RELIABLE writer, so a board finds out its histories are too shallow
 * from a number rather than from a client that times out.
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

/// Records every unicast datagram with the locator it was sent to.
struct CaptureTransport : ITransport {
    struct Sent {
        std::vector<Octet> bytes{};
        Locator            dest = LOCATOR_INVALID;
    };

    std::vector<Sent> unicast{};

    bool send_unicast(const BufRef& buf, const Locator& dest) noexcept override {
        unicast.push_back(Sent{std::vector<Octet>(buf.data, buf.data + buf.size), dest});
        return true;
    }

    bool send_multicast(const BufRef&, const Locator&) noexcept override { return true; }
};

/// What one reader was sent: whether it got DATA for SN 1, and whether a GAP
/// named SN 1; and the writer's count of evicted unacknowledged changes.
struct SentToReader {
    bool     data_sn1 = false;
    bool     gap_sn1 = false;
    uint32_t evicted = 0u;
};

SentToReader sent_to(const CaptureTransport& tx, const Locator& dest) {
    SentToReader seen{};
    ParseOps     ops{};
    ops.on_data = [](const ParseContext& c, const ParsedData& d) noexcept {
        auto* out = static_cast<SentToReader*>(c.user_data);
        out->data_sn1 = out->data_sn1 || d.writerSN == SequenceNumber::from_int64(1);
    };
    ops.on_gap = [](const ParseContext& c, const ParsedGap& g) noexcept {
        auto*                out = static_cast<SentToReader*>(c.user_data);
        const SequenceNumber one = SequenceNumber::from_int64(1);
        out->gap_sn1 =
            out->gap_sn1 || (!(one < g.gapStart) && one < gap_run_end(g.gapStart, g.gapList));
    };
    for (const auto& msg : tx.unicast) {
        if (msg.dest == dest) {
            (void)parse_message(msg.bytes.data(), static_cast<uint32_t>(msg.bytes.size()),
                                GuidPrefix{}, ops, &seen);
        }
    }

    return seen;
}

Guid make_guid(Octet p0, const EntityId& eid) {
    Guid g{};
    g.prefix.value[0] = p0;
    g.prefix.value[1] = 0x7Du;
    g.entityId = eid;
    return g;
}

/// Inline QoS naming the request a reply answers: SN 1 of the request writer `requester`.
InlineQos reply_to(const Guid& requester) {
    InlineQos qos{};
    qos.hasRelatedSampleIdentity = true;
    qos.relatedSampleIdentity.writerGuid = requester;
    qos.relatedSampleIdentity.sequenceNumber = SequenceNumber::from_int64(1);
    return qos;
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

const Locator kLocA = make_udpv4_locator(0xC0A80191u, 7411u);
const Locator kLocB = make_udpv4_locator(0xC0A80192u, 7411u);
const Octet   kPayload[4] = {1u, 2u, 3u, 4u};

/**
 * Client A's reply (SN 1) is lost, the service then answers client B
 * `b_replies` times before A's ACKNACK asking for SN 1 arrives, and the
 * writer's answer to A is returned.
 */
template <uint8_t Depth>
SentToReader repair_after(int b_replies, Octet prefix, std::string& err) {
    CaptureTransport tx{};
    const Guid       writer = make_guid(prefix, make_entity(1u, 2u, 3u, 0x02u));
    const Guid       a_reader = make_guid(prefix + 1u, make_entity(4u, 5u, 6u, 0x07u));
    const Guid       a_requester = make_guid(prefix + 1u, make_entity(7u, 8u, 9u, 0x03u));
    const Guid       b_reader = make_guid(prefix + 2u, make_entity(4u, 5u, 6u, 0x07u));
    const Guid       b_requester = make_guid(prefix + 2u, make_entity(7u, 8u, 9u, 0x03u));
    StatefulWriterStorage<Depth, 2> w(writer, ReliabilityKind::RELIABLE,
                                      DurabilityKind::TRANSIENT_LOCAL, tx, 5u, 1000u);
    w.release_replies_on_delivery(true);
    if (!w.matched_reader_add(a_reader, &kLocA, 1u, false)
        || !w.matched_reader_add(b_reader, &kLocB, 1u, false)
        || !w.write_async(ChangeKind::ALIVE, kPayload, 4u, reply_to(a_requester))) {
        err = "setup: match A and B and write A's reply";
        return SentToReader{};
    }

    for (int i = 0; i < b_replies; ++i) {
        if (!w.write_async(ChangeKind::ALIVE, kPayload, 4u, reply_to(b_requester))) {
            err = "setup: write B's replies";
            return SentToReader{};
        }
    }

    // The datagrams sent so far are A's lost reply and B's replies in flight.
    tx.unicast.clear();
    g_stateful_writer_test_now_ms = 100u;
    deliver(w, a_reader, acknack(a_reader, writer, 1, {1}, 2));
    w.tick(200u);
    SentToReader seen = sent_to(tx, kLocA);
    seen.evicted = w.evicted_unacknowledged();
    return seen;
}

}  // namespace

// §8.3.3.1, §8.4.7.1, §8.4.9.2.12; DDS-RPC 1.0 §7.8.2: a lost reply is
// resent on its requester's NACK while no more than depth - 1 later replies
// are in flight; one more reply displaces it, and the NACK is answered with
// a GAP, so the client never receives its reply. Depth 1 loses a reply to
// any concurrent client; depth 4 (RMW_ROSETTE_REPLY_DEPTH) repairs it with
// three other clients' replies in flight.
static bool tc_swr_8_4_7_013(std::string& err) {
    SentToReader seen = repair_after<4>(3, 0x10u, err);
    CHECK(err.empty() && seen.data_sn1 && !seen.gap_sn1 && seen.evicted == 0u,
          "depth 4: A's lost reply is resent with three other replies in flight");

    seen = repair_after<4>(4, 0x20u, err);
    CHECK(err.empty() && !seen.data_sn1 && seen.gap_sn1 && seen.evicted == 1u,
          "depth 4: a fourth reply in flight displaces A's, whose NACK gets a GAP, and the "
          "eviction is counted");

    seen = repair_after<1>(0, 0x30u, err);
    CHECK(err.empty() && seen.data_sn1 && !seen.gap_sn1 && seen.evicted == 0u,
          "depth 1: A's lost reply is resent while it is the only reply in flight");

    seen = repair_after<1>(1, 0x40u, err);
    CHECK(err.empty() && !seen.data_sn1 && seen.gap_sn1 && seen.evicted == 1u,
          "depth 1: one reply to another client displaces A's, A's reply is lost, and the "
          "eviction is counted");
    return true;
}

// §8.3.3.1: on a RELIABLE writer that is not a reply writer, evicting a
// change a matched reader has not acknowledged is counted, and evicting one
// every reader acknowledged, or one no reader was matched for, is not; a
// BEST_EFFORT writer counts nothing.
static bool tc_swr_8_4_7_014(std::string& err) {
    CaptureTransport            tx{};
    const Guid                  writer = make_guid(0x50u, make_entity(1u, 2u, 3u, 0x02u));
    const Guid                  a = make_guid(0x51u, make_entity(4u, 5u, 6u, 0x07u));
    StatefulWriterStorage<2, 1> kept(writer, ReliabilityKind::RELIABLE,
                                     DurabilityKind::TRANSIENT_LOCAL, tx, 5u, 1000u);
    for (int i = 0; i < 3; ++i) {
        CHECK(kept.write_async(ChangeKind::ALIVE, kPayload, 4u), "write with no reader matched");
    }

    CHECK(kept.evicted_unacknowledged() == 0u,
          "evicting a change no reader was matched for counts nothing");

    CHECK(kept.matched_reader_add(a, &kLocA, 1u, false), "match A");
    deliver(kept, a, acknack(a, writer, 4, {}, 1));
    CHECK(kept.write_async(ChangeKind::ALIVE, kPayload, 4u),
          "write SN 4 over the acknowledged SN 2");
    CHECK(kept.evicted_unacknowledged() == 0u,
          "evicting a change every reader acknowledged counts nothing");
    CHECK(kept.write_async(ChangeKind::ALIVE, kPayload, 4u), "write SN 5 over SN 3");
    CHECK(kept.write_async(ChangeKind::ALIVE, kPayload, 4u), "write SN 6 over unacknowledged SN 4");
    CHECK(kept.evicted_unacknowledged() == 1u,
          "evicting a change the matched reader has not acknowledged counts once");

    StatefulWriterStorage<1, 1> volatile_writer(writer, ReliabilityKind::RELIABLE,
                                                DurabilityKind::VOLATILE, tx, 5u, 1000u);
    CHECK(volatile_writer.matched_reader_add(a, &kLocA, 1u, false), "match A");
    for (int i = 0; i < 3; ++i) {
        CHECK(volatile_writer.write_async(ChangeKind::ALIVE, kPayload, 4u), "write");
    }

    CHECK(volatile_writer.evicted_unacknowledged() == 2u,
          "a VOLATILE depth-1 writer outpacing its reader counts each sample the reader missed");

    StatefulWriterStorage<1, 1> best_effort(writer, ReliabilityKind::BEST_EFFORT,
                                            DurabilityKind::TRANSIENT_LOCAL, tx, 5u, 1000u);
    CHECK(best_effort.matched_reader_add(a, &kLocA, 1u, false), "match A");
    for (int i = 0; i < 3; ++i) {
        CHECK(best_effort.write_async(ChangeKind::ALIVE, kPayload, 4u), "write");
    }

    CHECK(best_effort.evicted_unacknowledged() == 0u,
          "a BEST_EFFORT writer repairs nothing and counts nothing");
    return true;
}

/// Adds one change to @p w's history without a send pass.
static bool add_unsent(StatefulWriter& w) {
    BufRef       payload{};
    HistoryChange* change = w.new_change(ChangeKind::ALIVE, payload, 4u);
    if (change == nullptr) {
        return false;
    }

    memcpy(payload.data, kPayload, 4u);
    payload.size = 4u;
    change->payload.size = 4u;
    return w.add_change(static_cast<HistoryChange&&>(*change));
}

// §8.4.7.1, §8.7.2.2.1: a VOLATILE reader matched after SN 1 was written
// starts at the next sequence number, so SN 1 was never relevant to it and
// evicting SN 1 loses nothing it needed.
static bool tc_swr_8_4_7_015(std::string& err) {
    CaptureTransport            tx{};
    const Guid                  writer = make_guid(0x50u, make_entity(1u, 2u, 3u, 0x02u));
    const Guid                  a = make_guid(0x51u, make_entity(4u, 5u, 6u, 0x07u));
    StatefulWriterStorage<1, 1> w(writer, ReliabilityKind::RELIABLE, DurabilityKind::VOLATILE, tx,
                                  5u, 1000u);
    // add_change() rather than write_async(): a send pass would release SN 1
    // at once, with no reader matched, and leave nothing to evict.
    CHECK(add_unsent(w), "add SN 1 with no reader matched");
    CHECK(w.matched_reader_add(a, &kLocA, 1u, false), "match A after SN 1");
    CHECK(add_unsent(w), "add SN 2 over SN 1");
    CHECK(w.evicted_unacknowledged() == 0u,
          "evicting a change that was never relevant to the matched reader counts nothing");

    CHECK(add_unsent(w), "add SN 3 over unacknowledged SN 2");
    CHECK(w.evicted_unacknowledged() == 1u,
          "evicting a change that was relevant to the reader and unacknowledged counts once");
    return true;
}

void append_stateful_writer_tests_8_4_7_undelivered_eviction(std::vector<TestCase>& tests) {
    tests.push_back({"TC-SWR-8.4.7-013",
                     "§8.3.3.1/§8.4.9.2.12, DDS-RPC §7.8.2: a service reply lost on the wire is "
                     "resent on its requester's NACK while fewer than depth later replies are in "
                     "flight, and answered with a GAP, the reply lost, once one more displaces "
                     "it.",
                     tc_swr_8_4_7_013});
    tests.push_back({"TC-SWR-8.4.7-014",
                     "§8.3.3.1: a RELIABLE writer counts each change KEEP_LAST evicts before a "
                     "matched reader acknowledged it, and nothing for changes every reader has or "
                     "no reader was matched for; a BEST_EFFORT writer counts nothing.",
                     tc_swr_8_4_7_014});
    tests.push_back({"TC-SWR-8.4.7-015",
                     "§8.4.7.1, §8.7.2.2.1: evicting a change that was never relevant to a "
                     "reader matched after it was written is not counted; evicting one that was "
                     "relevant and unacknowledged is.",
                     tc_swr_8_4_7_015});
}

}  // namespace spec_test
