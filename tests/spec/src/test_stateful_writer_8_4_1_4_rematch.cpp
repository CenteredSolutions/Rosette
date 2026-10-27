/**
 * @file test_stateful_writer_8_4_1_4_rematch.cpp
 * @brief Reliable StatefulWriter: a reader whose acknowledgement goes backwards is matched afresh.
 *
 * A remote participant that loses this participant's SPDP lease (§8.5.3.2)
 * forgets every endpoint of it and matches them again when the next SPDP
 * announcement arrives. Its readers then start over, while this side, which
 * never lost the remote lease, still holds their old ReaderProxies. The
 * re-matched reader shows it by acknowledging less than before. The writer
 * must treat that reader as newly matched (§8.4.1.4 matched_reader_add):
 * a TRANSIENT_LOCAL writer replays its history (§8.7.2.2) and a VOLATILE
 * writer declares what was written earlier irrelevant (§8.7.2.2.1). A
 * writer that keeps the old proxy sends the reader nothing, ever: a stock
 * `ros2 node list` then never shows this participant's nodes again, because
 * `ros_discovery_info` is never redelivered.
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

/// Counts the DATA, HEARTBEAT and GAP submessages the writer sends.
struct SubmessageCounter : ITransport {
    int data = 0;
    int heartbeats = 0;
    int gaps = 0;
    int64_t last_data_sn = 0;
    int64_t last_gap_start = 0;

    bool send_unicast(const BufRef& buf, const Locator&) noexcept override {
        ParseOps ops{};
        ops.on_data = [](const ParseContext& c, const ParsedData& d) noexcept {
            auto* self = static_cast<SubmessageCounter*>(c.user_data);
            ++self->data;
            self->last_data_sn = d.writerSN.to_int64();
        };
        ops.on_heartbeat = [](const ParseContext& c, const ParsedHeartbeat&) noexcept {
            ++static_cast<SubmessageCounter*>(c.user_data)->heartbeats;
        };
        ops.on_gap = [](const ParseContext& c, const ParsedGap& g) noexcept {
            auto* self = static_cast<SubmessageCounter*>(c.user_data);
            ++self->gaps;
            self->last_gap_start = g.gapStart.to_int64();
        };
        (void)parse_message(buf.data, buf.size, GuidPrefix{}, ops, this);
        return true;
    }

    bool send_multicast(const BufRef&, const Locator&) noexcept override { return true; }

    void reset() noexcept { *this = SubmessageCounter{}; }
};

Guid make_guid(Octet p0, const EntityId& eid) {
    Guid g{};
    g.prefix.value[0] = p0;
    g.entityId = eid;
    return g;
}

/// Delivers an ACKNACK from `reader` acknowledging everything below `base`
/// and NACKing the `nacks` sequence numbers that follow it.
void acknack(StatefulWriter& w, const Guid& reader, const Guid& writer, int64_t base, int32_t count,
             std::initializer_list<int64_t> nacks = {}) {
    ParsedAckNack ack{};
    ack.readerId = reader.entityId;
    ack.writerId = writer.entityId;
    ack.readerSNState.clear();
    ack.readerSNState.bitmapBase = SequenceNumber::from_int64(base);
    for (const int64_t sn : nacks) {
        (void)ack.readerSNState.add(SequenceNumber::from_int64(sn));
    }

    ack.count = count;
    ParseContext ctx{};
    ctx.senderGuidPrefix = reader.prefix;
    w.on_acknack(ctx, ack);
}

/// Rematch listener that counts its calls and, as rmw_rosette's
/// `ros_discovery_info` writer does, writes a new change from the callback.
struct RepublishingListener {
    StatefulWriter* writer = nullptr;
    int calls = 0;

    static void on_rematch(void* self) noexcept {
        auto* l = static_cast<RepublishingListener*>(self);
        ++l->calls;
        const Octet payload[1] = {0x43u};
        (void)l->writer->write_async(ChangeKind::ALIVE, payload, 1u);
    }
};

}  // namespace

// §8.4.1.4, §8.7.2.2: a reader that acknowledged SN 3 and then announces
// itself again (ACKNACK base 1, higher count, as Fast DDS sends after
// re-matching) is replayed the TRANSIENT_LOCAL history and HEARTBEATed until
// it acknowledges again.
static bool tc_swr_8_4_1_4_004(std::string& err) {
    SubmessageCounter tx{};
    const Guid writer = make_guid(0x11u, make_entity(1u, 2u, 3u, 0x02u));
    const Guid reader = make_guid(0x22u, make_entity(4u, 5u, 6u, 0x07u));
    const Locator loc = make_udpv4_locator(0xC0A80179u, 7411u);
    StatefulWriterStorage<1, 1> w(writer, ReliabilityKind::RELIABLE,
                                  DurabilityKind::TRANSIENT_LOCAL, tx, 5u, 1000u);
    CHECK(w.matched_reader_add(reader, &loc, 1u, false), "match");
    RepublishingListener listener{&w};
    w.set_rematch_listener(&RepublishingListener::on_rematch, &listener);

    const Octet payload[1] = {0x42u};
    for (int i = 0; i < 3; ++i) {
        CHECK(w.write_async(ChangeKind::ALIVE, payload, 1u), "write");
    }

    g_stateful_writer_test_now_ms = 100u;
    acknack(w, reader, writer, 4, 1);
    tx.reset();
    w.tick(1000u);
    CHECK(tx.data == 0 && tx.heartbeats == 0, "a fully acknowledged reader is idle");
    CHECK(listener.calls == 0, "acknowledging more is not starting over");

    acknack(w, reader, writer, 1, 2);
    CHECK(listener.calls == 1, "the rematch listener runs once for the reader that started over");
    CHECK(tx.data == 0, "the replay waits for tick(), and so does the change written meanwhile");
    finish_replay(w, 1000u);
    CHECK(tx.data == 1 && tx.last_data_sn == 4,
          "the replay sends the history as it stands: the listener's SN 4 evicted SN 3 from "
          "this KEEP_LAST(1) history, so the reader gets SN 4");
    CHECK(tx.heartbeats >= 1, "and told the range it can request");
    const ReaderProxy* proxy = w.matched_reader_lookup(reader);
    CHECK(proxy != nullptr && proxy->highestAckedSN == SEQUENCENUMBER_ZERO,
          "nothing counts as acknowledged by the re-matched reader");
    CHECK(proxy->is_announcing(w.cache()), "it stays announcing until it acknowledges again");

    acknack(w, reader, writer, 5, 3);
    tx.reset();
    w.tick(2000u);
    CHECK(tx.heartbeats == 0, "once it acknowledges the replay it is idle again");
    return true;
}

// §8.4.1.4: a reader that restarts its ACKNACK count on re-matching is
// still recognised when it acknowledges nothing (base 1), while a delayed
// ACKNACK that acknowledges less but carries an old count is a duplicate
// and changes nothing.
static bool tc_swr_8_4_1_4_005(std::string& err) {
    SubmessageCounter tx{};
    const Guid writer = make_guid(0x33u, make_entity(1u, 2u, 3u, 0x02u));
    const Guid reader = make_guid(0x44u, make_entity(4u, 5u, 6u, 0x07u));
    const Locator loc = make_udpv4_locator(0xC0A8017Au, 7411u);
    StatefulWriterStorage<4, 1> w(writer, ReliabilityKind::RELIABLE,
                                  DurabilityKind::TRANSIENT_LOCAL, tx, 5u, 1000u);
    CHECK(w.matched_reader_add(reader, &loc, 1u, false), "match");

    const Octet payload[1] = {0x42u};
    for (int i = 0; i < 3; ++i) {
        CHECK(w.write_async(ChangeKind::ALIVE, payload, 1u), "write");
    }

    g_stateful_writer_test_now_ms = 100u;
    acknack(w, reader, writer, 2, 5);
    acknack(w, reader, writer, 4, 6);
    tx.reset();

    acknack(w, reader, writer, 3, 5);
    const ReaderProxy* proxy = w.matched_reader_lookup(reader);
    CHECK(proxy != nullptr && proxy->highestAckedSN == SequenceNumber::from_int64(3),
          "a reordered ACKNACK with an old count does not lower the acknowledgement");
    CHECK(tx.data == 0 && tx.heartbeats == 0, "and sends nothing");

    acknack(w, reader, writer, 1, 1);
    CHECK(proxy->highestAckedSN == SEQUENCENUMBER_ZERO,
          "an ACKNACK acknowledging nothing re-matches the reader whatever its count");
    finish_replay(w, 0u);
    CHECK(tx.data == 3, "and replays every cached change");
    return true;
}

// §8.4.1.4, §8.7.2.2.1: a re-matched reader of a VOLATILE writer is not
// sent what was written before it matched again; its NACKs for those
// changes, which an old proxy would silently ignore, are answered with GAP.
static bool tc_swr_8_4_1_4_006(std::string& err) {
    SubmessageCounter tx{};
    const Guid writer = make_guid(0x55u, make_entity(1u, 2u, 3u, 0x02u));
    const Guid rematched = make_guid(0x66u, make_entity(4u, 5u, 6u, 0x07u));
    const Guid lagging = make_guid(0x77u, make_entity(4u, 5u, 6u, 0x07u));
    const Locator loc = make_udpv4_locator(0xC0A8017Bu, 7411u);
    StatefulWriterStorage<4, 2> w(writer, ReliabilityKind::RELIABLE, DurabilityKind::VOLATILE, tx,
                                  5u, 1000u);
    CHECK(w.matched_reader_add(rematched, &loc, 1u, false), "match the reader that re-matches");
    CHECK(w.matched_reader_add(lagging, &loc, 1u, false), "match a reader that keeps SN 1-2 held");

    const Octet payload[1] = {0x42u};
    CHECK(w.write_async(ChangeKind::ALIVE, payload, 1u), "write SN 1");
    CHECK(w.write_async(ChangeKind::ALIVE, payload, 1u), "write SN 2");

    g_stateful_writer_test_now_ms = 100u;
    acknack(w, rematched, writer, 3, 1);
    tx.reset();

    acknack(w, rematched, writer, 1, 2, {1, 2});
    CHECK(tx.gaps == 1 && tx.last_gap_start == 1,
          "the changes written before the re-match are declared irrelevant");
    w.tick(1000u);
    CHECK(tx.data == 0, "and are not replayed to a VOLATILE reader");
    return true;
}

// §8.4.1.4: a newly matched reader's first ACKNACK acknowledges nothing,
// often with bitmapBase 0 (Fast DDS), and is not a restart; the same
// ACKNACK from a reader that has acknowledged changes is one.
static bool tc_swr_8_4_1_4_007(std::string& err) {
    SubmessageCounter tx{};
    const Guid writer = make_guid(0x88u, make_entity(1u, 2u, 3u, 0x02u));
    const Guid reader = make_guid(0x99u, make_entity(4u, 5u, 6u, 0x07u));
    const Locator loc = make_udpv4_locator(0xC0A8017Cu, 7411u);
    StatefulWriterStorage<4, 1> w(writer, ReliabilityKind::RELIABLE,
                                  DurabilityKind::TRANSIENT_LOCAL, tx, 5u, 1000u);
    RepublishingListener listener{&w};
    w.set_rematch_listener(&RepublishingListener::on_rematch, &listener);

    const Octet payload[1] = {0x42u};
    for (int i = 0; i < 3; ++i) {
        CHECK(w.write_async(ChangeKind::ALIVE, payload, 1u), "write");
    }

    CHECK(w.matched_reader_add(reader, &loc, 1u, false), "match after the writes");
    tx.reset();
    g_stateful_writer_test_now_ms = 100u;
    acknack(w, reader, writer, 0, 1);
    CHECK(listener.calls == 0, "a new reader's base-0 ACKNACK is not a restart");
    CHECK(tx.data == 0, "and draws no second replay");

    acknack(w, reader, writer, 4, 2);
    acknack(w, reader, writer, 0, 3);
    CHECK(listener.calls == 1, "the same ACKNACK after acknowledging SN 3 is a restart");
    return true;
}

void append_stateful_writer_tests_8_4_1_4_rematch(std::vector<TestCase>& tests) {
    tests.push_back({"TC-SWR-8.4.1.4-004",
                     "§8.4.1.4/§8.7.2.2: a reader whose cumulative ACK goes backwards is matched "
                     "afresh and replayed the TRANSIENT_LOCAL history.",
                     tc_swr_8_4_1_4_004});
    tests.push_back({"TC-SWR-8.4.1.4-005",
                     "§8.4.1.4: an ACKNACK acknowledging nothing re-matches its reader whatever "
                     "its count; a reordered old ACKNACK changes nothing.",
                     tc_swr_8_4_1_4_005});
    tests.push_back({"TC-SWR-8.4.1.4-006",
                     "§8.4.1.4/§8.7.2.2.1: a re-matched reader of a VOLATILE writer is sent GAP, "
                     "not replay, for changes written before it re-matched.",
                     tc_swr_8_4_1_4_006});
    tests.push_back({"TC-SWR-8.4.1.4-007",
                     "§8.4.1.4: a new reader's first ACKNACK (base 0 or 1) is not a restart and "
                     "does not call the rematch listener; after acknowledging, it is one.",
                     tc_swr_8_4_1_4_007});
}

}  // namespace spec_test
