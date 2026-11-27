// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file test_stateful_writer_8_4_15_fast_heartbeat.cpp
 * @brief Reliable StatefulWriter: the HEARTBEAT schedule each write starts.
 *
 * A reader answers only a HEARTBEAT, so a lost last change of a burst is
 * repaired only when the writer next asks (§8.4.15.4). After each write the
 * writer asks every reader behind the newest change at 50, 150, 350 and
 * 750 ms, until each has acknowledged it; a slow topic's writes carry the
 * HEARTBEAT in their own DATA; a NACK_FRAG moves the next round one wait
 * later. See "Heartbeats after a write" in stateful_writer.hpp.
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

/// One HEARTBEAT the writer sent, with where it went.
struct SentHeartbeat {
    Locator dest{};
    bool with_data = false;
    ParsedHeartbeat hb{};
};

/// Records every HEARTBEAT in the datagrams the writer sends.
struct HeartbeatTransport : ITransport {
    std::vector<SentHeartbeat> sent{};

    bool send_unicast(const BufRef& buf, const Locator& dest) noexcept override {
        struct Scan {
            HeartbeatTransport* self;
            Locator dest;
            bool data;
        } scan{this, dest, false};

        ParseOps ops{};
        ops.on_data = [](const ParseContext& c, const ParsedData&) noexcept {
            static_cast<Scan*>(c.user_data)->data = true;
        };
        ops.on_heartbeat = [](const ParseContext& c, const ParsedHeartbeat& hb) noexcept {
            auto* s = static_cast<Scan*>(c.user_data);
            s->self->sent.push_back({s->dest, s->data, hb});
        };
        (void)parse_message(buf.data, buf.size, GuidPrefix{}, ops, &scan);
        return true;
    }

    bool send_multicast(const BufRef&, const Locator&) noexcept override { return true; }

    uint32_t max_datagram_size() const noexcept override { return 65507u; }

    /// HEARTBEATs sent on their own (not with a DATA) to `dest`.
    int probes_to(const Locator& dest) const {
        int n = 0;
        for (const SentHeartbeat& s : sent) {
            if (!s.with_data && !s.hb.finalFlag && s.dest == dest) {
                ++n;
            }
        }

        return n;
    }

    int probes() const {
        int n = 0;
        for (const SentHeartbeat& s : sent) {
            if (!s.with_data && !s.hb.finalFlag) {
                ++n;
            }
        }

        return n;
    }
};

Guid make_guid(Octet p0, const EntityId& eid) {
    Guid g{};
    g.prefix.value[0] = p0;
    g.prefix.value[1] = 0x5Cu;
    g.entityId = eid;
    return g;
}

const Guid kWriter = make_guid(0x31u, make_entity(1u, 2u, 3u, 0x02u));
const Guid kReaderA = make_guid(0x41u, make_entity(4u, 5u, 6u, 0x07u));
const Guid kReaderB = make_guid(0x42u, make_entity(4u, 5u, 6u, 0x07u));
const Locator kLocA = make_udpv4_locator(0xC0A801A1u, 7411u);
const Locator kLocB = make_udpv4_locator(0xC0A801A2u, 7411u);
const Octet kPayload[2] = {0x12u, 0x34u};

/// Long enough that no periodic HEARTBEAT falls inside a test.
constexpr uint32_t kPeriod = 60000u;

using Writer = StatefulWriterStorage<8, 2>;

void acknowledge(StatefulWriter& w, const Guid& reader, int64_t next_sn, int32_t count,
                 uint32_t now_ms) {
    ParsedAckNack ack{};
    ack.readerId = reader.entityId;
    ack.writerId = kWriter.entityId;
    ack.readerSNState.clear();
    ack.readerSNState.bitmapBase = SequenceNumber::from_int64(next_sn);
    ack.count = count;
    ParseContext ctx{};
    ctx.senderGuidPrefix = reader.prefix;
    g_stateful_writer_test_now_ms = now_ms;
    w.on_acknack(ctx, ack);
}

bool write_at(StatefulWriter& w, uint32_t now_ms) {
    g_stateful_writer_test_now_ms = now_ms;
    return w.write_async(ChangeKind::ALIVE, kPayload, 2u);
}

}  // namespace

// §8.4.15.4: an unacknowledged write is probed with a non-final HEARTBEAT
// directed to the reader at 50, 150, 350 and 750 ms, and then not again
// before the periodic HEARTBEAT.
static bool tc_swr_8_4_15_006(std::string& err) {
    HeartbeatTransport tx{};
    Writer w(kWriter, ReliabilityKind::RELIABLE, DurabilityKind::VOLATILE, tx, 5u, kPeriod);
    CHECK(w.matched_reader_add(kReaderA, &kLocA, 1u, false), "match");
    CHECK(write_at(w, 1000u), "write");
    CHECK(w.tick(1000u) == 50u, "the first round is due 50 ms after the write");
    CHECK(w.ms_until_next_timer(1000u) == 50u, "ms_until_next_timer agrees with tick");

    const uint32_t rounds[] = {1050u, 1150u, 1350u, 1750u};
    int expected = 0;
    for (const uint32_t at : rounds) {
        (void)w.tick(at - 1u);
        CHECK(tx.probes() == expected, "no round before it is due");
        (void)w.tick(at);
        ++expected;
        CHECK(tx.probes() == expected, "one probe per round");
    }

    const SentHeartbeat& last = tx.sent.back();
    CHECK(last.dest == kLocA && last.hb.readerId == kReaderA.entityId,
          "the probe is directed to the reader");
    CHECK(last.hb.lastSN == SequenceNumber::from_int64(1), "the probe announces the write");
    CHECK(w.ms_until_next_timer(1750u) == kPeriod - 1750u,
          "after the last round only the periodic HEARTBEAT is pending");
    (void)w.tick(5000u);
    (void)w.tick(30000u);
    CHECK(tx.probes() == expected, "no probe after the last round");
    return true;
}

// §8.4.15.4: once the reader acknowledges the newest change the schedule
// ends, and a new write starts it again at 50 ms.
static bool tc_swr_8_4_15_007(std::string& err) {
    HeartbeatTransport tx{};
    Writer w(kWriter, ReliabilityKind::RELIABLE, DurabilityKind::VOLATILE, tx, 5u, kPeriod);
    CHECK(w.matched_reader_add(kReaderA, &kLocA, 1u, false), "match");
    CHECK(write_at(w, 1000u), "write SN 1");
    acknowledge(w, kReaderA, 2, 1, 1020u);
    CHECK(w.tick(1050u) == UINT32_MAX, "an acknowledged writer has no timer");
    CHECK(tx.probes() == 0, "an acknowledged write is not probed");

    CHECK(write_at(w, 2000u), "write SN 2");
    (void)w.tick(2050u);
    CHECK(tx.probes() == 1, "a new write is probed at 50 ms");
    CHECK(write_at(w, 2100u), "write SN 3");
    (void)w.tick(2149u);
    CHECK(tx.probes() == 1, "a write restarts the schedule");
    (void)w.tick(2150u);
    CHECK(tx.probes() == 2 && tx.sent.back().hb.lastSN == SequenceNumber::from_int64(3),
          "the restarted schedule probes 50 ms after the newest write");
    return true;
}

// §8.4.9.1, §8.4.15.4: only readers behind the newest change are probed; a
// reader that acknowledged it and a reader served best-effort get nothing.
static bool tc_swr_8_4_15_008(std::string& err) {
    HeartbeatTransport tx{};
    Writer w(kWriter, ReliabilityKind::RELIABLE, DurabilityKind::VOLATILE, tx, 5u, kPeriod);
    CHECK(w.matched_reader_add(kReaderA, &kLocA, 1u, false), "match A");
    CHECK(w.matched_reader_add(kReaderB, &kLocB, 1u, false), "match B");
    CHECK(write_at(w, 1000u), "write");
    acknowledge(w, kReaderB, 2, 1, 1010u);
    (void)w.tick(1050u);
    CHECK(tx.probes_to(kLocA) == 1, "the reader behind is probed");
    CHECK(tx.probes_to(kLocB) == 0, "the reader that acknowledged is not");

    HeartbeatTransport tx2{};
    Writer w2(kWriter, ReliabilityKind::RELIABLE, DurabilityKind::VOLATILE, tx2, 5u, kPeriod);
    CHECK(w2.matched_reader_add(kReaderA, &kLocA, 1u, false, ReliabilityKind::BEST_EFFORT),
          "match best-effort");
    CHECK(write_at(w2, 1000u), "write to best-effort");
    CHECK(w2.tick(1000u) == UINT32_MAX, "a best-effort reader leaves no timer");
    (void)w2.tick(1050u);
    CHECK(tx2.sent.empty(), "a best-effort reader receives no HEARTBEAT");
    return true;
}

// §8.4.15.4: a write at least 50 ms after the writer last asked carries the
// HEARTBEAT in its own datagram; a sooner one waits for depth/2 writes.
static bool tc_swr_8_4_15_009(std::string& err) {
    HeartbeatTransport tx{};
    Writer w(kWriter, ReliabilityKind::RELIABLE, DurabilityKind::VOLATILE, tx, 5u, kPeriod);
    CHECK(w.matched_reader_add(kReaderA, &kLocA, 1u, false), "match");
    CHECK(write_at(w, 1000u), "write 1");
    CHECK(tx.sent.size() == 1u && tx.sent[0].with_data, "the first write carries a HEARTBEAT");
    CHECK(write_at(w, 1020u), "write 2");
    CHECK(tx.sent.size() == 1u, "a write 20 ms later does not");
    CHECK(write_at(w, 1050u), "write 3");
    CHECK(tx.sent.size() == 2u && tx.sent[1].with_data && !tx.sent[1].hb.finalFlag,
          "a write 50 ms after the last HEARTBEAT carries one");
    (void)w.tick(1099u);
    CHECK(tx.probes() == 0, "a write's own HEARTBEAT and the schedule do not overlap");
    return true;
}

// §8.4.4, §8.3.7.7: a NACK_FRAG moves the next round one wait later, for
// every reader of the writer.
static bool tc_swr_8_4_15_010(std::string& err) {
    HeartbeatTransport tx{};
    Writer w(kWriter, ReliabilityKind::RELIABLE, DurabilityKind::VOLATILE, tx, 5u, kPeriod);
    CHECK(w.matched_reader_add(kReaderA, &kLocA, 1u, false), "match A");
    CHECK(w.matched_reader_add(kReaderB, &kLocB, 1u, false), "match B");
    CHECK(write_at(w, 1000u), "write");

    ParsedNackFrag nf{};
    nf.readerId = kReaderA.entityId;
    nf.writerId = kWriter.entityId;
    nf.writerSN = SequenceNumber::from_int64(1);
    nf.fragmentNumberState.clear();
    (void)nf.fragmentNumberState.add(1u);
    nf.count = 1;
    ParseContext ctx{};
    ctx.senderGuidPrefix = kReaderA.prefix;
    g_stateful_writer_test_now_ms = 1040u;
    w.on_nack_frag(ctx, nf);

    (void)w.tick(1050u);
    CHECK(tx.probes() == 0, "no round falls due inside one wait of the NACK_FRAG");
    CHECK(w.ms_until_next_timer(1050u) == 40u, "the round moves to 50 ms after the NACK_FRAG");
    (void)w.tick(1090u);
    CHECK(tx.probes_to(kLocA) == 1 && tx.probes_to(kLocB) == 1,
          "the deferred round probes every reader still behind");
    return true;
}

void append_stateful_writer_tests_8_4_15_fast_heartbeat(std::vector<TestCase>& tests) {
    tests.push_back({"TC-SWR-8.4.15-006",
                     "§8.4.15.4: an unacknowledged write is probed at 50, 150, 350 and 750 ms, "
                     "then only by the periodic HEARTBEAT.",
                     tc_swr_8_4_15_006});
    tests.push_back({"TC-SWR-8.4.15-007",
                     "§8.4.15.4: an ACKNACK of the newest change ends the schedule; a write "
                     "restarts it.",
                     tc_swr_8_4_15_007});
    tests.push_back({"TC-SWR-8.4.15-008",
                     "§8.4.9.1: only readers behind the newest change are probed; best-effort "
                     "readers never are.",
                     tc_swr_8_4_15_008});
    tests.push_back({"TC-SWR-8.4.15-009",
                     "§8.4.15.4: a write 50 ms after the last HEARTBEAT carries one in its "
                     "datagram.",
                     tc_swr_8_4_15_009});
    tests.push_back({"TC-SWR-8.4.15-010",
                     "§8.4.4: a NACK_FRAG moves the next round one wait later.",
                     tc_swr_8_4_15_010});
}

}  // namespace spec_test
