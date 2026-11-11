// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file test_stateful_reader_8_4_12_gap_nack_on_quiet.cpp
 * @brief A reliable reader requests a missing change without waiting for a HEARTBEAT.
 *
 * A writer resends a lost change only when a reader names it. The reader
 * sees the loss as soon as a later change arrives, and asks for it
 * RTPS_GAP_REPAIR_DELAY_MS later (WriterProxy::schedule_gap_repair())
 * instead of at the writer's next HEARTBEAT, then backs off by doubling
 * while the writer does not answer. These tests drive the reader with an
 * explicit clock (test_stateful_reader_clock_stub.cpp) and check the
 * ACKNACKs its timer pass sends.
 */

#include <vector>

#include "spec_test_common.hpp"

#include "rosette/rtps/stateful_reader.hpp"
#include "rosette/rtps/transport.hpp"

namespace spec_test {

extern uint32_t g_stateful_reader_test_now_ms;

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
    g.prefix.value[1] = 0x6Bu;
    g.entityId = eid;
    return g;
}

struct Fixture {
    CaptureTransport      tx{};
    Guid                  reader_guid = make_guid(0x91u, make_entity(6u, 6u, 6u, 0x04u));
    Guid                  writer_guid = make_guid(0x92u, make_entity(7u, 7u, 7u, 0x03u));
    Locator               loc = make_udpv4_locator(0xC0A801A2u, 7411u);
    DefaultStatefulReader r;
    size_t                baseline = 0;  ///< Messages sent before the test's first change.
    int32_t               hb_count = 0;

    explicit Fixture(ReliabilityKind reliability = ReliabilityKind::RELIABLE)
        : r(reader_guid, reliability, tx, nullptr, nullptr, 0u) {
        g_stateful_reader_test_now_ms = 1000u;
        (void)r.matched_writer_add(writer_guid, &loc, 1u);
        baseline = tx.unicast_msgs.size();  // the preemptive ACKNACK
    }

    ParseContext ctx() const {
        ParseContext c{};
        c.senderGuidPrefix = writer_guid.prefix;
        return c;
    }

    /// DATA `sn` arrives `at` ms after the start.
    void data(int64_t sn, uint32_t at) {
        static const Octet payload[5] = {0x00u, 0x01u, 0x00u, 0x00u, 0x42u};
        g_stateful_reader_test_now_ms = 1000u + at;
        ParsedData d{};
        d.readerId = reader_guid.entityId;
        d.writerId = writer_guid.entityId;
        d.writerSN = SequenceNumber::from_int64(sn);
        d.dataFlag = true;
        d.serializedPayload = payload;
        d.serializedPayloadSize = sizeof(payload);
        r.on_data(ctx(), d);
    }

    /// Runs the timer pass `at` ms after the start; returns the messages it sent.
    size_t tick(uint32_t at) {
        g_stateful_reader_test_now_ms = 1000u + at;
        const size_t before = tx.unicast_msgs.size();
        r.tick(g_stateful_reader_test_now_ms);
        return tx.unicast_msgs.size() - before;
    }

    /// Ticks every millisecond from `from` to `to` inclusive; returns the times a message went out.
    std::vector<uint32_t> run(uint32_t from, uint32_t to) {
        std::vector<uint32_t> sent{};
        for (uint32_t at = from; at <= to; ++at) {
            if (tick(at) != 0u) {
                sent.push_back(at);
            }
        }

        return sent;
    }

    bool last_acknack(ParsedAckNack& out) const {
        ParseOps ops{};
        ops.on_acknack = [](const ParseContext& c, const ParsedAckNack& a) noexcept {
            *static_cast<ParsedAckNack*>(c.user_data) = a;
        };
        const std::vector<Octet>& msg = tx.unicast_msgs.back();
        return parse_message(msg.data(), static_cast<uint32_t>(msg.size()), GuidPrefix{}, ops, &out)
               == ParseResult::OK;
    }
};

}  // namespace

// §8.4.12.2: a change missing behind a later one is requested once the
// delay passes, by a non-final ACKNACK whose base is the missing change.
static bool tc_srr_8_4_12_020(std::string& err) {
    Fixture f{};
    f.data(1, 0u);
    f.data(3, 0u);
    CHECK(f.r.ms_until_next_timer(1000u) == RTPS_GAP_REPAIR_DELAY_MS,
          "the timer is due one delay after the gap showed");
    CHECK(f.run(0u, RTPS_GAP_REPAIR_DELAY_MS - 1u).empty(), "nothing is sent within the delay");
    CHECK(f.tick(RTPS_GAP_REPAIR_DELAY_MS) == 1u, "one ACKNACK when the delay passes");
    ParsedAckNack ack{};
    CHECK(f.last_acknack(ack) && !ack.finalFlag
              && ack.readerSNState.bitmapBase == SequenceNumber::from_int64(2)
              && ack.readerSNState.test(0u) && !ack.readerSNState.test(1u),
          "it acknowledges SN 1 and requests exactly SN 2");
    return true;
}

// §8.4.12.2: unanswered requests back off by doubling and stop after
// RTPS_GAP_REPAIR_ROUNDS; a new change from the writer restores the rounds,
// and a stream of changes does not postpone a request.
static bool tc_srr_8_4_12_021(std::string& err) {
    constexpr uint32_t d = RTPS_GAP_REPAIR_DELAY_MS;
    Fixture            f{};
    f.data(1, 0u);
    f.data(3, 0u);
    f.data(4, d / 2u);
    f.data(5, d - 1u);
    const std::vector<uint32_t> sent = f.run(0u, 2000u);
    CHECK(sent == (std::vector<uint32_t>{d, 3u * d, 7u * d, 15u * d}),
          "requests at d, then waits of 2d, 4d, 8d; four rounds; later changes kept the first "
          "deadline");
    f.data(6, 2001u);
    CHECK(f.run(2002u, 2001u + 8u * d) == std::vector<uint32_t>{2001u + 8u * d},
          "a new change restores a round at the last, capped wait");
    return true;
}

// §8.4.12.2: an ACKNACK answering a HEARTBEAT is a request too, and pushes
// the timer's next request out by the doubled wait.
static bool tc_srr_8_4_12_022(std::string& err) {
    constexpr uint32_t d = RTPS_GAP_REPAIR_DELAY_MS;
    Fixture            f{};
    f.data(1, 0u);
    f.data(3, 0u);
    g_stateful_reader_test_now_ms = 1000u + 10u;
    ParsedHeartbeat hb{};
    hb.readerId = f.reader_guid.entityId;
    hb.writerId = f.writer_guid.entityId;
    hb.firstSN = SequenceNumber::from_int64(1);
    hb.lastSN = SequenceNumber::from_int64(3);
    hb.count = 1;
    const size_t before = f.tx.unicast_msgs.size();
    f.r.on_heartbeat(f.ctx(), hb);
    CHECK(f.tx.unicast_msgs.size() == before + 1u, "the HEARTBEAT is answered at once");
    CHECK(f.run(11u, 10u + 2u * d) == std::vector<uint32_t>{10u + 2u * d},
          "the timer waits the doubled delay after the HEARTBEAT's answer");
    return true;
}

// §8.4.12.2: a change that arrives out of order within the delay is not
// requested, and a best-effort reader never requests anything.
static bool tc_srr_8_4_12_023(std::string& err) {
    Fixture f{};
    f.data(1, 0u);
    f.data(3, 0u);
    f.data(2, RTPS_GAP_REPAIR_DELAY_MS / 2u);
    CHECK(f.run(0u, 1000u).empty(), "a gap closed by reordering sends nothing");
    CHECK(f.r.ms_until_next_timer(2000u) == UINT32_MAX, "and leaves no timer running");

    Fixture be{ReliabilityKind::BEST_EFFORT};
    be.data(1, 0u);
    be.data(3, 0u);
    CHECK(be.run(0u, 1000u).empty(), "a best-effort reader requests nothing");
    return true;
}

// §8.4.12.2: changes a HEARTBEAT shows MISSING with no later change to
// reveal them (the tail of a burst) are requested again after the doubled
// wait when the writer's resend does not arrive, not at its next HEARTBEAT.
static bool tc_srr_8_4_12_024(std::string& err) {
    constexpr uint32_t d = RTPS_GAP_REPAIR_DELAY_MS;
    Fixture            f{};
    f.data(1, 0u);
    ParsedHeartbeat hb{};
    hb.readerId = f.reader_guid.entityId;
    hb.writerId = f.writer_guid.entityId;
    hb.firstSN = SequenceNumber::from_int64(1);
    hb.lastSN = SequenceNumber::from_int64(3);
    hb.count = 1;
    const size_t before = f.tx.unicast_msgs.size();
    f.r.on_heartbeat(f.ctx(), hb);
    CHECK(f.tx.unicast_msgs.size() == before + 1u, "the HEARTBEAT is answered at once");
    CHECK(f.run(0u, 6u * d) == (std::vector<uint32_t>{2u * d, 6u * d}),
          "the unanswered request is repeated after 2d, then 4d");
    ParsedAckNack ack{};
    CHECK(f.last_acknack(ack) && ack.readerSNState.bitmapBase == SequenceNumber::from_int64(2)
              && ack.readerSNState.test(0u) && ack.readerSNState.test(1u),
          "each repeat requests SN 2 and 3");
    return true;
}

void append_stateful_reader_tests_8_4_12_gap_nack_on_quiet(std::vector<TestCase>& tests) {
    tests.push_back({"TC-SRR-8.4.12-020",
                     "§8.4.12.2: a change missing behind a later one is requested one delay later, "
                     "without a HEARTBEAT.",
                     tc_srr_8_4_12_020});
    tests.push_back({"TC-SRR-8.4.12-021",
                     "§8.4.12.2: unanswered gap requests double their wait and stop after the "
                     "rounds; a new change restores them; a stream does not postpone them.",
                     tc_srr_8_4_12_021});
    tests.push_back({"TC-SRR-8.4.12-022",
                     "§8.4.12.2: an ACKNACK answering a HEARTBEAT spends a gap-repair round.",
                     tc_srr_8_4_12_022});
    tests.push_back({"TC-SRR-8.4.12-023",
                     "§8.4.12.2: a gap closed by reordering within the delay, and a best-effort "
                     "reader, send no request.",
                     tc_srr_8_4_12_023});
    tests.push_back({"TC-SRR-8.4.12-024",
                     "§8.4.12.2: the tail of a burst a HEARTBEAT shows MISSING is requested again "
                     "after the doubled wait, not at the next HEARTBEAT.",
                     tc_srr_8_4_12_024});
}

}  // namespace spec_test
