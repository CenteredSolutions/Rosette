// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file test_stateful_reader_8_4_12_nack_frag_on_quiet.cpp
 * @brief Reliable StatefulReader: a partial change is repaired once its new fragments stop.
 *
 * A lost last fragment leaves no later fragment to show the gap, so a
 * reliable reader does not wait for the writer's HEARTBEAT: once a partial
 * change has stored no new fragment for RTPS_FRAGMENT_REPAIR_DELAY_MS it
 * sends a NACK_FRAG for the missing fragments on its own, for a bounded
 * number of rounds, doubling the wait after each one. Only a fragment the
 * change lacked counts as progress: duplicates and rejected fragments
 * neither postpone a request nor restore spent rounds. A change received
 * whole, or declared irrelevant by a HEARTBEAT or GAP, is requested no more.
 *
 * Requirement: REQ-SRR-8.4.12-NACKFRAG-ON-QUIET (stateful_reader.matrix.json).
 */

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

    uint32_t max_datagram_size() const noexcept override { return kMaxUdpDatagram; }

    bool send_unicast(const BufRef& buf, const Locator&) noexcept override {
        unicast_msgs.emplace_back(buf.data, buf.data + buf.size);
        return true;
    }

    bool send_multicast(const BufRef&, const Locator&) noexcept override { return true; }
};

/// The ACKNACK and NACK_FRAGs one captured message carries.
struct Reply {
    bool                        parsed = false;
    int                         acknacks = 0;
    std::vector<ParsedNackFrag> nack_frags{};
};

Reply inspect(const std::vector<Octet>& msg) {
    Reply    r{};
    ParseOps ops{};
    ops.on_acknack = [](const ParseContext& c, const ParsedAckNack&) noexcept {
        ++static_cast<Reply*>(c.user_data)->acknacks;
    };
    ops.on_nack_frag = [](const ParseContext& c, const ParsedNackFrag& n) noexcept {
        static_cast<Reply*>(c.user_data)->nack_frags.push_back(n);
    };
    r.parsed = parse_message(msg.data(), static_cast<uint32_t>(msg.size()), GuidPrefix{}, ops, &r)
               == ParseResult::OK;
    return r;
}

/// NACK_FRAGs across every captured message.
size_t count_nack_frags(const CaptureTransport& tx) {
    size_t n = 0u;
    for (const std::vector<Octet>& msg : tx.unicast_msgs) {
        n += inspect(msg).nack_frags.size();
    }

    return n;
}

Guid make_guid(Octet p0, const EntityId& eid) {
    Guid g{};
    g.prefix.value[0] = p0;
    g.prefix.value[1] = 0x4Du;
    g.entityId = eid;
    return g;
}

ParseContext from(const Guid& writer) {
    ParseContext ctx{};
    ctx.senderGuidPrefix = writer.prefix;
    ctx.sourceTimestamp = TIME_ZERO;
    return ctx;
}

/// Delivers fragment `frag` of change `sn`, by default one of 3 fragments of a 12-octet change.
void deliver_fragment(StatefulReader& r, const Guid& reader, const Guid& writer, int64_t sn,
                      uint32_t frag, uint16_t frag_size = 4u, uint32_t data_size = 12u) {
    static const Octet bytes[8] = {0xA0u, 0xA1u, 0xA2u, 0xA3u, 0xA4u, 0xA5u, 0xA6u, 0xA7u};
    ParsedDataFrag     df{};
    df.readerId = reader.entityId;
    df.writerId = writer.entityId;
    df.writerSN = SequenceNumber::from_int64(sn);
    df.fragmentStartingNum = frag;
    df.fragmentsInSubmessage = 1u;
    df.fragmentSize = frag_size;
    df.dataSize = data_size;
    df.fragmentPayload = bytes;
    df.fragmentPayloadSize = frag_size;
    r.on_data_frag(from(writer), df);
}

const Locator kLoc = make_udpv4_locator(0xC0A80166u, 7411u);

}  // namespace

// §8.4.12, §8.4.14: a partial change whose last fragment was lost is
// requested by NACK_FRAG one quiet period after its last fragment arrived,
// with no HEARTBEAT; a fragment arriving in the meantime pushes the deadline
// out, and the completed change leaves no timer behind.
static bool tc_srr_8_4_12_005(std::string& err) {
    CaptureTransport tx{};
    int              delivered = 0;
    const auto       count = [](const CacheChange&, void* n) noexcept {
        ++*static_cast<int*>(n);
    };
    const Guid            reader = make_guid(0x91u, make_entity(1u, 2u, 3u, 0x04u));
    const Guid            writer = make_guid(0xA2u, make_entity(4u, 5u, 6u, 0x03u));
    DefaultStatefulReader r(reader, ReliabilityKind::RELIABLE, tx, count, &delivered);
    CHECK(r.matched_writer_add(writer, &kLoc, 1u), "match");
    tx.unicast_msgs.clear();

    g_stateful_reader_test_now_ms = 1000u;
    deliver_fragment(r, reader, writer, 1, 1u);
    g_stateful_reader_test_now_ms = 1200u;
    deliver_fragment(r, reader, writer, 1, 2u);
    CHECK(r.ms_until_next_timer(1200u) == RTPS_FRAGMENT_REPAIR_DELAY_MS,
          "the driver is woken one quiet period after the latest fragment");
    r.tick(1000u + RTPS_FRAGMENT_REPAIR_DELAY_MS);
    CHECK(tx.unicast_msgs.empty(), "a fragment within the quiet period pushes the deadline out");

    r.tick(1200u + RTPS_FRAGMENT_REPAIR_DELAY_MS - 1u);
    CHECK(tx.unicast_msgs.empty(), "nothing is requested before the quiet period ends");
    r.tick(1200u + RTPS_FRAGMENT_REPAIR_DELAY_MS);
    CHECK(tx.unicast_msgs.size() == 1u, "one message once the quiet period ends");
    const Reply reply = inspect(tx.unicast_msgs[0]);
    CHECK(reply.parsed && reply.acknacks == 0 && reply.nack_frags.size() == 1u,
          "it carries one NACK_FRAG and no ACKNACK");
    const ParsedNackFrag& nf = reply.nack_frags[0];
    CHECK(nf.writerSN == SequenceNumber::from_int64(1) && nf.writerId == writer.entityId,
          "the NACK_FRAG names the partial change");
    CHECK(nf.fragmentNumberState.bitmapBase == 3u && nf.fragmentNumberState.test(0u)
              && nf.fragmentNumberState.numBits == 1u,
          "and exactly its lost last fragment");

    deliver_fragment(r, reader, writer, 1, 3u);
    CHECK(delivered == 1, "the repaired change is delivered");
    CHECK(r.ms_until_next_timer(1500u) == UINT32_MAX, "the completed change leaves no deadline");
    tx.unicast_msgs.clear();
    r.tick(5000u);
    CHECK(tx.unicast_msgs.empty(), "and no further NACK_FRAG");
    return true;
}

// §8.4.12: unanswered, the quiet-period NACK_FRAG is repeated at most
// RTPS_FRAGMENT_REPAIR_ROUNDS times, each wait double the last up to
// RTPS_FRAGMENT_REPAIR_MAX_DOUBLINGS doublings, after which the writer's
// HEARTBEAT drives repair; a best-effort reader, whose writer never repairs,
// sends none.
static bool tc_srr_8_4_12_006(std::string& err) {
    CaptureTransport      tx{};
    const Guid            reader = make_guid(0xB3u, make_entity(1u, 2u, 3u, 0x04u));
    const Guid            writer = make_guid(0xC4u, make_entity(4u, 5u, 6u, 0x03u));
    DefaultStatefulReader r(reader, ReliabilityKind::RELIABLE, tx);
    CHECK(r.matched_writer_add(writer, &kLoc, 1u), "match");
    tx.unicast_msgs.clear();

    g_stateful_reader_test_now_ms = 0u;
    deliver_fragment(r, reader, writer, 1, 1u);
    uint32_t due = 0u;
    for (uint32_t round = 0u; round < RTPS_FRAGMENT_REPAIR_ROUNDS; ++round) {
        const uint32_t doublings =
            round < RTPS_FRAGMENT_REPAIR_MAX_DOUBLINGS ? round : RTPS_FRAGMENT_REPAIR_MAX_DOUBLINGS;
        due += RTPS_FRAGMENT_REPAIR_DELAY_MS << doublings;
        r.tick(due - 1u);
        CHECK(tx.unicast_msgs.size() == round, "no NACK_FRAG before the doubled wait ends");
        r.tick(due);
        CHECK(tx.unicast_msgs.size() == round + 1u, "one NACK_FRAG when the doubled wait ends");
    }

    r.tick(due + (RTPS_FRAGMENT_REPAIR_DELAY_MS << 9u));
    CHECK(tx.unicast_msgs.size() == RTPS_FRAGMENT_REPAIR_ROUNDS, "none after the round limit");
    CHECK(r.ms_until_next_timer(due + 1u) == UINT32_MAX,
          "with the rounds spent no deadline remains");

    CaptureTransport      be_tx{};
    const Guid            be_reader = make_guid(0xD5u, make_entity(1u, 2u, 3u, 0x04u));
    DefaultStatefulReader be(be_reader, ReliabilityKind::BEST_EFFORT, be_tx);
    CHECK(be.matched_writer_add(writer, &kLoc, 1u), "best-effort match");
    be_tx.unicast_msgs.clear();
    deliver_fragment(be, be_reader, writer, 1, 1u);
    be.tick(10u * RTPS_FRAGMENT_REPAIR_DELAY_MS);
    CHECK(be_tx.unicast_msgs.empty(), "a best-effort reader never asks for fragments");
    return true;
}

// §8.4.12: a fragment arriving after a NACK_FRAG restores the rounds but
// keeps the doubled wait, so a writer pacing its fragments further apart than
// RTPS_FRAGMENT_REPAIR_DELAY_MS is not asked again at the first wait; a new
// sample starts again at the first wait.
static bool tc_srr_8_4_12_007(std::string& err) {
    CaptureTransport      tx{};
    const Guid            reader = make_guid(0xE6u, make_entity(1u, 2u, 3u, 0x04u));
    const Guid            writer = make_guid(0xF7u, make_entity(4u, 5u, 6u, 0x03u));
    DefaultStatefulReader r(reader, ReliabilityKind::RELIABLE, tx);
    CHECK(r.matched_writer_add(writer, &kLoc, 1u), "match");
    tx.unicast_msgs.clear();

    constexpr uint32_t kDelay = RTPS_FRAGMENT_REPAIR_DELAY_MS;
    g_stateful_reader_test_now_ms = 0u;
    deliver_fragment(r, reader, writer, 1, 1u);
    r.tick(kDelay);
    CHECK(tx.unicast_msgs.size() == 1u, "the first NACK_FRAG after one quiet period");

    const uint32_t paced = kDelay + 1u;
    g_stateful_reader_test_now_ms = paced;
    deliver_fragment(r, reader, writer, 1, 2u);
    CHECK(r.ms_until_next_timer(paced) == 2u * kDelay,
          "a fragment after a NACK_FRAG waits the doubled period");
    r.tick(paced + kDelay);
    CHECK(tx.unicast_msgs.size() == 1u, "nothing is asked at the first wait");
    r.tick(paced + 2u * kDelay);
    CHECK(tx.unicast_msgs.size() == 2u, "the NACK_FRAG follows the doubled wait");

    const uint32_t next = paced + 3u * kDelay;
    g_stateful_reader_test_now_ms = next;
    deliver_fragment(r, reader, writer, 2, 1u);
    CHECK(r.ms_until_next_timer(next) == kDelay, "a new sample starts at the first wait");
    return true;
}

// §8.4.12: only a fragment the change lacked is progress. A duplicate, and a
// fragment whose geometry disagrees with the change in assembly, arriving
// just before each deadline neither postpone the NACK_FRAG nor restore its
// rounds once spent.
static bool tc_srr_8_4_12_008(std::string& err) {
    CaptureTransport      tx{};
    const Guid            reader = make_guid(0x13u, make_entity(1u, 2u, 3u, 0x04u));
    const Guid            writer = make_guid(0x24u, make_entity(4u, 5u, 6u, 0x03u));
    DefaultStatefulReader r(reader, ReliabilityKind::RELIABLE, tx);
    CHECK(r.matched_writer_add(writer, &kLoc, 1u), "match");
    tx.unicast_msgs.clear();

    g_stateful_reader_test_now_ms = 0u;
    deliver_fragment(r, reader, writer, 1, 1u);
    uint32_t due = 0u;
    for (uint32_t round = 0u; round < RTPS_FRAGMENT_REPAIR_ROUNDS; ++round) {
        const uint32_t doublings =
            round < RTPS_FRAGMENT_REPAIR_MAX_DOUBLINGS ? round : RTPS_FRAGMENT_REPAIR_MAX_DOUBLINGS;
        due += RTPS_FRAGMENT_REPAIR_DELAY_MS << doublings;
        g_stateful_reader_test_now_ms = due - 1u;
        deliver_fragment(r, reader, writer, 1, 1u);
        deliver_fragment(r, reader, writer, 1, 2u, 6u, 12u);
        CHECK(r.ms_until_next_timer(due - 1u) == 1u,
              "a duplicate or rejected fragment leaves the deadline where it was");
        r.tick(due);
        CHECK(tx.unicast_msgs.size() == round + 1u, "the NACK_FRAG is sent on time");
    }

    const Reply reply = inspect(tx.unicast_msgs[0]);
    CHECK(reply.parsed && reply.nack_frags.size() == 1u, "each request is one NACK_FRAG");
    const FragmentNumberSet& missing = reply.nack_frags[0].fragmentNumberState;
    CHECK(missing.bitmapBase == 2u && missing.numBits == 2u && missing.test(0u) && missing.test(1u),
          "naming fragments 2 and 3: the rejected fragment 2 was not stored");

    g_stateful_reader_test_now_ms = due + 1u;
    deliver_fragment(r, reader, writer, 1, 1u);
    CHECK(r.ms_until_next_timer(due + 1u) == UINT32_MAX,
          "a duplicate after the rounds are spent does not restore them");
    return true;
}

// §8.4.12: a new fragment arriving after every round is spent restores a full
// set of rounds at the doubled wait the sample reached, and each of them
// requests only the fragments still missing.
static bool tc_srr_8_4_12_009(std::string& err) {
    CaptureTransport      tx{};
    const Guid            reader = make_guid(0x35u, make_entity(1u, 2u, 3u, 0x04u));
    const Guid            writer = make_guid(0x46u, make_entity(4u, 5u, 6u, 0x03u));
    DefaultStatefulReader r(reader, ReliabilityKind::RELIABLE, tx);
    CHECK(r.matched_writer_add(writer, &kLoc, 1u), "match");
    tx.unicast_msgs.clear();

    g_stateful_reader_test_now_ms = 0u;
    deliver_fragment(r, reader, writer, 1, 1u);
    uint32_t doublings = 0u;
    uint32_t due = 0u;
    for (uint32_t round = 0u; round < RTPS_FRAGMENT_REPAIR_ROUNDS; ++round) {
        due += RTPS_FRAGMENT_REPAIR_DELAY_MS << doublings;
        r.tick(due);
        if (doublings < RTPS_FRAGMENT_REPAIR_MAX_DOUBLINGS) {
            ++doublings;
        }
    }

    CHECK(tx.unicast_msgs.size() == RTPS_FRAGMENT_REPAIR_ROUNDS, "the first rounds are spent");
    CHECK(r.ms_until_next_timer(due + 1u) == UINT32_MAX, "and no deadline remains");

    const uint32_t late = due + 1u;
    g_stateful_reader_test_now_ms = late;
    deliver_fragment(r, reader, writer, 1, 2u);
    CHECK(r.ms_until_next_timer(late) == (RTPS_FRAGMENT_REPAIR_DELAY_MS << doublings),
          "a new fragment rearms the deadline at the doubled wait");

    tx.unicast_msgs.clear();
    due = late;
    for (uint32_t round = 0u; round < RTPS_FRAGMENT_REPAIR_ROUNDS; ++round) {
        due += RTPS_FRAGMENT_REPAIR_DELAY_MS << doublings;
        r.tick(due - 1u);
        CHECK(tx.unicast_msgs.size() == round, "no NACK_FRAG before the wait ends");
        r.tick(due);
        CHECK(tx.unicast_msgs.size() == round + 1u, "one NACK_FRAG of the restored rounds");
        const Reply reply = inspect(tx.unicast_msgs.back());
        CHECK(reply.parsed && reply.nack_frags.size() == 1u, "carried alone");
        const FragmentNumberSet& missing = reply.nack_frags[0].fragmentNumberState;
        CHECK(missing.bitmapBase == 3u && missing.numBits == 1u && missing.test(0u),
              "naming only fragment 3, the one still missing");
        if (doublings < RTPS_FRAGMENT_REPAIR_MAX_DOUBLINGS) {
            ++doublings;
        }
    }

    r.tick(due + (RTPS_FRAGMENT_REPAIR_DELAY_MS << 9u));
    CHECK(tx.unicast_msgs.size() == RTPS_FRAGMENT_REPAIR_ROUNDS,
          "the restored rounds are bounded too");
    return true;
}

/// How a partial change stops being wanted before its repair deadline.
enum class Superseded { kWholeData, kHeartbeat, kGap };

/// Runs one TC-SRR-8.4.12-010 case: fragment 1 of SN 1 arrives, `how` makes
/// SN 1 unwanted, and the deadline passes.
static bool superseded_partial_case(Superseded how, std::string& err) {
    CaptureTransport                 tx{};
    ProxyPoolStorage<WriterProxy, 1> proxies;
    FragmentAssemblyPoolStorage<1>   assembly;
    CHECK(proxies.reserve(1u), "a floor for the reader");
    const Guid               reader = make_guid(0x57u, make_entity(1u, 2u, 3u, 0x04u));
    const Guid               writer = make_guid(0x68u, make_entity(4u, 5u, 6u, 0x03u));
    SharedPoolStatefulReader r(reader, ReliabilityKind::RELIABLE, tx, proxies, 1u, 1u, assembly,
                               1u);
    CHECK(r.matched_writer_add(writer, &kLoc, 1u), "match");

    g_stateful_reader_test_now_ms = 0u;
    deliver_fragment(r, reader, writer, 1, 1u);
    CHECK(assembly.active_count() == 1u, "the partial change holds the slot");

    g_stateful_reader_test_now_ms = 1u;
    if (how == Superseded::kWholeData) {
        static const Octet payload[5] = {0x00u, 0x01u, 0x00u, 0x00u, 0x42u};
        ParsedData         data{};
        data.readerId = reader.entityId;
        data.writerId = writer.entityId;
        data.writerSN = SequenceNumber::from_int64(1);
        data.dataFlag = true;
        data.serializedPayload = payload;
        data.serializedPayloadSize = sizeof(payload);
        r.on_data(from(writer), data);
    } else if (how == Superseded::kHeartbeat) {
        ParsedHeartbeat hb{};
        hb.readerId = reader.entityId;
        hb.writerId = writer.entityId;
        hb.firstSN = SequenceNumber::from_int64(2);
        hb.lastSN = SequenceNumber::from_int64(2);
        hb.count = 1;
        r.on_heartbeat(from(writer), hb);
    } else {
        ParsedGap gap{};
        gap.readerId = reader.entityId;
        gap.writerId = writer.entityId;
        gap.gapStart = SequenceNumber::from_int64(1);
        gap.gapList.clear();
        gap.gapList.bitmapBase = SequenceNumber::from_int64(2);
        r.on_gap(from(writer), gap);
        CHECK(assembly.active_count() == 0u, "a GAP frees the slot at once");
    }

    tx.unicast_msgs.clear();
    r.tick(RTPS_FRAGMENT_REPAIR_DELAY_MS << RTPS_FRAGMENT_REPAIR_MAX_DOUBLINGS);
    CHECK(count_nack_frags(tx) == 0u, "no NACK_FRAG asks for the unwanted change");
    CHECK(assembly.active_count() == 0u, "and its slot is free");
    r.tick(10u * (RTPS_FRAGMENT_REPAIR_DELAY_MS << RTPS_FRAGMENT_REPAIR_MAX_DOUBLINGS));
    CHECK(count_nack_frags(tx) == 0u, "nor any later one");
    return true;
}

// §8.4.12, §8.4.10.4: a partial change received whole as a DATA, made lost by
// a HEARTBEAT's firstSN, or declared irrelevant by a GAP before its deadline
// is not requested, and the slot it held is freed.
static bool tc_srr_8_4_12_010(std::string& err) {
    if (!superseded_partial_case(Superseded::kWholeData, err)) {
        err = "whole DATA: " + err;
        return false;
    }

    if (!superseded_partial_case(Superseded::kHeartbeat, err)) {
        err = "HEARTBEAT firstSN: " + err;
        return false;
    }

    if (!superseded_partial_case(Superseded::kGap, err)) {
        err = "GAP: " + err;
        return false;
    }

    return true;
}

void append_stateful_reader_tests_8_4_12_nack_frag_on_quiet(std::vector<TestCase>& tests) {
    tests.push_back({"TC-SRR-8.4.12-005",
                     "§8.4.12/§8.4.14: a partial change whose fragments stop arriving is "
                     "requested by NACK_FRAG one quiet period after the last fragment, naming a "
                     "lost last fragment, with no HEARTBEAT.",
                     tc_srr_8_4_12_005});
    tests.push_back({"TC-SRR-8.4.12-006",
                     "§8.4.12: unanswered quiet-period NACK_FRAGs follow doubling waits and stop "
                     "after RTPS_FRAGMENT_REPAIR_ROUNDS; a best-effort reader sends none.",
                     tc_srr_8_4_12_006});
    tests.push_back({"TC-SRR-8.4.12-007",
                     "§8.4.12: a fragment arriving after a NACK_FRAG keeps the doubled wait; a "
                     "new sample starts again at RTPS_FRAGMENT_REPAIR_DELAY_MS.",
                     tc_srr_8_4_12_007});
    tests.push_back({"TC-SRR-8.4.12-008",
                     "§8.4.12: duplicate and rejected fragments neither postpone the quiet-period "
                     "NACK_FRAG nor restore its spent rounds.",
                     tc_srr_8_4_12_008});
    tests.push_back({"TC-SRR-8.4.12-009",
                     "§8.4.12: a new fragment after the rounds are spent restores a full set at "
                     "the doubled wait, each naming only the fragments still missing.",
                     tc_srr_8_4_12_009});
    tests.push_back({"TC-SRR-8.4.12-010",
                     "§8.4.12/§8.4.10.4: a partial change received whole, lost to a HEARTBEAT, "
                     "or GAPped before its deadline is not requested and its slot is freed.",
                     tc_srr_8_4_12_010});
}

}  // namespace spec_test
