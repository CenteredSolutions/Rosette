// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file test_stateful_reader_8_4_12_nack_frag_on_heartbeat.cpp
 * @brief Reliable StatefulReader: a partly received change is requested fragment by fragment.
 *
 * A writer's HEARTBEAT shows a fragmented change as missing until every
 * fragment has arrived. NACKing the whole change in the ACKNACK has the
 * writer resend all of its fragments back to back, and on a link whose
 * receive buffer holds fewer than all of them the same fragments are lost on
 * every retry. So the reader answers the HEARTBEAT with a NACK_FRAG naming
 * only the missing fragments, alongside an ACKNACK that leaves the change
 * out, as Fast DDS's reader does. A change the reader holds no fragments of,
 * including one refused because a shared assembly pool was full, is
 * requested whole.
 *
 * A lost last fragment leaves no later fragment to show the gap, so a
 * reliable reader does not wait for the writer's HEARTBEAT: once a partial
 * change has received nothing for RTPS_FRAGMENT_REPAIR_DELAY_MS it sends a
 * NACK_FRAG for the missing fragments on its own, for a bounded number of
 * rounds.
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
    uint32_t limit = kMaxUdpDatagram;

    uint32_t max_datagram_size() const noexcept override { return limit; }

    bool send_unicast(const BufRef& buf, const Locator&) noexcept override {
        unicast_msgs.emplace_back(buf.data, buf.data + buf.size);
        return true;
    }

    bool send_multicast(const BufRef&, const Locator&) noexcept override { return true; }
};

/// The ACKNACK and NACK_FRAGs one captured message carries.
struct Reply {
    bool parsed = false;
    int acknacks = 0;
    ParsedAckNack acknack{};
    std::vector<ParsedNackFrag> nack_frags{};
};

Reply inspect(const std::vector<Octet>& msg) {
    Reply r{};
    ParseOps ops{};
    ops.on_acknack = [](const ParseContext& c, const ParsedAckNack& a) noexcept {
        auto* out = static_cast<Reply*>(c.user_data);
        ++out->acknacks;
        out->acknack = a;
    };
    ops.on_nack_frag = [](const ParseContext& c, const ParsedNackFrag& n) noexcept {
        static_cast<Reply*>(c.user_data)->nack_frags.push_back(n);
    };
    r.parsed = parse_message(msg.data(), static_cast<uint32_t>(msg.size()), GuidPrefix{}, ops, &r)
               == ParseResult::OK;
    return r;
}

Guid make_guid(Octet p0, const EntityId& eid) {
    Guid g{};
    g.prefix.value[0] = p0;
    g.prefix.value[1] = 0x4Du;
    g.entityId = eid;
    return g;
}

/// Delivers fragment `frag` of a 3-fragment, 12-octet change `sn`.
void deliver_fragment(StatefulReader& r, const Guid& reader, const Guid& writer, int64_t sn,
                      uint32_t frag) {
    static const Octet bytes[4] = {0xA0u, 0xA1u, 0xA2u, 0xA3u};
    ParsedDataFrag df{};
    df.readerId = reader.entityId;
    df.writerId = writer.entityId;
    df.writerSN = SequenceNumber::from_int64(sn);
    df.fragmentStartingNum = frag;
    df.fragmentsInSubmessage = 1u;
    df.fragmentSize = 4u;
    df.dataSize = 12u;
    df.fragmentPayload = bytes;
    df.fragmentPayloadSize = sizeof(bytes);
    ParseContext ctx{};
    ctx.senderGuidPrefix = writer.prefix;
    ctx.sourceTimestamp = TIME_ZERO;
    r.on_data_frag(ctx, df);
}

void heartbeat(StatefulReader& r, const Guid& reader, const Guid& writer, int64_t last) {
    ParsedHeartbeat hb{};
    hb.readerId = reader.entityId;
    hb.writerId = writer.entityId;
    hb.firstSN = SequenceNumber::from_int64(1);
    hb.lastSN = SequenceNumber::from_int64(last);
    hb.count = 1;
    ParseContext ctx{};
    ctx.senderGuidPrefix = writer.prefix;
    g_stateful_reader_test_now_ms = 100u;
    r.on_heartbeat(ctx, hb);
}

const Locator kLoc = make_udpv4_locator(0xC0A80166u, 7411u);

}  // namespace

// §8.4.12, §8.3.7.5: on a link without IP fragmentation, a HEARTBEAT that
// shows a partly received change missing is answered with one message: an
// ACKNACK that acknowledges what precedes the change and NACKs nothing, and a
// NACK_FRAG naming exactly the fragments that did not arrive.
static bool tc_srr_8_4_12_002(std::string& err) {
    CaptureTransport tx{};
    tx.limit = 1472u;
    const Guid reader = make_guid(0x31u, make_entity(1u, 2u, 3u, 0x04u));
    const Guid writer = make_guid(0x42u, make_entity(4u, 5u, 6u, 0x03u));
    DefaultStatefulReader r(reader, ReliabilityKind::RELIABLE, tx);
    CHECK(r.matched_writer_add(writer, &kLoc, 1u), "match");

    deliver_fragment(r, reader, writer, 1, 1u);
    deliver_fragment(r, reader, writer, 1, 3u);
    tx.unicast_msgs.clear();
    heartbeat(r, reader, writer, 1);

    CHECK(tx.unicast_msgs.size() == 1u, "the HEARTBEAT is answered with one message");
    const Reply reply = inspect(tx.unicast_msgs[0]);
    CHECK(reply.parsed && reply.acknacks == 1, "the message parses and carries one ACKNACK");
    CHECK(reply.acknack.readerSNState.bitmapBase == SequenceNumber::from_int64(1),
          "the ACKNACK acknowledges only what precedes the partial change");
    CHECK(reply.acknack.readerSNState.numBits == 0u,
          "and NACKs nothing: the change is not resent whole");
    CHECK(reply.nack_frags.size() == 1u, "one NACK_FRAG accompanies it");
    const ParsedNackFrag& nf = reply.nack_frags[0];
    CHECK(nf.writerSN == SequenceNumber::from_int64(1) && nf.writerId == writer.entityId,
          "the NACK_FRAG names the partial change");
    CHECK(nf.fragmentNumberState.bitmapBase == 2u && nf.fragmentNumberState.test(0u)
              && !nf.fragmentNumberState.test(1u),
          "and exactly its missing fragment 2, not the fragments that arrived");
    return true;
}

// §8.4.12, §8.3.7.1: with IP fragmentation available, a change with no
// fragment received stays in the ACKNACK bitmap while a partly received one
// is left out of it and requested by NACK_FRAG.
static bool tc_srr_8_4_12_003(std::string& err) {
    CaptureTransport tx{};
    const Guid reader = make_guid(0x53u, make_entity(1u, 2u, 3u, 0x04u));
    const Guid writer = make_guid(0x64u, make_entity(4u, 5u, 6u, 0x03u));
    DefaultStatefulReader r(reader, ReliabilityKind::RELIABLE, tx);
    CHECK(r.matched_writer_add(writer, &kLoc, 1u), "match");

    deliver_fragment(r, reader, writer, 1, 2u);
    tx.unicast_msgs.clear();
    heartbeat(r, reader, writer, 2);

    CHECK(tx.unicast_msgs.size() == 1u, "the HEARTBEAT is answered with one message");
    const Reply reply = inspect(tx.unicast_msgs[0]);
    CHECK(reply.parsed && reply.acknacks == 1, "the message carries one ACKNACK");
    const SequenceNumberSet& state = reply.acknack.readerSNState;
    CHECK(state.bitmapBase == SequenceNumber::from_int64(1), "nothing is acknowledged yet");
    CHECK(!state.test(0u) && state.test(1u),
          "SN 2, with no fragment here, is NACKed; SN 1, partly here, is not");
    CHECK(reply.nack_frags.size() == 1u
              && reply.nack_frags[0].writerSN == SequenceNumber::from_int64(1),
          "SN 1 is requested by NACK_FRAG");
    const FragmentNumberSet& frags = reply.nack_frags[0].fragmentNumberState;
    CHECK(frags.bitmapBase == 1u && frags.test(0u) && !frags.test(1u) && frags.test(2u),
          "naming fragments 1 and 3");
    return true;
}

// §8.4.12, §8.4.14: a reader that shares a full assembly pool and holds no
// slot of it is refused a fragmented change; the change stays MISSING, is
// requested whole, and assembles when the writer resends it after a slot
// frees.
static bool tc_srr_8_4_12_004(std::string& err) {
    CaptureTransport tx{};
    ProxyPoolStorage<WriterProxy, 2> proxies;
    FragmentAssemblyPoolStorage<1> assembly;
    CHECK(proxies.reserve(2u), "one floor for each reader");
    int delivered = 0;
    const auto count = [](const CacheChange&, void* n) noexcept {
        ++*static_cast<int*>(n);
    };
    const Guid busy_guid = make_guid(0x71u, make_entity(1u, 2u, 3u, 0x04u));
    const Guid quiet_guid = make_guid(0x72u, make_entity(1u, 2u, 4u, 0x04u));
    const Guid busy_writer = make_guid(0x81u, make_entity(4u, 5u, 6u, 0x03u));
    const Guid quiet_writer = make_guid(0x82u, make_entity(4u, 5u, 7u, 0x03u));
    SharedPoolStatefulReader busy(busy_guid, ReliabilityKind::RELIABLE, tx, proxies, 1u, 1u,
                                  assembly, 1u);
    SharedPoolStatefulReader quiet(quiet_guid, ReliabilityKind::RELIABLE, tx, proxies, 1u, 1u,
                                   assembly, 1u, count, &delivered);
    CHECK(busy.matched_writer_add(busy_writer, &kLoc, 1u)
              && quiet.matched_writer_add(quiet_writer, &kLoc, 1u),
          "each reader matches its writer");

    deliver_fragment(busy, busy_guid, busy_writer, 1, 1u);
    for (uint32_t frag = 1u; frag <= 3u; ++frag) {
        deliver_fragment(quiet, quiet_guid, quiet_writer, 1, frag);
    }

    CHECK(delivered == 0, "the refused change is not delivered");
    tx.unicast_msgs.clear();
    heartbeat(quiet, quiet_guid, quiet_writer, 1);
    CHECK(tx.unicast_msgs.size() == 1u, "the HEARTBEAT is answered");
    const Reply reply = inspect(tx.unicast_msgs[0]);
    CHECK(reply.parsed && reply.acknacks == 1 && reply.acknack.readerSNState.test(0u)
              && reply.nack_frags.empty(),
          "the refused change is requested whole, with no NACK_FRAG");

    deliver_fragment(busy, busy_guid, busy_writer, 1, 2u);
    deliver_fragment(busy, busy_guid, busy_writer, 1, 3u);
    for (uint32_t frag = 1u; frag <= 3u; ++frag) {
        deliver_fragment(quiet, quiet_guid, quiet_writer, 1, frag);
    }

    CHECK(delivered == 1, "the resent change assembles once the busy reader's sample completes");
    CHECK(assembly.active_count() == 0u, "the shared slot is free again");
    return true;
}

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
// RTPS_FRAGMENT_REPAIR_ROUNDS times, after which the writer's HEARTBEAT
// drives repair; a best-effort reader, whose writer never repairs, sends none.
static bool tc_srr_8_4_12_006(std::string& err) {
    CaptureTransport      tx{};
    const Guid            reader = make_guid(0xB3u, make_entity(1u, 2u, 3u, 0x04u));
    const Guid            writer = make_guid(0xC4u, make_entity(4u, 5u, 6u, 0x03u));
    DefaultStatefulReader r(reader, ReliabilityKind::RELIABLE, tx);
    CHECK(r.matched_writer_add(writer, &kLoc, 1u), "match");
    tx.unicast_msgs.clear();

    g_stateful_reader_test_now_ms = 0u;
    deliver_fragment(r, reader, writer, 1, 1u);
    for (uint32_t round = 1u; round <= RTPS_FRAGMENT_REPAIR_ROUNDS + 2u; ++round) {
        r.tick(round * RTPS_FRAGMENT_REPAIR_DELAY_MS);
    }

    CHECK(tx.unicast_msgs.size() == RTPS_FRAGMENT_REPAIR_ROUNDS,
          "one NACK_FRAG per quiet period, up to the round limit");
    CHECK(r.ms_until_next_timer(10u * RTPS_FRAGMENT_REPAIR_DELAY_MS) == UINT32_MAX,
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

void append_stateful_reader_tests_8_4_12_nack_frag_on_heartbeat(std::vector<TestCase>& tests) {
    tests.push_back(
        {"TC-SRR-8.4.12-002",
         "§8.4.12/§8.3.7.5: on a narrow link a HEARTBEAT showing a partly received change missing "
         "gets an ACKNACK NACKing nothing plus a NACK_FRAG for exactly its missing fragments.",
         tc_srr_8_4_12_002});
    tests.push_back({"TC-SRR-8.4.12-003",
                     "§8.4.12/§8.3.7.1: a change with no fragment received stays in the ACKNACK "
                     "bitmap; a partly received one is left out and requested by NACK_FRAG.",
                     tc_srr_8_4_12_003});
    tests.push_back({"TC-SRR-8.4.12-004",
                     "§8.4.12/§8.4.14: a reader refused a fragmented change by a full shared "
                     "assembly pool requests it whole and assembles the resent change once a "
                     "slot frees.",
                     tc_srr_8_4_12_004});
    tests.push_back({"TC-SRR-8.4.12-005",
                     "§8.4.12/§8.4.14: a partial change whose fragments stop arriving is "
                     "requested by NACK_FRAG one quiet period after the last fragment, naming a "
                     "lost last fragment, with no HEARTBEAT.",
                     tc_srr_8_4_12_005});
    tests.push_back({"TC-SRR-8.4.12-006",
                     "§8.4.12: unanswered quiet-period NACK_FRAGs stop after "
                     "RTPS_FRAGMENT_REPAIR_ROUNDS; a best-effort reader sends none.",
                     tc_srr_8_4_12_006});
}

}  // namespace spec_test
