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
 * out, as Fast DDS's reader does.
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
}

}  // namespace spec_test
