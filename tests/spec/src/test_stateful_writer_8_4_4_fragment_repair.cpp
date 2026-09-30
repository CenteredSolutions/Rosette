/**
 * @file test_stateful_writer_8_4_4_fragment_repair.cpp
 * @brief Reliable StatefulWriter: a NACK_FRAG is repaired with exactly the fragments it names.
 *
 * A reader holding part of a fragmented change asks for the rest with a
 * NACK_FRAG. Resending every fragment would put the missing ones back behind
 * the ones the reader already has; on a link whose receive buffer takes fewer
 * than all of them, the same ones are lost on every retry. The writer
 * therefore resends only the named fragments, after nackResponseDelay like
 * any repair, and follows them with a HEARTBEAT.
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
    uint32_t limit = 200u;

    bool send_unicast(const BufRef& buf, const Locator&) noexcept override {
        unicast_msgs.emplace_back(buf.data, buf.data + buf.size);
        return true;
    }

    bool send_multicast(const BufRef&, const Locator&) noexcept override { return true; }

    uint32_t max_datagram_size() const noexcept override { return limit; }
};

/// Fragment numbers, HEARTBEATs and GAPs across every captured message.
struct Sent {
    std::vector<uint32_t> fragments{};
    uint32_t total_fragments = 0;
    int heartbeats = 0;
    int gaps = 0;
    ParsedGap gap{};
};

Sent inspect(const CaptureTransport& tx) {
    Sent sent{};
    ParseOps ops{};
    ops.on_data_frag = [](const ParseContext& c, const ParsedDataFrag& f) noexcept {
        auto* out = static_cast<Sent*>(c.user_data);
        out->fragments.push_back(f.fragmentStartingNum);
        out->total_fragments = (f.dataSize + f.fragmentSize - 1u) / f.fragmentSize;
    };
    ops.on_heartbeat = [](const ParseContext& c, const ParsedHeartbeat&) noexcept {
        ++static_cast<Sent*>(c.user_data)->heartbeats;
    };
    ops.on_gap = [](const ParseContext& c, const ParsedGap& g) noexcept {
        auto* out = static_cast<Sent*>(c.user_data);
        ++out->gaps;
        out->gap = g;
    };
    for (const auto& msg : tx.unicast_msgs) {
        (void)parse_message(msg.data(), static_cast<uint32_t>(msg.size()), GuidPrefix{}, ops,
                            &sent);
    }

    return sent;
}

Guid make_guid(Octet p0, const EntityId& eid) {
    Guid g{};
    g.prefix.value[0] = p0;
    g.prefix.value[1] = 0x6Eu;
    g.entityId = eid;
    return g;
}

ParsedNackFrag nack_frag(const Guid& reader, const Guid& writer, int64_t sn,
                         std::initializer_list<uint32_t> frags, int32_t count) {
    ParsedNackFrag nf{};
    nf.readerId = reader.entityId;
    nf.writerId = writer.entityId;
    nf.writerSN = SequenceNumber::from_int64(sn);
    nf.fragmentNumberState.clear();
    for (uint32_t f : frags) {
        (void)nf.fragmentNumberState.add(f);
    }

    nf.count = count;
    return nf;
}

void deliver(StatefulWriter& w, const Guid& reader, const ParsedNackFrag& nf, uint32_t now_ms) {
    ParseContext ctx{};
    ctx.senderGuidPrefix = reader.prefix;
    g_stateful_writer_test_now_ms = now_ms;
    w.on_nack_frag(ctx, nf);
}

const Locator kLoc = make_udpv4_locator(0xC0A80170u, 7411u);

}  // namespace

// §8.3.7.5, §8.4.14.1: a NACK_FRAG for fragment 2 of a fragmented change is
// answered, once nackResponseDelay has passed, with that one DATA_FRAG and a
// HEARTBEAT, and nothing before the delay.
static bool tc_swr_8_4_4_002(std::string& err) {
    CaptureTransport tx{};
    const Guid writer = make_guid(0x11u, make_entity(1u, 2u, 3u, 0x02u));
    const Guid reader = make_guid(0x22u, make_entity(4u, 5u, 6u, 0x07u));
    StatefulWriterStorage<4, 1> w(writer, ReliabilityKind::RELIABLE, DurabilityKind::VOLATILE, tx,
                                  5u, 1000u);
    CHECK(w.matched_reader_add(reader, &kLoc, 1u, false), "match");

    std::vector<Octet> payload(400u, 0x5Au);
    CHECK(w.write_async(ChangeKind::ALIVE, payload.data(), 400u), "write a fragmented change");
    CHECK(inspect(tx).total_fragments >= 3u, "the change goes out in at least three fragments");
    tx.unicast_msgs.clear();

    deliver(w, reader, nack_frag(reader, writer, 1, {2u}, 1), 100u);
    w.tick(103u);
    CHECK(tx.unicast_msgs.empty(), "nothing is resent before nackResponseDelay");

    w.tick(106u);
    const Sent sent = inspect(tx);
    CHECK(sent.fragments.size() == 1u && sent.fragments[0] == 2u,
          "exactly fragment 2 is resent, not the fragments the reader holds");
    CHECK(sent.heartbeats == 1, "a HEARTBEAT follows the repair");
    return true;
}

// §8.3.7.5: a NACK_FRAG naming fragments 1 and 3 resends those two; a
// NACK_FRAG with a count already processed is a duplicate and sends nothing;
// one for a change no longer held is answered with GAP.
static bool tc_swr_8_4_4_003(std::string& err) {
    CaptureTransport tx{};
    const Guid writer = make_guid(0x33u, make_entity(1u, 2u, 3u, 0x02u));
    const Guid reader = make_guid(0x44u, make_entity(4u, 5u, 6u, 0x07u));
    StatefulWriterStorage<1, 1> w(writer, ReliabilityKind::RELIABLE, DurabilityKind::VOLATILE, tx,
                                  5u, 1000u);
    CHECK(w.matched_reader_add(reader, &kLoc, 1u, false), "match");

    std::vector<Octet> payload(400u, 0x3Cu);
    CHECK(w.write_async(ChangeKind::ALIVE, payload.data(), 400u), "write SN 1");
    tx.unicast_msgs.clear();

    deliver(w, reader, nack_frag(reader, writer, 1, {1u, 3u}, 1), 200u);
    w.tick(206u);
    Sent sent = inspect(tx);
    CHECK(sent.fragments == (std::vector<uint32_t>{1u, 3u}), "fragments 1 and 3 are resent");

    tx.unicast_msgs.clear();
    deliver(w, reader, nack_frag(reader, writer, 1, {2u}, 1), 300u);
    w.tick(306u);
    CHECK(tx.unicast_msgs.empty(), "a repeated NACK_FRAG count is ignored");

    CHECK(w.write_async(ChangeKind::ALIVE, payload.data(), 400u), "write SN 2, evicting SN 1");
    tx.unicast_msgs.clear();
    deliver(w, reader, nack_frag(reader, writer, 1, {2u}, 2), 400u);
    w.tick(406u);
    sent = inspect(tx);
    CHECK(sent.fragments.empty(), "an evicted change sends no fragments");
    CHECK(sent.gaps == 1 && sent.gap.gapStart == SequenceNumber::from_int64(1),
          "and is answered with a GAP naming it");
    return true;
}

void append_stateful_writer_tests_8_4_4_fragment_repair(std::vector<TestCase>& tests) {
    tests.push_back({"TC-SWR-8.4.4-002",
                     "§8.3.7.5/§8.4.14.1: a NACK_FRAG is answered after nackResponseDelay with "
                     "exactly the named DATA_FRAG and a HEARTBEAT.",
                     tc_swr_8_4_4_002});
    tests.push_back({"TC-SWR-8.4.4-003",
                     "§8.3.7.5: named fragments only; a repeated NACK_FRAG count is ignored; a "
                     "change no longer held is answered with GAP.",
                     tc_swr_8_4_4_003});
}

}  // namespace spec_test
