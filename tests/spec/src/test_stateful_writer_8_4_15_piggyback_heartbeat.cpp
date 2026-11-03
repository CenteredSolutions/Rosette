// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file test_stateful_writer_8_4_15_piggyback_heartbeat.cpp
 * @brief Reliable StatefulWriter: every depth/2-th write carries a HEARTBEAT in the DATA's datagram.
 *
 * A reliable reader requests a lost change only in answer to a HEARTBEAT.
 * A writer that heartbeats only once per heartbeatPeriod leaves a reader
 * parked behind one lost datagram for up to that period, and KEEP_LAST has
 * evicted the change by the time it asks. The writer therefore piggybacks a
 * HEARTBEAT on the DATA of every depth/2-th write (§8.4.15.4), in the same
 * RTPS message: the DATA is padded to a 4-octet boundary so the HEARTBEAT
 * that follows is aligned (§9.4.1), or, when the DATA's bytes cannot be
 * padded or the message would exceed the datagram limit, the HEARTBEAT goes
 * in a datagram of its own.
 */

#include "spec_test_common.hpp"

#include "rosette/rtps/stateful_writer.hpp"

namespace spec_test {

extern uint32_t g_stateful_writer_test_now_ms;

#define CHECK(COND, MSG) do { if (!(COND)) { err = (MSG); return false; } } while (0)

namespace {

struct CaptureTransport : ITransport {
    std::vector<std::vector<Octet>> unicast_msgs{};
    uint32_t limit = 65507u;

    bool send_unicast(const BufRef& buf, const Locator&) noexcept override {
        unicast_msgs.emplace_back(buf.data, buf.data + buf.size);
        return true;
    }

    bool send_multicast(const BufRef&, const Locator&) noexcept override { return true; }

    uint32_t max_datagram_size() const noexcept override { return limit; }
};

/// What one captured datagram contains, in submessage order.
struct Datagram {
    bool parsed = false;
    int data = 0;
    int data_frags = 0;
    int heartbeats = 0;
    bool heartbeat_after_data = false;
    uint32_t payload_size = 0;
    std::vector<Octet> payload{};
    ParsedHeartbeat heartbeat{};
};

Datagram inspect(const std::vector<Octet>& msg) {
    Datagram d{};
    ParseOps ops{};
    ops.on_data = [](const ParseContext& c, const ParsedData& p) noexcept {
        auto* out = static_cast<Datagram*>(c.user_data);
        ++out->data;
        out->payload_size = p.serializedPayloadSize;
        out->payload.assign(p.serializedPayload, p.serializedPayload + p.serializedPayloadSize);
    };
    ops.on_data_frag = [](const ParseContext& c, const ParsedDataFrag&) noexcept {
        ++static_cast<Datagram*>(c.user_data)->data_frags;
    };
    ops.on_heartbeat = [](const ParseContext& c, const ParsedHeartbeat& hb) noexcept {
        auto* out = static_cast<Datagram*>(c.user_data);
        ++out->heartbeats;
        out->heartbeat = hb;
        out->heartbeat_after_data = out->data + out->data_frags > 0;
    };
    d.parsed = parse_message(msg.data(), static_cast<uint32_t>(msg.size()), GuidPrefix{}, ops, &d)
            == ParseResult::OK;
    return d;
}

Guid make_guid(Octet p0, const EntityId& eid) {
    Guid g{};
    g.prefix.value[0] = p0;
    g.prefix.value[1] = 0x5Bu;
    g.entityId = eid;
    return g;
}

const Locator kLoc = make_udpv4_locator(0xC0A80190u, 7411u);

}  // namespace

// §8.4.15.4, §9.4.1: with KEEP_LAST(4), writes 2 and 4 each go out as one
// datagram holding the DATA, padded to a 4-octet boundary, followed by a
// non-final HEARTBEAT to the reader that advertises the history; writes 1
// and 3 carry none.
static bool tc_swr_8_4_15_001(std::string& err) {
    CaptureTransport tx{};
    const Guid writer = make_guid(0x11u, make_entity(1u, 2u, 3u, 0x02u));
    const Guid reader = make_guid(0x22u, make_entity(4u, 5u, 6u, 0x07u));
    StatefulWriterStorage<4, 1> w(writer, ReliabilityKind::RELIABLE, DurabilityKind::VOLATILE, tx,
                                  5u, 1000u);
    CHECK(w.matched_reader_add(reader, &kLoc, 1u, false), "match");

    const Octet payload[3] = {0xAAu, 0xBBu, 0xCCu};
    for (int i = 1; i <= 4; ++i) {
        CHECK(w.write_async(ChangeKind::ALIVE, payload, 3u), "write");
    }

    CHECK(tx.unicast_msgs.size() == 4u, "one datagram per write: HEARTBEATs share the DATA's");
    for (size_t i = 0; i < 4u; ++i) {
        const Datagram d = inspect(tx.unicast_msgs[i]);
        const bool due = i == 1u || i == 3u;
        CHECK(d.parsed && d.data == 1, "every datagram parses and carries its DATA");
        CHECK(d.heartbeats == (due ? 1 : 0), "only writes 2 and 4 carry a HEARTBEAT");
        if (!due) {
            CHECK(d.payload_size == 7u && d.payload[3] == 0u,
                  "a DATA with nothing after it is not padded");
            continue;
        }

        CHECK(d.heartbeat_after_data, "the HEARTBEAT follows the DATA");
        CHECK(d.payload_size == 8u && d.payload[3] == 1u && d.payload[7] == 0u,
              "the DATA is padded by one octet, recorded in the encapsulation options");
        CHECK(d.payload[4] == 0xAAu && d.payload[6] == 0xCCu, "the payload itself is intact");
        CHECK(!d.heartbeat.finalFlag, "the HEARTBEAT asks the reader to answer");
        CHECK(d.heartbeat.readerId == reader.entityId, "the HEARTBEAT is addressed to the reader");
        CHECK(d.heartbeat.firstSN == SequenceNumber::from_int64(1)
                  && d.heartbeat.lastSN == SequenceNumber::from_int64(static_cast<int64_t>(i + 1)),
              "the HEARTBEAT advertises the history up to the change it rides with");
    }

    const Datagram second = inspect(tx.unicast_msgs[1]);
    const Datagram fourth = inspect(tx.unicast_msgs[3]);
    CHECK(fourth.heartbeat.count > second.heartbeat.count, "HEARTBEAT counts increase");
    return true;
}

// §8.4.15.4, §8.4.1.1: a periodic HEARTBEAT restarts the count, so the next
// piggybacked one comes depth/2 writes after it; a BEST_EFFORT writer never
// piggybacks.
static bool tc_swr_8_4_15_002(std::string& err) {
    CaptureTransport tx{};
    const Guid writer = make_guid(0x33u, make_entity(1u, 2u, 3u, 0x02u));
    const Guid reader = make_guid(0x44u, make_entity(4u, 5u, 6u, 0x07u));
    StatefulWriterStorage<4, 1> w(writer, ReliabilityKind::RELIABLE, DurabilityKind::VOLATILE, tx,
                                  5u, 1000u);
    CHECK(w.matched_reader_add(reader, &kLoc, 1u, false), "match");

    const Octet payload[4] = {1u, 2u, 3u, 4u};
    g_stateful_writer_test_now_ms = 0u;
    CHECK(w.write_async(ChangeKind::ALIVE, payload, 4u), "write 1");
    g_stateful_writer_test_now_ms = 1000u;
    w.tick(1000u);
    tx.unicast_msgs.clear();

    CHECK(w.write_async(ChangeKind::ALIVE, payload, 4u), "write 2");
    CHECK(tx.unicast_msgs.size() == 1u && inspect(tx.unicast_msgs[0]).heartbeats == 0,
          "one write after the periodic HEARTBEAT carries none");
    CHECK(w.write_async(ChangeKind::ALIVE, payload, 4u), "write 3");
    CHECK(tx.unicast_msgs.size() == 2u && inspect(tx.unicast_msgs[1]).heartbeats == 1,
          "the second write after it does");

    CaptureTransport be_tx{};
    StatefulWriterStorage<2, 1> be(writer, ReliabilityKind::BEST_EFFORT, DurabilityKind::VOLATILE,
                                   be_tx, 5u, 1000u);
    CHECK(be.matched_reader_add(reader, &kLoc, 1u, false), "match best-effort");
    for (int i = 0; i < 4; ++i) {
        CHECK(be.write_async(ChangeKind::ALIVE, payload, 4u), "best-effort write");
    }

    for (const auto& msg : be_tx.unicast_msgs) {
        CHECK(inspect(msg).heartbeats == 0, "a best-effort writer sends no HEARTBEAT");
    }

    return true;
}

// §9.4.5.3, §9.4.1: a preframed change framed in place carries its
// HEARTBEAT in the tailroom reserved after the payload; one whose payload
// does not end on a 4-octet boundary cannot be padded in place, so its
// HEARTBEAT follows in a datagram of its own. Either way, a later repair
// sends the stored payload unchanged.
static bool tc_swr_8_4_15_003(std::string& err) {
    const Guid writer = make_guid(0x55u, make_entity(1u, 2u, 3u, 0x02u));
    const Guid reader = make_guid(0x66u, make_entity(4u, 5u, 6u, 0x07u));
    const Octet aligned[8] = {0x00u, 0x03u, 0x00u, 0x00u, 1u, 2u, 3u, 4u};
    const Octet unaligned[7] = {0x00u, 0x03u, 0x00u, 0x00u, 1u, 2u, 3u};

    CaptureTransport tx{};
    StatefulWriterStorage<2, 1> w(writer, ReliabilityKind::RELIABLE, DurabilityKind::VOLATILE, tx,
                                  5u, 1000u);
    CHECK(w.matched_reader_add(reader, &kLoc, 1u, false), "match");
    CHECK(w.write_preframed_async(ChangeKind::ALIVE, aligned, 8u), "write aligned");
    CHECK(tx.unicast_msgs.size() == 1u, "one datagram");
    Datagram d = inspect(tx.unicast_msgs[0]);
    CHECK(d.parsed && d.data == 1 && d.heartbeats == 1 && d.heartbeat_after_data,
          "an aligned in-place DATA carries the HEARTBEAT in its datagram");
    CHECK(d.payload_size == 8u, "the in-place DATA is not padded");

    ParsedAckNack ack{};
    ack.readerId = reader.entityId;
    ack.writerId = writer.entityId;
    ack.readerSNState.clear();
    ack.readerSNState.bitmapBase = SequenceNumber::from_int64(1);
    ack.readerSNState.set(0u);
    ack.count = 1;
    ParseContext ctx{};
    ctx.senderGuidPrefix = reader.prefix;
    g_stateful_writer_test_now_ms = 100u;
    w.on_acknack(ctx, ack);
    g_stateful_writer_test_now_ms = 200u;
    w.tick(200u);
    bool repaired = false;
    for (size_t i = 1; i < tx.unicast_msgs.size(); ++i) {
        const Datagram r = inspect(tx.unicast_msgs[i]);
        if (r.data == 1) {
            repaired = r.payload_size == 8u && r.payload == d.payload;
        }
    }

    CHECK(repaired, "the repair resends the stored payload unchanged");

    CaptureTransport tx2{};
    StatefulWriterStorage<2, 1> w2(writer, ReliabilityKind::RELIABLE, DurabilityKind::VOLATILE, tx2,
                                   5u, 1000u);
    CHECK(w2.matched_reader_add(reader, &kLoc, 1u, false), "match second writer");
    CHECK(w2.write_preframed_async(ChangeKind::ALIVE, unaligned, 7u), "write unaligned");
    CHECK(tx2.unicast_msgs.size() == 2u, "the HEARTBEAT goes in a second datagram");
    d = inspect(tx2.unicast_msgs[0]);
    CHECK(d.parsed && d.data == 1 && d.heartbeats == 0 && d.payload_size == 7u,
          "the DATA datagram is unchanged");
    d = inspect(tx2.unicast_msgs[1]);
    CHECK(d.parsed && d.data == 0 && d.heartbeats == 1 && !d.heartbeat.finalFlag,
          "the second datagram is the non-final HEARTBEAT");
    return true;
}

// §8.4.14.1, §8.4.15.4: a fragmented change sends exactly one HEARTBEAT,
// every datagram within the limit, and a DATA that fills the limit sends its
// HEARTBEAT separately.
static bool tc_swr_8_4_15_004(std::string& err) {
    const Guid writer = make_guid(0x77u, make_entity(1u, 2u, 3u, 0x02u));
    const Guid reader = make_guid(0x88u, make_entity(4u, 5u, 6u, 0x07u));
    std::vector<Octet> big(1000u, 0x5Au);

    for (uint32_t limit : {600u, 614u}) {
        CaptureTransport tx{};
        tx.limit = limit;
        StatefulWriterStorage<1, 1> w(writer, ReliabilityKind::RELIABLE, DurabilityKind::VOLATILE,
                                      tx, 5u, 1000u);
        CHECK(w.matched_reader_add(reader, &kLoc, 1u, false), "match");
        CHECK(w.write_async(ChangeKind::ALIVE, big.data(), 1000u), "write fragmented");

        int frags = 0;
        int heartbeats = 0;
        for (const auto& msg : tx.unicast_msgs) {
            const Datagram d = inspect(msg);
            CHECK(d.parsed && msg.size() <= limit, "every datagram parses within the limit");
            frags += d.data_frags;
            heartbeats += d.heartbeats;
        }

        CHECK(frags >= 2 && heartbeats == 1, "the fragments are followed by exactly one HEARTBEAT");
    }

    // A DATA exactly at the limit leaves no room: 20 + 4 + 20 (DATA fixed
    // fields) + 4 (encapsulation) + 452 = 500 octets.
    CaptureTransport tx{};
    tx.limit = 500u;
    StatefulWriterStorage<1, 1> w(writer, ReliabilityKind::RELIABLE, DurabilityKind::VOLATILE, tx,
                                  5u, 1000u);
    CHECK(w.matched_reader_add(reader, &kLoc, 1u, false), "match");
    CHECK(w.write_async(ChangeKind::ALIVE, big.data(), 452u), "write at the limit");
    CHECK(tx.unicast_msgs.size() == 2u, "the HEARTBEAT goes in its own datagram");
    CHECK(tx.unicast_msgs[0].size() == 500u && inspect(tx.unicast_msgs[0]).heartbeats == 0,
          "the DATA fills the datagram alone");
    CHECK(inspect(tx.unicast_msgs[1]).heartbeats == 1, "the HEARTBEAT follows");
    return true;
}

void append_stateful_writer_tests_8_4_15_piggyback_heartbeat(std::vector<TestCase>& tests) {
    tests.push_back({"TC-SWR-8.4.15-001",
                     "§8.4.15.4/§9.4.1: every depth/2-th write carries a non-final HEARTBEAT after its padded DATA in one datagram.",
                     tc_swr_8_4_15_001});
    tests.push_back({"TC-SWR-8.4.15-002",
                     "§8.4.15.4/§8.4.1.1: a periodic HEARTBEAT restarts the piggyback count; best-effort writers never piggyback.",
                     tc_swr_8_4_15_002});
    tests.push_back({"TC-SWR-8.4.15-003",
                     "§9.4.5.3/§9.4.1: in-place DATA carries the HEARTBEAT in reserved tailroom when aligned, else in its own datagram; repairs are unchanged.",
                     tc_swr_8_4_15_003});
    tests.push_back({"TC-SWR-8.4.15-004",
                     "§8.4.14.1/§8.4.15.4: fragmented and limit-filling DATA each send exactly one HEARTBEAT within the datagram limit.",
                     tc_swr_8_4_15_004});
}

}  // namespace spec_test
