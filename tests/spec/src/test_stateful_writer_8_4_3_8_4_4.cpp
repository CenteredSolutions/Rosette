#include "spec_test_common.hpp"

#include <cstdio>

#include "rosette/rtps/fragment_assembler.hpp"
#include "rosette/rtps/stateful_writer.hpp"

namespace spec_test {

extern uint32_t g_stateful_writer_test_now_ms;

#define CHECK(COND, MSG) do { if (!(COND)) { err = (MSG); return false; } } while (0)

namespace {

struct CaptureTransport : ITransport {
    std::vector<std::vector<Octet>> unicast_msgs{};

    bool send_unicast(const BufRef& buf, const Locator&) noexcept override {
        unicast_msgs.emplace_back(buf.data, buf.data + buf.size);
        return true;
    }

    bool send_multicast(const BufRef&, const Locator&) noexcept override {
        return true;
    }
};

struct PacketCap {
    int data_calls = 0;
    int hb_calls = 0;
    ParsedHeartbeat hb{};
};

Guid make_guid(Octet p0, Octet p1, Octet p2, const EntityId& eid) {
    Guid g{};
    g.prefix.value[0] = p0;
    g.prefix.value[1] = p1;
    g.prefix.value[2] = p2;
    g.entityId = eid;
    return g;
}

SequenceNumberSet make_empty_sn_set_from(const SequenceNumber& base) {
    SequenceNumberSet s{};
    s.clear();
    s.bitmapBase = base;
    s.numBits = 0u;
    return s;
}

} // namespace

static bool tc_swr_8_4_3_001(std::string& err) {
    CaptureTransport tx{};
    const Guid writer_guid = make_guid(0xC1u, 0xC2u, 0xC3u, make_entity(1u, 1u, 1u, 0xC2u));
    const Guid reader_guid = make_guid(0xD1u, 0xD2u, 0xD3u, make_entity(2u, 2u, 2u, 0xC7u));
    const Locator loc = make_udpv4_locator(0xC0A80144u, 7411u);

    StatefulWriter<1, 1> w(writer_guid,
                           ReliabilityKind::RELIABLE,
                           DurabilityKind::VOLATILE,
                           tx);
    CHECK(w.matched_reader_add(reader_guid, &loc, 1u, false),
          "matched_reader_add should succeed");

    const Octet payload[1] = {0x42u};
    CHECK(w.write_async(ChangeKind::ALIVE, payload, 1u),
          "write_async should succeed");

    auto* proxy = w.matched_reader_lookup(reader_guid);
    CHECK(proxy != nullptr, "proxy lookup should succeed");

    ParsedAckNack ack{};
    ack.readerId = reader_guid.entityId;
    ack.writerId = writer_guid.entityId;
    ack.readerSNState = make_empty_sn_set_from(SequenceNumber::from_int64(2));
    ack.count = 1;
    ack.finalFlag = true;

    ParseContext ctx{};
    ctx.senderGuidPrefix = reader_guid.prefix;
    g_stateful_writer_test_now_ms = 100u;
    w.on_acknack(ctx, ack);
    CHECK(proxy->highestAckedSN.to_int64() == 1, "cumulative ACK should advance highestAckedSN to SN=1");

    ack.readerSNState = make_empty_sn_set_from(SequenceNumber::from_int64(3));
    ack.count = 1;  // duplicate count should be ignored.
    w.on_acknack(ctx, ack);
    CHECK(proxy->highestAckedSN.to_int64() == 1, "duplicate ACKNACK count should not update ACK state");
    return true;
}

static bool tc_swr_8_4_4_001(std::string& err) {
    CaptureTransport tx{};
    const Guid writer_guid = make_guid(0xE1u, 0xE2u, 0xE3u, make_entity(1u, 3u, 5u, 0xC2u));
    const Guid reader_guid = make_guid(0xF1u, 0xF2u, 0xF3u, make_entity(2u, 4u, 6u, 0xC7u));
    const Locator loc = make_udpv4_locator(0xC0A80155u, 7411u);

    StatefulWriter<1, 1> w(writer_guid,
                           ReliabilityKind::RELIABLE,
                           DurabilityKind::VOLATILE,
                           tx,
                           5u,
                           1000u);
    CHECK(w.matched_reader_add(reader_guid, &loc, 1u, false),
          "matched_reader_add should succeed");

    const Octet payload[2] = {0xABu, 0xCDu};
    CHECK(w.write_async(ChangeKind::ALIVE, payload, 2u),
          "initial reliable write should succeed");
    tx.unicast_msgs.clear();

    ParsedNackFrag nf{};
    nf.readerId = reader_guid.entityId;
    nf.writerId = writer_guid.entityId;
    nf.writerSN = SequenceNumber::from_int64(1);
    nf.count = 1;

    ParseContext ctx{};
    ctx.senderGuidPrefix = reader_guid.prefix;
    g_stateful_writer_test_now_ms = 200u;
    w.on_nack_frag(ctx, nf);

    w.tick(203u);
    CHECK(tx.unicast_msgs.empty(), "tick before nackResponseDelay should not retransmit");

    w.tick(206u);
    CHECK(tx.unicast_msgs.size() == 2u,
          "nackResponseDelay expiry should send retransmitted DATA and directed HEARTBEAT");

    int data_packets = 0;
    int hb_packets = 0;
    ParsedHeartbeat last_hb{};
    for (const auto& pkt : tx.unicast_msgs) {
        PacketCap cap{};
        ParseOps ops{};
        ops.on_data = [](const ParseContext& ctx, const ParsedData&) noexcept {
            auto* c = static_cast<PacketCap*>(ctx.user_data);
            c->data_calls++;
        };
        ops.on_heartbeat = [](const ParseContext& ctx, const ParsedHeartbeat& hb) noexcept {
            auto* c = static_cast<PacketCap*>(ctx.user_data);
            c->hb_calls++;
            c->hb = hb;
        };
        CHECK(parse_message(pkt.data(),
                            static_cast<uint32_t>(pkt.size()),
                            GuidPrefix{},
                            ops,
                            &cap) == ParseResult::OK,
              "retransmit packet should parse");
        data_packets += cap.data_calls;
        hb_packets += cap.hb_calls;
        if (cap.hb_calls > 0) last_hb = cap.hb;
    }
    CHECK(data_packets == 1, "expected one retransmitted DATA packet");
    CHECK(hb_packets == 1, "expected one directed HEARTBEAT packet");
    CHECK(last_hb.finalFlag, "post-retransmit directed HEARTBEAT should set finalFlag=true");
    CHECK(last_hb.readerId == reader_guid.entityId,
          "post-retransmit directed HEARTBEAT should target requesting readerId");
    return true;
}

// Counts HEARTBEATs in captured messages and records the last one.
static int count_heartbeats(const CaptureTransport& tx, ParsedHeartbeat* last) {
    int total = 0;
    for (const auto& msg : tx.unicast_msgs) {
        PacketCap cap{};
        ParseOps ops{};
        ops.on_heartbeat = [](const ParseContext& ctx, const ParsedHeartbeat& hb) noexcept {
            auto* c = static_cast<PacketCap*>(ctx.user_data);
            c->hb = hb;
            c->hb_calls++;
        };
        (void)parse_message(msg.data(), static_cast<uint32_t>(msg.size()), GuidPrefix{}, ops, &cap);
        total += cap.hb_calls;
        if (cap.hb_calls > 0 && last) *last = cap.hb;
    }
    return total;
}

// §8.4.15.4 (reader-initiated exchange): a preemptive ACKNACK that
// requests nothing but acknowledges less than the writer has written is
// answered with a HEARTBEAT directed at that reader.
static bool tc_swr_8_4_3_002(std::string& err) {
    CaptureTransport tx{};
    const Guid writer_guid = make_guid(0xE1u, 0xE2u, 0xE3u, make_entity(1u, 1u, 1u, 0x03u));
    const Guid reader_guid = make_guid(0xF1u, 0xF2u, 0xF3u, make_entity(2u, 2u, 2u, 0x04u));
    const Locator loc = make_udpv4_locator(0xC0A80145u, 7411u);
    StatefulWriter<2, 1> w(writer_guid, ReliabilityKind::RELIABLE,
                           DurabilityKind::VOLATILE, tx);
    const Octet payload[1] = {0x42u};
    CHECK(w.write_async(ChangeKind::ALIVE, payload, 1u), "write_async should succeed");
    CHECK(w.matched_reader_add(reader_guid, &loc, 1u, false), "matched_reader_add should succeed");
    tx.unicast_msgs.clear();

    ParseContext ctx{};
    ctx.senderGuidPrefix = reader_guid.prefix;
    ParsedAckNack ack{};
    ack.readerId = reader_guid.entityId;
    ack.writerId = writer_guid.entityId;
    ack.readerSNState = make_empty_sn_set_from(SequenceNumber::from_int64(0));
    ack.count = 1;
    w.on_acknack(ctx, ack);

    ParsedHeartbeat hb{};
    CHECK(count_heartbeats(tx, &hb) == 1, "preemptive ACKNACK must elicit one HEARTBEAT");
    CHECK(hb.readerId == reader_guid.entityId, "the HEARTBEAT must be directed at the reader");
    CHECK(hb.lastSN == SequenceNumber::from_int64(1), "HEARTBEAT must advertise the written SN");

    // An ACKNACK that acknowledges everything written needs no answer.
    tx.unicast_msgs.clear();
    ack.readerSNState = make_empty_sn_set_from(SequenceNumber::from_int64(2));
    ack.count = 2;
    w.on_acknack(ctx, ack);
    CHECK(count_heartbeats(tx, nullptr) == 0, "an up-to-date reader must not get a HEARTBEAT");
    return true;
}

// §8.4.7.1: acknowledgment state for DDS wait_for_acknowledgments.
static bool tc_swr_8_4_7_001(std::string& err) {
    CaptureTransport tx{};
    const Guid writer_guid = make_guid(0xA1u, 0xA2u, 0xA3u, make_entity(1u, 1u, 1u, 0x03u));
    const Guid reader_guid = make_guid(0xB1u, 0xB2u, 0xB3u, make_entity(2u, 2u, 2u, 0x04u));
    const Locator loc = make_udpv4_locator(0xC0A80146u, 7411u);
    StatefulWriter<2, 1> w(writer_guid, ReliabilityKind::RELIABLE, DurabilityKind::VOLATILE, tx);
    CHECK(w.acknowledged_by_all(), "an empty history is trivially acknowledged");
    CHECK(w.matched_reader_add(reader_guid, &loc, 1u, false), "matched_reader_add should succeed");
    const Octet payload[1] = {0x42u};
    CHECK(w.write_async(ChangeKind::ALIVE, payload, 1u), "write_async should succeed");
    CHECK(!w.acknowledged_by_all(), "a sample not yet acknowledged keeps the wait going");

    ParseContext ctx{};
    ctx.senderGuidPrefix = reader_guid.prefix;
    ParsedAckNack ack{};
    ack.readerId = reader_guid.entityId;
    ack.writerId = writer_guid.entityId;
    ack.readerSNState = make_empty_sn_set_from(SequenceNumber::from_int64(2));
    ack.count = 1;
    w.on_acknack(ctx, ack);
    CHECK(w.acknowledged_by_all(), "a cumulative ACK of every SN completes the wait");

    StatefulWriter<2, 1> be(writer_guid, ReliabilityKind::BEST_EFFORT, DurabilityKind::VOLATILE, tx);
    CHECK(be.matched_reader_add(reader_guid, &loc, 1u, false), "matched_reader_add should succeed");
    CHECK(be.write_async(ChangeKind::ALIVE, payload, 1u), "write_async should succeed");
    CHECK(be.acknowledged_by_all(), "a BEST_EFFORT writer never waits for acknowledgments");
    return true;
}

// §8.4.7.1: the writer's next timed action bounds how long the participant sleeps.
static bool tc_swr_8_4_7_002(std::string& err) {
    CaptureTransport tx{};
    const Guid writer_guid = make_guid(0xC1u, 0xC2u, 0xC3u, make_entity(1u, 1u, 1u, 0x03u));
    const Guid reader_guid = make_guid(0xD1u, 0xD2u, 0xD3u, make_entity(2u, 2u, 2u, 0x04u));
    const Locator loc = make_udpv4_locator(0xC0A80147u, 7411u);
    StatefulWriter<2, 1> w(writer_guid, ReliabilityKind::RELIABLE, DurabilityKind::VOLATILE, tx,
                           /*nack_delay_ms=*/200u, /*hb_period_ms=*/2000u);
    CHECK(w.ms_until_next_timer(500u) == UINT32_MAX,
          "without matched readers there is nothing to heartbeat or repair");
    CHECK(w.matched_reader_add(reader_guid, &loc, 1u, false), "matched_reader_add should succeed");
    CHECK(w.ms_until_next_timer(500u) == 1500u, "next HEARTBEAT is one period after the last");

    const Octet payload[1] = {0x42u};
    CHECK(w.write_async(ChangeKind::ALIVE, payload, 1u), "write_async should succeed");
    ParseContext ctx{};
    ctx.senderGuidPrefix = reader_guid.prefix;
    ParsedAckNack nack{};
    nack.readerId = reader_guid.entityId;
    nack.writerId = writer_guid.entityId;
    nack.readerSNState = make_empty_sn_set_from(SequenceNumber::from_int64(1));
    nack.readerSNState.numBits = 1u;
    nack.readerSNState.set(0u);
    nack.count = 1;
    g_stateful_writer_test_now_ms = 600u;
    w.on_acknack(ctx, nack);
    CHECK(w.ms_until_next_timer(600u) == 200u, "a NACK arms nackResponseDelay, the sooner timer");
    CHECK(w.ms_until_next_timer(900u) == 0u, "an overdue repair reports zero");
    return true;
}

// A transport with a small datagram limit that records what is sent.
struct SmallMtuTransport : ITransport {
    uint32_t limit = 600u;
    std::vector<std::vector<Octet>> sent{};
    bool send_unicast(const BufRef& buf, const Locator&) noexcept override {
        sent.emplace_back(buf.data, buf.data + buf.size);
        return true;
    }
    bool send_multicast(const BufRef&, const Locator&) noexcept override { return true; }
    uint32_t max_datagram_size() const noexcept override { return limit; }
};

// §8.4.14.1 / §9.4.5.4: a sample larger than the datagram limit is sent as
// DATA_FRAG submessages, every datagram within the limit, and reassembles to
// the complete serializedPayload (CDR_LE header + data) with its inline QoS.
static bool tc_swr_8_4_14_001(std::string& err) {
    SmallMtuTransport tx{};
    const Guid writer_guid = make_guid(0xE1u, 0xE2u, 0xE3u, make_entity(1u, 1u, 1u, 0x03u));
    const Guid reader_guid = make_guid(0xF1u, 0xF2u, 0xF3u, make_entity(2u, 2u, 2u, 0x04u));
    const Locator loc = make_udpv4_locator(0xC0A80148u, 7411u);
    StatefulWriter<1, 1> w(writer_guid, ReliabilityKind::RELIABLE, DurabilityKind::VOLATILE, tx);
    CHECK(w.matched_reader_add(reader_guid, &loc, 1u, false), "matched_reader_add should succeed");

    std::vector<Octet> payload(1800u);
    for (size_t i = 0; i < payload.size(); ++i) payload[i] = static_cast<Octet>(i * 7u + 3u);
    InlineQos qos{};
    qos.hasRelatedSampleIdentity = true;
    qos.relatedSampleIdentity.writerGuid = reader_guid;
    qos.relatedSampleIdentity.sequenceNumber = SequenceNumber::from_int64(9);
    CHECK(w.write_async(ChangeKind::ALIVE, payload.data(), static_cast<uint32_t>(payload.size()), qos),
          "write_async should succeed");

    CHECK(tx.sent.size() > 1u, "a sample over the limit must be split into several datagrams");
    struct Collect {
        FragmentAssembler<1> assembler{};
        CacheChange complete{};
        int fragments = 0;
        bool done = false;
        bool qos_everywhere = true;
    } c{};
    ParseOps ops{};
    ops.on_data_frag = [](const ParseContext& ctx, const ParsedDataFrag& f) noexcept {
        auto* col = static_cast<Collect*>(ctx.user_data);
        ++col->fragments;
        col->qos_everywhere = col->qos_everywhere && f.inlineQosPresent &&
                              f.inlineQos.hasRelatedSampleIdentity;
        const Guid wg{ctx.senderGuidPrefix, f.writerId};
        if (col->assembler.on_data_frag(wg, f.writerSN, f.fragmentStartingNum,
                                        f.fragmentsInSubmessage, f.fragmentSize, f.dataSize,
                                        f.fragmentPayload, f.fragmentPayloadSize, TIME_ZERO,
                                        col->complete, f.inlineQosPresent ? &f.inlineQos : nullptr)) {
            col->done = true;
        }
    };
    for (const auto& msg : tx.sent) {
        CHECK(msg.size() <= tx.limit, "no datagram may exceed the transport's datagram limit");
        CHECK(parse_message(msg.data(), static_cast<uint32_t>(msg.size()), GuidPrefix{}, ops, &c) ==
                  ParseResult::OK,
              "every fragment datagram must parse");
    }
    CHECK(c.fragments == static_cast<int>(tx.sent.size()), "one DATA_FRAG per datagram");
    CHECK(c.qos_everywhere, "inline QoS must ride in every fragment");
    CHECK(c.done, "the fragments must reassemble into a complete sample");
    CHECK(c.complete.payload.size == payload.size() + 4u,
          "the serializedPayload is the CDR header plus the data");
    if (!(c.complete.payload.data[0] == 0x00u && c.complete.payload.data[1] == 0x01u)) {
        char b[96];
        std::snprintf(b, sizeof(b), "fragment 1 must carry the CDR_LE header; got %02x %02x %02x %02x %02x %02x",
                      c.complete.payload.data[0], c.complete.payload.data[1], c.complete.payload.data[2],
                      c.complete.payload.data[3], c.complete.payload.data[4], c.complete.payload.data[5]);
        err = b;
        return false;
    }
    bool same = true;
    for (size_t i = 0; i < payload.size(); ++i) same = same && c.complete.payload.data[4 + i] == payload[i];
    CHECK(same, "reassembled data must equal the written sample");
    CHECK(c.complete.inlineQos.hasRelatedSampleIdentity &&
              c.complete.inlineQos.relatedSampleIdentity == qos.relatedSampleIdentity,
          "the completed change keeps the sample's related sample identity");

    // A sample that fits is still sent as one DATA.
    tx.sent.clear();
    const Octet small[16] = {};
    CHECK(w.write_async(ChangeKind::ALIVE, small, sizeof(small)), "small write should succeed");
    CHECK(tx.sent.size() == 1u, "a sample within the limit stays a single DATA datagram");
    return true;
}

void append_stateful_writer_tests_8_4_3_8_4_4(std::vector<TestCase>& tests) {
    tests.push_back({"TC-SWR-8.4.3-001",
                     "§8.4.3: ACKNACK cumulative ACK updates ReaderProxy acknowledgment state; duplicate ACKNACK counts are ignored.",
                     tc_swr_8_4_3_001});
    tests.push_back({"TC-SWR-8.4.3-002",
                     "§8.4.15.4: a preemptive ACKNACK (no NACK bits, base below the next SN) is answered with a directed HEARTBEAT; an up-to-date ACK is not.",
                     tc_swr_8_4_3_002});
    tests.push_back({"TC-SWR-8.4.7-001",
                     "§8.4.7.1: acknowledged_by_all() is true for an empty history, false until every matched reader ACKs the newest SN, and always true for BEST_EFFORT.",
                     tc_swr_8_4_7_001});
    tests.push_back({"TC-SWR-8.4.7-002",
                     "§8.4.7.1: ms_until_next_timer() is unbounded without readers, the heartbeat remainder with readers, and the nackResponseDelay remainder after a NACK.",
                     tc_swr_8_4_7_002});
    tests.push_back({"TC-SWR-8.4.14-001",
                     "§8.4.14.1/§9.4.5.4: samples over the datagram limit go out as DATA_FRAG, every datagram within the limit, reassembling to the CDR header plus data with inline QoS in every fragment and in the result.",
                     tc_swr_8_4_14_001});
    tests.push_back({"TC-SWR-8.4.4-001",
                     "§8.4.4/§8.4.1.1: NACK_FRAG requests arm nackResponseDelay and trigger deferred retransmit plus directed final HEARTBEAT.",
                     tc_swr_8_4_4_001});
}

} // namespace spec_test
