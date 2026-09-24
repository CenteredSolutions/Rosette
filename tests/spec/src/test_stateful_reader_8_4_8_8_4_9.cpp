#include "spec_test_common.hpp"

#include "rosette/rtps/stateful_reader.hpp"
#include "rosette/rtps/transport.hpp"

namespace spec_test {

extern uint32_t g_stateful_reader_test_now_ms;

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

struct ChangeCapture {
    int calls = 0;
    SequenceNumber last_sn = SEQUENCENUMBER_UNKNOWN;
    std::vector<Octet> payload{};
};

void on_change_capture(const CacheChange& change, void* user) noexcept {
    auto* cap = static_cast<ChangeCapture*>(user);
    cap->calls++;
    cap->last_sn = change.sequenceNumber;
    cap->payload.assign(change.payload.data, change.payload.data + change.payload.size);
}

Guid make_guid(Octet p0, Octet p1, Octet p2, const EntityId& eid) {
    Guid g{};
    g.prefix.value[0] = p0;
    g.prefix.value[1] = p1;
    g.prefix.value[2] = p2;
    g.entityId = eid;
    return g;
}

} // namespace

static bool tc_srr_8_4_8_001(std::string& err) {
    CaptureTransport tx{};
    ChangeCapture cap{};
    const Guid reader_guid = make_guid(0x31u, 0x32u, 0x33u, make_entity(1u, 1u, 1u, 0xC7u));
    const Guid writer_guid = make_guid(0x41u, 0x42u, 0x43u, make_entity(2u, 2u, 2u, 0xC2u));
    const Locator loc = make_udpv4_locator(0xC0A80122u, 7411u);

    StatefulReader r(reader_guid,
                     ReliabilityKind::RELIABLE,
                     tx,
                     on_change_capture,
                     &cap);
    CHECK(r.matched_writer_add(writer_guid, &loc, 1u), "matched_writer_add should succeed");

    const Octet wire_payload[7] = {0x00u, 0x01u, 0x00u, 0x00u, 0xAAu, 0xBBu, 0xCCu};
    ParsedData data{};
    data.readerId = reader_guid.entityId;
    data.writerId = writer_guid.entityId;
    data.writerSN = SequenceNumber::from_int64(1);
    data.dataFlag = true;
    data.keyFlag = false;
    data.inlineQosPresent = false;
    data.serializedPayload = wire_payload;
    data.serializedPayloadSize = sizeof(wire_payload);

    ParseContext ctx{};
    ctx.senderGuidPrefix = writer_guid.prefix;
    ctx.sourceTimestamp = TIME_ZERO;

    r.on_data(ctx, data);
    r.on_data(ctx, data); // Duplicate SN should be ignored.

    CHECK(cap.calls == 1, "duplicate DATA should not trigger callback twice");
    CHECK(cap.last_sn.to_int64() == 1, "accepted DATA should keep writerSN");
    CHECK(cap.payload.size() == sizeof(wire_payload), "payload size should be preserved");
    CHECK(cap.payload[4] == 0xAAu && cap.payload[5] == 0xBBu && cap.payload[6] == 0xCCu,
          "payload bytes should be copied into reader cache change");
    return true;
}

// §8.4.12.2: a reliable reader delivers a retransmitted change that fills a
// gap (tracked MISSING), even though a later SN already arrived, and drops
// true duplicates.
static bool tc_srr_8_4_8_002(std::string& err) {
    CaptureTransport tx{};
    ChangeCapture cap{};
    const Guid reader_guid = make_guid(0x35u, 0x36u, 0x37u, make_entity(1u, 1u, 1u, 0x04u));
    const Guid writer_guid = make_guid(0x45u, 0x46u, 0x47u, make_entity(2u, 2u, 2u, 0x03u));
    const Locator loc = make_udpv4_locator(0xC0A80123u, 7411u);
    StatefulReader r(reader_guid, ReliabilityKind::RELIABLE, tx, on_change_capture, &cap);
    CHECK(r.matched_writer_add(writer_guid, &loc, 1u), "matched_writer_add should succeed");

    const Octet wire_payload[5] = {0x00u, 0x01u, 0x00u, 0x00u, 0x77u};
    ParsedData data{};
    data.readerId = reader_guid.entityId;
    data.writerId = writer_guid.entityId;
    data.dataFlag = true;
    data.serializedPayload = wire_payload;
    data.serializedPayloadSize = sizeof(wire_payload);
    ParseContext ctx{};
    ctx.senderGuidPrefix = writer_guid.prefix;

    auto deliver = [&](int64_t sn) {
        data.writerSN = SequenceNumber::from_int64(sn);
        r.on_data(ctx, data);
    };
    deliver(1);
    deliver(3);                        // SN 2 lost in transit
    CHECK(cap.calls == 2, "SN 1 and SN 3 are delivered");
    deliver(2);                        // repair retransmission
    CHECK(cap.calls == 3 && cap.last_sn.to_int64() == 2,
          "a retransmitted SN filling a gap must be delivered");
    deliver(2);
    deliver(3);
    CHECK(cap.calls == 3, "repeats of delivered SNs must be dropped");
    return true;
}

static bool tc_srr_8_4_9_001(std::string& err) {
    CaptureTransport tx{};
    const Guid reader_guid = make_guid(0x51u, 0x52u, 0x53u, make_entity(3u, 3u, 3u, 0xC7u));
    const Guid writer_guid = make_guid(0x61u, 0x62u, 0x63u, make_entity(4u, 4u, 4u, 0xC2u));
    const Locator loc = make_udpv4_locator(0xC0A80133u, 7411u);

    StatefulReader r(reader_guid,
                     ReliabilityKind::RELIABLE,
                     tx,
                     nullptr,
                     nullptr,
                     0u);
    CHECK(r.matched_writer_add(writer_guid, &loc, 1u), "matched_writer_add should succeed");

    const Octet wire_payload[5] = {0x00u, 0x01u, 0x00u, 0x00u, 0x11u};
    ParsedData data{};
    data.readerId = reader_guid.entityId;
    data.writerId = writer_guid.entityId;
    data.writerSN = SequenceNumber::from_int64(1);
    data.dataFlag = true;
    data.serializedPayload = wire_payload;
    data.serializedPayloadSize = sizeof(wire_payload);

    ParseContext data_ctx{};
    data_ctx.senderGuidPrefix = writer_guid.prefix;
    r.on_data(data_ctx, data);

    ParsedHeartbeat hb{};
    hb.readerId = reader_guid.entityId;
    hb.writerId = writer_guid.entityId;
    hb.firstSN = SequenceNumber::from_int64(1);
    hb.lastSN = SequenceNumber::from_int64(3);
    hb.count = 1;
    hb.finalFlag = true;

    ParseContext hb_ctx{};
    hb_ctx.senderGuidPrefix = writer_guid.prefix;
    g_stateful_reader_test_now_ms = 100u;
    r.on_heartbeat(hb_ctx, hb);

    CHECK(tx.unicast_msgs.size() == 1u,
          "HEARTBEAT with missing changes should trigger immediate ACKNACK when delay is zero");

    struct AckCap {
        int calls = 0;
        ParsedAckNack ack{};
    } cap{};

    ParseOps ops{};
    ops.on_acknack = [](const ParseContext& ctx, const ParsedAckNack& msg) noexcept {
        auto* c = static_cast<AckCap*>(ctx.user_data);
        c->calls++;
        c->ack = msg;
    };

    CHECK(parse_message(tx.unicast_msgs[0].data(),
                        static_cast<uint32_t>(tx.unicast_msgs[0].size()),
                        GuidPrefix{},
                        ops,
                        &cap) == ParseResult::OK,
          "ACKNACK response should parse");
    CHECK(cap.calls == 1, "expected one ACKNACK submessage");
    CHECK(cap.ack.readerId == reader_guid.entityId, "ACKNACK readerId should be local reader");
    CHECK(cap.ack.writerId == writer_guid.entityId, "ACKNACK writerId should target remote writer");
    CHECK(!cap.ack.finalFlag, "ACKNACK should carry missing bitmap and finalFlag=false");
    CHECK(cap.ack.readerSNState.bitmapBase.to_int64() == 2,
          "missing sequence base should begin at SN=2");
    CHECK(cap.ack.readerSNState.numBits >= 2u &&
          cap.ack.readerSNState.test(0u) &&
          cap.ack.readerSNState.test(1u),
          "ACKNACK bitmap should mark SN=2 and SN=3 missing");
    return true;
}

static bool tc_srr_8_4_9_002(std::string& err) {
    CaptureTransport tx{};
    const Guid reader_guid = make_guid(0x71u, 0x72u, 0x73u, make_entity(5u, 5u, 5u, 0xC7u));
    const Guid writer_guid = make_guid(0x81u, 0x82u, 0x83u, make_entity(6u, 6u, 6u, 0xC2u));
    const Locator loc = make_udpv4_locator(0xC0A80144u, 7411u);

    StatefulReader r(reader_guid, ReliabilityKind::RELIABLE, tx);
    CHECK(r.matched_writer_add(writer_guid, &loc, 1u), "matched_writer_add should succeed");

    const Octet frag_payload[4] = {0x10u, 0x11u, 0x12u, 0x13u};
    ParsedDataFrag df{};
    df.readerId = reader_guid.entityId;
    df.writerId = writer_guid.entityId;
    df.writerSN = SequenceNumber::from_int64(10);
    df.fragmentStartingNum = 1u;
    df.fragmentsInSubmessage = 1u;
    df.fragmentSize = 4u;
    df.dataSize = 8u;
    df.fragmentPayload = frag_payload;
    df.fragmentPayloadSize = sizeof(frag_payload);

    ParseContext data_ctx{};
    data_ctx.senderGuidPrefix = writer_guid.prefix;
    data_ctx.sourceTimestamp = TIME_ZERO;
    r.on_data_frag(data_ctx, df);

    ParsedHeartbeatFrag hb{};
    hb.readerId = reader_guid.entityId;
    hb.writerId = writer_guid.entityId;
    hb.writerSN = SequenceNumber::from_int64(10);
    hb.lastFragmentNum = 2u;
    hb.count = 1;

    ParseContext hb_ctx{};
    hb_ctx.senderGuidPrefix = writer_guid.prefix;
    r.on_heartbeat_frag(hb_ctx, hb);

    CHECK(tx.unicast_msgs.size() == 1u,
          "HEARTBEAT_FRAG should trigger NACK_FRAG for missing fragments");

    struct NackFragCap {
        int calls = 0;
        ParsedNackFrag msg{};
    } cap{};

    ParseOps ops{};
    ops.on_nack_frag = [](const ParseContext& ctx, const ParsedNackFrag& msg) noexcept {
        auto* c = static_cast<NackFragCap*>(ctx.user_data);
        c->calls++;
        c->msg = msg;
    };

    CHECK(parse_message(tx.unicast_msgs[0].data(),
                        static_cast<uint32_t>(tx.unicast_msgs[0].size()),
                        GuidPrefix{},
                        ops,
                        &cap) == ParseResult::OK,
          "NACK_FRAG response should parse");
    CHECK(cap.calls == 1, "expected one NACK_FRAG submessage");
    CHECK(cap.msg.readerId == reader_guid.entityId, "NACK_FRAG readerId should be local reader");
    CHECK(cap.msg.writerId == writer_guid.entityId, "NACK_FRAG writerId should target remote writer");
    CHECK(cap.msg.writerSN.to_int64() == 10, "NACK_FRAG should target fragmented sample SN");
    CHECK(cap.msg.fragmentNumberState.bitmapBase == 2u,
          "missing fragment base should start at fragment #2");
    CHECK(cap.msg.fragmentNumberState.numBits >= 1u && cap.msg.fragmentNumberState.test(0u),
          "fragment #2 should be requested in NACK_FRAG bitmap");
    return true;
}

// §8.4.7.1 heartbeatResponseDelay: a reader reports exactly how long the
// participant may sleep before the pending ACKNACK is due, and nothing
// once it is sent.
static bool tc_srr_8_4_7_001(std::string& err) {
    CaptureTransport tx{};
    const Guid reader_guid = make_guid(0x71u, 0x72u, 0x73u, make_entity(5u, 5u, 5u, 0x04u));
    const Guid writer_guid = make_guid(0x81u, 0x82u, 0x83u, make_entity(6u, 6u, 6u, 0x03u));
    const Locator loc = make_udpv4_locator(0xC0A80144u, 7411u);
    StatefulReader r(reader_guid, ReliabilityKind::RELIABLE, tx, nullptr, nullptr, 200u);
    CHECK(r.ms_until_next_timer(0u) == UINT32_MAX, "an unmatched reader has no timer");
    CHECK(r.matched_writer_add(writer_guid, &loc, 1u), "matched_writer_add should succeed");
    CHECK(r.ms_until_next_timer(0u) == UINT32_MAX, "no ACKNACK is pending before a HEARTBEAT");

    ParsedHeartbeat hb{};
    hb.readerId = reader_guid.entityId;
    hb.writerId = writer_guid.entityId;
    hb.firstSN = SequenceNumber::from_int64(1);
    hb.lastSN = SequenceNumber::from_int64(2);
    hb.count = 1;
    ParseContext ctx{};
    ctx.senderGuidPrefix = writer_guid.prefix;
    g_stateful_reader_test_now_ms = 100u;
    r.on_heartbeat(ctx, hb);

    CHECK(r.ms_until_next_timer(100u) == 200u, "full heartbeatResponseDelay remains right after the HEARTBEAT");
    CHECK(r.ms_until_next_timer(250u) == 50u, "the remaining delay shrinks with time");
    CHECK(r.ms_until_next_timer(400u) == 0u, "an overdue ACKNACK reports zero");
    const size_t before = tx.unicast_msgs.size();
    r.tick(400u);
    CHECK(tx.unicast_msgs.size() == before + 1u, "tick sends the due ACKNACK");
    CHECK(r.ms_until_next_timer(400u) == UINT32_MAX, "no timer remains once the ACKNACK is sent");
    return true;
}

void append_stateful_reader_tests_8_4_8_8_4_9(std::vector<TestCase>& tests) {
    tests.push_back({"TC-SRR-8.4.8-001",
                     "§8.4.8: on_data accepts new DATA as CacheChange and ignores duplicates whose writerSN is not newer than highestSNReceived.",
                     tc_srr_8_4_8_001});
    tests.push_back({"TC-SRR-8.4.8-002",
                     "§8.4.12.2: a retransmitted change filling a gap below the highest received SN is delivered; repeats of delivered SNs are dropped.",
                     tc_srr_8_4_8_002});
    tests.push_back({"TC-SRR-8.4.9-001",
                     "§8.3.7.5/§8.4.9: on_heartbeat schedules ACKNACK when missing changes exist; with heartbeatResponseDelay=0 it is sent immediately.",
                     tc_srr_8_4_9_001});
    tests.push_back({"TC-SRR-8.4.9-002",
                     "§8.3.7.4/§8.4.12: on_heartbeat_frag computes missing fragment numbers and sends NACK_FRAG for incomplete DATA_FRAG samples.",
                     tc_srr_8_4_9_002});
    tests.push_back({"TC-SRR-8.4.7-001",
                     "§8.4.7.1: ms_until_next_timer() reports the remaining heartbeatResponseDelay of a pending ACKNACK, zero when due, and no timer when none is pending.",
                     tc_srr_8_4_7_001});
}

} // namespace spec_test
