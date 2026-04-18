#include "spec_test_common.hpp"

#include "rtps/stateful_writer.hpp"

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
    CHECK(w.write_reliable(ChangeKind::ALIVE, payload, 1u),
          "write_reliable should succeed");

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
    CHECK(w.write_reliable(ChangeKind::ALIVE, payload, 2u),
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

void append_stateful_writer_tests_8_4_3_8_4_4(std::vector<TestCase>& tests) {
    tests.push_back({"TC-SWR-8.4.3-001",
                     "§8.4.3: ACKNACK cumulative ACK updates ReaderProxy acknowledgment state; duplicate ACKNACK counts are ignored.",
                     tc_swr_8_4_3_001});
    tests.push_back({"TC-SWR-8.4.4-001",
                     "§8.4.4/§8.4.1.1: NACK_FRAG requests arm nackResponseDelay and trigger deferred retransmit plus directed final HEARTBEAT.",
                     tc_swr_8_4_4_001});
}

} // namespace spec_test
