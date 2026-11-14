// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

#include "spec_test_common.hpp"

#include "rosette/rtps/stateful_writer.hpp"

namespace spec_test {

extern uint32_t g_stateful_writer_test_now_ms;

#define CHECK(COND, MSG) do { if (!(COND)) { err = (MSG); return false; } } while (0)

namespace {

struct NullTransport : ITransport {
    bool send_unicast(const BufRef&, const Locator&) noexcept override { return true; }
    bool send_multicast(const BufRef&, const Locator&) noexcept override { return true; }
};

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

Guid make_guid(Octet p0, Octet p1, Octet p2, const EntityId& eid) {
    Guid g{};
    g.prefix.value[0] = p0;
    g.prefix.value[1] = p1;
    g.prefix.value[2] = p2;
    g.entityId = eid;
    return g;
}

bool add_one_change(StatefulWriterStorage<2, 1>& w, Octet byte) {
    BufRef payload{};
    HistoryChange* ch = w.new_change(ChangeKind::ALIVE, payload, 16u);
    if (ch == nullptr || !payload.valid()) return false;
    payload.data[0] = byte;
    payload.size = 1u;
    ch->payload.size = 1u;
    return w.add_change(static_cast<HistoryChange&&>(*ch));
}

} // namespace

static bool tc_swr_8_2_2_001(std::string& err) {
    NullTransport tx{};
    const Guid writer_guid = make_guid(0x10u, 0x11u, 0x12u, make_entity(1u, 1u, 1u, 0xC2u));
    const Guid reader_guid = make_guid(0x20u, 0x21u, 0x22u, make_entity(2u, 2u, 2u, 0xC7u));
    const Locator loc = make_udpv4_locator(0xC0A80112u, 7411u);

    StatefulWriterStorage<2, 1> w(writer_guid,
                           ReliabilityKind::RELIABLE,
                           DurabilityKind::VOLATILE,
                           tx);
    CHECK(w.matched_reader_add(reader_guid, &loc, 1u, false),
          "matched_reader_add should succeed");
    CHECK(add_one_change(w, 0xA1u), "first add_change should succeed");
    CHECK(add_one_change(w, 0xA2u), "second add_change should succeed");

    auto* proxy = w.matched_reader_lookup(reader_guid);
    CHECK(proxy != nullptr, "proxy lookup should succeed");
    CHECK(proxy->next_unsent_change(w.cache()).to_int64() == 1,
          "reader proxy should track lowest unsent SN first");

    w.remove_change(SequenceNumber::from_int64(1));
    CHECK(w.cache().get_change(SequenceNumber::from_int64(1)) == nullptr,
          "remove_change should erase SN from writer history cache");
    CHECK(proxy->next_unsent_change(w.cache()).to_int64() == 2,
          "a removed SN is no longer an unsent change of the proxy");
    return true;
}

static bool tc_swr_8_2_2_002(std::string& err) {
    CaptureTransport tx{};
    const Guid writer_guid = make_guid(0x30u, 0x31u, 0x32u, make_entity(3u, 3u, 3u, 0xC2u));
    const Guid reader_guid = make_guid(0x40u, 0x41u, 0x42u, make_entity(4u, 4u, 4u, 0xC7u));
    const Locator loc = make_udpv4_locator(0xC0A80134u, 7411u);

    StatefulWriterStorage<2, 1> w(writer_guid,
                           ReliabilityKind::RELIABLE,
                           DurabilityKind::VOLATILE,
                           tx,
                           50u,
                           10u);
    CHECK(w.matched_reader_add(reader_guid, &loc, 1u, false),
          "matched_reader_add should succeed");
    CHECK(add_one_change(w, 0xB1u), "first add_change should succeed");
    CHECK(add_one_change(w, 0xB2u), "second add_change should succeed");

    g_stateful_writer_test_now_ms = 10u;
    w.tick(10u);
    CHECK(tx.unicast_msgs.size() == 1u, "tick should emit heartbeat");

    struct HbCap {
        int calls = 0;
        ParsedHeartbeat hb{};
    } cap{};
    ParseOps ops{};
    ops.on_heartbeat = [](const ParseContext& ctx, const ParsedHeartbeat& hb) noexcept {
        auto* c = static_cast<HbCap*>(ctx.user_data);
        c->calls++;
        c->hb = hb;
    };
    CHECK(parse_message(tx.unicast_msgs[0].data(),
                        static_cast<uint32_t>(tx.unicast_msgs[0].size()),
                        GuidPrefix{},
                        ops,
                        &cap) == ParseResult::OK,
          "heartbeat packet should parse");
    CHECK(cap.calls == 1, "expected one heartbeat");
    CHECK(cap.hb.firstSN.to_int64() == 1 && cap.hb.lastSN.to_int64() == 2,
          "heartbeat range should reflect history cache min/max sequence numbers");

    CHECK(add_one_change(w, 0xB3u), "third add_change should evict oldest");
    tx.unicast_msgs.clear();
    g_stateful_writer_test_now_ms = 20u;
    w.tick(20u);
    CHECK(tx.unicast_msgs.size() == 1u, "tick should emit heartbeat after eviction");
    cap = HbCap{};
    CHECK(parse_message(tx.unicast_msgs[0].data(),
                        static_cast<uint32_t>(tx.unicast_msgs[0].size()),
                        GuidPrefix{},
                        ops,
                        &cap) == ParseResult::OK,
          "heartbeat packet should parse after eviction");
    CHECK(cap.calls == 1, "expected one heartbeat after eviction");
    CHECK(cap.hb.firstSN.to_int64() == 2 && cap.hb.lastSN.to_int64() == 3,
          "heartbeat range should shift with history cache eviction");
    return true;
}

static bool tc_swr_8_2_2_003(std::string& err) {
    CaptureTransport tx{};
    const Guid writer_guid = make_guid(0x50u, 0x51u, 0x52u, make_entity(5u, 5u, 5u, 0xC2u));
    const Guid reader_guid = make_guid(0x60u, 0x61u, 0x62u, make_entity(6u, 6u, 6u, 0xC7u));
    const Locator loc = make_udpv4_locator(0xC0A80156u, 7411u);

    StatefulWriterStorage<2, 1> w(writer_guid,
                           ReliabilityKind::RELIABLE,
                           DurabilityKind::VOLATILE,
                           tx,
                           5u,
                           100000u);
    CHECK(w.matched_reader_add(reader_guid, &loc, 1u, false),
          "matched_reader_add should succeed");

    const Octet payload[2] = {0xCAu, 0xFEu};
    CHECK(w.write_async(ChangeKind::ALIVE, payload, 2u),
          "initial reliable write should succeed");
    tx.unicast_msgs.clear();

    SequenceNumberSet nack_set{};
    nack_set.clear();
    nack_set.bitmapBase = SequenceNumber::from_int64(1);
    CHECK(nack_set.set(0u), "nack bitmap setup should succeed");

    ParsedAckNack ack{};
    ack.readerId = reader_guid.entityId;
    ack.writerId = writer_guid.entityId;
    ack.readerSNState = nack_set;
    ack.count = 1;
    ack.finalFlag = false;

    ParseContext ctx{};
    ctx.senderGuidPrefix = reader_guid.prefix;
    g_stateful_writer_test_now_ms = 100u;
    w.on_acknack(ctx, ack);

    w.tick(103u);
    CHECK(tx.unicast_msgs.empty(), "tick before nackResponseDelay should not retransmit");

    w.tick(106u);
    CHECK(tx.unicast_msgs.size() == 2u,
          "ACKNACK requested change should be retransmitted from cache after nackResponseDelay");

    struct PacketCap {
        int data_calls = 0;
        int hb_calls = 0;
    };
    int data_calls = 0;
    int hb_calls = 0;
    for (const auto& pkt : tx.unicast_msgs) {
        PacketCap cap{};
        ParseOps ops{};
        ops.on_data = [](const ParseContext& parse_ctx, const ParsedData&) noexcept {
            auto* c = static_cast<PacketCap*>(parse_ctx.user_data);
            c->data_calls++;
        };
        ops.on_heartbeat = [](const ParseContext& parse_ctx, const ParsedHeartbeat&) noexcept {
            auto* c = static_cast<PacketCap*>(parse_ctx.user_data);
            c->hb_calls++;
        };
        CHECK(parse_message(pkt.data(),
                            static_cast<uint32_t>(pkt.size()),
                            GuidPrefix{},
                            ops,
                            &cap) == ParseResult::OK,
              "retransmit packet should parse");
        data_calls += cap.data_calls;
        hb_calls += cap.hb_calls;
    }

    CHECK(data_calls == 1, "expected one DATA retransmit for ACKNACK-requested cached change");
    CHECK(hb_calls == 1, "expected one directed HEARTBEAT after retransmit");
    return true;
}

void append_stateful_writer_tests_8_2_2(std::vector<TestCase>& tests) {
    tests.push_back({"TC-SWR-8.2.2-001",
                     "§8.2.2/§8.3.1 interaction: removing a CacheChange from WriterHistoryCache removes the same sequence from matched ReaderProxy tracking.",
                     tc_swr_8_2_2_001});
    tests.push_back({"TC-SWR-8.2.2-002",
                     "§8.2.2/§8.3.1/§8.4.2.2 interaction: HEARTBEAT firstSN/lastSN reflects WriterHistoryCache min/max sequence numbers across eviction.",
                     tc_swr_8_2_2_002});
    tests.push_back({"TC-SWR-8.2.2-003",
                     "§8.2.2/§8.4.3 interaction: ACKNACK-requested sequence numbers are retransmitted from WriterHistoryCache after nackResponseDelay.",
                     tc_swr_8_2_2_003});
}

} // namespace spec_test
