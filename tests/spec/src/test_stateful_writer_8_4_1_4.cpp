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
    std::vector<Locator> unicast_dests{};

    bool send_unicast(const BufRef& buf, const Locator& dest) noexcept override {
        unicast_msgs.emplace_back(buf.data, buf.data + buf.size);
        unicast_dests.push_back(dest);
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

} // namespace

static bool tc_swr_8_4_1_4_001(std::string& err) {
    NullTransport tx{};
    const Guid writer_guid = make_guid(0x10u, 0x20u, 0x30u, make_entity(1u, 2u, 3u, 0xC2u));
    StatefulWriterStorage<2, 1> w(writer_guid,
                           ReliabilityKind::BEST_EFFORT,
                           DurabilityKind::VOLATILE,
                           tx);

    for (int i = 0; i < 3; ++i) {
        BufRef payload{};
        CacheChange* ch = w.new_change(ChangeKind::ALIVE, payload, 32u);
        CHECK(ch != nullptr, "new_change should allocate staging change");
        CHECK(payload.valid(), "new_change should return valid payload");
        payload.data[0] = static_cast<Octet>(0xA0u + i);
        payload.size = 1u;
        ch->payload.size = 1u;
        CHECK(w.add_change(static_cast<CacheChange&&>(*ch)), "add_change should succeed");
    }

    const auto min_sn = w.cache().get_seq_num_min().to_int64();
    const auto max_sn = w.cache().get_seq_num_max().to_int64();
    CHECK(min_sn == 2, "KEEP_LAST(2) should evict oldest sequence number");
    CHECK(max_sn == 3, "latest sequence number should be retained");
    CHECK(w.cache().get_change(SequenceNumber::from_int64(1)) == nullptr,
          "evicted SN should not remain in writer cache");
    return true;
}

static bool tc_swr_8_4_1_4_002(std::string& err) {
    CaptureTransport tx{};
    const Guid writer_guid = make_guid(0x40u, 0x41u, 0x42u, make_entity(4u, 5u, 6u, 0xC2u));
    const Guid reader_guid = make_guid(0x70u, 0x71u, 0x72u, make_entity(7u, 8u, 9u, 0xC7u));
    const Locator loc = make_udpv4_locator(0xC0A80101u, 7411u);
    StatefulWriterStorage<1, 2> w(writer_guid,
                           ReliabilityKind::RELIABLE,
                           DurabilityKind::VOLATILE,
                           tx);
    CHECK(w.matched_reader_add(reader_guid, &loc, 1u, false),
          "matched_reader_add should succeed for first reader");

    const Octet data[3] = {0xAAu, 0xBBu, 0xCCu};
    CHECK(w.write_async(ChangeKind::ALIVE, data, 3u),
          "write_async should succeed and dispatch data");

    CHECK(tx.unicast_msgs.size() == 1u, "write_async should emit one DATA packet");
    CHECK(tx.unicast_dests.size() == 1u && tx.unicast_dests[0] == loc,
          "writer should send to matched reader best locator");

    struct DataCap {
        int data_calls = 0;
        ParsedData data{};
        std::vector<Octet> payload{};
    } cap{};
    ParseOps ops{};
    ops.on_data = [](const ParseContext& ctx, const ParsedData& d) noexcept {
        auto* c = static_cast<DataCap*>(ctx.user_data);
        c->data_calls++;
        c->data = d;
        c->payload.assign(d.serializedPayload, d.serializedPayload + d.serializedPayloadSize);
    };

    CHECK(parse_message(tx.unicast_msgs[0].data(),
                        static_cast<uint32_t>(tx.unicast_msgs[0].size()),
                        GuidPrefix{},
                        ops,
                        &cap) == ParseResult::OK,
          "dispatched packet should parse as RTPS DATA");
    CHECK(cap.data_calls == 1, "expected exactly one DATA submessage");
    CHECK(cap.data.writerSN.to_int64() == 1, "first write should carry SN=1");
    CHECK(cap.data.readerId == reader_guid.entityId, "DATA readerId should target matched reader");
    CHECK(cap.payload.size() >= 7u, "serialized payload should include encapsulation + user bytes");
    CHECK(cap.payload[4] == 0xAAu && cap.payload[5] == 0xBBu && cap.payload[6] == 0xCCu,
          "DATA payload should include user bytes after encapsulation header");
    return true;
}

static bool tc_swr_8_4_1_4_003(std::string& err) {
    CaptureTransport tx{};
    const Guid writer_guid = make_guid(0x80u, 0x81u, 0x82u, make_entity(1u, 1u, 1u, 0xC2u));
    const Guid reader_guid = make_guid(0x90u, 0x91u, 0x92u, make_entity(2u, 2u, 2u, 0xC7u));
    const Locator loc = make_udpv4_locator(0xC0A80122u, 7411u);
    StatefulWriterStorage<2, 1> w(writer_guid,
                           ReliabilityKind::BEST_EFFORT,
                           DurabilityKind::TRANSIENT_LOCAL,
                           tx);

    BufRef payload{};
    CacheChange* ch = w.new_change(ChangeKind::ALIVE, payload, 32u);
    CHECK(ch != nullptr, "new_change should succeed");
    payload.data[0] = 0x11u;
    payload.size = 1u;
    ch->payload.size = 1u;
    CHECK(w.add_change(static_cast<CacheChange&&>(*ch)), "add_change should succeed");
    CHECK(tx.unicast_msgs.empty(), "no reader is matched yet, so nothing is sent");

    CHECK(w.matched_reader_add(reader_guid, &loc, 1u, false),
          "matched_reader_add should succeed");
    auto* proxy = w.matched_reader_lookup(reader_guid);
    CHECK(proxy != nullptr, "matched_reader_lookup should find added proxy");
    CHECK(proxy->next_unsent_change() == nullptr,
          "TRANSIENT_LOCAL replay should be sent at match time, leaving nothing unsent");
    CHECK(tx.unicast_msgs.size() == 1u && tx.unicast_dests[0] == loc,
          "the cached change should be replayed to the new reader's locator");
    return true;
}

void append_stateful_writer_tests_8_4_1_4(std::vector<TestCase>& tests) {
    tests.push_back({"TC-SWR-8.4.1.4-001",
                     "§8.4.1.4/§8.3.3: add_change assigns monotonically increasing sequence numbers and respects KEEP_LAST cache eviction.",
                     tc_swr_8_4_1_4_001});
    tests.push_back({"TC-SWR-8.4.1.4-002",
                     "§8.4.1.4: matched_reader_add plus write_async sends DATA to the matched reader locator with writer-assigned SN.",
                     tc_swr_8_4_1_4_002});
    tests.push_back({"TC-SWR-8.4.1.4-003",
                     "§8.7.2.2: a TRANSIENT_LOCAL writer replays its cached changes to a newly matched reader immediately.",
                     tc_swr_8_4_1_4_003});
}

} // namespace spec_test
