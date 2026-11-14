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

/// DATA sequence numbers and HEARTBEATs found in one datagram.
struct Submessages {
    std::vector<int64_t> data_sns{};
    std::vector<ParsedHeartbeat> heartbeats{};
};

/// @brief Parses @p msg into @p out; false when it does not parse.
bool parse_submessages(const std::vector<Octet>& msg, Submessages& out) {
    ParseOps ops{};
    ops.on_data = [](const ParseContext& ctx, const ParsedData& d) noexcept {
        static_cast<Submessages*>(ctx.user_data)->data_sns.push_back(d.writerSN.to_int64());
    };
    ops.on_heartbeat = [](const ParseContext& ctx, const ParsedHeartbeat& hb) noexcept {
        static_cast<Submessages*>(ctx.user_data)->heartbeats.push_back(hb);
    };
    return parse_message(msg.data(), static_cast<uint32_t>(msg.size()), GuidPrefix{}, ops, &out)
           == ParseResult::OK;
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

static bool tc_swr_8_4_1_4_001(std::string& err) {
    NullTransport tx{};
    const Guid writer_guid = make_guid(0x10u, 0x20u, 0x30u, make_entity(1u, 2u, 3u, 0xC2u));
    StatefulWriterStorage<2, 1> w(writer_guid,
                           ReliabilityKind::BEST_EFFORT,
                           DurabilityKind::VOLATILE,
                           tx);

    for (uint32_t i = 0; i < 3u; ++i) {
        BufRef payload{};
        HistoryChange ch = w.new_change(ChangeKind::ALIVE, payload, 32u);
        CHECK(ch.payload.valid(), "new_change should allocate the change's payload");
        CHECK(payload.valid(), "new_change should return valid payload");
        payload.data[0] = static_cast<Octet>(0xA0u + i);
        payload.size = 1u;
        ch.payload.size = 1u;
        CHECK(w.add_change(static_cast<HistoryChange&&>(ch)), "add_change should succeed");
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
    HistoryChange ch = w.new_change(ChangeKind::ALIVE, payload, 32u);
    CHECK(ch.payload.valid(), "new_change should succeed");
    payload.data[0] = 0x11u;
    payload.size = 1u;
    ch.payload.size = 1u;
    CHECK(w.add_change(static_cast<HistoryChange&&>(ch)), "add_change should succeed");
    CHECK(tx.unicast_msgs.empty(), "no reader is matched yet, so nothing is sent");

    CHECK(w.matched_reader_add(reader_guid, &loc, 1u, false),
          "matched_reader_add should succeed");
    auto* proxy = w.matched_reader_lookup(reader_guid);
    CHECK(proxy != nullptr, "matched_reader_lookup should find added proxy");
    CHECK(tx.unicast_msgs.empty() && proxy->replaying && w.replay_pending(),
          "matching sends nothing itself: the replay is left to tick()");
    CHECK(w.ms_until_next_timer(0u) == 0u, "and the writer asks to be ticked at once");

    w.tick(0u);
    CHECK(proxy->next_unsent_change(w.cache()) == SEQUENCENUMBER_UNKNOWN,
          "the next tick() sends the replay, leaving nothing unsent");
    CHECK(!proxy->replaying && !w.replay_pending(), "and ends it");
    CHECK(tx.unicast_msgs.size() == 1u && tx.unicast_dests[0] == loc,
          "the cached change should be replayed to the new reader's locator");

    Submessages sent{};
    CHECK(parse_submessages(tx.unicast_msgs[0], sent), "the replay should parse");
    CHECK(sent.data_sns.size() == 1u && sent.heartbeats.empty(),
          "a BEST_EFFORT writer replays the DATA without a HEARTBEAT");
    return true;
}

static bool tc_swr_8_4_1_4_008(std::string& err) {
    CaptureTransport tx{};
    const Guid writer_guid = make_guid(0x80u, 0x81u, 0x82u, make_entity(1u, 1u, 1u, 0xC2u));
    const Guid reader_guid = make_guid(0x90u, 0x91u, 0x92u, make_entity(2u, 2u, 2u, 0xC7u));
    const Locator loc = make_udpv4_locator(0xC0A80122u, 7411u);
    StatefulWriterStorage<4, 1> w(writer_guid, ReliabilityKind::RELIABLE,
                                  DurabilityKind::TRANSIENT_LOCAL, tx);

    const Octet data[4] = {0x11u, 0x22u, 0x33u, 0x44u};
    CHECK(w.write_async(ChangeKind::ALIVE, data, 4u), "first write should succeed");
    CHECK(w.write_async(ChangeKind::ALIVE, data, 4u), "second write should succeed");
    CHECK(tx.unicast_msgs.empty(), "no reader is matched yet, so nothing is sent");

    CHECK(w.matched_reader_add(reader_guid, &loc, 1u, false), "matched_reader_add should succeed");
    finish_replay(w, 0u);

    std::vector<int64_t> data_sns{};
    std::vector<ParsedHeartbeat> heartbeats{};
    std::size_t heartbeat_msg = tx.unicast_msgs.size();
    for (std::size_t i = 0; i < tx.unicast_msgs.size(); ++i) {
        Submessages sent{};
        CHECK(parse_submessages(tx.unicast_msgs[i], sent), "every replay datagram should parse");
        data_sns.insert(data_sns.end(), sent.data_sns.begin(), sent.data_sns.end());
        if (!sent.heartbeats.empty()) {
            heartbeat_msg = i;
        }

        heartbeats.insert(heartbeats.end(), sent.heartbeats.begin(), sent.heartbeats.end());
    }

    CHECK(data_sns == (std::vector<int64_t>{1, 2}), "both cached changes are replayed in order");
    CHECK(heartbeats.size() == 1u, "the replay carries exactly one HEARTBEAT");
    CHECK(heartbeat_msg == tx.unicast_msgs.size() - 1u,
          "the HEARTBEAT goes with the newest change, after every DATA it advertises");

    const ParsedHeartbeat& hb = heartbeats[0];
    CHECK(hb.readerId == reader_guid.entityId && hb.writerId == writer_guid.entityId,
          "the HEARTBEAT names the new reader and this writer");
    CHECK(hb.firstSN.to_int64() == 1 && hb.lastSN.to_int64() == 2,
          "the HEARTBEAT spans the replayed cache, SN 1 to 2");
    CHECK(!hb.finalFlag, "the HEARTBEAT is non-final, so the reader acknowledges at once");
    return true;
}

// TC-SWR-8.4.1.4-009: writers sharing a ReaderProxy pool bind by its
// admission rule: a busy writer borrows what a quiet one is not promised, the
// quiet one still gets its floor, an unmatch frees a proxy for either, a
// write reaches exactly the bound readers, and a destroyed writer returns
// every proxy.
bool tc_swr_8_4_1_4_009(std::string& err) {
    CaptureTransport tx{};
    ProxyPoolStorage<ReaderProxy, 3> pool;
    CHECK(pool.reserve(2u), "two floors of 1 must fit a pool of 3");

    const Guid busy_guid = make_guid(0x01u, 0x02u, 0x03u, make_entity(0u, 0u, 1u, 0x03u));
    const Guid quiet_guid = make_guid(0x01u, 0x02u, 0x03u, make_entity(0u, 0u, 2u, 0x03u));
    const Locator loc = make_udpv4_locator(0xC0A80150u, 7411u);
    Guid readers[4]{};
    for (Octet i = 0u; i < 4u; ++i) {
        readers[i] = make_guid(0x40u, 0x41u, i, make_entity(9u, 9u, i, 0xC7u));
    }

    SharedPoolStatefulWriter<1> quiet(quiet_guid, ReliabilityKind::BEST_EFFORT,
                                      DurabilityKind::VOLATILE, tx, pool, 1u, 3u);
    {
        SharedPoolStatefulWriter<1> busy(busy_guid, ReliabilityKind::BEST_EFFORT,
                                         DurabilityKind::VOLATILE, tx, pool, 1u, 3u);
        CHECK(busy.matched_reader_add(readers[0], &loc, 1u), "busy binds its floor");
        CHECK(busy.matched_reader_add(readers[1], &loc, 1u), "busy borrows the unpromised proxy");
        CHECK(!busy.matched_reader_add(readers[2], &loc, 1u),
              "busy must not take the proxy promised to quiet's floor");
        CHECK(busy.matched_reader_count() == 2u, "a refused match is not counted");
        CHECK(quiet.matched_reader_add(readers[3], &loc, 1u), "quiet binds its floor");

        busy.matched_reader_remove(readers[0]);
        CHECK(busy.matched_reader_count() == 1u && pool.free_count() == 1u,
              "an unmatch returns the proxy to the pool");
        CHECK(busy.matched_reader_add(readers[2], &loc, 1u), "busy rebinds within its floor");

        const Octet data[4] = {1u, 2u, 3u, 4u};
        tx.unicast_msgs.clear();
        CHECK(busy.write_async(ChangeKind::ALIVE, data, 4u), "busy writes");
        CHECK(tx.unicast_msgs.size() == 2u, "the DATA goes to exactly busy's two bound readers");
        CHECK(busy.matched_reader_lookup(readers[1]) != nullptr
                  && busy.matched_reader_lookup(readers[2]) != nullptr
                  && busy.matched_reader_lookup(readers[3]) == nullptr,
              "busy looks up its own readers and not quiet's");
    }

    CHECK(pool.free_count() == 2u && pool.reserved() == 1u,
          "a destroyed writer returns every proxy, and its floor stays promised to its slot");
    CHECK(quiet.matched_reader_add(readers[0], &loc, 1u), "quiet borrows what busy returned");
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
                     "§8.7.2.2: a TRANSIENT_LOCAL writer replays its cached changes to a newly "
                     "matched reader from the next tick(), not from matched_reader_add().",
                     tc_swr_8_4_1_4_003});
    tests.push_back(
        {"TC-SWR-8.4.1.4-008",
         "§8.7.2.2/§8.4.15.4: a RELIABLE TRANSIENT_LOCAL writer replays its cache to a newly "
         "matched reader with a non-final HEARTBEAT on the newest change spanning the cache.",
         tc_swr_8_4_1_4_008});
    tests.push_back(
        {"TC-SWR-8.4.1.4-009",
         "§8.4.1.1/§8.4.1.4: writers sharing a ReaderProxy pool bind by its admission rule, an "
         "unmatch or a destroyed writer returns proxies to it, and a write reaches exactly the "
         "bound readers.",
         tc_swr_8_4_1_4_009});
}

} // namespace spec_test
