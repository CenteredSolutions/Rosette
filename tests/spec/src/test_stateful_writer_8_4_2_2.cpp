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

Guid make_guid(Octet p0, Octet p1, Octet p2, const EntityId& eid) {
    Guid g{};
    g.prefix.value[0] = p0;
    g.prefix.value[1] = p1;
    g.prefix.value[2] = p2;
    g.entityId = eid;
    return g;
}

} // namespace

static bool tc_swr_8_4_2_2_001(std::string& err) {
    CaptureTransport tx{};
    const Guid writer_guid = make_guid(0xA1u, 0xA2u, 0xA3u, make_entity(1u, 2u, 3u, 0xC2u));
    const Guid reader_guid = make_guid(0xB1u, 0xB2u, 0xB3u, make_entity(4u, 5u, 6u, 0xC7u));
    const Locator loc = make_udpv4_locator(0xC0A80133u, 7411u);

    StatefulWriter<1, 1> w(writer_guid,
                           ReliabilityKind::RELIABLE,
                           DurabilityKind::VOLATILE,
                           tx,
                           50u,
                           10u);
    CHECK(w.matched_reader_add(reader_guid, &loc, 1u, false),
          "matched_reader_add should succeed");

    g_stateful_writer_test_now_ms = 0u;
    w.tick(0u);
    CHECK(tx.unicast_msgs.empty(), "tick before heartbeat period should not emit HEARTBEAT");

    g_stateful_writer_test_now_ms = 10u;
    w.tick(10u);
    CHECK(tx.unicast_msgs.size() == 1u, "tick at heartbeat period should emit one HEARTBEAT");

    struct HbCap {
        int hb_calls = 0;
        ParsedHeartbeat hb{};
    } cap{};

    ParseOps ops{};
    ops.on_heartbeat = [](const ParseContext& ctx, const ParsedHeartbeat& hb) noexcept {
        auto* c = static_cast<HbCap*>(ctx.user_data);
        c->hb_calls++;
        c->hb = hb;
    };

    CHECK(parse_message(tx.unicast_msgs[0].data(),
                        static_cast<uint32_t>(tx.unicast_msgs[0].size()),
                        GuidPrefix{},
                        ops,
                        &cap) == ParseResult::OK,
          "HEARTBEAT packet should parse");
    CHECK(cap.hb_calls == 1, "expected one HEARTBEAT callback");
    CHECK(!cap.hb.finalFlag, "periodic heartbeat should set finalFlag=false");
    CHECK(cap.hb.readerId == entity_id::UNKNOWN, "periodic heartbeat should be broadcast to UNKNOWN readerId");
    return true;
}

void append_stateful_writer_tests_8_4_2_2(std::vector<TestCase>& tests) {
    tests.push_back({"TC-SWR-8.4.2.2-001",
                     "§8.4.2.2: tick() emits periodic HEARTBEAT over heartbeatPeriod with FinalFlag cleared for broadcast.",
                     tc_swr_8_4_2_2_001});
}

} // namespace spec_test
