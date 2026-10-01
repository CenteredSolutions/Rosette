#include "spec_test_common.hpp"

#include "rosette/rtps/message_builder.hpp"

namespace spec_test {

#define CHECK(COND, MSG) do { if (!(COND)) { err = (MSG); return false; } } while (0)

struct CtrlCapture {
    int hb_calls = 0;
    int ack_calls = 0;
    int gap_calls = 0;
    int frag_calls = 0;
    ParsedHeartbeat hb{};
    ParsedAckNack ack{};
    ParsedGap gap{};
    ParsedDataFrag frag{};
};

static void on_hb(const ParseContext& ctx, const ParsedHeartbeat& hb) noexcept {
    auto* c = static_cast<CtrlCapture*>(ctx.user_data);
    c->hb_calls++;
    c->hb = hb;
}
static void on_ack(const ParseContext& ctx, const ParsedAckNack& a) noexcept {
    auto* c = static_cast<CtrlCapture*>(ctx.user_data);
    c->ack_calls++;
    c->ack = a;
}
static void on_gap(const ParseContext& ctx, const ParsedGap& g) noexcept {
    auto* c = static_cast<CtrlCapture*>(ctx.user_data);
    c->gap_calls++;
    c->gap = g;
}
static void on_frag(const ParseContext& ctx, const ParsedDataFrag& f) noexcept {
    auto* c = static_cast<CtrlCapture*>(ctx.user_data);
    c->frag_calls++;
    c->frag = f;
}

static bool parse_ctrl(const BufRef& msg, CtrlCapture& cap) {
    ParseOps ops{};
    ops.on_heartbeat = on_hb;
    ops.on_acknack = on_ack;
    ops.on_gap = on_gap;
    ops.on_data_frag = on_frag;
    return parse_message(msg.data, msg.size, GuidPrefix{}, ops, &cap) == ParseResult::OK;
}

static bool tc_bld_9_4_5_5_001(std::string& err) {
    MessageBuilder b{};
    CHECK(b.begin(GuidPrefix{}, 256u), "begin failed");
    CHECK(b.append_heartbeat(make_entity(1u,2u,3u,0xC7u),
                             make_entity(4u,5u,6u,0xC2u),
                             SequenceNumber::from_int64(10),
                             SequenceNumber::from_int64(20),
                             3,
                             true,
                             true),
          "append_heartbeat failed");
    BufRef out = b.finish();
    CtrlCapture cap{};
    CHECK(parse_ctrl(out, cap), "parse failed");
    CHECK(cap.hb_calls == 1, "expected heartbeat callback");
    CHECK(cap.hb.finalFlag && cap.hb.livelinessFlag, "heartbeat flags mismatch");
    CHECK(cap.hb.firstSN.to_int64() == 10 && cap.hb.lastSN.to_int64() == 20, "heartbeat SN mismatch");
    out.reset();
    return true;
}

static bool tc_bld_9_4_5_7_001(std::string& err) {
    MessageBuilder b{};
    CHECK(b.begin(GuidPrefix{}, 256u), "begin failed");
    SequenceNumberSet sn{};
    sn.clear();
    sn.bitmapBase = SequenceNumber::from_int64(1);
    sn.numBits = 0;
    CHECK(b.append_acknack(make_entity(1u,2u,3u,0xC7u),
                           make_entity(4u,5u,6u,0xC2u),
                           sn, 7, true),
          "append_acknack failed");
    BufRef out = b.finish();
    CtrlCapture cap{};
    CHECK(parse_ctrl(out, cap), "parse failed");
    CHECK(cap.ack_calls == 1, "expected acknack callback");
    CHECK(cap.ack.finalFlag, "expected ACKNACK final flag");
    out.reset();
    return true;
}

static bool tc_bld_9_4_5_11_001(std::string& err) {
    MessageBuilder b{};
    CHECK(b.begin(GuidPrefix{}, 256u), "begin failed");
    SequenceNumberSet gap_list{};
    gap_list.clear();
    gap_list.bitmapBase = SequenceNumber::from_int64(11);
    gap_list.numBits = 0;
    CHECK(b.append_gap(make_entity(1u,2u,3u,0xC7u),
                       make_entity(4u,5u,6u,0xC2u),
                       SequenceNumber::from_int64(10),
                       gap_list),
          "append_gap failed");
    BufRef out = b.finish();
    CtrlCapture cap{};
    CHECK(parse_ctrl(out, cap), "parse failed");
    CHECK(cap.gap_calls == 1, "expected gap callback");
    CHECK(cap.gap.gapStart.to_int64() == 10, "gap start mismatch");
    out.reset();
    return true;
}

static bool tc_bld_9_4_5_4_001(std::string& err) {
    MessageBuilder b{};
    CHECK(b.begin(GuidPrefix{}, 256u), "begin failed");
    const Octet frag[2] = {0xABu, 0xCDu};
    CHECK(b.append_data_frag(make_entity(1u,2u,3u,0xC7u),
                             make_entity(4u,5u,6u,0xC2u),
                             SequenceNumber::from_int64(3),
                             1u, 1u, 64u, 64u, frag, 2u),
          "append_data_frag failed");
    BufRef out = b.finish();
    CtrlCapture cap{};
    CHECK(parse_ctrl(out, cap), "parse failed");
    CHECK(cap.frag_calls == 1, "expected data_frag callback");
    CHECK(cap.frag.fragmentPayloadSize == 2u, "fragment payload size mismatch");
    out.reset();
    return true;
}

static bool tc_bld_9_4_5_1_001(std::string& err) {
    MessageBuilder b{};
    CHECK(b.begin(GuidPrefix{}, 128u), "begin with constrained buffer failed");
    BufRef payload = buf_alloc(256u);
    CHECK(payload.valid(), "payload alloc failed");
    for (uint32_t i = 0; i < 200u; ++i) payload.data[i] = static_cast<Octet>(i & 0xFFu);
    payload.size = 200u;
    const bool ok = b.append_data(make_entity(1u,2u,3u,0xC7u),
                                  make_entity(4u,5u,6u,0xC2u),
                                  SequenceNumber::from_int64(1),
                                  payload);
    CHECK(!ok, "append_data should fail when submessage exceeds available capacity");
    b.abort();
    payload.reset();
    return true;
}

void append_builder_tests_9_4_5_ctrl(std::vector<TestCase>& tests) {
    tests.push_back({"TC-BLD-9.4.5.5-001",
                     "append_heartbeat() serializes HEARTBEAT fields and F/L flags.",
                     tc_bld_9_4_5_5_001});
    tests.push_back({"TC-BLD-9.4.5.7-001",
                     "append_acknack() serializes ACKNACK including final flag and SN state.",
                     tc_bld_9_4_5_7_001});
    tests.push_back({"TC-BLD-9.4.5.11-001",
                     "append_gap() serializes GAP reader/writer IDs and sequence set.",
                     tc_bld_9_4_5_11_001});
    tests.push_back({"TC-BLD-9.4.5.4-001",
                     "append_data_frag() serializes DATA_FRAG fixed fields and fragment payload.",
                     tc_bld_9_4_5_4_001});
    tests.push_back({"TC-BLD-9.4.5.1-001",
                     "append operations fail safely when message buffer capacity is insufficient.",
                     tc_bld_9_4_5_1_001});
}

} // namespace spec_test
