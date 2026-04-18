#include "spec_test_common.hpp"

namespace spec_test {

#define CHECK(COND, MSG) do { if (!(COND)) { err = (MSG); return false; } } while (0)

static bool tc_9_4_5_7_001(std::string& err) {
    std::vector<Octet> msg{};
    append_header(msg);
    std::vector<Octet> ack{};
    append_entity_id(ack, make_entity(1u, 2u, 3u, 0xC7u));
    append_entity_id(ack, make_entity(4u, 5u, 6u, 0xC2u));
    write_i32(ack, 0, true);
    write_u32(ack, 1, true);
    write_u32(ack, 257u, true); // invalid > 256
    write_i32(ack, 1, true);
    append_submessage(msg, SubmessageKind::ACKNACK, FLAG_E, ack);

    ParseOps ops{};
    const ParseResult r = parse_message(msg.data(), static_cast<uint32_t>(msg.size()), GuidPrefix{}, ops, nullptr);
    CHECK(r == ParseResult::MALFORMED, "expected MALFORMED for invalid ACKNACK SN set size");
    return true;
}

static bool tc_9_4_5_7_002(std::string& err) {
    std::vector<Octet> msg{};
    append_header(msg);
    std::vector<Octet> ack{};
    append_entity_id(ack, make_entity(1u, 2u, 3u, 0xC7u));
    append_entity_id(ack, make_entity(4u, 5u, 6u, 0xC2u));
    write_i32(ack, 0, true);
    write_u32(ack, 1, true);
    write_u32(ack, 0u, true);
    write_i32(ack, 2, true);
    append_submessage(msg, SubmessageKind::ACKNACK, static_cast<Octet>(FLAG_E | acknack_flags::F), ack);

    struct AckCapture {
        ParsedAckNack got{};
        int calls = 0;
    } cap{};

    ParseOps ops{};
    ops.on_acknack = [](const ParseContext& ctx, const ParsedAckNack& a) noexcept {
        auto* c = static_cast<AckCapture*>(ctx.user_data);
        c->got = a;
        c->calls++;
    };

    const ParseResult r = parse_message(msg.data(), static_cast<uint32_t>(msg.size()), GuidPrefix{}, ops, &cap);
    CHECK(r == ParseResult::OK, "expected OK");
    CHECK(cap.calls == 1, "expected ACKNACK callback");
    CHECK(cap.got.finalFlag, "expected final flag decoded");
    return true;
}

void append_tests_9_4_5_7(std::vector<TestCase>& tests) {
    tests.push_back({"TC-9.4.5.7-001", "Invalid ACKNACK SequenceNumberSet bounds are malformed.", tc_9_4_5_7_001});
    tests.push_back({"TC-9.4.5.7-002", "ACKNACK final flag is decoded from submessage flags.", tc_9_4_5_7_002});
}

} // namespace spec_test
