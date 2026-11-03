// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

#include "spec_test_common.hpp"

namespace spec_test {

#define CHECK(COND, MSG) do { if (!(COND)) { err = (MSG); return false; } } while (0)

static bool tc_9_4_2_001(std::string& err) {
    const Octet buf[19] = {};
    ParseOps ops{};
    const ParseResult r = parse_message(buf, sizeof(buf), GuidPrefix{}, ops, nullptr);
    CHECK(r == ParseResult::BAD_HEADER, "expected BAD_HEADER for short message");
    return true;
}

static bool tc_9_4_2_002(std::string& err) {
    std::vector<Octet> msg{};
    append_header(msg);
    msg[0] = 'X';
    ParseOps ops{};
    const ParseResult r = parse_message(msg.data(), static_cast<uint32_t>(msg.size()), GuidPrefix{}, ops, nullptr);
    CHECK(r == ParseResult::BAD_HEADER, "expected BAD_HEADER for invalid magic");
    return true;
}

static bool tc_9_4_2_003(std::string& err) {
    std::vector<Octet> msg{};
    append_header(msg);
    msg[4] = 3u;
    ParseOps ops{};
    const ParseResult r = parse_message(msg.data(), static_cast<uint32_t>(msg.size()), GuidPrefix{}, ops, nullptr);
    CHECK(r == ParseResult::VERSION_MISMATCH, "expected VERSION_MISMATCH for major version");
    return true;
}

static bool tc_9_4_2_004(std::string& err) {
    std::vector<Octet> msg{};
    append_header(msg);

    const EntityId reader = make_entity(0x10u, 0x11u, 0x12u, 0xC7u);
    const EntityId writer = make_entity(0x20u, 0x21u, 0x22u, 0xC2u);
    const auto data_payload = make_data_payload(true, reader, writer,
                                                SequenceNumber::from_int64(1),
                                                16u, {}, {0x55u});
    append_submessage(msg, SubmessageKind::DATA, static_cast<Octet>(FLAG_E | data_flags::D), data_payload);

    TestCapture cap{};
    ParseOps ops = make_ops_all(cap);
    const ParseResult r = parse_message(msg.data(), static_cast<uint32_t>(msg.size()), GuidPrefix{}, ops, &cap);
    CHECK(r == ParseResult::OK, "expected OK");
    CHECK(cap.data_calls == 1, "expected DATA callback");
    CHECK(cap.data_ctx.senderGuidPrefix.value[0] == 0xA1u &&
          cap.data_ctx.senderGuidPrefix.value[1] == 0xB2u,
          "expected sender guid prefix from RTPS header");
    return true;
}

void append_tests_9_4_2(std::vector<TestCase>& tests) {
    tests.push_back({"TC-9.4.2-001", "RTPS fixed header length is required; shorter messages are rejected.", tc_9_4_2_001});
    tests.push_back({"TC-9.4.2-002", "RTPS protocol magic bytes must identify the message.", tc_9_4_2_002});
    tests.push_back({"TC-9.4.2-003", "Unsupported major protocol versions are rejected.", tc_9_4_2_003});
    tests.push_back({"TC-9.4.2-004", "Header sender GUID prefix is propagated into ParseContext.", tc_9_4_2_004});
}

} // namespace spec_test
