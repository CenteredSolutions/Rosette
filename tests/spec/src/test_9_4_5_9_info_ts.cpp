// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

#include "spec_test_common.hpp"

namespace spec_test {

#define CHECK(COND, MSG) do { if (!(COND)) { err = (MSG); return false; } } while (0)

static bool tc_9_4_5_9_001(std::string& err) {
    std::vector<Octet> msg{};
    append_header(msg);
    append_submessage(msg, SubmessageKind::INFO_TS, static_cast<Octet>(FLAG_E | 0x02u),
                      {0x00u, 0x00u, 0x00u, 0x00u}, true, 4u);

    const EntityId reader = make_entity(1u, 2u, 3u, 0xC7u);
    const EntityId writer = make_entity(4u, 5u, 6u, 0xC2u);
    const auto data_payload = make_data_payload(true, reader, writer,
                                                SequenceNumber::from_int64(1),
                                                16u, {}, {0x11u});
    append_submessage(msg, SubmessageKind::DATA, static_cast<Octet>(FLAG_E | data_flags::D), data_payload);

    TestCapture cap{};
    ParseOps ops = make_ops_all(cap);
    const ParseResult r = parse_message(msg.data(), static_cast<uint32_t>(msg.size()), GuidPrefix{}, ops, &cap);
    CHECK(r == ParseResult::OK, "expected OK");
    CHECK(cap.data_calls == 1, "expected DATA callback");
    CHECK(cap.data_ctx.sourceTimestamp.seconds == TIME_INVALID.seconds &&
          cap.data_ctx.sourceTimestamp.fraction == TIME_INVALID.fraction,
          "expected source timestamp invalidated");
    return true;
}

static bool tc_9_4_5_9_002(std::string& err) {
    std::vector<Octet> msg{};
    append_header(msg);
    std::vector<Octet> ts{};
    write_i32(ts, 123, true);
    write_u32(ts, 456u, true);
    append_submessage(msg, SubmessageKind::INFO_TS, FLAG_E, ts);

    const EntityId reader = make_entity(1u, 2u, 3u, 0xC7u);
    const EntityId writer = make_entity(4u, 5u, 6u, 0xC2u);
    const auto data_payload = make_data_payload(true, reader, writer,
                                                SequenceNumber::from_int64(2),
                                                16u, {}, {0x22u});
    append_submessage(msg, SubmessageKind::DATA, static_cast<Octet>(FLAG_E | data_flags::D), data_payload);

    TestCapture cap{};
    ParseOps ops = make_ops_all(cap);
    const ParseResult r = parse_message(msg.data(), static_cast<uint32_t>(msg.size()), GuidPrefix{}, ops, &cap);
    CHECK(r == ParseResult::OK, "expected OK");
    CHECK(cap.data_calls == 1, "expected DATA callback");
    CHECK(cap.data_ctx.sourceTimestamp.seconds == 123 &&
          cap.data_ctx.sourceTimestamp.fraction == 456u,
          "expected INFO_TS timestamp propagated");
    return true;
}

void append_tests_9_4_5_9(std::vector<TestCase>& tests) {
    tests.push_back({"TC-9.4.5.9-001", "INFO_TS invalidate flag clears source timestamp context.", tc_9_4_5_9_001});
    tests.push_back({"TC-9.4.5.9-002", "INFO_TS timestamp value propagates to following submessages.", tc_9_4_5_9_002});
}

} // namespace spec_test
