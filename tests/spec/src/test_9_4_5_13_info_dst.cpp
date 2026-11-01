// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

#include "spec_test_common.hpp"

namespace spec_test {

#define CHECK(COND, MSG) do { if (!(COND)) { err = (MSG); return false; } } while (0)

static bool tc_9_4_5_13_001(std::string& err) {
    std::vector<Octet> msg{};
    append_header(msg);
    GuidPrefix dst{};
    dst.value[0] = 0xAAu;
    dst.value[1] = 0xBBu;
    std::vector<Octet> info_dst{};
    info_dst.insert(info_dst.end(), dst.value, dst.value + 12u);
    append_submessage(msg, SubmessageKind::INFO_DST, FLAG_E, info_dst);

    const EntityId reader = make_entity(1u, 2u, 3u, 0xC7u);
    const EntityId writer = make_entity(4u, 5u, 6u, 0xC2u);
    const auto data_payload = make_data_payload(true, reader, writer,
                                                SequenceNumber::from_int64(3),
                                                16u, {}, {0x10u});
    append_submessage(msg, SubmessageKind::DATA, static_cast<Octet>(FLAG_E | data_flags::D), data_payload);

    TestCapture cap{};
    ParseOps ops = make_ops_all(cap);
    const ParseResult r = parse_message(msg.data(), static_cast<uint32_t>(msg.size()), GuidPrefix{}, ops, &cap);
    CHECK(r == ParseResult::OK, "expected OK");
    CHECK(cap.data_calls == 1, "expected DATA callback");
    CHECK(cap.data_ctx.destGuidPrefix.value[0] == 0xAAu &&
          cap.data_ctx.destGuidPrefix.value[1] == 0xBBu,
          "expected INFO_DST propagated to ParseContext");
    return true;
}

void append_tests_9_4_5_13(std::vector<TestCase>& tests) {
    tests.push_back({"TC-9.4.5.13-001", "INFO_DST updates destination GuidPrefix for following submessages.", tc_9_4_5_13_001});
}

} // namespace spec_test

