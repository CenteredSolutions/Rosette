// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

#include "spec_test_common.hpp"

namespace spec_test {

#define CHECK(COND, MSG) do { if (!(COND)) { err = (MSG); return false; } } while (0)

static bool tc_9_4_5_1_001(std::string& err) {
    std::vector<Octet> msg{};
    append_header(msg);
    append_submessage(msg, static_cast<SubmessageKind>(0x7Fu), FLAG_E, {0x11u, 0x22u, 0x33u});

    TestCapture cap{};
    ParseOps ops = make_ops_all(cap);
    const ParseResult r = parse_message(msg.data(), static_cast<uint32_t>(msg.size()), GuidPrefix{}, ops, &cap);
    CHECK(r == ParseResult::OK, "expected OK");
    CHECK(cap.unknown_calls == 1, "expected one unknown callback");
    CHECK(cap.unknown_raw_len == 3u, "expected unknown raw length 3");
    return true;
}

static bool tc_9_4_5_1_002(std::string& err) {
    std::vector<Octet> msg{};
    append_header(msg);
    const EntityId reader = make_entity(1u, 2u, 3u, 0xC7u);
    const EntityId writer = make_entity(4u, 5u, 6u, 0xC2u);
    const auto data_payload = make_data_payload(true, reader, writer,
                                                SequenceNumber::from_int64(7),
                                                16u, {}, {0xDEu, 0xADu});
    append_submessage(msg, SubmessageKind::DATA, static_cast<Octet>(FLAG_E | data_flags::D),
                      data_payload, true, 0u);

    TestCapture cap{};
    ParseOps ops = make_ops_all(cap);
    const ParseResult r = parse_message(msg.data(), static_cast<uint32_t>(msg.size()), GuidPrefix{}, ops, &cap);
    CHECK(r == ParseResult::OK, "expected OK");
    CHECK(cap.data_calls == 1, "expected DATA callback");
    CHECK(cap.data_payload.size() == 2u, "expected payload size 2");
    return true;
}

static bool tc_9_4_5_1_003(std::string& err) {
    std::vector<Octet> msg{};
    append_header(msg);
    append_submessage(msg, SubmessageKind::PAD, FLAG_E, {0x00u, 0x01u}, true, 100u);

    ParseOps ops{};
    const ParseResult r = parse_message(msg.data(), static_cast<uint32_t>(msg.size()), GuidPrefix{}, ops, nullptr);
    CHECK(r == ParseResult::MALFORMED, "expected MALFORMED for extent overflow");
    return true;
}

static bool tc_9_4_5_1_004(std::string& err) {
    std::vector<Octet> msg{};
    append_header(msg);
    append_submessage(msg, SubmessageKind::PAD, FLAG_E, {0x00u});
    msg.push_back(0x00u);
    msg.push_back(0x00u);
    msg.push_back(0x00u);

    const EntityId reader = make_entity(1u, 2u, 3u, 0xC7u);
    const EntityId writer = make_entity(4u, 5u, 6u, 0xC2u);
    const auto data_payload = make_data_payload(true, reader, writer,
                                                SequenceNumber::from_int64(1),
                                                16u, {}, {0xABu});
    append_submessage(msg, SubmessageKind::DATA, static_cast<Octet>(FLAG_E | data_flags::D), data_payload);

    TestCapture cap{};
    ParseOps ops = make_ops_all(cap);
    const ParseResult r = parse_message(msg.data(), static_cast<uint32_t>(msg.size()), GuidPrefix{}, ops, &cap);
    CHECK(r == ParseResult::OK, "expected OK");
    CHECK(cap.data_calls == 1, "expected DATA callback after padding alignment");
    return true;
}

static bool tc_9_4_5_1_005(std::string& err) {
    std::vector<Octet> msg{};
    append_header(msg);
    msg.push_back(static_cast<Octet>(SubmessageKind::PAD));
    msg.push_back(FLAG_E);
    msg.push_back(0x01u); // incomplete header (needs 2-byte length)
    ParseOps ops{};
    const ParseResult r = parse_message(msg.data(), static_cast<uint32_t>(msg.size()), GuidPrefix{}, ops, nullptr);
    CHECK(r == ParseResult::MALFORMED, "expected MALFORMED for truncated submessage header");
    return true;
}

// A DATA behind a zero-length INFO_TS (invalidate) or PAD; the length-0 rule
// of §9.4.5.1.3 gives those two kinds an empty body, not the rest of the message.
static ParseResult parse_with_empty_lead(SubmessageKind lead, Octet lead_flags, TestCapture& cap) {
    std::vector<Octet> msg{};
    append_header(msg);
    append_submessage(msg, lead, lead_flags, {}, true, 0u);

    const EntityId reader = make_entity(1u, 2u, 3u, 0xC7u);
    const EntityId writer = make_entity(4u, 5u, 6u, 0xC2u);
    const auto     data_payload =
        make_data_payload(true, reader, writer, SequenceNumber::from_int64(3), 16u, {}, {0x5Au});
    append_submessage(msg, SubmessageKind::DATA, static_cast<Octet>(FLAG_E | data_flags::D),
                      data_payload);

    ParseOps ops = make_ops_all(cap);
    return parse_message(msg.data(), static_cast<uint32_t>(msg.size()), GuidPrefix{}, ops, &cap);
}

static bool tc_9_4_5_1_006(std::string& err) {
    TestCapture       cap{};
    const ParseResult r =
        parse_with_empty_lead(SubmessageKind::INFO_TS, static_cast<Octet>(FLAG_E | 0x02u), cap);
    CHECK(r == ParseResult::OK, "expected OK");
    CHECK(cap.data_calls == 1, "expected the DATA after a zero-length INFO_TS dispatched");
    CHECK(cap.data_ctx.sourceTimestamp.seconds == TIME_INVALID.seconds
              && cap.data_ctx.sourceTimestamp.fraction == TIME_INVALID.fraction,
          "expected source timestamp invalidated");
    return true;
}

static bool tc_9_4_5_1_007(std::string& err) {
    TestCapture       cap{};
    const ParseResult r = parse_with_empty_lead(SubmessageKind::PAD, FLAG_E, cap);
    CHECK(r == ParseResult::OK, "expected OK");
    CHECK(cap.data_calls == 1, "expected the DATA after a zero-length PAD dispatched");
    return true;
}

static bool tc_9_4_5_1_008(std::string& err) {
    // Without the invalidate flag a zero-length INFO_TS lacks its timestamp.
    TestCapture       cap{};
    const ParseResult r = parse_with_empty_lead(SubmessageKind::INFO_TS, FLAG_E, cap);
    CHECK(r == ParseResult::MALFORMED, "expected MALFORMED for INFO_TS missing its timestamp");
    CHECK(cap.data_calls == 0, "expected no DATA after the invalid INFO_TS");
    return true;
}

void append_tests_9_4_5_1(std::vector<TestCase>& tests) {
    tests.push_back({"TC-9.4.5.1-001", "Unknown SubmessageKind is skipped without aborting parse.", tc_9_4_5_1_001});
    tests.push_back({"TC-9.4.5.1-002", "submessageLength=0 extends to end of message for the last submessage.", tc_9_4_5_1_002});
    tests.push_back({"TC-9.4.5.1-003", "Submessage extent overflow beyond message bounds is malformed.", tc_9_4_5_1_003});
    tests.push_back({"TC-9.4.5.1-004", "Submessage processing honors 32-bit boundary alignment padding.", tc_9_4_5_1_004});
    tests.push_back({"TC-9.4.5.1-005", "Truncated submessage headers are malformed.", tc_9_4_5_1_005});
    tests.push_back({"TC-9.4.5.1-006",
                     "A zero-length INFO_TS has an empty body; the next submessage follows.",
                     tc_9_4_5_1_006});
    tests.push_back({"TC-9.4.5.1-007",
                     "A zero-length PAD has an empty body; the next submessage follows.",
                     tc_9_4_5_1_007});
    tests.push_back({"TC-9.4.5.1-008",
                     "A zero-length INFO_TS without the invalidate flag is malformed.",
                     tc_9_4_5_1_008});
}

} // namespace spec_test
