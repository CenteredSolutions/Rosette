// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

#include "spec_test_common.hpp"

namespace spec_test {

#define CHECK(COND, MSG) do { if (!(COND)) { err = (MSG); return false; } } while (0)

static bool tc_9_4_5_11_001(std::string& err) {
    std::vector<Octet> msg{};
    append_header(msg);
    std::vector<Octet> gap{};
    append_entity_id(gap, make_entity(1u, 2u, 3u, 0xC7u));
    append_entity_id(gap, make_entity(4u, 5u, 6u, 0xC2u));
    write_i32(gap, 0, true);
    write_u32(gap, 10u, true);
    write_i32(gap, 0, true);
    write_u32(gap, 11u, true);
    write_u32(gap, 0u, true);
    append_submessage(msg, SubmessageKind::GAP, FLAG_E, gap);

    TestCapture cap{};
    ParseOps ops = make_ops_all(cap);
    const ParseResult r = parse_message(msg.data(), static_cast<uint32_t>(msg.size()), GuidPrefix{}, ops, &cap);
    CHECK(r == ParseResult::OK, "expected OK");
    CHECK(cap.gap_calls == 1, "expected GAP callback");
    CHECK(cap.gap.gapStart.low == 10u, "expected gapStart low");
    return true;
}

static bool tc_9_4_5_11_002(std::string& err) {
    std::vector<Octet> msg{};
    append_header(msg);
    std::vector<Octet> gap{};
    append_entity_id(gap, make_entity(1u, 2u, 3u, 0xC7u));
    append_entity_id(gap, make_entity(4u, 5u, 6u, 0xC2u));
    write_i32(gap, 0, false);
    write_u32(gap, 0x01020304u, false);
    write_i32(gap, 0, false);
    write_u32(gap, 0x05060708u, false);
    write_u32(gap, 0u, false);
    append_submessage(msg, SubmessageKind::GAP, 0x00u, gap, false);

    TestCapture cap{};
    ParseOps ops = make_ops_all(cap);
    const ParseResult r = parse_message(msg.data(), static_cast<uint32_t>(msg.size()), GuidPrefix{}, ops, &cap);
    CHECK(r == ParseResult::OK, "expected OK");
    CHECK(cap.gap_calls == 1, "expected GAP callback");
    CHECK(cap.gap.gapStart.low == 0x01020304u, "expected big-endian GAP decode");
    return true;
}

// A GAP from gapStart, then a DATA that must not be dispatched when the GAP is invalid.
static ParseResult parse_gap_then_data(int64_t gap_start, TestCapture& cap) {
    std::vector<Octet> msg{};
    append_header(msg);
    const EntityId       reader = make_entity(1u, 2u, 3u, 0xC7u);
    const EntityId       writer = make_entity(4u, 5u, 6u, 0xC2u);
    const SequenceNumber start = SequenceNumber::from_int64(gap_start);
    std::vector<Octet>   gap{};
    append_entity_id(gap, reader);
    append_entity_id(gap, writer);
    write_i32(gap, start.high, true);
    write_u32(gap, start.low, true);
    write_i32(gap, 0, true);
    write_u32(gap, 2u, true);
    write_u32(gap, 0u, true);
    append_submessage(msg, SubmessageKind::GAP, FLAG_E, gap);

    const auto data_payload =
        make_data_payload(true, reader, writer, SequenceNumber::from_int64(2), 16u, {}, {0x33u});
    append_submessage(msg, SubmessageKind::DATA, static_cast<Octet>(FLAG_E | data_flags::D),
                      data_payload);

    ParseOps ops = make_ops_all(cap);
    return parse_message(msg.data(), static_cast<uint32_t>(msg.size()), GuidPrefix{}, ops, &cap);
}

static bool tc_9_4_5_11_003(std::string& err) {
    TestCapture       cap{};
    const ParseResult r = parse_gap_then_data(0, cap);
    CHECK(r == ParseResult::MALFORMED, "expected MALFORMED for gapStart 0");
    CHECK(cap.gap_calls == 0 && cap.data_calls == 0, "expected nothing dispatched");
    return true;
}

static bool tc_9_4_5_11_004(std::string& err) {
    TestCapture       cap{};
    const ParseResult r = parse_gap_then_data(-5, cap);
    CHECK(r == ParseResult::MALFORMED, "expected MALFORMED for a negative gapStart");
    CHECK(cap.gap_calls == 0 && cap.data_calls == 0, "expected nothing dispatched");
    return true;
}

void append_tests_9_4_5_11(std::vector<TestCase>& tests) {
    tests.push_back({"TC-9.4.5.11-001", "GAP fields are decoded and dispatched.", tc_9_4_5_11_001});
    tests.push_back({"TC-9.4.5.11-002", "GAP scalar fields follow submessage endianness.", tc_9_4_5_11_002});
    tests.push_back({"TC-9.4.5.11-003", "A GAP with gapStart 0 is invalid and ends the parse.",
                     tc_9_4_5_11_003});
    tests.push_back({"TC-9.4.5.11-004",
                     "A GAP with a negative gapStart is invalid and ends the parse.",
                     tc_9_4_5_11_004});
}

} // namespace spec_test

