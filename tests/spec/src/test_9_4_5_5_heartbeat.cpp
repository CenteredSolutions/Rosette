// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

#include "spec_test_common.hpp"

namespace spec_test {

#define CHECK(COND, MSG)  \
    do {                  \
        if (!(COND)) {    \
            err = (MSG);  \
            return false; \
        }                 \
    } while (0)

// A HEARTBEAT announcing [first, last], then a DATA that must not be dispatched
// when the HEARTBEAT is invalid (§8.3.4.1: it invalidates the rest of the message).
static ParseResult parse_heartbeat_then_data(int64_t first, int64_t last, TestCapture& cap) {
    std::vector<Octet> msg{};
    append_header(msg);
    const EntityId       reader = make_entity(1u, 2u, 3u, 0xC7u);
    const EntityId       writer = make_entity(4u, 5u, 6u, 0xC2u);
    const SequenceNumber first_sn = SequenceNumber::from_int64(first);
    const SequenceNumber last_sn = SequenceNumber::from_int64(last);
    std::vector<Octet>   hb{};
    append_entity_id(hb, reader);
    append_entity_id(hb, writer);
    write_i32(hb, first_sn.high, true);
    write_u32(hb, first_sn.low, true);
    write_i32(hb, last_sn.high, true);
    write_u32(hb, last_sn.low, true);
    write_i32(hb, 7, true);
    append_submessage(msg, SubmessageKind::HEARTBEAT, FLAG_E, hb);

    const auto data_payload =
        make_data_payload(true, reader, writer, SequenceNumber::from_int64(4), 16u, {}, {0x44u});
    append_submessage(msg, SubmessageKind::DATA, static_cast<Octet>(FLAG_E | data_flags::D),
                      data_payload);

    ParseOps ops = make_ops_all(cap);
    return parse_message(msg.data(), static_cast<uint32_t>(msg.size()), GuidPrefix{}, ops, &cap);
}

static bool tc_9_4_5_5_001(std::string& err) {
    TestCapture       cap{};
    const ParseResult r = parse_heartbeat_then_data(2, 9, cap);
    CHECK(r == ParseResult::OK, "expected OK");
    CHECK(cap.heartbeat_calls == 1, "expected HEARTBEAT callback");
    CHECK(cap.heartbeat.firstSN.to_int64() == 2 && cap.heartbeat.lastSN.to_int64() == 9,
          "expected firstSN 2 and lastSN 9");
    CHECK(cap.heartbeat.count == 7, "expected count 7");
    CHECK(cap.data_calls == 1, "expected the DATA after it dispatched");
    return true;
}

static bool tc_9_4_5_5_002(std::string& err) {
    // An empty history announces lastSN = firstSN - 1, as Rosette's own writer does.
    TestCapture       cap{};
    const ParseResult r = parse_heartbeat_then_data(1, 0, cap);
    CHECK(r == ParseResult::OK, "expected OK for the empty-history HEARTBEAT");
    CHECK(cap.heartbeat_calls == 1 && cap.data_calls == 1, "expected both dispatched");
    return true;
}

static bool tc_9_4_5_5_003(std::string& err) {
    for (const int64_t first : {int64_t{0}, int64_t{-3}}) {
        TestCapture       cap{};
        const ParseResult r = parse_heartbeat_then_data(first, 5, cap);
        CHECK(r == ParseResult::MALFORMED, "expected MALFORMED for firstSN < 1");
        CHECK(cap.heartbeat_calls == 0 && cap.data_calls == 0, "expected nothing dispatched");
    }

    return true;
}

static bool tc_9_4_5_5_004(std::string& err) {
    TestCapture       cap{};
    const ParseResult r = parse_heartbeat_then_data(10, 3, cap);
    CHECK(r == ParseResult::MALFORMED, "expected MALFORMED for lastSN < firstSN - 1");
    CHECK(cap.heartbeat_calls == 0 && cap.data_calls == 0, "expected nothing dispatched");
    return true;
}

void append_tests_9_4_5_5(std::vector<TestCase>& tests) {
    tests.push_back(
        {"TC-9.4.5.5-001", "HEARTBEAT fields are decoded and dispatched.", tc_9_4_5_5_001});
    tests.push_back({"TC-9.4.5.5-002",
                     "A HEARTBEAT with lastSN = firstSN - 1 (empty history) is valid.",
                     tc_9_4_5_5_002});
    tests.push_back({"TC-9.4.5.5-003",
                     "A HEARTBEAT with firstSN < 1 is invalid and ends the parse.",
                     tc_9_4_5_5_003});
    tests.push_back({"TC-9.4.5.5-004",
                     "A HEARTBEAT with lastSN < firstSN - 1 is invalid and ends the parse.",
                     tc_9_4_5_5_004});
}

}  // namespace spec_test
