// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

#include "spec_test_common.hpp"

namespace spec_test {

#define CHECK(COND, MSG) do { if (!(COND)) { err = (MSG); return false; } } while (0)

static bool tc_9_4_5_3_001(std::string& err) {
    std::vector<Octet> msg{};
    append_header(msg);
    const EntityId reader = make_entity(0x10u, 0x11u, 0x12u, 0xC7u);
    const EntityId writer = make_entity(0x20u, 0x21u, 0x22u, 0xC2u);
    const auto data_payload = make_data_payload(true, reader, writer,
                                                SequenceNumber::from_int64(42),
                                                16u, {}, {0x01u, 0x02u, 0x03u});
    append_submessage(msg, SubmessageKind::DATA, static_cast<Octet>(FLAG_E | data_flags::D), data_payload);

    TestCapture cap{};
    ParseOps ops = make_ops_all(cap);
    const ParseResult r = parse_message(msg.data(), static_cast<uint32_t>(msg.size()), GuidPrefix{}, ops, &cap);
    CHECK(r == ParseResult::OK, "expected OK");
    CHECK(cap.data_calls == 1, "expected DATA callback");
    CHECK(cap.data.dataFlag && !cap.data.keyFlag, "expected DATA flags");
    CHECK(cap.data.writerSN.to_int64() == 42, "expected writer SN 42");
    return true;
}

static bool tc_9_4_5_3_002(std::string& err) {
    std::vector<Octet> msg{};
    append_header(msg);
    const EntityId reader = make_entity(0x01u, 0x02u, 0x03u, 0xC7u);
    const EntityId writer = make_entity(0x04u, 0x05u, 0x06u, 0xC2u);
    SequenceNumber sn{};
    sn.high = 0x01020304;
    sn.low = 0xA1B2C3D4u;
    const auto data_payload = make_data_payload(false, reader, writer, sn, 16u, {}, {0xEEu});
    append_submessage(msg, SubmessageKind::DATA, data_flags::D, data_payload, false);

    TestCapture cap{};
    ParseOps ops = make_ops_all(cap);
    const ParseResult r = parse_message(msg.data(), static_cast<uint32_t>(msg.size()), GuidPrefix{}, ops, &cap);
    CHECK(r == ParseResult::OK, "expected OK");
    CHECK(cap.data_calls == 1, "expected DATA callback");
    CHECK(cap.data.writerSN.high == 0x01020304 && cap.data.writerSN.low == 0xA1B2C3D4u,
          "expected big-endian sequence number decode");
    return true;
}

static bool tc_9_4_5_3_003(std::string& err) {
    std::vector<Octet> msg{};
    append_header(msg);
    const EntityId reader = make_entity(1u, 1u, 1u, 0xC7u);
    const EntityId writer = make_entity(2u, 2u, 2u, 0xC2u);
    const auto data_payload = make_data_payload(true, reader, writer,
                                                SequenceNumber::from_int64(5),
                                                20u, {}, {0x99u});
    append_submessage(msg, SubmessageKind::DATA, static_cast<Octet>(FLAG_E | data_flags::D), data_payload);

    TestCapture cap{};
    ParseOps ops = make_ops_all(cap);
    const ParseResult r = parse_message(msg.data(), static_cast<uint32_t>(msg.size()), GuidPrefix{}, ops, &cap);
    CHECK(r == ParseResult::OK, "expected OK");
    CHECK(cap.data_calls == 1, "expected DATA callback");
    CHECK(cap.data_payload.size() == 1u && cap.data_payload[0] == 0x99u,
          "expected payload after octetsToInlineQos extension bytes");
    return true;
}

static bool tc_9_4_5_3_004(std::string& err) {
    std::vector<Octet> msg{};
    append_header(msg);
    const EntityId reader = make_entity(1u, 2u, 3u, 0xC7u);
    const EntityId writer = make_entity(4u, 5u, 6u, 0xC2u);

    std::vector<Octet> inline_qos{};
    write_u16(inline_qos, inline_qos_pid::RELATED_SAMPLE_IDENTITY_CUSTOM, true);
    write_u16(inline_qos, sample_identity_wire_size(), true);
    for (int i = 0; i < 24; ++i) inline_qos.push_back(static_cast<Octet>(i + 1));
    write_u16(inline_qos, inline_qos_pid::SENTINEL, true);
    write_u16(inline_qos, 0u, true);

    const auto data_payload = make_data_payload(true, reader, writer,
                                                SequenceNumber::from_int64(8),
                                                16u, inline_qos, {0x42u});
    append_submessage(msg, SubmessageKind::DATA,
                      static_cast<Octet>(FLAG_E | data_flags::D | data_flags::Q),
                      data_payload);

    TestCapture cap{};
    ParseOps ops = make_ops_all(cap);
    const ParseResult r = parse_message(msg.data(), static_cast<uint32_t>(msg.size()), GuidPrefix{}, ops, &cap);
    CHECK(r == ParseResult::OK, "expected OK");
    CHECK(cap.data.inlineQos.hasRelatedSampleIdentity, "expected related sample identity from custom PID");
    return true;
}

static bool tc_9_4_5_3_005(std::string& err) {
    std::vector<Octet> msg{};
    append_header(msg);
    const EntityId reader = make_entity(1u, 2u, 3u, 0xC7u);
    const EntityId writer = make_entity(4u, 5u, 6u, 0xC2u);

    std::vector<Octet> inline_qos{};
    write_u16(inline_qos, 0x7777u, true); // unknown PID
    write_u16(inline_qos, 4u, true);
    inline_qos.push_back(0xAAu);
    inline_qos.push_back(0xBBu);
    inline_qos.push_back(0xCCu);
    inline_qos.push_back(0xDDu);
    write_u16(inline_qos, inline_qos_pid::SENTINEL, true);
    write_u16(inline_qos, 0u, true);

    const auto data_payload = make_data_payload(true, reader, writer,
                                                SequenceNumber::from_int64(8),
                                                16u, inline_qos, {0x42u});
    append_submessage(msg, SubmessageKind::DATA,
                      static_cast<Octet>(FLAG_E | data_flags::D | data_flags::Q),
                      data_payload);

    TestCapture cap{};
    ParseOps ops = make_ops_all(cap);
    const ParseResult r = parse_message(msg.data(), static_cast<uint32_t>(msg.size()), GuidPrefix{}, ops, &cap);
    CHECK(r == ParseResult::OK, "expected OK");
    CHECK(!cap.data.inlineQos.hasRelatedSampleIdentity, "expected unknown PID to be skipped");
    CHECK(cap.data_payload.size() == 1u && cap.data_payload[0] == 0x42u,
          "expected payload preserved after unknown inline QoS param");
    return true;
}

void append_tests_9_4_5_3(std::vector<TestCase>& tests) {
    tests.push_back({"TC-9.4.5.3-001", "DATA flags and payload are decoded and delivered to callback.", tc_9_4_5_3_001});
    tests.push_back({"TC-9.4.5.3-002", "DATA scalar fields follow submessage endianness flag.", tc_9_4_5_3_002});
    tests.push_back({"TC-9.4.5.3-003", "octetsToInlineQos offset is honored before payload parsing.", tc_9_4_5_3_003});
    tests.push_back({"TC-9.4.5.3-004", "DATA inline QoS accepts related sample identity custom PID variant.", tc_9_4_5_3_004});
    tests.push_back({"TC-9.4.5.3-005", "Unknown inline QoS parameters are skipped safely.", tc_9_4_5_3_005});
}

} // namespace spec_test
