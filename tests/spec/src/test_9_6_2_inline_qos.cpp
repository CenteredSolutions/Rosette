// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

#include "spec_test_common.hpp"

namespace spec_test {

#define CHECK(COND, MSG) do { if (!(COND)) { err = (MSG); return false; } } while (0)

static bool tc_9_6_2_001(std::string& err) {
    std::vector<Octet> msg{};
    append_header(msg);
    const EntityId reader = make_entity(1u, 2u, 3u, 0xC7u);
    const EntityId writer = make_entity(4u, 5u, 6u, 0xC2u);

    SampleIdentity sid{};
    sid.writerGuid.prefix.value[0] = 0x44u;
    sid.writerGuid.entityId = make_entity(0xAAu, 0xBBu, 0xCCu, 0xC2u);
    sid.sequenceNumber = SequenceNumber::from_int64(1234);
    const auto qos = make_related_sample_identity_qos(sid, true);

    const auto data_payload = make_data_payload(true, reader, writer,
                                                SequenceNumber::from_int64(7),
                                                16u, qos, {0x01u});
    append_submessage(msg, SubmessageKind::DATA,
                      static_cast<Octet>(FLAG_E | data_flags::D | data_flags::Q),
                      data_payload);

    TestCapture cap{};
    ParseOps ops = make_ops_all(cap);
    const ParseResult r = parse_message(msg.data(), static_cast<uint32_t>(msg.size()), GuidPrefix{}, ops, &cap);
    CHECK(r == ParseResult::OK, "expected OK");
    CHECK(cap.data_calls == 1, "expected DATA callback");
    CHECK(cap.data.inlineQos.hasRelatedSampleIdentity, "expected related sample identity present");
    CHECK(cap.data.inlineQos.relatedSampleIdentity.sequenceNumber.to_int64() == 1234,
          "expected related sample identity sequence number");
    return true;
}

static bool tc_9_6_2_002(std::string& err) {
    std::vector<Octet> msg{};
    append_header(msg);
    const EntityId reader = make_entity(1u, 2u, 3u, 0xC7u);
    const EntityId writer = make_entity(4u, 5u, 6u, 0xC2u);
    std::vector<Octet> malformed_qos{};
    write_u16(malformed_qos, inline_qos_pid::RELATED_SAMPLE_IDENTITY, true);
    write_u16(malformed_qos, sample_identity_wire_size(), true);
    malformed_qos.push_back(0xAAu);

    const auto data_payload = make_data_payload(true, reader, writer,
                                                SequenceNumber::from_int64(7),
                                                16u, malformed_qos, {0x01u});
    append_submessage(msg, SubmessageKind::DATA,
                      static_cast<Octet>(FLAG_E | data_flags::D | data_flags::Q),
                      data_payload);

    ParseOps ops{};
    const ParseResult r = parse_message(msg.data(), static_cast<uint32_t>(msg.size()), GuidPrefix{}, ops, nullptr);
    CHECK(r == ParseResult::MALFORMED, "expected MALFORMED for truncated inline QoS");
    return true;
}

void append_tests_9_6_2(std::vector<TestCase>& tests) {
    tests.push_back({"TC-9.6.2-001", "InlineQoS ParameterList decodes RelatedSampleIdentity and sentinel termination.", tc_9_6_2_001});
    tests.push_back({"TC-9.6.2-002", "Malformed InlineQoS ParameterList causes parse failure.", tc_9_6_2_002});
}

} // namespace spec_test

