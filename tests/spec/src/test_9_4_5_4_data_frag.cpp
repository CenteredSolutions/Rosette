#include "spec_test_common.hpp"

namespace spec_test {

#define CHECK(COND, MSG) do { if (!(COND)) { err = (MSG); return false; } } while (0)

static bool tc_9_4_5_4_001(std::string& err) {
    std::vector<Octet> msg{};
    append_header(msg);
    const EntityId reader = make_entity(1u, 2u, 3u, 0xC7u);
    const EntityId writer = make_entity(4u, 5u, 6u, 0xC2u);
    const auto frag = make_data_frag_payload(true, reader, writer,
                                             SequenceNumber::from_int64(9),
                                             3u, 2u, 128u, 512u,
                                             {0xCAu, 0xFEu});
    append_submessage(msg, SubmessageKind::DATA_FRAG, FLAG_E, frag);

    TestCapture cap{};
    ParseOps ops = make_ops_all(cap);
    const ParseResult r = parse_message(msg.data(), static_cast<uint32_t>(msg.size()), GuidPrefix{}, ops, &cap);
    CHECK(r == ParseResult::OK, "expected OK");
    CHECK(cap.data_frag_calls == 1, "expected DATA_FRAG callback");
    CHECK(cap.data_frag.fragmentStartingNum == 3u, "expected fragmentStartingNum");
    return true;
}

static bool tc_9_4_5_4_002(std::string& err) {
    std::vector<Octet> msg{};
    append_header(msg);
    const EntityId reader = make_entity(1u, 2u, 3u, 0xC7u);
    const EntityId writer = make_entity(4u, 5u, 6u, 0xC2u);
    auto frag = make_data_frag_payload(true, reader, writer,
                                       SequenceNumber::from_int64(10),
                                       1u, 1u, 64u, 64u, {0xF0u});

    std::vector<Octet> with_q{};
    write_u16(with_q, 0u, true);      // extraFlags
    write_u16(with_q, 28u, true);     // octetsToInlineQos
    with_q.insert(with_q.end(), frag.begin() + 4, frag.begin() + 32); // fixed fields after first 4 bytes
    auto qos = make_sentinel_only_qos(true);
    with_q.insert(with_q.end(), qos.begin(), qos.end());
    with_q.push_back(0xF0u);
    append_submessage(msg, SubmessageKind::DATA_FRAG, static_cast<Octet>(FLAG_E | data_frag_flags::Q), with_q);

    TestCapture cap{};
    ParseOps ops = make_ops_all(cap);
    const ParseResult r = parse_message(msg.data(), static_cast<uint32_t>(msg.size()), GuidPrefix{}, ops, &cap);
    CHECK(r == ParseResult::OK, "expected OK");
    CHECK(cap.data_frag_calls == 1, "expected DATA_FRAG callback with Q flag");
    CHECK(cap.data_frag.fragmentPayloadSize == 1u, "expected single-byte fragment payload");
    return true;
}

void append_tests_9_4_5_4(std::vector<TestCase>& tests) {
    tests.push_back({"TC-9.4.5.4-001", "DATA_FRAG fixed fields and fragment payload are decoded.", tc_9_4_5_4_001});
    tests.push_back({"TC-9.4.5.4-002", "DATA_FRAG Q flag inline QoS is consumed before fragment payload.", tc_9_4_5_4_002});
}

} // namespace spec_test
