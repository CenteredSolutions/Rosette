#include "spec_test_common.hpp"

#include "rtps/message_builder.hpp"

namespace spec_test {

#define CHECK(COND, MSG) do { if (!(COND)) { err = (MSG); return false; } } while (0)

struct DataCapture {
    int calls = 0;
    ParsedData data{};
    std::vector<Octet> payload{};
};

static void on_data_capture(const ParseContext& ctx, const ParsedData& d) noexcept {
    auto* c = static_cast<DataCapture*>(ctx.user_data);
    c->calls++;
    c->data = d;
    c->payload.assign(d.serializedPayload, d.serializedPayload + d.serializedPayloadSize);
}

static bool parse_single_data(const BufRef& msg, DataCapture& out) {
    ParseOps ops{};
    ops.on_data = on_data_capture;
    return parse_message(msg.data, msg.size, GuidPrefix{}, ops, &out) == ParseResult::OK;
}

static bool tc_bld_9_4_5_3_001(std::string& err) {
    MessageBuilder b{};
    GuidPrefix gp{};
    gp.value[0] = 0x33u;
    CHECK(b.begin(gp, 256u), "begin should succeed");

    BufRef payload = buf_alloc(64u);
    CHECK(payload.valid(), "payload alloc failed");
    payload.data[0] = 0xAAu;
    payload.data[1] = 0xBBu;
    payload.size = 2u;

    const EntityId reader = make_entity(1u, 2u, 3u, 0xC7u);
    const EntityId writer = make_entity(4u, 5u, 6u, 0xC2u);
    CHECK(b.append_data(reader, writer, SequenceNumber::from_int64(7), payload),
          "append_data should succeed");

    BufRef out = b.finish();
    CHECK(out.valid(), "finish should produce message");

    DataCapture cap{};
    CHECK(parse_single_data(out, cap), "built DATA message should parse");
    CHECK(cap.calls == 1, "expected one DATA callback");
    CHECK(cap.data.writerSN.to_int64() == 7, "writer SN mismatch");
    CHECK(cap.payload.size() >= 6u, "payload should include encapsulation + body");

    out.reset();
    payload.reset();
    return true;
}

static bool tc_bld_9_4_5_3_002(std::string& err) {
    MessageBuilder b{};
    CHECK(b.begin(GuidPrefix{}, 256u), "begin should succeed");
    BufRef payload = buf_alloc(64u);
    CHECK(payload.valid(), "payload alloc failed");
    payload.data[0] = 0x42u;
    payload.size = 1u;

    InlineQos qos{};
    qos.hasRelatedSampleIdentity = true;
    qos.relatedSampleIdentity.writerGuid.prefix.value[0] = 0x5Au;
    qos.relatedSampleIdentity.writerGuid.entityId = make_entity(0xAAu, 0xBBu, 0xCCu, 0xC2u);
    qos.relatedSampleIdentity.sequenceNumber = SequenceNumber::from_int64(1234);

    CHECK(b.append_data(make_entity(1u,2u,3u,0xC7u),
                        make_entity(4u,5u,6u,0xC2u),
                        SequenceNumber::from_int64(9),
                        payload,
                        false,
                        qos),
          "append_data with inline qos should succeed");

    BufRef out = b.finish();
    DataCapture cap{};
    CHECK(parse_single_data(out, cap), "built DATA+Q message should parse");
    CHECK(cap.calls == 1, "expected one DATA callback");
    CHECK(cap.data.inlineQosPresent, "Q flag not parsed");
    CHECK(cap.data.inlineQos.hasRelatedSampleIdentity, "related sample identity missing");
    CHECK(cap.data.inlineQos.relatedSampleIdentity.sequenceNumber.to_int64() == 1234,
          "related sample identity sequence number mismatch");

    out.reset();
    payload.reset();
    return true;
}

static bool tc_bld_9_4_5_3_003(std::string& err) {
    BufRef payload = buf_alloc_with_headroom(16u, kRtpsDataHeadroom);
    CHECK(payload.valid(), "alloc with headroom failed");
    payload.data[0] = 0x11u;
    payload.data[1] = 0x22u;
    payload.size = 2u;

    MessageBuilder b{};
    CHECK(b.begin_with_payload(static_cast<BufRef&&>(payload)), "begin_with_payload failed");
    CHECK(b.prepend_data_message(GuidPrefix{},
                                 make_entity(1u,2u,3u,0xC7u),
                                 make_entity(4u,5u,6u,0xC2u),
                                 SequenceNumber::from_int64(99)),
          "prepend_data_message failed");

    BufRef out = b.finish();
    DataCapture cap{};
    CHECK(parse_single_data(out, cap), "prepended message should parse");
    CHECK(cap.calls == 1, "expected one DATA callback");
    CHECK(cap.data.writerSN.to_int64() == 99, "writer SN mismatch");
    CHECK(cap.payload.size() == 2u, "payload size mismatch");
    CHECK(cap.payload[0] == 0x11u && cap.payload[1] == 0x22u, "payload bytes mismatch");
    out.reset();
    return true;
}

static bool tc_bld_9_4_5_3_004(std::string& err) {
    BufRef payload = buf_alloc_with_headroom(16u, kRtpsDataHeadroom);
    CHECK(payload.valid(), "alloc with headroom failed");
    payload.size = 1u;
    payload.data[0] = 0x77u;

    MessageBuilder b{};
    CHECK(b.begin_with_payload(static_cast<BufRef&&>(payload)), "begin_with_payload failed");
    InlineQos qos{};
    qos.hasRelatedSampleIdentity = true;
    qos.relatedSampleIdentity.sequenceNumber = SequenceNumber::from_int64(1);
    const bool ok = b.prepend_data_message(GuidPrefix{},
                                           make_entity(1u,2u,3u,0xC7u),
                                           make_entity(4u,5u,6u,0xC2u),
                                           SequenceNumber::from_int64(1),
                                           false,
                                           qos);
    CHECK(!ok, "prepend_data_message should reject inline qos in MVP");
    b.abort();
    return true;
}

void append_builder_tests_9_4_5_data(std::vector<TestCase>& tests) {
    tests.push_back({"TC-BLD-9.4.5.3-001",
                     "append_data() serializes a parseable DATA submessage with fixed fields and payload.",
                     tc_bld_9_4_5_3_001});
    tests.push_back({"TC-BLD-9.4.5.3-002",
                     "append_data() sets Q flag and serializes related sample identity inline QoS when provided.",
                     tc_bld_9_4_5_3_002});
    tests.push_back({"TC-BLD-9.4.5.3-003",
                     "prepend_data_message() builds parseable DATA framing over headroom-backed payload.",
                     tc_bld_9_4_5_3_003});
    tests.push_back({"TC-BLD-9.4.5.3-004",
                     "prepend_data_message() rejects non-empty inline qos in current MVP path.",
                     tc_bld_9_4_5_3_004});
}

} // namespace spec_test

