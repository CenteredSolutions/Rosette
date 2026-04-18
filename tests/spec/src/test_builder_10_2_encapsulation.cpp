#include "spec_test_common.hpp"

#include "rtps/message_builder.hpp"

namespace spec_test {

#define CHECK(COND, MSG) do { if (!(COND)) { err = (MSG); return false; } } while (0)

struct DataCap {
    int calls = 0;
    std::vector<Octet> payload{};
};

static void on_data(const ParseContext& ctx, const ParsedData& d) noexcept {
    auto* c = static_cast<DataCap*>(ctx.user_data);
    c->calls++;
    c->payload.assign(d.serializedPayload, d.serializedPayload + d.serializedPayloadSize);
}

static bool tc_bld_10_2_001(std::string& err) {
    MessageBuilder b{};
    CHECK(b.begin(GuidPrefix{}, 256u), "begin failed");
    BufRef payload = buf_alloc(64u);
    CHECK(payload.valid(), "payload alloc failed");
    payload.data[0] = 0xDEu;
    payload.data[1] = 0xADu;
    payload.size = 2u;

    CHECK(b.append_data(make_entity(1u,2u,3u,0xC7u),
                        make_entity(4u,5u,6u,0xC2u),
                        SequenceNumber::from_int64(1),
                        payload),
          "append_data failed");

    BufRef msg = b.finish();
    ParseOps ops{};
    ops.on_data = on_data;
    DataCap cap{};
    CHECK(parse_message(msg.data, msg.size, GuidPrefix{}, ops, &cap) == ParseResult::OK,
          "parse_message failed");
    CHECK(cap.calls == 1, "expected DATA callback");
    CHECK(cap.payload.size() >= 6u, "serializedPayload should include encapsulation + user payload");
    CHECK(cap.payload[0] == 0x01u && cap.payload[1] == 0x00u &&
          cap.payload[2] == 0x00u && cap.payload[3] == 0x00u,
          "encapsulation header should be CDR_LE + zero options");
    CHECK(cap.payload[4] == 0xDEu && cap.payload[5] == 0xADu,
          "user payload should follow encapsulation header");

    msg.reset();
    payload.reset();
    return true;
}

void append_builder_tests_10_2(std::vector<TestCase>& tests) {
    tests.push_back({"TC-BLD-10.2-001",
                     "append_data() prepends CDR_LE encapsulation header (0x0001,0x0000) before user payload.",
                     tc_bld_10_2_001});
}

} // namespace spec_test

