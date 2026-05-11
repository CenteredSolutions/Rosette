#include "spec_test_common.hpp"

#include "rosette/rtps/message_builder.hpp"

namespace spec_test {

#define CHECK(COND, MSG) do { if (!(COND)) { err = (MSG); return false; } } while (0)

static bool tc_bld_9_4_1_001(std::string& err) {
    MessageBuilder b{};
    GuidPrefix gp{};
    gp.value[0] = 0x11u;
    gp.value[1] = 0x22u;

    CHECK(b.begin(gp, 128u), "begin should succeed");
    BufRef out = b.finish();
    CHECK(out.valid(), "finish should return valid buffer");
    CHECK(out.size == sizeof(RTPSMessageHeader), "header-only message size should match RTPS header");

    RTPSMessageHeader hdr{};
    memcpy(&hdr, out.data, sizeof(hdr));
    CHECK(rtps_header_valid(hdr), "header should contain RTPS magic");
    CHECK(hdr.guidPrefix.value[0] == 0x11u && hdr.guidPrefix.value[1] == 0x22u,
          "guid prefix should match begin() input");
    out.reset();
    return true;
}

void append_builder_tests_9_4_1(std::vector<TestCase>& tests) {
    tests.push_back({"TC-BLD-9.4.1-001",
                     "begin() writes RTPS message header with protocol magic and caller guidPrefix.",
                     tc_bld_9_4_1_001});
}

} // namespace spec_test

