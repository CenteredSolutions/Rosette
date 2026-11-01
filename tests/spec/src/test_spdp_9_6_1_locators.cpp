// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

#include "spec_test_common.hpp"

#include "rosette/rtps/spdp.hpp"

namespace spec_test {

#define CHECK(COND, MSG) do { if (!(COND)) { err = (MSG); return false; } } while (0)

static bool tc_spdp_9_6_1_001(std::string& err) {
    CHECK(spdp_multicast_port(0u) == 7400u, "domain 0 multicast port should be 7400");
    CHECK(spdp_multicast_port(1u) == 7650u, "domain 1 multicast port should be 7650");
    CHECK(spdp_unicast_port(0u, 0u) == 7410u, "domain 0 participant 0 unicast port should be 7410");
    CHECK(spdp_unicast_port(0u, 5u) == 7420u, "domain 0 participant 5 unicast port should be 7420");
    return true;
}

static bool tc_spdp_9_6_1_002(std::string& err) {
    const Locator m = spdp_multicast_locator_v4(3u);
    CHECK(m.kind == locator_kind::UDPv4, "SPDP v4 multicast locator should be UDPv4");
    CHECK(m.port == spdp_multicast_port(3u), "SPDP v4 multicast locator port mismatch");
    CHECK(m.address[12] == 0xEFu && m.address[13] == 0xFFu &&
          m.address[14] == 0x00u && m.address[15] == 0x01u,
          "SPDP v4 multicast locator address should be 239.255.0.1");
    return true;
}

static bool tc_spdp_9_6_1_003(std::string& err) {
    const Locator m = spdp_multicast_locator_v6(2u);
    CHECK(m.kind == locator_kind::UDPv6, "SPDP v6 multicast locator should be UDPv6");
    CHECK(m.port == spdp_multicast_port(2u), "SPDP v6 multicast locator port mismatch");
    CHECK(m.address[0] == 0xFFu && m.address[1] == 0x02u,
          "SPDP v6 multicast address should start with FF02::");
    CHECK(m.address[10] == 0xFFu && m.address[11] == 0xFFu &&
          m.address[12] == 0xEFu && m.address[13] == 0xFFu &&
          m.address[14] == 0x00u && m.address[15] == 0x01u,
          "SPDP v6 multicast locator tail should encode 239.255.0.1");
    return true;
}

void append_spdp_tests_9_6_1(std::vector<TestCase>& tests) {
    tests.push_back({"TC-SPDP-9.6.1-001",
                     "§9.6.1 Table 9.35: SPDP well-known multicast/unicast ports follow PB + DG*domain + d constants.",
                     tc_spdp_9_6_1_001});
    tests.push_back({"TC-SPDP-9.6.1-002",
                     "§9.6.1: SPDP IPv4 discovery multicast locator resolves to 239.255.0.1:<port> for the domain.",
                     tc_spdp_9_6_1_002});
    tests.push_back({"TC-SPDP-9.6.1-003",
                     "§9.6.1: SPDP IPv6 discovery multicast locator resolves to FF02::FFFF:239.255.0.1:<port>.",
                     tc_spdp_9_6_1_003});
}

} // namespace spec_test
