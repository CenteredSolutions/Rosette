// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file mdns_wire_test.cpp
 * @brief Host checks for the DNS wire codec and the mDNS address policy.
 *
 * These exercise the parts of `net/dns_wire.hpp` and `net/mdns.hpp` that have no
 * socket dependency: query encoding, response parsing with compression
 * pointers, name normalisation, and the source-first address rule.
 *
 * Build and run standalone:
 * @code
 *   c++ -std=c++17 -Wall -Wextra -I include \
 *       tests/host/mdns_wire_test.cpp -o /tmp/mdns_wire_test && /tmp/mdns_wire_test
 * @endcode
 * or through CMake: `cmake -S tests/host -B build/tests-host && ctest --test-dir build/tests-host`.
 */

#include "rosette/net/dns_wire.hpp"
#include "rosette/net/mdns.hpp"

#include <array>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>

namespace {

int g_failures = 0;

void check(bool condition, const char* what)
{
    std::printf("[%s] %s\n", condition ? "PASS" : "FAIL", what);
    if (!condition) {
        ++g_failures;
    }
}

namespace dns = rosette::net::dns;
namespace mdns_detail = rosette::net::mdns_detail;

// --- A response for "agent.local" -> 192.168.1.50 -------------------------------
const std::vector<uint8_t> kAResponse = {
    0x00, 0x00, 0x84, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00,
    // question: agent.local  A  IN
    0x05, 'a', 'g', 'e', 'n', 't', 0x05, 'l', 'o', 'c', 'a', 'l', 0x00,
    0x00, 0x01, 0x00, 0x01,
    // answer: PTR-compressed name -> A  cache-flush|IN  ttl 120  rdlen 4
    0xC0, 0x0C, 0x00, 0x01, 0x80, 0x01, 0x00, 0x00, 0x00, 0x78, 0x00, 0x04,
    0xC0, 0xA8, 0x01, 0x32,
};

// --- DNS-SD reply to a PTR query for "_microrosagent._udp.local" ---------------
// PTR (answer) + SRV + A (additional), wired together with 0xC0 pointers the way
// Avahi packs a full service resolution into one datagram.
const std::vector<uint8_t> kServiceResponse = {
    0x00, 0x00, 0x84, 0x00, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x02,
    // question (offset 12): _microrosagent._udp.local  PTR  IN
    0x0E, '_', 'm', 'i', 'c', 'r', 'o', 'r', 'o', 's', 'a', 'g', 'e', 'n', 't',
    0x04, '_', 'u', 'd', 'p', 0x05, 'l', 'o', 'c', 'a', 'l', 0x00,
    0x00, 0x0C, 0x00, 0x01,
    // answer PTR (offset 43): name -> q(12); rdata = "agent" + ptr(12)
    0xC0, 0x0C, 0x00, 0x0C, 0x00, 0x01, 0x00, 0x00, 0x00, 0x78, 0x00, 0x08,
    0x05, 'a', 'g', 'e', 'n', 't', 0xC0, 0x0C,
    // additional SRV (offset 63): name -> instance label at offset 55
    0xC0, 0x37, 0x00, 0x21, 0x00, 0x01, 0x00, 0x00, 0x00, 0x78, 0x00, 0x13,
    0x00, 0x00, 0x00, 0x00, 0x22, 0xB8,  // priority, weight, port 8888
    0x05, 'a', 'g', 'e', 'n', 't', 0x05, 'l', 'o', 'c', 'a', 'l', 0x00,
    // additional A (offset 94): name -> "agent.local" at offset 81
    0xC0, 0x51, 0x00, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x78, 0x00, 0x04,
    0xC0, 0xA8, 0x01, 0x32,
};

// --- SRV record whose target name is a self-referential compression pointer ----
const std::vector<uint8_t> kPointerLoop = {
    0x00, 0x00, 0x84, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00,
    0xC0, 0x0C, 0x00, 0x21, 0x00, 0x01, 0x00, 0x00, 0x00, 0x78, 0x00, 0x08,
    0x00, 0x00, 0x00, 0x00, 0x22, 0xB8, 0xC0, 0x1E,  // target ptr -> offset 30
};

void test_query_builder()
{
    uint8_t query[128] = {};
    const size_t length =
        dns::QueryBuilder(query, sizeof(query))
            .build("_microrosagent._udp.local", dns::kTypePtr,
                   /*unicast_response=*/true);

    check(length == 43u, "PTR query is 43 bytes");
    check(rosette::net::inet::load_be16(query + 4) == 1u,
          "query carries exactly one question");
    check(query[12] == 0x0E && std::memcmp(query + 13, "_microrosagent", 14) == 0,
          "first label is _microrosagent");
    check(rosette::net::inet::load_be16(query + 39) == dns::kTypePtr,
          "QTYPE is PTR");
    check(rosette::net::inet::load_be16(query + 41) ==
              (dns::kClassIn | dns::kQuUnicastResponseBit),
          "QCLASS is IN with the unicast-response bit");
}

void test_parse_a_response()
{
    std::array<uint8_t, 4> address{};
    const bool found =
        mdns_detail::find_first_a(kAResponse.data(), kAResponse.size(), address);
    check(found, "A record found in A response");
    check(address == std::array<uint8_t, 4>{192u, 168u, 1u, 50u},
          "A record decodes to 192.168.1.50");
}

void test_parse_service_response()
{
    dns::Srv srv{};
    const bool srv_found = mdns_detail::find_first_srv(
        kServiceResponse.data(), kServiceResponse.size(), srv);
    check(srv_found, "SRV record found in service response");
    check(srv.port == 8888u, "SRV port is 8888");
    check(std::strcmp(srv.target, "agent.local") == 0,
          "SRV target decompresses to agent.local");

    std::array<uint8_t, 4> address{};
    const bool a_found = mdns_detail::find_first_a(
        kServiceResponse.data(), kServiceResponse.size(), address);
    check(a_found, "A record found in service response");
    check(address == std::array<uint8_t, 4>{192u, 168u, 1u, 50u},
          "service A record decodes to 192.168.1.50");
}

void test_pointer_loop_is_rejected()
{
    dns::Srv srv{};
    const bool found = mdns_detail::find_first_srv(kPointerLoop.data(),
                                                  kPointerLoop.size(), srv);
    check(!found, "self-referential SRV target is rejected, not followed");
}

void test_response_matching()
{
    check(mdns_detail::response_answers(kAResponse.data(), kAResponse.size(),
                                        "agent.local"),
          "A response is accepted for its own question");
    check(!mdns_detail::response_answers(kAResponse.data(), kAResponse.size(),
                                         "other.local"),
          "A response is rejected for an unrelated question");
    check(mdns_detail::response_answers(kServiceResponse.data(),
                                        kServiceResponse.size(),
                                        "_microrosagent._udp.local"),
          "service response is accepted for its service type");
    check(!mdns_detail::response_answers(kServiceResponse.data(),
                                         kServiceResponse.size(),
                                         "_other._udp.local"),
          "service response is rejected for a different service type");
}

void test_name_normalisation()
{
    char out[64] = {};
    check(mdns_detail::to_mdns_name("agent", out, sizeof(out)) &&
              std::strcmp(out, "agent.local") == 0,
          "bare label gains .local");
    check(mdns_detail::to_mdns_name("agent.local", out, sizeof(out)) &&
              std::strcmp(out, "agent.local") == 0,
          "qualified .local name is unchanged");
    check(mdns_detail::to_mdns_name("box.lan", out, sizeof(out)) &&
              std::strcmp(out, "box.lan") == 0,
          "other qualified names are left verbatim");
    check(!mdns_detail::to_mdns_name("", out, sizeof(out)),
          "empty name is rejected");
}

void test_routable_classification()
{
    using A = std::array<uint8_t, 4>;
    check(mdns_detail::is_routable_v4(A{192u, 168u, 1u, 50u}), "192.168.1.50 routable");
    check(mdns_detail::is_routable_v4(A{10u, 1u, 2u, 3u}), "10.1.2.3 routable");
    check(!mdns_detail::is_routable_v4(A{127u, 0u, 0u, 1u}), "127.0.0.1 not routable");
    check(!mdns_detail::is_routable_v4(A{0u, 0u, 0u, 0u}), "0.0.0.0 not routable");
    check(!mdns_detail::is_routable_v4(A{169u, 254u, 3u, 9u}), "link-local not routable");
    check(!mdns_detail::is_routable_v4(A{224u, 0u, 0u, 251u}), "multicast not routable");
}

void test_address_policy()
{
    using A = std::array<uint8_t, 4>;
    A out{};

    check(mdns_detail::choose_agent_address(A{192u, 168u, 1u, 9u}, true,
                                            A{172u, 17u, 0u, 2u}, out) &&
              out == A{192u, 168u, 1u, 9u},
          "routable source wins over an advertised A record");

    check(mdns_detail::choose_agent_address(A{127u, 0u, 0u, 1u}, true,
                                            A{192u, 168u, 1u, 9u}, out) &&
              out == A{192u, 168u, 1u, 9u},
          "loopback source falls back to the routable A record");

    check(!mdns_detail::choose_agent_address(A{127u, 0u, 0u, 1u}, true,
                                             A{127u, 0u, 0u, 1u}, out),
          "loopback source and loopback A record resolve to nothing");
}

}  // namespace

int main()
{
    test_query_builder();
    test_parse_a_response();
    test_parse_service_response();
    test_pointer_loop_is_rejected();
    test_response_matching();
    test_name_normalisation();
    test_routable_classification();
    test_address_policy();

    std::printf("\n%s (%d failure%s)\n", g_failures == 0 ? "OK" : "FAILED",
                g_failures, g_failures == 1 ? "" : "s");
    return g_failures == 0 ? 0 : 1;
}
