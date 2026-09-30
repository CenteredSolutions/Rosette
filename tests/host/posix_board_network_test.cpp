/**
 * @file posix_board_network_test.cpp
 * @brief The host board network: config checking, bring-up, and the address it reports.
 *
 * The host address is pinned with `ROSETTE_LOCAL_IP` before the first call,
 * so the result does not depend on the machine's routing table. The refused
 * configs run first because a refused bring-up starts nothing, while an
 * accepted one is final for the process.
 *
 * References: rosette/net/board_network.hpp; src/net/posix/board_network.cpp.
 */

#include <cstdint>
#include <cstdlib>

#include "rosette/net/board_network.hpp"
#include "rosette/net/network_config.hpp"
#include "test_util.hpp"

namespace {

namespace net = rosette::net;
using net::BringUpResult;
using net::Method;

constexpr uint32_t kHost = 0xC000020Au;  // 192.0.2.10 (RFC 5737 TEST-NET-1)

/// TC-NET-BRD-001: a static address and SLAAC are refused, and a refusal starts nothing.
void refuses_what_the_host_cannot_run() {
    net::NetworkConfig fixed{};
    fixed.v4 = {Method::static_ipv4};
    fixed.static_v4.address = {192u, 0u, 2u, 20u};
    check(net::bring_up_board_network("host", fixed) == BringUpResult::unsupported_config,
          "001: static IPv4 refused");

    net::NetworkConfig slaac{};
    slaac.v6 = {Method::slaac};
    check(net::bring_up_board_network("host", slaac) == BringUpResult::unsupported_config,
          "001: SLAAC refused");
    check(!net::board_network_ready(), "001: nothing started");
}

/// TC-NET-BRD-002: the address and sockets are valid before bring-up, and bring-up reports them.
void brings_up_on_the_host_address() {
    check(net::board_ipv4_address() == kHost, "002: address known before bring-up");

    const auto socket = net::board_socket_provider().open(net::SocketDomain::ipv4);
    check(socket != nullptr && socket->is_open(), "002: socket opens before bring-up");

    check(net::bring_up_board_network("host") == BringUpResult::ready, "002: default config ready");
    const net::AddressSnapshot& snapshot = net::board_address_snapshot();
    check(snapshot.assigned && snapshot.v4.addresses.size() == 1u, "002: one IPv4 address");
    const auto& a = snapshot.v4.address();
    check(a[0] == 192u && a[1] == 0u && a[2] == 2u && a[3] == 10u, "002: it is the host's");
    check(snapshot.v4.source == net::AddressSource::manual && snapshot.v6.addresses.empty(),
          "002: source manual, no IPv6");
    check(net::bring_up_board_network("host") == BringUpResult::ready,
          "002: a second call is ready");
}

/// TC-NET-BRD-003: the watch sees what the board network functions see.
void watch_matches_the_board_network() {
    const net::NetworkWatch watch = net::board_network_watch();
    watch.poll(0u);
    check(watch.ready() && net::board_network_ready(), "003: watch ready");
    check(&watch.snapshot() == &net::board_address_snapshot(), "003: same snapshot");
    check(net::board_send_timeouts() == 0u, "003: no send timeouts reported");
}

}  // namespace

int main() {
    setenv("ROSETTE_LOCAL_IP", "192.0.2.10", 1);
    refuses_what_the_host_cannot_run();
    brings_up_on_the_host_address();
    watch_matches_the_board_network();
    return test_summary();
}
