/**
 * @file posix_board_network_prefix_test.cpp
 * @brief The host board network reports the netmask of the interface holding its address.
 *
 * The host address is computed once per process, so this runs in its own
 * executable with `ROSETTE_LOCAL_IP` pinned to 127.0.0.1, which every POSIX
 * host holds on its loopback interface with netmask 255.0.0.0.
 *
 * References: rosette/net/board_network.hpp; ports/net/posix/posix_board_network.cpp;
 * rmw_rosette/network_port.hpp (network_ipv4_prefix_len()).
 */

#include <cstdlib>

#include "rosette/net/board_network.hpp"
#include "test_util.hpp"

namespace {

namespace net = rosette::net;

/// TC-NET-BRD-004: the snapshot carries the prefix length of the interface holding the address.
void reports_the_interface_prefix() {
    check(net::board_ipv4_address() == 0x7F000001u, "004: address is 127.0.0.1");
    const net::AddressSnapshot& snapshot = net::board_address_snapshot();
    check(snapshot.v4.addresses.size() == 1u, "004: one IPv4 address");
    check(snapshot.v4.primary().prefix_len == 8u, "004: loopback interface's /8 prefix");
}

}  // namespace

int main() {
    setenv("ROSETTE_LOCAL_IP", "127.0.0.1", 1);
    reports_the_interface_prefix();
    return test_summary();
}
