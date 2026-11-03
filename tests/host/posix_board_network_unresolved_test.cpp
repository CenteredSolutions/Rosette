// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file posix_board_network_unresolved_test.cpp
 * @brief The host board network reports no address when `ROSETTE_INTERFACE` names no interface.
 *
 * The interface is resolved once per process, so this runs in its own
 * executable. An unresolvable setting must never fall back to another
 * interface: the board network reports no address, which makes rmw_init()
 * fail instead of announcing the process on a network the user did not ask for.
 *
 * References: ports/net/posix/posix_host_interface.hpp;
 * ports/net/posix/posix_board_network.cpp; rmw_rosette/network_port.hpp.
 */

#include <cstdlib>

#include "../../ports/net/posix/posix_host_interface.hpp"
#include "rosette/net/board_network.hpp"
#include "test_util.hpp"

namespace {

namespace net = rosette::net;

/// TC-NET-BRD-005: an interface that does not exist leaves the board network without an address.
void reports_no_address() {
    check(net::board_ipv4_address() == 0u, "005: no address");
    const net::AddressSnapshot& snapshot = net::board_address_snapshot();
    check(!snapshot.assigned && snapshot.v4.addresses.empty(), "005: snapshot unassigned");
    check(!net::board_network_ready(), "005: not ready");
    check(rosette::net::posix::host_interface_error()
              == rosette::net::posix::InterfaceError::no_such_interface,
          "005: the reason is no_such_interface");
}

}  // namespace

int main() {
    setenv("ROSETTE_INTERFACE", "rosette-none0", 1);
    reports_no_address();
    return test_summary();
}
