/**
 * @file posix_host_interface_test.cpp
 * @brief `ROSETTE_INTERFACE` resolution, and multicast sent on the chosen interface.
 *
 * Loopback is the one interface every POSIX host has, so the resolution
 * cases name it by name and by address, and the multicast case joins and
 * sends on it. The failure cases use a name and an address (RFC 5737
 * TEST-NET-1) that no host holds.
 *
 * References: ports/net/posix/posix_host_interface.hpp;
 * rosette/net/socket.hpp (Socket::set_multicast_interface); Linux ip(7).
 */

#include <array>
#include <cstdint>
#include <cstring>

#include "../../ports/net/posix/posix_host_interface.hpp"
#include "../../ports/net/posix/posix_socket.hpp"
#include "test_util.hpp"

namespace {

namespace net = rosette::net;
namespace posix = rosette::net::posix;

constexpr uint32_t kLoopback = 0x7F000001u;

/// TC-NET-IFC-001: loopback resolves by name and by address to lo at 127.0.0.1/8.
void resolves_by_name_and_address() {
    posix::HostInterface by_address{};
    check(posix::resolve_host_interface("127.0.0.1", by_address) == posix::InterfaceError::none,
          "001: 127.0.0.1 resolves");
    check(by_address.address == kLoopback && by_address.prefix_len == 8u, "001: to 127.0.0.1/8");

    posix::HostInterface by_name{};
    check(posix::resolve_host_interface(by_address.name, by_name) == posix::InterfaceError::none,
          "001: its name resolves");
    check(by_name.address == kLoopback && std::strcmp(by_name.name, by_address.name) == 0,
          "001: to the same interface");
}

/// TC-NET-IFC-002: an unknown name and an unheld address are refused, each with its own reason.
void refuses_what_no_interface_is() {
    posix::HostInterface out{};
    check(posix::resolve_host_interface("rosette-none0", out)
              == posix::InterfaceError::no_such_interface,
          "002: unknown name is no_such_interface");
    check(
        posix::resolve_host_interface("192.0.2.10", out) == posix::InterfaceError::address_not_held,
        "002: unheld address is address_not_held");
    check(out.address == 0u, "002: output untouched");
    check(std::strcmp(posix::describe(posix::InterfaceError::address_not_held),
                      "no interface holds that address")
              == 0,
          "002: describe() words the reason");
}

/// TC-NET-IFC-003: a list names several interfaces and is refused.
void refuses_a_list() {
    posix::HostInterface out{};
    check(posix::resolve_host_interface("lo,eth0", out) == posix::InterfaceError::several_named,
          "003: list is several_named");
}

/// TC-NET-IFC-004: the default is an address the host holds, or loopback.
void defaults_to_a_held_address() {
    posix::HostInterface chosen{};
    check(posix::resolve_host_interface(nullptr, chosen) == posix::InterfaceError::none,
          "004: default resolves");
    check(chosen.address != 0u, "004: to an address");

    posix::HostInterface empty{};
    check(posix::resolve_host_interface("", empty) == posix::InterfaceError::none
              && empty.address == chosen.address,
          "004: empty is the default");
}

/// TC-NET-IFC-006: a group datagram sent with the egress interface set reaches a member joined
/// there.
void sends_multicast_on_the_chosen_interface() {
    posix::PosixSocketProvider   provider;
    auto                         receiver = provider.open();
    auto                         sender = provider.open();
    const std::array<uint8_t, 4> lo{127u, 0u, 0u, 1u};
    const std::array<uint8_t, 4> group{239u, 255u, 0u, 1u};
    constexpr uint16_t           kPort = 17410u;

    net::SocketMulticastMembership membership{};
    std::memcpy(membership.group.data(), group.data(), group.size());
    std::memcpy(membership.interface.data(), lo.data(), lo.size());
    membership.port = kPort;
    check(receiver->joinMulticastGroup(membership) == 0, "006: receiver joins on loopback");
    check(receiver->bind(net::make_socket_address_v4({0u, 0u, 0u, 0u}, kPort)) == 0,
          "006: receiver binds the group port");
    (void)receiver->set_receive_timeout(1000u);

    check(sender->set_multicast_interface(net::make_socket_address_v4(lo, 0u)) == 0,
          "006: egress set to loopback");
    check(sender->set_multicast_interface(net::make_socket_address_v4({192u, 0u, 2u, 10u}, 0u))
              == static_cast<int>(net::SocketError::invalid_argument),
          "006: an unheld address is invalid_argument");
    check(sender->set_multicast_interface(net::make_socket_address_v4(lo, 0u)) == 0,
          "006: egress back on loopback");

    const char payload[] = "rosette";
    check(sender->send_to(payload, sizeof(payload), net::make_socket_address_v4(group, kPort))
              == static_cast<std::ptrdiff_t>(sizeof(payload)),
          "006: group datagram sent");

    char                 received[16] = {};
    net::SocketAddress   from{};
    const std::ptrdiff_t got = receiver->recv_from(received, sizeof(received), &from);
    check(
        got == static_cast<std::ptrdiff_t>(sizeof(payload)) && std::strcmp(received, payload) == 0,
        "006: member received it");
}

}  // namespace

int main() {
    resolves_by_name_and_address();
    refuses_what_no_interface_is();
    refuses_a_list();
    defaults_to_a_held_address();
    sends_multicast_on_the_chosen_interface();
    return test_summary();
}
