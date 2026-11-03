// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file network_config_test.cpp
 * @brief `NetworkConfig`: checking a config against a backend, ordering providers by it, and the
 * ready predicates.
 *
 * The capabilities are the WIZnet W6100's (IPv4 DHCP, link-local, static;
 * IPv6 SLAAC) and the W5100S's (no IPv6). TC-NET-CFG-005 runs the providers
 * a config selects under a real `AddressManager`, so the list order is shown
 * to decide which address the interface gets.
 *
 * References: rosette/net/network_config.hpp; RFC 3927 §1.9.
 */

#include <array>
#include <cstdint>

#include "fake_interface.hpp"
#include "rosette/net/address_manager.hpp"
#include "rosette/net/ipv4ll_provider.hpp"
#include "rosette/net/network_config.hpp"
#include "rosette/net/static_provider.hpp"
#include "test_util.hpp"

namespace {

namespace net = rosette::net;
using net::ConfigError;
using net::Method;

constexpr std::array<uint8_t, 6> kMac{0x00u, 0x08u, 0xDCu, 0x54u, 0x75u, 0x30u};

const net::NetworkCapabilities kW6100{
    {Method::dhcp, Method::ipv4_link_local, Method::static_ipv4},
    {Method::slaac},
};

const net::NetworkCapabilities kW5100s{
    {Method::dhcp, Method::ipv4_link_local, Method::static_ipv4},
    {},
};

/// @brief A snapshot holding the given IPv4 and IPv6 addresses (all zero for none).
net::AddressSnapshot snapshot(std::array<uint8_t, 4> v4, std::array<uint8_t, 16> v6 = {}) {
    net::AddressSnapshot s{};
    if (v4 != std::array<uint8_t, 4>{}) {
        (void)s.v4.addresses.add(net::AddressFamily::ipv4, {v4[0], v4[1], v4[2], v4[3]}, 16u);
    }

    if (v6 != std::array<uint8_t, 16>{}) {
        (void)s.v6.addresses.add(net::AddressFamily::ipv6, v6, 64u);
    }

    return s;
}

void tc_accepts_well_formed_configs() {
    check(net::check_config(net::NetworkConfig{}, kW6100) == ConfigError::none,
          "001: the default config (DHCP, then link-local) is accepted");

    // The firmware-facing spelling, designated initializers in declaration order.
    const net::NetworkConfig sketch{
        .v4 = {Method::dhcp, Method::ipv4_link_local},
        .v6 = {Method::slaac},
        .upgrade_probe_ms = 300'000u,
        .ready_when = net::ready::any_v4,
    };
    check(net::check_config(sketch, kW6100) == ConfigError::none,
          "001: DHCP, link-local, and SLAAC on the W6100");

    net::NetworkConfig fixed{};
    fixed.v4 = {Method::static_ipv4, Method::ipv4_link_local};
    fixed.static_v4.address = {192u, 168u, 1u, 50u};
    check(net::check_config(fixed, kW5100s) == ConfigError::none,
          "001: a static address with a link-local fallback");
}

void tc_rejects_each_defect() {
    net::NetworkConfig c{};
    c.v4 = {};
    check(net::check_config(c, kW6100) == ConfigError::no_ipv4_method, "002: no IPv4 method");

    c = {};
    c.v4 = {Method::dhcp, Method::none, Method::ipv4_link_local};
    check(net::check_config(c, kW6100) == ConfigError::gap_in_list, "002: a gap in the list");

    c = {};
    c.v4 = {Method::dhcp, Method::dhcp};
    check(net::check_config(c, kW6100) == ConfigError::duplicate_method, "002: a duplicate");

    c = {};
    c.v4 = {Method::slaac};
    check(net::check_config(c, kW6100) == ConfigError::wrong_family, "002: SLAAC listed as IPv4");

    c = {};
    c.v6 = {Method::dhcp};
    check(net::check_config(c, kW6100) == ConfigError::wrong_family, "002: DHCPv4 listed as IPv6");

    c = {};
    c.v6 = {Method::slaac};
    check(net::check_config(c, kW5100s) == ConfigError::unsupported_method,
          "002: SLAAC on a part without IPv6 is refused, not skipped");

    c = {};
    c.v4 = {Method::static_ipv4};
    check(net::check_config(c, kW6100) == ConfigError::missing_static_address,
          "002: static_ipv4 without an address");

    c = {};
    c.ready_when = nullptr;
    check(net::check_config(c, kW6100) == ConfigError::no_ready_predicate,
          "002: no ready predicate");
}

void tc_binds_providers_in_list_order() {
    FakeInterface iface;
    net::Ipv4LinkLocalProvider zeroconf{kMac, iface};
    net::StaticAddressProvider fixed{net::StaticAddressProvider::v4({10u, 0u, 0u, 5u}), iface};
    const net::MethodBinding bindings[] = {
        {Method::static_ipv4, &fixed},
        {Method::ipv4_link_local, &zeroconf},
    };

    net::AddressProvider* out[net::kMaxMethodsPerFamily] = {};
    std::size_t n =
        net::bind_methods({Method::ipv4_link_local, Method::static_ipv4}, bindings, 2u, out);
    check(n == 2u && out[0] == &zeroconf && out[1] == &fixed,
          "003: providers follow the config's order, not the backend's");

    n = net::bind_methods({Method::static_ipv4}, bindings, 2u, out);
    check(n == 1u && out[0] == &fixed, "003: an unlisted provider is left out");

    n = net::bind_methods({}, bindings, 2u, out);
    check(n == 0u, "003: an empty list selects nothing");
}

void tc_ready_predicates() {
    const net::AddressSnapshot none = snapshot({});
    const net::AddressSnapshot link_local = snapshot({169u, 254u, 17u, 78u});
    const net::AddressSnapshot leased = snapshot({192u, 168u, 32u, 79u});
    const net::AddressSnapshot v6_only =
        snapshot({}, {0xFEu, 0x80u, 0u, 0u, 0u, 0u, 0u, 0u, 0x02u, 0x08u, 0xDCu, 0xFFu, 0xFEu,
                      0x54u, 0x75u, 0x30u});

    check(!net::ready::any_v4(none) && net::ready::any_v4(link_local) && net::ready::any_v4(leased),
          "004: any_v4 accepts link-local and leased addresses");
    check(!net::ready::routable_v4(link_local) && net::ready::routable_v4(leased),
          "004: routable_v4 waits past a 169.254 address");
    check(!net::ready::any_address(none) && net::ready::any_address(v6_only)
              && !net::ready::any_v4(v6_only),
          "004: any_address accepts an IPv6-only board");
}

/// @brief Runs the IPv4 list a config selects until it is stable; returns the address the interface
/// got.
std::array<uint8_t, 16> run_config(const net::MethodList& methods) {
    FakeInterface iface;
    net::Ipv4LinkLocalProvider zeroconf{kMac, iface};
    net::StaticAddressProvider fixed{net::StaticAddressProvider::v4({10u, 0u, 0u, 5u}), iface};
    const net::MethodBinding bindings[] = {
        {Method::ipv4_link_local, &zeroconf},
        {Method::static_ipv4, &fixed},
    };
    net::AddressProvider* list[net::kMaxMethodsPerFamily] = {};
    const std::size_t count = net::bind_methods(methods, bindings, 2u, list);
    net::AddressManager mgr{list, count};
    mgr.begin();
    for (uint32_t now = 0u; now < 30000u && !mgr.stable(); now += 10u) {
        (void)mgr.poll(now);
    }

    return iface.last_address;
}

void tc_list_order_decides_the_address() {
    const std::array<uint8_t, 16> fixed_first =
        run_config({Method::static_ipv4, Method::ipv4_link_local});
    check(fixed_first[0] == 10u && fixed_first[3] == 5u, "005: static first: the static address");

    const std::array<uint8_t, 16> ll_first =
        run_config({Method::ipv4_link_local, Method::static_ipv4});
    check(ll_first[0] == 169u && ll_first[1] == 254u, "005: link-local first: a 169.254 address");
}

}  // namespace

int main() {
    tc_accepts_well_formed_configs();
    tc_rejects_each_defect();
    tc_binds_providers_in_list_order();
    tc_ready_predicates();
    tc_list_order_decides_the_address();
    return test_summary();
}
