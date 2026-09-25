/**
 * @file examples/rclc_chatter/wiznet_network.cpp
 * @brief WIZnet bring-up and rmw_rosette's network port; see wiznet_network.hpp.
 *
 * The chip objects are function-scope statics in one place so their
 * construction order (bus, device, sockets, link, addressing) is explicit and
 * each lives for the whole program, as WiznetDevice holds its bus by reference.
 */

#include "wiznet_network.hpp"

#include <array>
#include <cstdio>

#include "board.h"
#include "rmw_rosette/network_port.hpp"
#include "rosette/net/address.hpp"
#include "rosette/net/ipv4ll_provider.hpp"
#include "rosette/net/wiznet/board.hpp"
#include "rosette/net/wiznet/dhcp_provider.hpp"
#include "rosette/net/wiznet/interface.hpp"
#include "rosette/net/wiznet/link.hpp"
#include "rosette/net/wiznet/rx_interrupt_pico.hpp"
#include "rosette/platform/platform.hpp"

#include "pico/stdlib.h"

namespace {

namespace wiz = rosette::net::wiznet;
namespace net = rosette::net;
namespace pf = rosette::platform;

/// @brief The board's chip objects, constructed on first use.
struct WiznetStack {
    std::array<uint8_t, 6> mac;
    wiz::BoardBus bus;
    wiz::BoardDevice device;
    wiz::BoardSocketProvider sockets;
    wiz::BoardLink link;
    wiz::WiznetInterface<wiz::BoardChip, wiz::BoardBus> iface;
    wiz::DhcpAddressProvider<wiz::BoardChip, wiz::BoardBus> dhcp;
    net::Ipv4LinkLocalProvider zeroconf;
    net::AddressProvider* providers[2];
    net::AddressManager v4;
    net::NetworkAddress address;

    static std::array<uint8_t, 6> board_mac() noexcept {
        std::array<uint8_t, 6> mac{};
        board_get_mac_address(mac.data());
        return mac;
    }

    explicit WiznetStack(const char* hostname) noexcept
        : mac(board_mac()),
          bus(wiz::make_board_bus()),
          device(bus),
          sockets(device),
          link(device, sockets, mac),
          iface(device),
          dhcp(device, sockets, iface, wiz::dhcp_config(hostname)),
          zeroconf(mac, iface),
          providers{&dhcp, &zeroconf},
          v4(providers),
          address(v4, &iface) {}
};

WiznetStack* g_stack = nullptr;

}  // namespace

namespace rclc_chatter {

void bring_up_wiznet_network(const char* hostname) noexcept {
    static WiznetStack stack(hostname);
    g_stack = &stack;

    while (!stack.link.bring_link_up()) {
        std::printf("[net] waiting for chip / PHY link\n");
        sleep_ms(2000);
    }
    std::printf("[net] PHY link up\n");

    stack.address.begin();
    uint32_t next_note = pf::monotonic_ms() + 2000u;
    while (!stack.address.has_address()) {
        stack.address.poll(pf::monotonic_ms());
        if (static_cast<int32_t>(pf::monotonic_ms() - next_note) >= 0) {
            next_note += 2000u;
            std::printf("[net] waiting for an address (DHCP, then link-local)\n");
        }
        sleep_ms(10);
    }
    const auto a = stack.address.snapshot().v4.address;
    std::printf("[net] address %u.%u.%u.%u\n", a[0], a[1], a[2], a[3]);

    wiz::attach_rx_interrupt(ROSETTE_PIN_W5100S_INT, stack.sockets);
}

void poll_wiznet_network(uint32_t now_ms) noexcept {
    if (g_stack != nullptr) g_stack->address.poll(now_ms);
}

uint32_t wiznet_send_timeouts() noexcept {
    return g_stack != nullptr ? g_stack->device.send_timeouts() : 0u;
}

}  // namespace rclc_chatter

namespace rmw_rosette {

/**
 * @brief The WIZnet chip's hardware UDP sockets.
 *
 * @return The board's socket provider. Valid only after
 *   rclc_chatter::bring_up_wiznet_network(); rcl_init() runs after it.
 */
rosette::net::SocketProvider& network_socket_provider() noexcept {
    return g_stack->sockets;
}

/**
 * @brief The address DHCP or IPv4 link-local assigned.
 *
 * @return Address in host byte order, or 0 before bring-up.
 */
uint32_t network_ipv4_address() noexcept {
    if (g_stack == nullptr) return 0u;
    const auto a = g_stack->address.snapshot().v4.address;
    return (uint32_t{a[0]} << 24) | (uint32_t{a[1]} << 16) | (uint32_t{a[2]} << 8) | uint32_t{a[3]};
}

}  // namespace rmw_rosette
