/**
 * @file src/net/wiznet/board_network.cpp
 * @brief The board's WIZnet network; see rosette/net/wiznet/board_network.hpp.
 *
 * The chip objects live in one struct, constructed on the first
 * bring_up_board_network() call, so their construction order (bus, device,
 * sockets, link, addressing) is explicit and each lives for the whole
 * program; WiznetDevice holds its bus by reference.
 *
 * IPv6 addressing exists only on parts with hardware IPv6: `BoardIpv6` is an
 * empty struct on the W5100S and a SLAAC provider with its own AddressManager
 * on the W6100 / W6300, chosen at compile time from `Chip::has_hw_ipv6()`.
 */

#include "rosette/net/wiznet/board_network.hpp"

#include <array>
#include <cstdio>

#include "board.h"
#include "rosette/net/address.hpp"
#include "rosette/net/inet.h"
#include "rosette/net/ipv4ll_provider.hpp"
#include "rosette/net/wiznet/board.hpp"
#include "rosette/net/wiznet/dhcp_provider.hpp"
#include "rosette/net/wiznet/interface.hpp"
#include "rosette/net/wiznet/link.hpp"
#include "rosette/net/wiznet/rx_interrupt_pico.hpp"
#include "rosette/net/wiznet/slaac_provider.hpp"
#include "rosette/platform/platform.hpp"

namespace rosette::net::wiznet {
namespace {

namespace pf = rosette::platform;

/// Interval between progress lines while waiting for link or an address.
constexpr uint32_t kProgressPeriodMs = 2000u;

/// @brief IPv6 addressing for a part without hardware IPv6: none.
template <typename Chip, bool = Chip::has_hw_ipv6()>
struct BoardIpv6 {
    BoardIpv6(BoardDevice&, BoardSocketProvider&, NetInterface&,
              const std::array<uint8_t, 6>&) noexcept {}

    AddressManager* manager() noexcept { return nullptr; }
};

/// @brief IPv6 addressing on the W6100 / W6300: SLAAC, one provider.
template <typename Chip>
struct BoardIpv6<Chip, true> {
    WiznetSlaacProvider<Chip, BoardBus> slaac;
    AddressProvider* providers[1];
    AddressManager v6;

    BoardIpv6(BoardDevice& device, BoardSocketProvider& sockets, NetInterface& iface,
              const std::array<uint8_t, 6>& mac) noexcept
        : slaac(device, sockets, iface, mac), providers{&slaac}, v6(providers) {}

    AddressManager* manager() noexcept { return &v6; }
};

/// @brief The board's chip driver and addressing, in construction order.
struct BoardNetwork {
    std::array<uint8_t, 6> mac;
    BoardBus bus;
    BoardDevice device;
    BoardSocketProvider sockets;
    BoardLink link;
    WiznetInterface<BoardChip, BoardBus> iface;
    DhcpAddressProvider<BoardChip, BoardBus> dhcp;
    Ipv4LinkLocalProvider zeroconf;
    AddressProvider* providers[2];
    AddressManager v4;
    BoardIpv6<BoardChip> ipv6;
    NetworkAddress address;

    /// @brief The board's MAC address (board_get_mac_address()).
    static std::array<uint8_t, 6> board_mac() noexcept {
        std::array<uint8_t, 6> mac{};
        board_get_mac_address(mac.data());
        return mac;
    }

    explicit BoardNetwork(const char* hostname) noexcept
        : mac(board_mac()),
          bus(make_board_bus()),
          device(bus),
          sockets(device),
          link(device, sockets, mac),
          iface(device),
          dhcp(device, sockets, iface, dhcp_config(hostname)),
          zeroconf(mac, iface),
          providers{&dhcp, &zeroconf},
          v4(providers),
          ipv6(device, sockets, iface, mac),
          address(v4, ipv6.manager(), &iface) {}
};

BoardNetwork* g_network = nullptr;

}  // namespace

void bring_up_board_network(const char* hostname) noexcept {
    if (g_network != nullptr) return;
    static BoardNetwork network(hostname);

    while (!network.link.bring_link_up()) {
        std::printf("[net] waiting for chip / PHY link\n");
        pf::sleep_ms(kProgressPeriodMs);
    }
    std::printf("[net] PHY link up\n");

    network.address.begin();
    uint32_t next_note = pf::monotonic_ms() + kProgressPeriodMs;
    while (!network.address.snapshot().v4.has_address()) {
        network.address.poll(pf::monotonic_ms());
        if (static_cast<int32_t>(pf::monotonic_ms() - next_note) >= 0) {
            next_note += kProgressPeriodMs;
            std::printf("[net] waiting for an address (DHCP, then link-local)\n");
        }
        pf::sleep_ms(10u);
    }

    const auto& a = network.address.snapshot().v4.address();
    std::printf("[net] address %u.%u.%u.%u\n", a[0], a[1], a[2], a[3]);

    // Link-local SLAAC binds within about a second of DAD; a global address
    // follows only if a router advertises a prefix, so poll_board_network()
    // keeps the manager running and the global address is reported later.
    while (network.ipv6.manager() != nullptr && !network.address.snapshot().v6.has_address()) {
        network.address.poll(pf::monotonic_ms());
        pf::sleep_ms(10u);
    }

    for (const AddressEntry& entry : network.address.snapshot().v6.addresses) {
        char text[48];
        if (inet::ntop6(entry.address.data(), text, sizeof(text)) != nullptr) {
            std::printf("[net] IPv6 address %s/%u\n", text, entry.prefix_len);
        }
    }

    attach_rx_interrupt(ROSETTE_PIN_WIZNET_INT, network.sockets);
    g_network = &network;
}

void poll_board_network(uint32_t now_ms) noexcept {
    if (g_network != nullptr) g_network->address.poll(now_ms);
}

rosette::net::SocketProvider& board_socket_provider() noexcept {
    return g_network->sockets;
}

uint32_t board_ipv4_address() noexcept {
    if (g_network == nullptr) return 0u;
    const auto& a = g_network->address.snapshot().v4.address();
    return (uint32_t{a[0]} << 24) | (uint32_t{a[1]} << 16) | (uint32_t{a[2]} << 8) | uint32_t{a[3]};
}

const AddressSnapshot& board_address_snapshot() noexcept {
    static const AddressSnapshot kNone{};
    return g_network != nullptr ? g_network->address.snapshot() : kNone;
}

uint32_t board_send_timeouts() noexcept {
    return g_network != nullptr ? g_network->device.send_timeouts() : 0u;
}

}  // namespace rosette::net::wiznet
