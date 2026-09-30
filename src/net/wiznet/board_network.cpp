/**
 * @file src/net/wiznet/board_network.cpp
 * @brief The board's WIZnet network; see rosette/net/wiznet/board_network.hpp.
 *
 * The chip objects and every provider the board can run live in one struct,
 * constructed on the first accepted bring_up_board_network() call, so their
 * construction order (bus, device, sockets, link, providers, managers) is
 * explicit and each lives for the whole program; WiznetDevice holds its bus
 * by reference. The config decides which providers the managers run and in
 * what order (bind_methods()); the others are constructed but never started.
 *
 * IPv6 addressing exists only on parts with hardware IPv6: `BoardIpv6` is an
 * empty struct on the W5100S and a SLAAC provider with its own AddressManager
 * on the W6100 / W6300, chosen at compile time from `Chip::has_hw_ipv6()`.
 *
 * `BoardNetwork::poll()` is the one step both bring-up and
 * poll_board_network() take: it retries the chip until it answers, then runs
 * the addresses and the SNTP client that keeps the wall clock on UTC.
 * Bring-up is that step in a loop with a predicate and a deadline, which is
 * why a bring-up that times out loses nothing.
 */

#include "rosette/net/wiznet/board_network.hpp"

#include <array>
#include <cstdio>

#include "board.h"
#include "rosette/net/address.hpp"
#include "rosette/net/inet.h"
#include "rosette/net/ipv4ll_provider.hpp"
#include "rosette/net/sntp_client.hpp"
#include "rosette/net/static_provider.hpp"
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

/// Interval between progress lines while bring-up waits.
constexpr uint32_t kProgressPeriodMs = 2000u;

/// Interval between attempts to bring up a chip that did not answer.
constexpr uint32_t kChipRetryMs = 1000u;

bool due(uint32_t deadline_ms, uint32_t now_ms) noexcept {
    return static_cast<int32_t>(now_ms - deadline_ms) >= 0;
}

/// @brief IPv6 addressing for a part without hardware IPv6: none.
template <typename Chip, bool = Chip::has_hw_ipv6()>
struct BoardIpv6 {
    BoardIpv6(BoardDevice&, BoardSocketProvider&, NetInterface&, const std::array<uint8_t, 6>&,
              const MethodList&) noexcept {}

    AddressManager* manager() noexcept { return nullptr; }
};

/// @brief IPv6 addressing on the W6100 / W6300: SLAAC when the config lists it.
template <typename Chip>
struct BoardIpv6<Chip, true> {
    WiznetSlaacProvider<Chip, BoardBus> slaac;
    AddressProvider* providers[kMaxMethodsPerFamily]{};
    std::size_t count;
    AddressManager v6;

    BoardIpv6(BoardDevice& device, BoardSocketProvider& sockets, NetInterface& iface,
              const std::array<uint8_t, 6>& mac, const MethodList& methods) noexcept
        : slaac(device, sockets, iface, mac), count(bind(methods)), v6(providers, count) {}

    std::size_t bind(const MethodList& methods) noexcept {
        const MethodBinding bindings[] = {{Method::slaac, &slaac}};
        return bind_methods(methods, bindings, 1u, providers);
    }

    AddressManager* manager() noexcept { return count > 0u ? &v6 : nullptr; }
};

/// @brief The manager policy a config asks for.
AddressManagerPolicy policy_for(const NetworkConfig& config) noexcept {
    AddressManagerPolicy policy{};
    policy.upgrade_probe_ms = config.upgrade_probe_ms;
    return policy;
}

/// @brief The chip driver, every provider the board can run, and the managers the config selects.
struct BoardNetwork {
    std::array<uint8_t, 6> mac;
    BoardBus bus;
    BoardDevice device;
    BoardSocketProvider sockets;
    BoardLink link;
    WiznetInterface<BoardChip, BoardBus> iface;
    DhcpAddressProvider dhcp;
    Ipv4LinkLocalProvider zeroconf;
    StaticAddressProvider fixed;
    AddressProvider* v4_providers[kMaxMethodsPerFamily]{};
    std::size_t v4_count;
    AddressManager v4;
    BoardIpv6<BoardChip> ipv6;
    NetworkAddress address;
    SntpClient sntp;
    ReadyPredicate ready_when;
    bool chip_ready = false;
    uint32_t chip_retry_ms;

    /// @brief The board's MAC address (board_get_mac_address()).
    static std::array<uint8_t, 6> board_mac() noexcept {
        std::array<uint8_t, 6> mac{};
        board_get_mac_address(mac.data());
        return mac;
    }

    /// @brief The static provider's settings from the config's `static_v4`.
    static StaticAddressProvider::Config static_config(const StaticIpv4& fixed) noexcept {
        return StaticAddressProvider::v4(fixed.address, fixed.prefix_len, fixed.gateway);
    }

    BoardNetwork(const char* hostname, const NetworkConfig& config) noexcept
        : mac(board_mac()),
          bus(make_board_bus()),
          device(bus),
          sockets(device),
          link(device, mac),
          iface(device),
          dhcp(sockets, mac, iface, dhcp_config(hostname)),
          zeroconf(mac, iface),
          fixed(static_config(config.static_v4), iface),
          v4_count(bind_v4(config.v4)),
          v4(v4_providers, v4_count, policy_for(config)),
          ipv6(device, sockets, iface, mac, config.v6),
          address(v4, ipv6.manager(), &iface),
          sntp(sockets, config.time, wall_clock_for_sntp()),
          ready_when(config.ready_when),
          chip_retry_ms(pf::monotonic_ms()) {
        // Before the link's bring-up runs init_socket_mode(); kBoardRxLayoutKib
        // is checked at compile time, so this always takes.
        (void)device.set_rx_layout(kBoardRxLayoutKib);
    }

    /// @brief Fills `v4_providers` in the order the config lists IPv4 methods.
    std::size_t bind_v4(const MethodList& methods) noexcept {
        const MethodBinding bindings[] = {
            {Method::dhcp, &dhcp},
            {Method::ipv4_link_local, &zeroconf},
            {Method::static_ipv4, &fixed},
        };
        return bind_methods(methods, bindings, 3u, v4_providers);
    }

    /**
     * @brief One step of bringing the network up and keeping it up.
     *
     * Until the chip answers, retries bring_chip_up() every kChipRetryMs;
     * once it does, routes the INT pin and starts the address managers,
     * which from then on follow carrier (NetworkAddress), then gives the
     * SNTP client its turn. Blocks only while an SNTP exchange is in flight
     * (at most `SntpConfig::reply_timeout_ms`, once per poll interval).
     *
     * @param[in] now_ms Monotonic milliseconds.
     */
    void poll(uint32_t now_ms) noexcept {
        if (!chip_ready) {
            if (!due(chip_retry_ms, now_ms)) {
                return;
            }

            chip_retry_ms = now_ms + kChipRetryMs;
            if (!link.bring_chip_up()) {
                return;
            }

            chip_ready = true;
            std::printf("[net] chip ready\n");
            attach_rx_interrupt(ROSETTE_PIN_WIZNET_INT, sockets);
            address.begin();
        }

        address.poll(now_ms);
        sntp.poll(now_ms, address.snapshot());
    }

    /// @brief Whether the config's ready predicate accepts the current addresses.
    bool ready() const noexcept { return chip_ready && ready_when(address.snapshot()); }

    /// @brief One `[net]` line naming what bring-up is waiting for.
    void print_progress() const noexcept {
        if (!chip_ready) {
            std::printf("[net] waiting for the chip to answer\n");
        } else if (!address.link_up()) {
            std::printf("[net] waiting for PHY link\n");
        } else {
            std::printf("[net] waiting for an address\n");
        }
    }
};

/// @brief Prints every address the board holds, one `[net]` line each.
void print_addresses(const AddressSnapshot& snapshot) noexcept {
    for (const AddressEntry& entry : snapshot.v4.addresses) {
        const auto& a = entry.address;
        std::printf("[net] address %u.%u.%u.%u/%u\n", a[0], a[1], a[2], a[3], entry.prefix_len);
    }

    for (const AddressEntry& entry : snapshot.v6.addresses) {
        char text[48];
        if (inet::ntop6(entry.address.data(), text, sizeof(text)) != nullptr) {
            std::printf("[net] IPv6 address %s/%u\n", text, entry.prefix_len);
        }
    }
}

/**
 * @brief Steps the network until its ready predicate holds or the timeout passes.
 *
 * @param[in,out] network     The board network, constructed.
 * @param[in]     timeout_ms  Longest wait; pf::kWaitForever waits for ever.
 * @return `ready` or `timed_out`.
 */
BringUpResult wait_until_ready(BoardNetwork& network, uint32_t timeout_ms) noexcept {
    const uint32_t start = pf::monotonic_ms();
    uint32_t next_note = start + kProgressPeriodMs;
    for (;;) {
        const uint32_t now = pf::monotonic_ms();
        network.poll(now);
        if (network.ready()) {
            print_addresses(network.address.snapshot());
            return BringUpResult::ready;
        }

        if (timeout_ms != pf::kWaitForever && now - start >= timeout_ms) {
            std::printf("[net] not ready after %lu ms; still trying\n",
                        static_cast<unsigned long>(timeout_ms));
            return BringUpResult::timed_out;
        }

        if (due(next_note, now)) {
            next_note += kProgressPeriodMs;
            network.print_progress();
        }

        pf::sleep_ms(10u);
    }
}

BoardNetwork* g_network = nullptr;

}  // namespace

NetworkCapabilities board_network_capabilities() noexcept {
    NetworkCapabilities capabilities{};
    capabilities.v4 = {Method::dhcp, Method::ipv4_link_local, Method::static_ipv4};
    if constexpr (BoardChip::has_hw_ipv6()) {
        capabilities.v6 = {Method::slaac};
    }

    return capabilities;
}

NetworkConfig board_default_network() noexcept {
    NetworkConfig config{};
    if constexpr (BoardChip::has_hw_ipv6()) {
        config.v6 = {Method::slaac};
    }

    return config;
}

BringUpResult bring_up_board_network(const char* hostname, const NetworkConfig& config) noexcept {
    if (g_network != nullptr) {
        return g_network->ready() ? BringUpResult::ready : BringUpResult::timed_out;
    }

    const ConfigError error = check_config(config, board_network_capabilities());
    if (error != ConfigError::none) {
        std::printf("[net] network config refused: %s\n", describe(error));
        return BringUpResult::unsupported_config;
    }

    static BoardNetwork network(hostname, config);
    g_network = &network;
    return wait_until_ready(network, config.ready_timeout_ms);
}

void poll_board_network(uint32_t now_ms) noexcept {
    if (g_network != nullptr) {
        g_network->poll(now_ms);
    }
}

bool board_network_ready() noexcept {
    return g_network != nullptr && g_network->ready();
}

NetworkWatch board_network_watch() noexcept {
    return {poll_board_network, board_network_ready, board_address_snapshot};
}

rosette::net::SocketProvider& board_socket_provider() noexcept {
    return g_network->sockets;
}

uint32_t board_ipv4_address() noexcept {
    if (g_network == nullptr) {
        return 0u;
    }

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
