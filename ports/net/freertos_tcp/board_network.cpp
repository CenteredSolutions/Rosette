// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file ports/net/freertos_tcp/board_network.cpp
 * @brief The board network on FreeRTOS+TCP; see rosette/net/board_network.hpp.
 *
 * FreeRTOS+TCP runs its own addressing in the IP task: DHCPv4 with an RFC
 * 3927-style link-local fallback (ipconfigDHCP_FALL_BACK_AUTO_IP), a fixed
 * address, and IPv6 router advertisement (ipconfigUSE_RA). Rosette's address
 * providers are therefore not used here. The board network maps the
 * firmware's NetworkConfig onto FreeRTOS+TCP endpoints, starts the IP task,
 * and reads the endpoints back into the AddressSnapshot every other backend
 * reports.
 *
 * Bring-up, once per program:
 *   1. the board registers its interface and reports its MAC
 *      (ports/net/freertos_tcp/board_interface.hpp);
 *   2. one IPv4 endpoint: DHCP with the link-local fallback, or the fixed
 *      address; and, when the config lists SLAAC, an fe80:: endpoint whose
 *      interface identifier is the MAC's modified EUI-64 and an endpoint that
 *      takes its prefix from router advertisements;
 *   3. FreeRTOS_IPInit_Multi() starts the IP task, which brings the link up
 *      and runs DHCP and RA on its own from then on.
 *
 * Which config the stack can run follows from how it runs DHCP. The
 * link-local fallback is compiled in, so a DHCP endpoint always falls back
 * to 169.254/16 when no server answers, and the fallback is the only way the
 * stack takes a link-local address. The IPv4 lists it runs are therefore
 * {dhcp, ipv4_link_local} and {static_ipv4}; every other list is refused with
 * ConfigError::unsupported_order. Once on link-local the stack does not
 * return to DHCP until the link goes down and up again, so
 * NetworkConfig::upgrade_probe_ms has no effect on this backend.
 *
 * The SNTP client runs as on the WIZnet boards, from NetworkConfig::time.
 * FreeRTOS+TCP does not report DHCP option 42, so the server is the
 * config's.
 *
 * Concurrency: the IP task writes the endpoints while the network thread
 * reads them in poll_board_network(). Each field read is a single aligned
 * word or a byte array the stack writes before marking the endpoint up, so a
 * snapshot taken mid-change reports the old or the new address, never a mix
 * that the stack has marked up. bring_up_board_network() and
 * poll_board_network() run in a task, after the scheduler has started:
 * bring-up sleeps while the IP task works.
 */

#include "rosette/net/board_network.hpp"

#include <array>
#include <cstdio>
#include <cstring>

extern "C" {
#include "FreeRTOS.h"
#include "FreeRTOS_IP.h"
#include "FreeRTOS_Routing.h"
#include "NetworkBufferManagement.h"
}

#include "ports/net/freertos_tcp/board_interface.hpp"
#include "ports/net/freertos_tcp/socket.hpp"
#include "rosette/net/inet.h"
#include "rosette/net/sntp_client.hpp"
#include "rosette/platform/platform.hpp"

/// Whether this build of the stack can run IPv6 SLAAC (router advertisements).
#define ROSETTE_FREERTOS_TCP_SLAAC ((ipconfigUSE_IPv6 != 0) && (ipconfigUSE_RA != 0))

namespace rosette::net::freertos {
namespace {

namespace pf = rosette::platform;

/// Interval between progress lines while bring-up waits.
constexpr uint32_t kProgressPeriodMs = 2000u;

/// The host name pcApplicationHostnameHook() gives DHCP (RFC 2132 §3.14).
const char* g_hostname = "rosette";

/// @brief The one socket provider; it holds no state, so it is valid before bring-up too.
FreeRtosSocketProvider& socket_provider() noexcept {
    static FreeRtosSocketProvider provider;
    return provider;
}

/// @brief Whether a list names exactly `first` then `second`, padded with Method::none.
bool lists_exactly(const MethodList& methods, Method first, Method second) noexcept {
    return methods[0] == first && methods[1] == second && methods[2] == Method::none;
}

/**
 * @brief Refuses the method lists FreeRTOS+TCP cannot run in the order given.
 *
 * check_config() has already confirmed that every listed method is one this
 * backend can run; this adds the stack's ordering rule (see the file
 * comment): IPv4 is {dhcp, ipv4_link_local} or {static_ipv4}, and IPv6 is
 * empty or {slaac}.
 *
 * @param[in] config A config check_config() accepted.
 * @return ConfigError::none, or ConfigError::unsupported_order.
 *
 * Verification:
 * - Correct means: the default config and a static-only config pass; {dhcp}
 *   alone, {ipv4_link_local, dhcp}, and {dhcp, ipv4_link_local, static_ipv4}
 *   are refused.
 * - Covered by: bring-up on the MIMXRT1020-EVK with the default config; the
 *   refusals have no test yet.
 */
ConfigError check_stack_order(const NetworkConfig& config) noexcept {
    const bool v4_ok = lists_exactly(config.v4, Method::dhcp, Method::ipv4_link_local)
                       || lists_exactly(config.v4, Method::static_ipv4, Method::none);
    const bool v6_ok = lists_exactly(config.v6, Method::none, Method::none)
                       || lists_exactly(config.v6, Method::slaac, Method::none);
    return (v4_ok && v6_ok) ? ConfigError::none : ConfigError::unsupported_order;
}

/// @brief A prefix length as the dotted netmask FreeRTOS_FillEndPoint() takes.
std::array<uint8_t, 4> netmask_octets(uint8_t prefix_len) noexcept {
    const uint32_t mask = prefix_len == 0u ? 0u : ~uint32_t{0} << (32u - prefix_len);
    return {static_cast<uint8_t>(mask >> 24), static_cast<uint8_t>(mask >> 16),
            static_cast<uint8_t>(mask >> 8), static_cast<uint8_t>(mask)};
}

/// @brief The number of leading one bits in a network-order netmask.
uint8_t prefix_len_of(uint32_t netmask_network_order) noexcept {
    return static_cast<uint8_t>(__builtin_popcount(netmask_network_order));
}

/**
 * @brief The fe80::/64 address whose interface identifier is the MAC's modified EUI-64.
 *
 * @param[in] mac The station address.
 * @return fe80::xxxx:xxff:fexx:xxxx, with the universal/local bit of the
 *   first octet inverted.
 *
 * References: RFC 4291 §2.5.6 and Appendix A; RFC 4862 §5.3.
 */
std::array<uint8_t, 16> eui64_link_local(const MacAddress& mac) noexcept {
    return {0xFEu,  0x80u,  0u,
            0u,     0u,     0u,
            0u,     0u,     static_cast<uint8_t>(mac[0] ^ 0x02u),
            mac[1], mac[2], 0xFFu,
            0xFEu,  mac[3], mac[4],
            mac[5]};
}

/// @brief The FreeRTOS+TCP objects bring-up creates, and the snapshot read back from them.
struct BoardNetwork {
    MacAddress          mac{};
    NetworkInterface_t* iface = nullptr;
    NetworkEndPoint_t   v4_endpoint{};
#if ROSETTE_FREERTOS_TCP_SLAAC
    NetworkEndPoint_t v6_link_local{};
    NetworkEndPoint_t v6_global{};
#endif
    bool            static_v4 = false;
    bool            started = false;
    SntpClient      sntp;
    ReadyPredicate  ready_when;
    AddressSnapshot snapshot{};

    explicit BoardNetwork(const NetworkConfig& config) noexcept
        : static_v4(lists_method(config.v4, Method::static_ipv4)),
          sntp(socket_provider(), config.time, wall_clock_for_sntp()),
          ready_when(config.ready_when) {}

    void start(const NetworkConfig& config) noexcept;
    void refresh_snapshot() noexcept;

    /// @brief One step: reads the endpoints, then gives the SNTP client its turn.
    void poll(uint32_t now_ms) noexcept {
        refresh_snapshot();
        if (started) {
            sntp.poll(now_ms, snapshot);
        }
    }

    /// @brief Whether the IP task is running and the ready predicate accepts the addresses.
    bool ready() const noexcept { return started && ready_when(snapshot); }

    /// @brief One `[net]` line naming what bring-up is waiting for.
    void print_progress() const noexcept {
        if (!started) {
            std::printf("[net] the IP task did not start\n");
        } else if (iface->pfGetPhyLinkStatus != nullptr && iface->pfGetPhyLinkStatus(iface) == 0) {
            std::printf("[net] waiting for PHY link\n");
        } else {
            std::printf("[net] waiting for an address\n");
        }
    }
};

/**
 * @brief Registers the board's interface, fills the endpoints the config asks for, and starts the
 * IP task.
 *
 * @param[in] config A config check_config() and check_stack_order() accepted.
 *
 * Verification:
 * - Correct means: with the default config the IPv4 endpoint asks for DHCP
 *   and, after a lease, reports the server's address; with a static config
 *   it reports `static_v4` without sending a DHCPDISCOVER; with SLAAC the
 *   board answers ping6 on its EUI-64 fe80:: address.
 * - Covered by: bring-up on the MIMXRT1020-EVK (DHCP lease from the bench's
 *   dnsmasq, ping, ping6).
 *
 * References:
 * - FreeRTOS+TCP V4 "Multiple Interfaces and Endpoints": FreeRTOS_FillEndPoint(),
 *   FreeRTOS_FillEndPoint_IPv6(), bWantDHCP, bWantRA, FreeRTOS_IPInit_Multi()
 */
void BoardNetwork::start(const NetworkConfig& config) noexcept {
    iface = add_board_network_interface(mac);
    if (iface == nullptr) {
        std::printf("[net] the board offers no network interface\n");
        return;
    }

    const StaticIpv4&            fixed = config.static_v4;
    const std::array<uint8_t, 4> none{};
    const std::array<uint8_t, 4> mask = netmask_octets(fixed.prefix_len);
    FreeRTOS_FillEndPoint(iface, &v4_endpoint, static_v4 ? fixed.address.data() : none.data(),
                          static_v4 ? mask.data() : none.data(),
                          static_v4 ? fixed.gateway.data() : none.data(), none.data(), mac.data());
    v4_endpoint.bits.bWantDHCP = static_v4 ? pdFALSE_UNSIGNED : pdTRUE_UNSIGNED;

#if ROSETTE_FREERTOS_TCP_SLAAC
    if (lists_method(config.v6, Method::slaac)) {
        IPv6_Address_t address{};
        IPv6_Address_t prefix{};
        const auto     link_local = eui64_link_local(mac);
        std::memcpy(address.ucBytes, link_local.data(), link_local.size());
        std::memcpy(prefix.ucBytes, link_local.data(), 8u);
        FreeRTOS_FillEndPoint_IPv6(iface, &v6_link_local, &address, &prefix, 64u, nullptr, nullptr,
                                   mac.data());

        // The router's prefix replaces the empty one when an advertisement arrives.
        const IPv6_Address_t unspecified{};
        FreeRTOS_FillEndPoint_IPv6(iface, &v6_global, &unspecified, &unspecified, 64u, nullptr,
                                   nullptr, mac.data());
        v6_global.bits.bWantRA = pdTRUE_UNSIGNED;
    }
#endif

    started = FreeRTOS_IPInit_Multi() == pdPASS;
    std::printf("[net] FreeRTOS+TCP on %02x:%02x:%02x:%02x:%02x:%02x, IPv4 by %s\n", mac[0], mac[1],
                mac[2], mac[3], mac[4], mac[5], static_v4 ? "static address" : "DHCP");
}

/**
 * @brief Rebuilds the snapshot from the endpoints the stack has marked up.
 *
 * Each up endpoint with a non-zero address becomes one entry of its family.
 * The IPv4 source is `manual` for a fixed address, `zeroconf` for a
 * 169.254/16 address, and `dhcp` otherwise; IPv6 entries are `slaac`. A
 * router-discovery endpoint counts only once a router has answered: when
 * none does, FreeRTOS+TCP marks it up anyway (eRAStateFailed) on its default
 * settings, an unspecified prefix with a random interface ID, which is no
 * SLAAC address (RFC 4862 §5.5.3 forms one only from an advertised prefix).
 *
 * Verification:
 * - Correct means: every listed address is one an up endpoint holds, an
 *   endpoint going down (link loss) removes its address on the next poll,
 *   and on a link with no router only the fe80:: address is listed.
 * - Covered by: bring-up on the MIMXRT1020-EVK (the `[net]` address lines
 *   match the DHCP server's lease table; tools/udp_echo_probe.py reads the
 *   list from the status port and reaches every address on it).
 *
 * References: RFC 4862 §5.5.3 (prefix information); FreeRTOS+TCP
 * FreeRTOS_RA.c, xRAProcess_HandleWaitStates().
 */
void BoardNetwork::refresh_snapshot() noexcept {
    snapshot.active = started;
    snapshot.v4.addresses.clear();
    snapshot.v6.addresses.clear();
    for (NetworkEndPoint_t* ep = FreeRTOS_FirstEndPoint(iface); ep != nullptr;
         ep = FreeRTOS_NextEndPoint(iface, ep)) {
        if (FreeRTOS_IsEndPointUp(ep) == pdFALSE) {
            continue;
        }

        std::array<uint8_t, 16> address{};
        if (ENDPOINT_IS_IPv4(ep)) {
            const uint32_t host = FreeRTOS_ntohl(ep->ipv4_settings.ulIPAddress);
            if (host == 0u) {
                continue;
            }

            address = {static_cast<uint8_t>(host >> 24), static_cast<uint8_t>(host >> 16),
                       static_cast<uint8_t>(host >> 8), static_cast<uint8_t>(host)};
            (void)snapshot.v4.addresses.add(AddressFamily::ipv4, address,
                                            prefix_len_of(ep->ipv4_settings.ulNetMask));
        }
#if ROSETTE_FREERTOS_TCP_SLAAC
        else {
            // With no router heard, the stack still brings the endpoint up on its fallback
            // settings: no prefix and a random interface ID, which nothing on the link reaches.
            if (ep->bits.bWantRA != 0u && ep->xRAData.bits.bRouterReplied == 0u) {
                continue;
            }

            std::memcpy(address.data(), ep->ipv6_settings.xIPAddress.ucBytes, address.size());
            if (address != std::array<uint8_t, 16>{}) {
                (void)snapshot.v6.addresses.add(
                    AddressFamily::ipv6, address,
                    static_cast<uint8_t>(ep->ipv6_settings.uxPrefixLength));
            }
        }
#endif
    }

    const bool v4_held = snapshot.v4.has_address();
    snapshot.v4.state =
        v4_held ? AddressPhase::bound : (started ? AddressPhase::acquiring : AddressPhase::idle);
    snapshot.v4.source = static_v4                               ? AddressSource::manual
                         : (v4_held && snapshot.v4.link_local()) ? AddressSource::zeroconf
                                                                 : AddressSource::dhcp;
    snapshot.v6.source = AddressSource::slaac;
    snapshot.v6.state = snapshot.v6.has_address() ? AddressPhase::bound : AddressPhase::idle;
    snapshot.assigned = v4_held || snapshot.v6.has_address();
}

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
 * @param[in,out] network     The board network, started.
 * @param[in]     timeout_ms  Longest wait; pf::kWaitForever waits for ever.
 * @return `ready` or `timed_out`.
 */
BringUpResult wait_until_ready(BoardNetwork& network, uint32_t timeout_ms) noexcept {
    const uint32_t start = pf::monotonic_ms();
    uint32_t       next_note = start + kProgressPeriodMs;
    for (;;) {
        const uint32_t now = pf::monotonic_ms();
        network.poll(now);
        if (network.ready()) {
            print_addresses(network.snapshot);
            return BringUpResult::ready;
        }

        if (timeout_ms != pf::kWaitForever && now - start >= timeout_ms) {
            std::printf("[net] not ready after %lu ms; still trying\n",
                        static_cast<unsigned long>(timeout_ms));
            return BringUpResult::timed_out;
        }

        if (static_cast<int32_t>(now - next_note) >= 0) {
            next_note += kProgressPeriodMs;
            network.print_progress();
        }

        pf::sleep_ms(50u);
    }
}

BoardNetwork* g_network = nullptr;

}  // namespace
}  // namespace rosette::net::freertos

namespace rosette::net {

using freertos::BoardNetwork;
using freertos::g_network;

NetworkCapabilities board_network_capabilities() noexcept {
    NetworkCapabilities capabilities{};
    capabilities.v4 = {Method::dhcp, Method::ipv4_link_local, Method::static_ipv4};
#if ROSETTE_FREERTOS_TCP_SLAAC
    capabilities.v6 = {Method::slaac};
#endif
    return capabilities;
}

NetworkConfig board_default_network() noexcept {
    NetworkConfig config{};
#if ROSETTE_FREERTOS_TCP_SLAAC
    config.v6 = {Method::slaac};
#endif
    return config;
}

BringUpResult bring_up_board_network(const char* hostname, const NetworkConfig& config) noexcept {
    if (g_network != nullptr) {
        return g_network->ready() ? BringUpResult::ready : BringUpResult::timed_out;
    }

    ConfigError error = check_config(config, board_network_capabilities());
    if (error == ConfigError::none) {
        error = freertos::check_stack_order(config);
    }

    if (error != ConfigError::none) {
        std::printf("[net] network config refused: %s\n", describe(error));
        return BringUpResult::unsupported_config;
    }

    if (hostname != nullptr) {
        freertos::g_hostname = hostname;
    }

    static BoardNetwork network(config);
    g_network = &network;
    network.start(config);
    return freertos::wait_until_ready(network, config.ready_timeout_ms);
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
    return freertos::socket_provider();
}

uint32_t board_ipv4_address() noexcept {
    if (g_network == nullptr || !g_network->snapshot.v4.has_address()) {
        return 0u;
    }

    const auto& a = g_network->snapshot.v4.address();
    return (uint32_t{a[0]} << 24) | (uint32_t{a[1]} << 16) | (uint32_t{a[2]} << 8) | uint32_t{a[3]};
}

const AddressSnapshot& board_address_snapshot() noexcept {
    static const AddressSnapshot kNone{};
    return g_network != nullptr ? g_network->snapshot : kNone;
}

uint32_t board_send_timeouts() noexcept {
    return 0u;
}

NetworkBufferUse board_network_buffer_use() noexcept {
    const uint32_t count = ipconfigNUM_NETWORK_BUFFER_DESCRIPTORS;
    return {count, count - static_cast<uint32_t>(uxGetMinimumFreeNetworkBuffers())};
}

}  // namespace rosette::net

/// @brief The host name FreeRTOS+TCP's DHCP client registers: bring_up_board_network()'s.
extern "C" const char* pcApplicationHostnameHook(void) {
    return rosette::net::freertos::g_hostname;
}
