/**
 * @file net/wiznet/w5x00_link.cpp
 * @brief Implementation of WIZnet chip bring-up and DHCP.
 *
 * `begin()` runs a small state machine: size the chip's 16 KiB of TX/RX memory
 * across the sockets, wait for the PHY to report link, then either drive the
 * ioLibrary DHCP client to a lease or apply the static fallback. The assigned
 * address is pushed to `rosette_platform_set_ipv4_addr()` so the rest of the
 * firmware can gate on "network up".
 *
 * References:
 * - WIZnet ioLibrary `wizchip_init()`, `wizchip_setnetinfo()`, `ctlwizchip()`
 * - WIZnet ioLibrary `Internet/DHCP/dhcp.c`: `DHCP_init` / `DHCP_run` / `DHCP_time_handler`
 */

#include "rosette/net/wiznet/w5x00_link.hpp"

#include "rosette/platform/host_runtime.hpp"

extern "C" {
#include "wizchip_conf.h"
#include "dhcp.h"
}

#include <cstring>

namespace rosette::net::wiznet {

namespace {

// 2 KiB TX + 2 KiB RX per socket on a 4-socket W5100S (8 KiB each bank).
constexpr uint8_t kTxSizes[8] = {2u, 2u, 2u, 2u, 0u, 0u, 0u, 0u};
constexpr uint8_t kRxSizes[8] = {2u, 2u, 2u, 2u, 0u, 0u, 0u, 0u};

constexpr uint32_t kPhyLinkTimeoutMs = 5000u;
constexpr uint32_t kDhcpTimeoutMs = 15000u;

// DHCP work buffer; one datagram's worth is plenty for the client.
uint8_t g_dhcp_buffer[548];

uint32_t to_host_order(const std::array<uint8_t, 4>& ip) noexcept
{
    return (static_cast<uint32_t>(ip[0]) << 24) | (static_cast<uint32_t>(ip[1]) << 16) |
           (static_cast<uint32_t>(ip[2]) << 8) | static_cast<uint32_t>(ip[3]);
}

}  // namespace

WiznetLink::WiznetLink(const std::array<uint8_t, 6>& mac,
                       LinkMode mode,
                       const StaticAddress& fallback,
                       uint8_t dhcp_socket) noexcept
    : mac_(mac), mode_(mode), fallback_(fallback), dhcp_socket_(dhcp_socket)
{}

void WiznetLink::apply_netinfo(const std::array<uint8_t, 4>& ip,
                               const std::array<uint8_t, 4>& sn,
                               const std::array<uint8_t, 4>& gw,
                               const std::array<uint8_t, 4>& dns,
                               bool from_dhcp) noexcept
{
    wiz_NetInfo info{};
    std::memcpy(info.mac, mac_.data(), 6u);
    std::memcpy(info.ip, ip.data(), 4u);
    std::memcpy(info.sn, sn.data(), 4u);
    std::memcpy(info.gw, gw.data(), 4u);
    std::memcpy(info.dns, dns.data(), 4u);
    info.dhcp = from_dhcp ? NETINFO_DHCP : NETINFO_STATIC;
    wizchip_setnetinfo(&info);

    ip_ = ip;
    ready_ = (ip[0] != 0u) || (ip[1] != 0u) || (ip[2] != 0u) || (ip[3] != 0u);
    rosette_platform_set_ipv4_addr(ready_ ? to_host_order(ip) : 0u);
}

bool WiznetLink::begin() noexcept
{
    if (ready_) {
        return true;
    }

    if (!chip_ready_) {
        if (wizchip_init(const_cast<uint8_t*>(kTxSizes),
                         const_cast<uint8_t*>(kRxSizes)) != 0) {
            return false;
        }
        chip_ready_ = true;
    }

    // Wait for PHY link.
    const uint32_t phy_start = rosette_platform_get_ms();
    for (;;) {
        uint8_t link = 0u;
        ctlwizchip(CW_GET_PHYLINK, &link);
        if (link == PHY_LINK_ON) {
            break;
        }
        if ((rosette_platform_get_ms() - phy_start) >= kPhyLinkTimeoutMs) {
            return false;
        }
        rosette_platform_delay_ms(100u);
    }

    if (mode_ == LinkMode::static_ip) {
        apply_netinfo(fallback_.ip, fallback_.netmask, fallback_.gateway,
                      fallback_.dns, false);
        return ready_;
    }

    // DHCP: set the MAC first so DISCOVER goes out with it, then run the client.
    apply_netinfo({0u, 0u, 0u, 0u}, {0u, 0u, 0u, 0u}, {0u, 0u, 0u, 0u},
                  {0u, 0u, 0u, 0u}, true);
    DHCP_init(dhcp_socket_, g_dhcp_buffer);
    dhcp_started_ = true;
    last_tick_ms_ = rosette_platform_get_ms();

    const uint32_t dhcp_start = rosette_platform_get_ms();
    for (;;) {
        poll();
        if (ready_) {
            return true;
        }
        if ((rosette_platform_get_ms() - dhcp_start) >= kDhcpTimeoutMs) {
            // Give up on DHCP; use the static fallback so the board still boots.
            apply_netinfo(fallback_.ip, fallback_.netmask, fallback_.gateway,
                          fallback_.dns, false);
            return ready_;
        }
        rosette_platform_delay_ms(10u);
    }
}

void WiznetLink::poll() noexcept
{
    if (!dhcp_started_) {
        return;
    }

    const uint32_t now = rosette_platform_get_ms();
    if ((now - last_tick_ms_) >= 1000u) {
        last_tick_ms_ = now;
        DHCP_time_handler();
    }

    const uint8_t state = DHCP_run();
    if (state == DHCP_IP_ASSIGN || state == DHCP_IP_CHANGED) {
        std::array<uint8_t, 4> ip{}, sn{}, gw{}, dns{};
        getIPfromDHCP(ip.data());
        getSNfromDHCP(sn.data());
        getGWfromDHCP(gw.data());
        getDNSfromDHCP(dns.data());
        apply_netinfo(ip, sn, gw, dns, true);
    }
}

}  // namespace rosette::net::wiznet
