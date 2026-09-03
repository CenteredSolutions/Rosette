/**
 * @file net/wiznet/link.hpp
 * @brief WIZnet chip bring-up: reset, buffer sizing, PHY link, addressing.
 *
 * Register-level replacement for the old ioLibrary `WiznetLink`. Built on
 * `WiznetDevice<Chip, Bus>` and a `WiznetSocketProvider`, so the same code path
 * serves the poll-mode (no-RTOS) board and, with a FreeRTOS executor, an RTOS
 * build.
 *
 *     namespace wiz = rosette::net::wiznet;
 *     auto bus = wiz::make_board_bus();
 *     wiz::BoardDevice device{bus};
 *     wiz::BoardSocketProvider sockets{device};
 *     wiz::BoardLink link{device, sockets, mac, wiz::LinkMode::dhcp};
 *
 *     while (!link.begin()) { rosette_platform_delay_ms(500u); }  // PHY + DHCP
 *     // link.ipv4() is valid; call link.poll() ~1 Hz for lease renewal
 *
 * `board_init()` must have configured SPI and toggled the chip reset line
 * before the first `begin()`.
 */

#pragma once

#include <array>
#include <cstdint>
#include <cstdio>
#include <variant>

#include "rosette/platform/host_runtime.hpp"

#include "board.hpp"
#include "device.hpp"
#include "dhcp.hpp"
#include "executor.hpp"
#include "socket_provider.hpp"

namespace rosette::net::wiznet {

/// Addressing policy for `WiznetLink`.
enum class LinkMode : uint8_t {
    dhcp,      ///< Request a lease; fall back to `fallback` if it fails.
    static_ip, ///< Use `fallback` directly.
};

/// Static IPv4 settings for `LinkMode::static_ip` or DHCP fallback.
struct StaticAddress {
    std::array<uint8_t, 4> ip{192u, 168u, 1u, 200u};
    std::array<uint8_t, 4> netmask{255u, 255u, 255u, 0u};
    std::array<uint8_t, 4> gateway{192u, 168u, 1u, 1u};
    std::array<uint8_t, 4> dns{8u, 8u, 8u, 8u};
};

template<typename Chip, typename Bus,
         template<typename> class ExecutorT = DefaultExecutor>
class WiznetLink {
public:
    using device_type = WiznetDevice<Chip, Bus>;
    using provider_type = WiznetSocketProvider<Chip, Bus, ExecutorT>;
    using dhcp_type = DhcpPoller<Chip, Bus, ExecutorT>;

    WiznetLink(device_type& device,
               provider_type& provider,
               const std::array<uint8_t, 6>& mac,
               LinkMode mode = LinkMode::dhcp,
               const StaticAddress& fallback = {},
               const DhcpConfig& dhcp_config = {}) noexcept
        : device_(device)
        , dhcp_(device, provider, dhcp_config)
        , mac_(mac)
        , mode_(mode)
        , fallback_(fallback) {}

    WiznetLink(const WiznetLink&) = delete;
    WiznetLink& operator=(const WiznetLink&) = delete;

    /**
     * @brief Verifies the chip, sizes its buffers, waits for PHY link, and
     * assigns an address.
     *
     * Idempotent once the link is up. Blocks through the PHY wait and, for
     * DHCP, one full exchange; on DHCP failure applies the static fallback so
     * the board still boots.
     *
     * @return `true` when a usable IPv4 address is assigned.
     */
    bool begin() noexcept
    {
        if (ready_) {
            return true;
        }

        if (!chip_ready_) {
            if (!device_.verify()) {
                std::printf("[wiznet] chip verify failed\n");
                return false;
            }
            auto result = device_.init_socket_mode(mac_);
            if (std::holds_alternative<typename device_type::InvalidChipVersion>(result)) {
                const auto& err =
                    std::get<typename device_type::InvalidChipVersion>(result);
                std::printf("[wiznet] invalid chip version: 0x%02x (expected 0x%02x)\n",
                            err.actual, err.expected);
                return false;
            }
            chip_ready_ = true;
        }

        const uint32_t phy_start = rosette_platform_get_ms();
        while (!device_.is_link_up()) {
            if ((rosette_platform_get_ms() - phy_start) >= kPhyLinkTimeoutMs) {
                std::printf("[wiznet] PHY link timeout\n");
                return false;
            }
            rosette_platform_delay_ms(100u);
        }

        if (mode_ == LinkMode::static_ip) {
            device_.configure_ipv4(fallback_.ip, fallback_.netmask, fallback_.gateway);
            apply_address(fallback_.ip);
            return ready_;
        }

        dhcp_.set_callbacks(&on_dhcp_assign, &on_dhcp_change, this);
        if (dhcp_.acquire()) {
            return ready_;  // on_dhcp_assign has applied the address
        }

        std::printf("[wiznet] DHCP failed; applying static fallback\n");
        device_.configure_ipv4(fallback_.ip, fallback_.netmask, fallback_.gateway);
        apply_address(fallback_.ip);
        return ready_;
    }

    /// Services DHCP lease renewal. Call about once per second.
    void poll() noexcept
    {
        if (mode_ == LinkMode::dhcp) {
            dhcp_.poll();
        }
    }

    bool ready() const noexcept { return ready_; }
    std::array<uint8_t, 4> ipv4() const noexcept { return ip_; }

    /// The DHCP driver, for callers that want lease details (DNS, NTP, T1).
    dhcp_type& dhcp() noexcept { return dhcp_; }

private:
    static constexpr uint32_t kPhyLinkTimeoutMs = 5000u;

    static void on_dhcp_assign(const DhcpLease& lease, void* user) noexcept
    {
        static_cast<WiznetLink*>(user)->apply_address(lease.ip);
    }

    static void on_dhcp_change(const DhcpLease& /*old_lease*/,
                               const DhcpLease& new_lease,
                               void* user) noexcept
    {
        static_cast<WiznetLink*>(user)->apply_address(new_lease.ip);
    }

    void apply_address(const std::array<uint8_t, 4>& ip) noexcept
    {
        ip_ = ip;
        ready_ = (ip[0] | ip[1] | ip[2] | ip[3]) != 0u;
        rosette_platform_set_ipv4_addr(
            ready_ ? ((static_cast<uint32_t>(ip[0]) << 24) |
                      (static_cast<uint32_t>(ip[1]) << 16) |
                      (static_cast<uint32_t>(ip[2]) << 8) |
                       static_cast<uint32_t>(ip[3]))
                   : 0u);
    }

    device_type& device_;
    dhcp_type dhcp_;
    std::array<uint8_t, 6> mac_{};
    LinkMode mode_;
    StaticAddress fallback_;
    std::array<uint8_t, 4> ip_{};
    bool chip_ready_ = false;
    bool ready_ = false;
};

using BoardLink = WiznetLink<BoardChip, BoardBus>;

}  // namespace rosette::net::wiznet
