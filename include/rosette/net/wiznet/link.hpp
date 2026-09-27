/**
 * @file net/wiznet/link.hpp
 * @brief WIZnet chip bring-up: verify the part, size its buffers, wait for the PHY link.
 *
 * `WiznetLink` gets the chip to the point where addressing can start: the
 * part answers with the expected version, runs in socket mode with the MAC
 * programmed, and reports carrier. Addresses are then the job of the
 * `AddressProvider`s (`dhcp_provider.hpp`, `ipv4ll_provider.hpp`,
 * `slaac_provider.hpp`) under an `AddressManager`.
 *
 *     namespace wiz = rosette::net::wiznet;
 *     auto bus = wiz::make_board_bus();
 *     wiz::BoardDevice device{bus};
 *     wiz::BoardLink link{device, mac};
 *     while (!link.bring_link_up()) { rosette_platform_delay_ms(500u); }
 *
 * `board_init()` must have configured SPI and toggled the chip reset line
 * before the first `bring_link_up()`.
 */

#pragma once

#include <array>
#include <cstdint>
#include <cstdio>
#include <variant>

#include "rosette/platform/host_runtime.hpp"

#include "board.hpp"
#include "device.hpp"

namespace rosette::net::wiznet {

template <typename Chip, typename Bus>
class WiznetLink {
public:
    using device_type = WiznetDevice<Chip, Bus>;

    WiznetLink(device_type& device, const std::array<uint8_t, 6>& mac) noexcept
        : device_(device), mac_(mac) {}

    WiznetLink(const WiznetLink&) = delete;
    WiznetLink& operator=(const WiznetLink&) = delete;

    /**
     * @brief Verifies the chip, puts it in socket mode, and waits for PHY link.
     *
     * Leaves the chip ready for an `AddressProvider` to take over addressing;
     * assigns no address. The chip is verified once; later calls only wait
     * for the link. Blocks for at most kPhyLinkTimeoutMs.
     *
     * @return `true` when the chip is ready and the PHY link is up.
     */
    bool bring_link_up() noexcept {
        if (link_ready_) {
            return true;
        }

        if (!chip_ready_) {
            if (!device_.verify()) {
                std::printf("[wiznet] chip verify failed\n");
                return false;
            }
            auto result = device_.init_socket_mode(mac_);
            if (std::holds_alternative<typename device_type::InvalidChipVersion>(result)) {
                const auto& err = std::get<typename device_type::InvalidChipVersion>(result);
                std::printf("[wiznet] invalid chip version: 0x%02x (expected 0x%02x)\n", err.actual,
                            err.expected);
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

        link_ready_ = true;
        return true;
    }

private:
    static constexpr uint32_t kPhyLinkTimeoutMs = 5000u;

    device_type& device_;
    std::array<uint8_t, 6> mac_{};
    bool chip_ready_ = false;
    bool link_ready_ = false;
};

using BoardLink = WiznetLink<BoardChip, BoardBus>;

}  // namespace rosette::net::wiznet
