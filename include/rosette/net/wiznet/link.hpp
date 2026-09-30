/**
 * @file net/wiznet/link.hpp
 * @brief WIZnet chip bring-up: verify the part and put it in socket mode with the board's MAC.
 *
 * `WiznetLink` gets the chip to the point where addressing can start: the
 * part answers with the expected version and runs in socket mode with the
 * MAC programmed. It does not wait for the PHY: carrier comes and goes for
 * the life of the board, so `NetworkAddress` (`net/address.hpp`) watches it
 * through `WiznetInterface::link_state()` and starts the address providers
 * (`dhcp_provider.hpp`, `ipv4ll_provider.hpp`, `slaac_provider.hpp`) whenever
 * the link comes up.
 *
 *     namespace wiz = rosette::net::wiznet;
 *     auto bus = wiz::make_board_bus();
 *     wiz::BoardDevice device{bus};
 *     wiz::BoardLink link{device, mac};
 *     while (!link.bring_chip_up()) { rosette::platform::sleep_ms(500u); }
 *
 * `board_init()` must have configured SPI and toggled the chip reset line
 * before the first `bring_chip_up()`.
 */

#pragma once

#include <array>
#include <cstdint>
#include <cstdio>
#include <variant>

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
     * @brief Verifies the chip and puts it in socket mode with the board's MAC.
     *
     * Leaves the chip ready for an `AddressProvider` to take over addressing;
     * assigns no address and does not wait for the PHY link. The chip is set
     * up once; later calls return `true` at once. Takes a few SPI
     * transactions, so a caller may retry it from a superloop.
     *
     * @return `true` when the chip is in socket mode; `false` when it did not
     *   answer or reported an unexpected version (printed), in which case the
     *   call may be retried.
     *
     * Verification:
     * - Correct means: after `true`, socket opens succeed and the chip's
     *   SHAR holds `mac`; a missing or wrong part never yields `true`.
     * - Covered by: every WIZnet example on the W5100S-EVB-Pico2 and the
     *   W6100-EVB-Pico (the boot log reaches `[net] chip ready`).
     *
     * References: W5100S / W6100 / W6300 datasheets, VERR / CIDR and MR registers.
     */
    bool bring_chip_up() noexcept {
        if (chip_ready_) {
            return true;
        }

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
        return true;
    }

private:
    device_type& device_;
    std::array<uint8_t, 6> mac_{};
    bool chip_ready_ = false;
};

using BoardLink = WiznetLink<BoardChip, BoardBus>;

}  // namespace rosette::net::wiznet
