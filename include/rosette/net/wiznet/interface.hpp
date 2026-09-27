#pragma once

/**
 * @file net/wiznet/interface.hpp
 * @brief `NetInterface` for a WIZnet hardware-socket chip.
 *
 * `apply()` programs `SIPR` / `SUBR` / `GAR` (writing `SIPR` makes the chip emit
 * a gratuitous ARP — the RFC 3927 announce), or zeroes them when the lease has
 * no address. `conflict()` drains the chip's IP-conflict latch, which
 * `WiznetDevice` sets under the bus lock from either the FreeRTOS ISR-dispatch
 * task (`service_socket_interrupts()`) or a superloop register read.
 *
 * On parts with hardware IPv6 (`Chip::has_hw_ipv6()`), an IPv6 lease programs
 * the chip's two IPv6 address registers: a link-local lease writes `LLAR` and
 * withdraws the global address (`GUAR`, `SUB6R`), a routable lease writes
 * `GUAR`, its prefix mask, and the default router (`GA6R`, from
 * `gateway_v6`) and leaves `LLAR` in place, and an empty lease
 * clears both. `WiznetSlaacProvider` (`slaac_provider.hpp`) issues them in
 * that order. On other parts an IPv6 lease is ignored.
 */

#include <array>
#include <cstddef>
#include <cstdint>

#include "rosette/net/address_state.hpp"
#include "rosette/net/interface.hpp"

#include "device.hpp"

namespace rosette::net::wiznet {

template <typename Chip, typename Bus>
class WiznetInterface : public rosette::net::NetInterface {
public:
    using device_type = WiznetDevice<Chip, Bus>;

    explicit WiznetInterface(device_type& device) noexcept : device_(device) {}

    void apply(const rosette::net::AddressLease& lease) noexcept override {
        if (lease.family == rosette::net::AddressFamily::ipv6) {
            if constexpr (Chip::has_hw_ipv6()) apply_ipv6(lease);
            return;
        }
        if (!lease.has_address) {
            device_.configure_ipv4({}, {}, {});
            return;
        }
        const std::array<uint8_t, 4> ip{lease.address[0], lease.address[1],
                                        lease.address[2], lease.address[3]};
        device_.configure_ipv4(ip, rosette::net::Ipv4PrefixToMask(lease.prefix_len),
                               lease.gateway_v4);
    }

    bool conflict() noexcept override {
        (void)device_.poll_common_interrupt();    // superloop refresh; ~free on RTOS
        return device_.take_ip_conflict();
    }

    rosette::net::LinkState link_state() noexcept override {
        return device_.is_link_up() ? rosette::net::LinkState::up
                                    : rosette::net::LinkState::down;
    }

private:
    void apply_ipv6(const rosette::net::AddressLease& lease) noexcept {
        static constexpr std::array<uint8_t, 16> kNone{};
        if (!lease.has_address) {
            device_.configure_ipv6_global(kNone, kNone, kNone);
            device_.configure_ipv6_link_local(kNone);
        } else if (lease.link_local()) {
            device_.configure_ipv6_global(kNone, kNone, kNone);
            device_.configure_ipv6_link_local(lease.address);
        } else {
            device_.configure_ipv6_global(lease.address, prefix_mask(lease.prefix_len), lease.gateway_v6);
        }
    }

    /// @brief The 128-bit mask with the first `bits` bits set (SUB6R format).
    static std::array<uint8_t, 16> prefix_mask(uint8_t bits) noexcept {
        std::array<uint8_t, 16> mask{};
        for (std::size_t i = 0; i < mask.size() && bits > 0u; ++i) {
            const uint8_t take = bits >= 8u ? 8u : bits;
            mask[i] = static_cast<uint8_t>(0xFFu << (8u - take));
            bits = static_cast<uint8_t>(bits - take);
        }
        return mask;
    }

    device_type& device_;
};

}    // namespace rosette::net::wiznet
