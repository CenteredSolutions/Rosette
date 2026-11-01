// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

#pragma once

/**
 * @file ports/net/wiznet/interface.hpp
 * @brief `NetInterface` for a WIZnet hardware-socket chip.
 *
 * `apply()` programs `SIPR` / `SUBR` / `GAR` (writing `SIPR` makes the chip emit
 * a gratuitous ARP — the RFC 3927 announce), or zeroes them when the lease has
 * no address. An IPv4 link-local address is programmed with mask 0.0.0.0 and
 * no gateway: the chip then ARPs for every destination directly, which is
 * what RFC 3927 §2.6.2 requires of a host holding only a link-local address,
 * so it reaches peers on the link that hold routable addresses. With the /16
 * mask the chip would send those packets to the absent gateway, and each
 * failed ARP stalls the socket's next send for the chip's retry time
 * (about 1.6 s, measured on the W6100). `conflict()` reads the chip's
 * IP-conflict flag (WiznetDevice::poll_common_interrupt()) and drains it.
 *
 * On parts with hardware IPv6 (`Chip::has_hw_ipv6()`), an IPv6 lease is the
 * whole IPv6 configuration: its routable entry goes to `GUAR` with its prefix
 * mask (`SUB6R`) and the default router (`GA6R`, from `gateway_v6`), and its
 * link-local entry goes to `LLAR`; a register with no matching entry is
 * cleared. `LLAR` is written only when its value changes, so refreshing the
 * global address never re-announces the link-local one. On other parts an
 * IPv6 lease is ignored.
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
            if constexpr (Chip::has_hw_ipv6()) {
                apply_ipv6(lease);
            }

            return;
        }

        if (!lease.has_address()) {
            device_.configure_ipv4({}, {}, {});
            return;
        }

        const auto& entry = lease.primary();
        const std::array<uint8_t, 4> ip{entry.address[0], entry.address[1], entry.address[2],
                                        entry.address[3]};
        if (entry.link_local()) {
            device_.configure_ipv4(ip, {}, {});
            return;
        }

        device_.configure_ipv4(ip, rosette::net::Ipv4PrefixToMask(entry.prefix_len),
                               lease.gateway_v4);
    }

    bool conflict() noexcept override {
        (void)device_.poll_common_interrupt();  // one register read
        return device_.take_ip_conflict();
    }

    rosette::net::LinkState link_state() noexcept override {
        return device_.is_link_up() ? rosette::net::LinkState::up
                                    : rosette::net::LinkState::down;
    }

private:
    void apply_ipv6(const rosette::net::AddressLease& lease) noexcept {
        static constexpr std::array<uint8_t, 16> kNone{};
        const rosette::net::AddressEntry* global = nullptr;
        const rosette::net::AddressEntry* link_local = nullptr;
        for (const auto& entry : lease.addresses) {
            if (entry.routable() && global == nullptr) {
                global = &entry;
            } else if (entry.link_local() && link_local == nullptr) {
                link_local = &entry;
            }
        }

        if (global != nullptr) {
            device_.configure_ipv6_global(global->address, prefix_mask(global->prefix_len),
                                          lease.gateway_v6);
        } else {
            device_.configure_ipv6_global(kNone, kNone, kNone);
        }

        const std::array<uint8_t, 16>& wanted = link_local != nullptr ? link_local->address : kNone;
        if (device_.ipv6_link_local() != wanted) {
            device_.configure_ipv6_link_local(wanted);
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
