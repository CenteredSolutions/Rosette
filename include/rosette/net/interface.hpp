// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

#pragma once

/**
 * @file net/interface.hpp
 * @brief The one backend hook an `AddressProvider` needs from the hardware.
 *
 * A provider that is not itself the chip driver — the static and RFC 3927
 * link-local providers — needs three things from the interface: write the chosen
 * address onto it, learn when the hardware sees a duplicate, and know whether
 * the physical link has carrier. All are hardware-specific (WIZnet programs
 * `SIPR`/`SUBR`/`GAR`, raises an IP-conflict interrupt bit, and exposes PHY
 * link status; a software stack calls `netif` APIs). `NetInterface` is that
 * hook, and it is the only one.
 *
 * WIZnet supplies `wiznet::WiznetInterface<Chip,Bus>`. A DHCP client writes its
 * own lease, so its provider only ever calls `conflict()`. The FreeRTOS+TCP
 * observer owns nothing and uses `NullNetInterface` (the stack handles carrier).
 */

#include "address_provider.hpp"

namespace rosette::net {

/// Physical-link carrier state, read live from `NetInterface::link_state()`.
enum class LinkState : uint8_t {
    down = 0u,
    up,
};

class NetInterface {
public:
    virtual ~NetInterface() = default;

    /**
     * @brief Program @p lease onto the interface, or clear the family's address
     *        when `lease.has_address` is false.
     *
     * Writing an IPv4 address is expected to emit a gratuitous ARP so the
     * hardware's own duplicate detection can fire — the RFC 3927 "announce".
     */
    virtual void apply(const AddressLease& lease) noexcept = 0;

    /**
     * @brief Consume a pending duplicate-address signal.
     *
     * @return `true` once per detected conflict, then `false` until the next.
     */
    virtual bool conflict() noexcept = 0;

    /**
     * @brief Whether the physical link currently has carrier.
     *
     * Read live each call. `NetworkAddress` supervises this: a carrier drop
     * releases the address, and its return kicks a fresh acquisition. Backends
     * whose stack owns link state (FreeRTOS+TCP) leave this `LinkState::up`.
     */
    virtual LinkState link_state() noexcept { return LinkState::up; }

    /// Convenience: `link_state() == LinkState::up`.
    bool link_up() noexcept { return link_state() == LinkState::up; }
};

/// Discards writes, never reports a conflict, always up — host tests / observe-only stacks.
class NullNetInterface : public NetInterface {
public:
    void apply(const AddressLease&) noexcept override {}
    bool conflict() noexcept override { return false; }
};

/// Shared no-op instance for provider default arguments.
inline NetInterface& null_net_interface() noexcept {
    static NullNetInterface instance;
    return instance;
}

}    // namespace rosette::net
