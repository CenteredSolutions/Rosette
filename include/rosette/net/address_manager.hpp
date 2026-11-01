// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

#pragma once

/**
 * @file net/address_manager.hpp
 * @brief Drives an ordered list of `AddressProvider`s for one address family.
 *
 * The list is in preference order, and exactly one provider holds the
 * interface at a time. Acquisition tries the providers in order and the first
 * to bind wins. A provider that reports `failed`, or that has not bound
 * before `provider_timeout_ms`, hands off to the next. When the list is
 * exhausted, or the holder later loses its address, the manager waits an
 * exponentially growing backoff and restarts from provider[0].
 *
 * While a lower-preference provider holds the interface (IPv4 link-local
 * after DHCP gave up), the manager keeps looking for a better address: every
 * `upgrade_probe_ms` it probes the providers ahead of the holder, one at a
 * time, with `AddressProvider::begin_probe()`. A prober acquires without
 * writing the interface or reading its conflict signal, and the holder keeps
 * being polled, so the address on the chip stays in use and defended
 * throughout. When a prober binds, the manager stops the holder and the
 * prober claims the interface (RFC 3927 §1.9: a host stops using its
 * link-local address once it has a routable one). A provider that cannot
 * probe (the default) is only retried by the backoff restart.
 *
 * The manager knows nothing about snapshots, IPv6, or the platform runtime:
 * it runs a list and exposes the current `lease()` and `stable()`.
 * `NetworkAddress` (`net/address.hpp`) combines a v4 and an optional v6
 * manager into the interface's overall state and publishes it.
 *
 *     static wiz::WiznetInterface<wiz::BoardChip, wiz::BoardBus> iface{device};
 *     static wiz::DhcpAddressProvider dhcp{sockets, mac, iface};
 *     static rosette::net::Ipv4LinkLocalProvider zeroconf{mac, iface};
 *     static rosette::net::AddressProvider* v4[] = {&dhcp, &zeroconf};
 *     static rosette::net::AddressManager   mgr{v4};
 *     static rosette::net::NetworkAddress   net{mgr};
 *
 * The logic is defined in src/net/address_manager.cpp.
 */

#include <cstddef>
#include <cstdint>

#include "address_provider.hpp"

namespace rosette::net {

struct AddressManagerPolicy {
    /// Move to the next provider if the current one has not bound by here
    /// (0 = wait on it forever). Also bounds one upgrade probe.
    uint32_t provider_timeout_ms = 15000u;
    /// First wait after the list is exhausted / a bound provider is lost;
    /// doubles each subsequent full pass, capped at 8×.
    uint32_t retry_backoff_ms = 30000u;
    /// While a lower-preference provider holds the interface, the interval
    /// between rounds of probes of the providers ahead of it (0 = never probe).
    uint32_t upgrade_probe_ms = 60000u;
};

enum class AddressState : uint8_t {
    down = 0u,  ///< `begin()` not called, or `stop()`ed.
    acquiring,  ///< Working through the provider list.
    stable,     ///< A provider is bound.
};

class AddressManager {
public:
    AddressManager(AddressProvider* const* providers, std::size_t count,
                   const AddressManagerPolicy& policy = {}) noexcept
        : providers_(providers), count_(count), policy_(policy), probe_(count) {}

    template <std::size_t N>
    explicit AddressManager(AddressProvider* const (&providers)[N],
                            const AddressManagerPolicy& policy = {}) noexcept
        : AddressManager(providers, N, policy) {}

    /// @brief Starts provider[0]; its timeout counts from the first `poll()`.
    void begin() noexcept;

    /// @brief Stops every provider and forgets the lease.
    void stop() noexcept;

    /**
     * @brief One cooperative step. Call every superloop iteration with the loop's `now_ms`.
     *
     * Polls the holder (or the provider being tried), moves along the list
     * on failure or timeout, and, while a lower-preference provider holds a
     * bound address, runs the upgrade probe schedule.
     *
     * @param[in] now_ms  Monotonic milliseconds.
     * @return The manager's state after this step.
     *
     * Verification:
     * - Correct means: the first provider to bind wins; failure or timeout
     *   advances the list; exhaustion or loss waits the backoff and restarts
     *   from provider[0]; while provider k > 0 holds, a probe of a provider
     *   ahead of it that binds becomes the holder and provider k is stopped,
     *   and a probe that fails leaves provider k's address untouched until
     *   the next round.
     * - Covered by: tests/host/address_manager_test.cpp;
     *   tests/host/address_upgrade_test.cpp (TC-NET-UPG-004, TC-NET-UPG-005).
     *
     * References: RFC 3927 §1.9, §2.11 (link-local only while no routable address).
     */
    AddressState poll(uint32_t now_ms) noexcept;

    const AddressLease& lease() const noexcept { return lease_; }

    AddressState state() const noexcept { return state_; }

    bool stable() const noexcept { return state_ == AddressState::stable; }

    /// @brief Whether an upgrade probe is running.
    bool probing() const noexcept { return probe_ < count_; }

private:
    void enter(std::size_t index, uint32_t now_ms) noexcept;
    void exhaust(uint32_t now_ms) noexcept;
    void advance(uint32_t now_ms) noexcept;
    void step_probe(uint32_t now_ms) noexcept;
    void start_probe(std::size_t from, uint32_t now_ms) noexcept;
    void abort_probe() noexcept;
    void promote(uint32_t now_ms) noexcept;

    AddressProvider* const* providers_;
    std::size_t count_;
    AddressManagerPolicy policy_;

    AddressLease lease_{};
    AddressState state_ = AddressState::down;
    std::size_t active_ = 0u;
    std::size_t probe_;  ///< Index being probed; `count_` when none.
    uint32_t entered_ms_ = 0u;
    uint32_t retry_at_ms_ = 0u;
    uint32_t backoff_ms_ = 0u;
    uint32_t probe_at_ms_ = 0u;
    uint32_t probe_entered_ms_ = 0u;
    bool started_ = false;
    bool entry_pending_ = false;  ///< provider[0] started by begin(); stamp its entry time.
};

}  // namespace rosette::net
