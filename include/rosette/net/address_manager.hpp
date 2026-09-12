#pragma once

/**
 * @file net/address_manager.hpp
 * @brief Drives an ordered list of `AddressProvider`s for one address family.
 *
 * The whole rule: try the providers in registration order, the first to bind
 * wins. A provider that reports `failed`, or that has not bound before
 * `provider_timeout_ms`, hands off to the next. When the list is exhausted — or
 * a bound provider is later lost — the manager waits an exponentially-growing
 * backoff and restarts from provider[0] (the network may have genuinely come
 * back, so a preferred provider like DHCP is re-tried ahead of a fallback).
 *
 * The manager knows nothing about snapshots, IPv6, or the platform runtime — it
 * just runs a list and exposes the current `lease()` and `stable()`.
 * `NetworkAddress` (`net/address.hpp`) combines a v4 and an optional v6
 * manager into the interface's overall state and publishes it.
 *
 * Firmware declares the providers directly (see examples/led_files/main.cpp):
 *
 *     static wiz::WiznetInterface<wiz::BoardChip, wiz::BoardBus> iface{device};
 *     static wiz::DhcpAddressProvider<wiz::BoardChip, wiz::BoardBus> dhcp{device, sockets, iface};
 *     static rosette::net::Ipv4LinkLocalProvider zeroconf{mac, iface};
 *     static rosette::net::AddressProvider* v4[] = {&dhcp, &zeroconf};
 *     static rosette::net::AddressManager   mgr{v4};
 *     static rosette::net::NetworkAddress   net{mgr};
 */

#include <cstddef>
#include <cstdint>

#include "rosette/platform/host_runtime.hpp"

#include "address_provider.hpp"

namespace rosette::net {

struct AddressManagerPolicy {
    /// Move to the next provider if the current one has not bound by here
    /// (0 = wait on it forever).
    uint32_t provider_timeout_ms = 15000u;
    /// First wait after the list is exhausted / a bound provider is lost;
    /// doubles each subsequent full pass, capped at 8×.
    uint32_t retry_backoff_ms = 30000u;
};

enum class AddressState : uint8_t {
    down = 0u,     ///< `begin()` not called, or `stop()`ed.
    acquiring,     ///< Working through the provider list.
    stable,        ///< A provider is bound.
};

class AddressManager {
public:
    AddressManager(AddressProvider* const* providers, std::size_t count,
                   const AddressManagerPolicy& policy = {}) noexcept
            : providers_(providers), count_(count), policy_(policy) {}

    template <std::size_t N>
    explicit AddressManager(AddressProvider* const (&providers)[N],
                            const AddressManagerPolicy& policy = {}) noexcept
            : AddressManager(providers, N, policy) {}

    void begin() noexcept {
        started_ = true;
        backoff_ms_ = 0u;
        lease_ = AddressLease{};
        state_ = AddressState::acquiring;
        if (count_ > 0u) {
            enter(0u, rosette_platform_get_ms());
        }
    }

    void stop() noexcept {
        for (std::size_t i = 0u; i < count_; ++i) {
            providers_[i]->stop();
        }
        started_ = false;
        active_ = 0u;
        state_ = AddressState::down;
        lease_ = AddressLease{};
    }

    /// One cooperative step. Call every superloop iteration with the loop's `now_ms`.
    AddressState poll(uint32_t now_ms) noexcept {
        if (!started_ || count_ == 0u) {
            return state_;
        }
        const uint32_t now = now_ms;

        if (active_ == count_) {    // between passes, waiting out the backoff
            if (static_cast<int32_t>(now - retry_at_ms_) < 0) {
                return state_;
            }
            enter(0u, now);
        }

        lease_ = providers_[active_]->poll(now);

        switch (lease_.state) {
        case AddressPhase::bound:
            backoff_ms_ = 0u;
            state_ = AddressState::stable;
            break;

        case AddressPhase::failed:
            // A bound provider that lost its address, or a plain acquisition
            // failure on the last rung: restart the whole cascade. Otherwise
            // just step to the next provider.
            if (state_ == AddressState::stable || active_ + 1u >= count_) {
                exhaust(now);
            } else {
                enter(active_ + 1u, now);
            }
            break;

        case AddressPhase::acquiring:
        case AddressPhase::idle:
            if (state_ != AddressState::stable) {
                state_ = AddressState::acquiring;
                if (policy_.provider_timeout_ms != 0u &&
                    static_cast<int32_t>(
                            now - (entered_ms_ + policy_.provider_timeout_ms)) >= 0) {
                    if (active_ + 1u >= count_) {
                        exhaust(now);
                    } else {
                        enter(active_ + 1u, now);
                    }
                }
            }
            // If we were stable, a provider now only `acquiring` is a renewal
            // in progress — hold `stable` until it reports `failed`.
            break;
        }
        return state_;
    }

    const AddressLease& lease() const noexcept { return lease_; }
    AddressState state() const noexcept { return state_; }
    bool stable() const noexcept { return state_ == AddressState::stable; }

private:
    void enter(std::size_t idx, uint32_t now) noexcept {
        if (active_ < count_ && idx != active_) {
            providers_[active_]->stop();
        }
        active_ = idx;
        entered_ms_ = now;
        providers_[idx]->begin();
    }

    void exhaust(uint32_t now) noexcept {
        if (active_ < count_) {
            providers_[active_]->stop();
        }
        active_ = count_;
        const uint32_t base = policy_.retry_backoff_ms;
        const uint32_t cap = base <= 0xFFFFFFFFu / 8u ? base * 8u : 0xFFFFFFFFu;
        backoff_ms_ = backoff_ms_ == 0u ? base
                      : (backoff_ms_ < cap / 2u ? backoff_ms_ * 2u : cap);
        retry_at_ms_ = now + backoff_ms_;
        state_ = AddressState::acquiring;
        lease_ = lease_.empty();
    }

    AddressProvider* const* providers_;
    std::size_t count_;
    AddressManagerPolicy policy_;

    AddressLease lease_{};
    AddressState state_ = AddressState::down;
    std::size_t active_ = 0u;
    uint32_t entered_ms_ = 0u;
    uint32_t retry_at_ms_ = 0u;
    uint32_t backoff_ms_ = 0u;
    bool started_ = false;
};

}    // namespace rosette::net
