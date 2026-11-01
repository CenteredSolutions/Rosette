// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file src/net/address_manager.cpp
 * @brief The per-family provider list runner; see rosette/net/address_manager.hpp.
 */

#include "rosette/net/address_manager.hpp"

namespace rosette::net {
namespace {

bool due(uint32_t deadline_ms, uint32_t now_ms) noexcept {
    return static_cast<int32_t>(now_ms - deadline_ms) >= 0;
}

}  // namespace

void AddressManager::begin() noexcept {
    abort_probe();
    started_ = true;
    backoff_ms_ = 0u;
    lease_ = AddressLease{};
    state_ = AddressState::acquiring;
    if (count_ > 0u) {
        active_ = 0u;
        providers_[0]->begin();
        entry_pending_ = true;
    }
}

void AddressManager::stop() noexcept {
    for (std::size_t i = 0u; i < count_; ++i) {
        providers_[i]->stop();
    }

    probe_ = count_;
    started_ = false;
    active_ = 0u;
    state_ = AddressState::down;
    lease_ = AddressLease{};
}

AddressState AddressManager::poll(uint32_t now_ms) noexcept {
    if (!started_ || count_ == 0u) {
        return state_;
    }

    if (entry_pending_) {
        entry_pending_ = false;
        entered_ms_ = now_ms;
    }

    if (active_ == count_) {  // between passes, waiting out the backoff
        if (!due(retry_at_ms_, now_ms)) {
            return state_;
        }

        enter(0u, now_ms);
    }

    const bool was_stable = stable();
    lease_ = providers_[active_]->poll(now_ms);
    switch (lease_.state) {
        case AddressPhase::bound:
            backoff_ms_ = 0u;
            state_ = AddressState::stable;
            if (!was_stable) {
                probe_at_ms_ = now_ms + policy_.upgrade_probe_ms;
            }

            break;
        case AddressPhase::failed:
            // A holder that lost its address, or a failure on the last rung,
            // restarts the whole cascade; otherwise step to the next provider.
            if (state_ == AddressState::stable || active_ + 1u >= count_) {
                exhaust(now_ms);
            } else {
                enter(active_ + 1u, now_ms);
            }

            break;
        case AddressPhase::acquiring:
        case AddressPhase::idle:
            // While stable, a provider only `acquiring` is renewing: hold `stable`
            // until it reports `failed`.
            if (state_ != AddressState::stable) {
                state_ = AddressState::acquiring;
                if (policy_.provider_timeout_ms != 0u
                    && due(entered_ms_ + policy_.provider_timeout_ms, now_ms)) {
                    advance(now_ms);
                }
            }

            break;
    }

    if (stable() && active_ > 0u && active_ < count_) {
        step_probe(now_ms);
    }

    return state_;
}

void AddressManager::enter(std::size_t index, uint32_t now_ms) noexcept {
    abort_probe();
    if (active_ < count_ && index != active_) {
        providers_[active_]->stop();
    }

    active_ = index;
    entered_ms_ = now_ms;
    providers_[index]->begin();
}

void AddressManager::exhaust(uint32_t now_ms) noexcept {
    abort_probe();
    if (active_ < count_) {
        providers_[active_]->stop();
    }

    active_ = count_;
    const uint32_t base = policy_.retry_backoff_ms;
    const uint32_t cap = base <= 0xFFFFFFFFu / 8u ? base * 8u : 0xFFFFFFFFu;
    backoff_ms_ = backoff_ms_ == 0u ? base : (backoff_ms_ < cap / 2u ? backoff_ms_ * 2u : cap);
    retry_at_ms_ = now_ms + backoff_ms_;
    state_ = AddressState::acquiring;
    lease_ = lease_.empty();
}

void AddressManager::advance(uint32_t now_ms) noexcept {
    if (active_ + 1u >= count_) {
        exhaust(now_ms);
    } else {
        enter(active_ + 1u, now_ms);
    }
}

void AddressManager::step_probe(uint32_t now_ms) noexcept {
    if (policy_.upgrade_probe_ms == 0u) {
        return;
    }

    if (!probing()) {
        if (due(probe_at_ms_, now_ms)) {
            start_probe(0u, now_ms);
        }

        return;
    }

    const AddressPhase phase = providers_[probe_]->poll(now_ms).state;
    if (phase == AddressPhase::bound) {
        promote(now_ms);
        return;
    }

    const bool timed_out = policy_.provider_timeout_ms != 0u
                           && due(probe_entered_ms_ + policy_.provider_timeout_ms, now_ms);
    if (phase == AddressPhase::failed || timed_out) {
        providers_[probe_]->stop();
        start_probe(probe_ + 1u, now_ms);
    }
}

void AddressManager::start_probe(std::size_t from, uint32_t now_ms) noexcept {
    for (std::size_t i = from; i < active_; ++i) {
        if (providers_[i]->begin_probe()) {
            probe_ = i;
            probe_entered_ms_ = now_ms;
            return;
        }
    }

    probe_ = count_;
    probe_at_ms_ = now_ms + policy_.upgrade_probe_ms;
}

void AddressManager::abort_probe() noexcept {
    if (probing()) {
        providers_[probe_]->stop();
    }

    probe_ = count_;
}

void AddressManager::promote(uint32_t now_ms) noexcept {
    // The holder releases the interface before the prober writes its address,
    // so the prober's write is the last one and the chip ends up holding it.
    providers_[active_]->stop();
    active_ = probe_;
    probe_ = count_;
    providers_[active_]->claim();
    entered_ms_ = now_ms;
    probe_at_ms_ = now_ms + policy_.upgrade_probe_ms;
    lease_ = providers_[active_]->poll(now_ms);
}

}  // namespace rosette::net
