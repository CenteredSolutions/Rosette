// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

#pragma once

/**
 * @file net/ipv4ll_provider.hpp
 * @brief RFC 3927 IPv4 link-local ("zeroconf" / AutoIP) `AddressProvider`.
 *
 * Picks a `169.254.x.y` candidate deterministically from the interface MAC and
 * a conflict counter (`ipv4_link_local_candidate()`), so a board reclaims the
 * same address on every boot and after every link flap, and moves to the next
 * candidate only after a conflict. It writes the candidate through the
 * `NetInterface` — which emits a gratuitous ARP — waits a settle window while
 * watching `NetInterface::conflict()`, then holds the address. A first conflict
 * while bound is defended once with a re-announce; a repeat within
 * `DEFEND_INTERVAL` relinquishes the address and re-picks (RFC 3927 §2.5).
 *
 * Entirely non-blocking: a plain cooperative FSM stepped from `poll(now_ms)`.
 * There is no ARP-probe call and no internal clock — the hardware's own
 * duplicate detection (surfaced by `NetInterface::conflict()`) is the single
 * collision signal, for both the pre-claim settle and steady state.
 *
 * As the terminal fallback in an IPv4 provider list it never reports `failed`;
 * it holds link-local connectivity and keeps defending it.
 */

#include <array>
#include <cstdint>

#include "address_candidate.hpp"
#include "address_provider.hpp"
#include "interface.hpp"

namespace rosette::net {

class Ipv4LinkLocalProvider : public AddressProvider {
public:
    // RFC 3927 §2.2.1 timing (milliseconds).
    static constexpr uint32_t kSettleMs = 3000u;        ///< PROBE_WAIT + PROBE_NUM·PROBE_MAX
    static constexpr uint8_t kMaxConflicts = 10u;       ///< MAX_CONFLICTS
    static constexpr uint32_t kRateLimitMs = 60000u;    ///< RATE_LIMIT_INTERVAL
    static constexpr uint32_t kDefendMs = 10000u;       ///< DEFEND_INTERVAL

    enum class State : uint8_t {
        idle = 0u,
        settling,
        bound,
        defending,
        rate_limited,
    };

    Ipv4LinkLocalProvider(const std::array<uint8_t, 6>& mac,
                          NetInterface& iface = null_net_interface()) noexcept
        : iface_(iface), mac_(mac) {
        lease_.family = AddressFamily::ipv4;
        lease_.source = AddressSource::zeroconf;
    }

    void begin() noexcept override {
        (void)iface_.conflict();    // drop a stale signal from a prior owner
        conflicts_ = 0u;
        attempt_ = 0u;
        state_ = State::settling;
        need_pick_ = true;
        rebuild(false);
    }

    void stop() noexcept override {
        if (state_ == State::bound || state_ == State::defending) {
            clear_address();
        }
        state_ = State::idle;
        need_pick_ = false;
        candidate_ = {};
        rebuild(false);
    }

    const AddressLease& poll(uint32_t now_ms) noexcept override {
        if (need_pick_) {
            need_pick_ = false;
            pick_and_announce(now_ms);
            rebuild(false);
            return lease_;
        }
        const bool conflict = iface_.conflict();
        tick(now_ms, conflict);
        rebuild(conflict);
        return lease_;
    }

    State state() const noexcept { return state_; }
    std::array<uint8_t, 4> candidate() const noexcept { return candidate_; }
    uint8_t conflict_count() const noexcept { return conflicts_; }

private:
    std::array<uint8_t, 16> candidate_bytes() const noexcept {
        return {candidate_[0], candidate_[1], candidate_[2], candidate_[3]};
    }

    AddressLease candidate_lease(bool has_address) const noexcept {
        AddressLease l = lease_.empty();
        if (has_address) {
            l.add(candidate_bytes(), 16u);
        }

        return l;
    }

    void announce() noexcept { iface_.apply(candidate_lease(true)); }
    void clear_address() noexcept { iface_.apply(candidate_lease(false)); }

    // `attempt_` numbers the candidates tried since begin() and keeps counting
    // across rate-limit pauses, so a pause never cycles back to a taken address.
    void pick_and_announce(uint32_t now) noexcept {
        candidate_ = ipv4_link_local_candidate(mac_, attempt_++);
        announce();
        state_ = State::settling;
        settle_until_ms_ = now + kSettleMs;
    }

    void tick(uint32_t now, bool conflict) noexcept {
        switch (state_) {
        case State::settling:
            if (conflict) {
                note_conflict();
                if (conflicts_ >= kMaxConflicts) {
                    state_ = State::rate_limited;
                    settle_until_ms_ = now + kRateLimitMs;
                } else {
                    pick_and_announce(now);
                }
            } else if (static_cast<int32_t>(now - settle_until_ms_) >= 0) {
                state_ = State::bound;
            }
            return;

        case State::bound:
            if (conflict) {
                note_conflict();
                announce();    // RFC 3927 §2.5: defend once
                state_ = State::defending;
                defend_until_ms_ = now + kDefendMs;
            }
            return;

        case State::defending:
            if (conflict) {
                note_conflict();
                pick_and_announce(now);    // repeat conflict — relinquish
            } else if (static_cast<int32_t>(now - defend_until_ms_) >= 0) {
                state_ = State::bound;
            }
            return;

        case State::rate_limited:
            if (static_cast<int32_t>(now - settle_until_ms_) >= 0) {
                conflicts_ = 0u;
                pick_and_announce(now);
            }
            return;

        case State::idle:
        default:
            return;
        }
    }

    void note_conflict() noexcept {
        if (conflicts_ < 0xFFu) {
            ++conflicts_;
        }
    }

    void rebuild(bool conflict_edge) noexcept {
        set_flag(lease_.flags, AddressFlags::duplicate, conflict_edge);
        lease_.addresses.clear();
        const bool have = state_ == State::bound || state_ == State::defending;
        if (have) {
            lease_.add(candidate_bytes(), 16u);
            lease_.state = AddressPhase::bound;
        } else {
            lease_.state = (state_ == State::idle) ? AddressPhase::idle
                                                   : AddressPhase::acquiring;
        }
    }

    NetInterface& iface_;
    std::array<uint8_t, 6> mac_;
    AddressLease lease_{};
    std::array<uint8_t, 4> candidate_{};
    State state_ = State::idle;
    uint32_t settle_until_ms_ = 0u;
    uint32_t defend_until_ms_ = 0u;
    uint8_t conflicts_ = 0u;
    uint32_t attempt_ = 0u;
    bool need_pick_ = false;
};

}    // namespace rosette::net
