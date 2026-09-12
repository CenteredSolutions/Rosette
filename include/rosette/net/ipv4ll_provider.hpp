#pragma once

/**
 * @file net/ipv4ll_provider.hpp
 * @brief RFC 3927 IPv4 link-local ("zeroconf" / AutoIP) `AddressProvider`.
 *
 * Picks a `169.254.x.y` candidate deterministically from the interface MAC
 * (stable across reboots so a board reclaims its address), writes it through the
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
            : iface_(iface), prng_{seed_from_mac(mac)} {
        lease_.family = AddressFamily::ipv4;
        lease_.source = AddressSource::zeroconf;
        lease_.prefix_len = 16u;
    }

    void begin() noexcept override {
        (void)iface_.conflict();    // drop a stale signal from a prior owner
        conflicts_ = 0u;
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
    struct Xorshift {
        uint32_t s;
        uint32_t next() noexcept {
            s ^= s << 13;
            s ^= s >> 17;
            s ^= s << 5;
            return s;
        }
    };

    static uint32_t seed_from_mac(const std::array<uint8_t, 6>& mac) noexcept {
        uint32_t seed = (static_cast<uint32_t>(mac[2]) << 24) |
                        (static_cast<uint32_t>(mac[3]) << 16) |
                        (static_cast<uint32_t>(mac[4]) << 8) |
                        static_cast<uint32_t>(mac[5]);
        seed ^= (static_cast<uint32_t>(mac[0]) << 8) | mac[1];
        return seed != 0u ? seed : 0x1234567u;
    }

    void pick_candidate() noexcept {
        const uint32_t r = prng_.next();
        candidate_ = {169u, 254u,
                      static_cast<uint8_t>(1u + (r % 254u)),        // 1..254
                      static_cast<uint8_t>((r >> 8) & 0xFFu)};      // 0..255
    }

    AddressLease candidate_lease(bool has_address) const noexcept {
        AddressLease l{};
        l.family = AddressFamily::ipv4;
        l.source = AddressSource::zeroconf;
        l.prefix_len = 16u;
        l.has_address = has_address;
        if (has_address) {
            l.address = {candidate_[0], candidate_[1], candidate_[2], candidate_[3]};
        }
        return l;
    }

    void announce() noexcept { iface_.apply(candidate_lease(true)); }
    void clear_address() noexcept { iface_.apply(candidate_lease(false)); }

    void pick_and_announce(uint32_t now) noexcept {
        pick_candidate();
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
        lease_.source = AddressSource::zeroconf;
        lease_.prefix_len = 16u;
        set_flag(lease_.flags, AddressFlags::duplicate, conflict_edge);
        const bool have = state_ == State::bound || state_ == State::defending;
        lease_.has_address = have;
        if (have) {
            lease_.address = {candidate_[0], candidate_[1], candidate_[2],
                              candidate_[3], 0};
            lease_.state = AddressPhase::bound;
        } else {
            lease_.address = {};
            lease_.state = (state_ == State::idle) ? AddressPhase::idle
                                                   : AddressPhase::acquiring;
        }
    }

    NetInterface& iface_;
    Xorshift prng_;
    AddressLease lease_{};
    std::array<uint8_t, 4> candidate_{};
    State state_ = State::idle;
    uint32_t settle_until_ms_ = 0u;
    uint32_t defend_until_ms_ = 0u;
    uint8_t conflicts_ = 0u;
    bool need_pick_ = false;
};

}    // namespace rosette::net
