/**
 * @file ipv4ll_provider_test.cpp
 * @brief RFC 3927 link-local provider: deterministic candidate, non-blocking
 *        settle, conflict handling, defend-once-then-abandon. No clock hook —
 *        `poll(now_ms)` takes the timestamp directly.
 */

#include "fake_interface.hpp"
#include "test_util.hpp"

#include "rosette/net/ipv4ll_provider.hpp"

#include <array>

using rosette::net::AddressPhase;
using rosette::net::Ipv4LinkLocalProvider;
using State = Ipv4LinkLocalProvider::State;

namespace {
const std::array<uint8_t, 6> kMac{0x02, 0x08, 0xDC, 0x11, 0x22, 0x33};
constexpr uint32_t kSettle = Ipv4LinkLocalProvider::kSettleMs;
}

int main() {
    // ── deterministic candidate from the MAC ────────────────────────────
    {
        FakeInterface a, b;
        Ipv4LinkLocalProvider p1{kMac, a};
        Ipv4LinkLocalProvider p2{kMac, b};
        p1.begin();
        p2.begin();
        p1.poll(0);
        p2.poll(0);
        check(p1.candidate() == p2.candidate(), "same MAC -> same first candidate");
        const auto c = p1.candidate();
        check(c[0] == 169u && c[1] == 254u && c[2] >= 1u && c[2] <= 254u,
              "candidate is a valid 169.254/16 address");
        check(a.has_address && a.last_address[0] == 169u,
              "first poll announces the candidate through the interface");
    }

    // ── settle with no conflict -> bound ───────────────────────────────
    {
        FakeInterface iface;
        Ipv4LinkLocalProvider p{kMac, iface};
        p.begin();
        check(p.poll(0).state == AddressPhase::acquiring, "settling reports acquiring");
        const auto& l = p.poll(kSettle + 100u);
        check(p.state() == State::bound, "bound after the settle window");
        check(l.state == AddressPhase::bound && l.has_address, "lease has the address");
        check(l.address[0] == 169u && l.source == rosette::net::AddressSource::zeroconf,
              "link-local address, zeroconf source");
        check(l.link_local() && !l.routable(), "link-local, not routable");
    }

    // ── conflict during settle -> new candidate ────────────────────────
    {
        FakeInterface iface;
        Ipv4LinkLocalProvider p{kMac, iface};
        p.begin();
        p.poll(0);
        const auto first = p.candidate();
        iface.signal_conflict();
        const auto& l = p.poll(500u);
        check(p.candidate() != first, "new candidate after a conflict");
        check(p.state() == State::settling, "re-enters settling");
        check(l.duplicate(), "the lease latches the conflict for one poll");
        check(!p.poll(600u).duplicate(), "and clears it next poll");
    }

    // ── MAX_CONFLICTS -> rate_limited ─────────────────────────────────
    {
        FakeInterface iface;
        Ipv4LinkLocalProvider p{kMac, iface};
        p.begin();
        p.poll(0);
        uint32_t t = 0u;
        for (int i = 0; i < Ipv4LinkLocalProvider::kMaxConflicts + 2 &&
                        p.state() != State::rate_limited; ++i) {
            iface.signal_conflict();
            p.poll(t += 100u);
        }
        check(p.state() == State::rate_limited, "10 conflicts -> rate_limited");
        check(p.poll(t).state == AddressPhase::acquiring, "rate_limited still reports acquiring");
        p.poll(t + Ipv4LinkLocalProvider::kRateLimitMs + 100u);
        check(p.state() == State::settling, "leaves rate_limited after the interval");
    }

    // ── bound: defend once, abandon on a repeat conflict ──────────────
    {
        FakeInterface iface;
        Ipv4LinkLocalProvider p{kMac, iface};
        p.begin();
        p.poll(0);
        p.poll(kSettle + 100u);
        check(p.state() == State::bound, "bound");
        const auto held = p.candidate();
        const int applies_before = iface.applies;

        iface.signal_conflict();
        p.poll(kSettle + 200u);
        check(p.state() == State::defending, "first conflict -> defending");
        check(iface.applies == applies_before + 1, "one defensive re-announce");
        check(p.candidate() == held, "still holding the address while defending");

        iface.signal_conflict();
        p.poll(kSettle + 300u);
        check(p.state() == State::settling && p.candidate() != held,
              "repeat conflict within DEFEND_INTERVAL -> relinquish, re-pick");
    }

    // ── stop() clears the interface address ───────────────────────────
    {
        FakeInterface iface;
        Ipv4LinkLocalProvider p{kMac, iface};
        p.begin();
        p.poll(0);
        p.poll(kSettle + 100u);
        const int clears_before = iface.clears;
        p.stop();
        check(iface.clears == clears_before + 1, "stop() clears the bound address");
        check(p.poll(0).state == AddressPhase::idle, "idle after stop()");
    }

    return test_summary();
}
