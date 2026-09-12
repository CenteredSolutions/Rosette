/**
 * @file address_manager_test.cpp
 * @brief `AddressManager`: serial list, first-to-bind wins, fall-through on
 *        failure / timeout, restart-from-top after exhaustion or loss.
 */

#include "fake_runtime.hpp"
#include "mock_provider.hpp"
#include "test_util.hpp"

#include "rosette/net/address_manager.hpp"

using namespace rosette::net;

namespace {
AddressManagerPolicy fast_policy() {
    AddressManagerPolicy p{};
    p.provider_timeout_ms = 5000u;
    p.retry_backoff_ms = 10000u;
    return p;
}
}  // namespace

int main() {
    // ── first provider to bind wins ───────────────────────────────────────
    {
        fake::reset();
        MockProvider dhcp, zc;
        AddressProvider* list[] = {&dhcp, &zc};
        AddressManager mgr{list, fast_policy()};
        mgr.begin();
        check(dhcp.started && !zc.started, "provider[0] started, provider[1] dormant");
        check(mgr.state() == AddressState::acquiring, "acquiring");

        dhcp.bind();
        mgr.poll(fake::now());
        check(mgr.stable(), "stable once a provider binds");
        check(!zc.started, "the fallback is never started");
        check(mgr.lease().has_address, "lease exposes the bound address");
    }

    // ── a failed provider hands off to the next ───────────────────────────
    {
        fake::reset();
        MockProvider a, b;
        AddressProvider* list[] = {&a, &b};
        AddressManager mgr{list, fast_policy()};
        mgr.begin();

        a.set_phase(AddressPhase::failed);
        mgr.poll(fake::now());
        check(!a.started && b.started, "advanced to provider[1] on failed");

        b.bind({169u, 254u, 3u, 9u});
        mgr.poll(fake::now());
        check(mgr.lease().has_address && mgr.lease().link_local(),
              "publishes the bound address; 169.254 is link-local");
    }

    // ── a provider that never binds times out ─────────────────────────────
    {
        fake::reset();
        MockProvider a, b;
        AddressProvider* list[] = {&a, &b};
        AddressManager mgr{list, fast_policy()};
        mgr.begin();
        a.set_phase(AddressPhase::acquiring);
        mgr.poll(fake::now());
        check(a.started && !b.started, "still on provider[0] before the timeout");
        fake::advance(6000u);
        mgr.poll(fake::now());
        check(b.started, "advanced on the provider timeout");
    }

    // ── exhausting the list: backoff, then restart from the top ───────────
    {
        fake::reset();
        MockProvider a, b;
        AddressProvider* list[] = {&a, &b};
        AddressManager mgr{list, fast_policy()};
        mgr.begin();

        a.set_phase(AddressPhase::failed);
        mgr.poll(fake::now());                       // -> b
        b.set_phase(AddressPhase::failed);
        mgr.poll(fake::now());                       // list exhausted
        check(mgr.state() == AddressState::acquiring, "still acquiring during the backoff");
        const int a_begins = a.begins;
        mgr.poll(fake::now());
        check(a.begins == a_begins, "does not restart before the backoff elapses");

        fake::advance(11000u);
        a.set_phase(AddressPhase::acquiring);
        mgr.poll(fake::now());
        check(a.begins > a_begins, "restarts from provider[0] after the backoff");

        a.bind();
        mgr.poll(fake::now());
        check(mgr.stable(), "recovers to stable");
    }

    // ── a bound provider that is later lost restarts the cascade ──────────
    {
        fake::reset();
        MockProvider dhcp, zc;
        AddressProvider* list[] = {&dhcp, &zc};
        AddressManager mgr{list, fast_policy()};
        mgr.begin();
        dhcp.bind();
        mgr.poll(fake::now());
        check(mgr.stable(), "stable");

        // Renewal in progress must NOT drop us.
        dhcp.set_phase(AddressPhase::acquiring);
        mgr.poll(fake::now());
        check(mgr.stable(), "acquiring (renewal) keeps us stable");
        check(!zc.started, "and does not advance the cascade");

        // Hard loss -> full restart from the top after the backoff.
        dhcp.set_phase(AddressPhase::failed);
        mgr.poll(fake::now());
        check(!mgr.stable() && !zc.started, "hard loss exhausts, waits (not straight to zc)");
        const int dhcp_begins = dhcp.begins;
        fake::advance(11000u);
        dhcp.set_phase(AddressPhase::acquiring);
        mgr.poll(fake::now());
        check(dhcp.begins > dhcp_begins, "re-cascade restarts from provider[0]");
    }

    // ── no providers: stays down-ish, never crashes ──────────────────────
    {
        fake::reset();
        AddressProvider* none[1] = {nullptr};
        AddressManager mgr{none, 0u, fast_policy()};
        mgr.begin();
        mgr.poll(fake::now());
        check(!mgr.stable(), "empty manager never claims stable");
    }

    return test_summary();
}
