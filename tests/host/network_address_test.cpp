/**
 * @file network_address_test.cpp
 * @brief `NetworkAddress`: combines the per-family managers into one snapshot,
 *        publishes to the platform, fires one on_change, supervises the link.
 */

#include "fake_interface.hpp"
#include "fake_runtime.hpp"
#include "mock_provider.hpp"
#include "test_util.hpp"

#include "rosette/net/address.hpp"

using namespace rosette::net;

int main() {
    // ── v4 + v6 combine into one snapshot; platform publish; on_change ────
    {
        fake::reset();
        MockProvider dhcp{AddressFamily::ipv4};
        MockProvider slaac{AddressFamily::ipv6};
        AddressProvider* v4l[] = {&dhcp};
        AddressProvider* v6l[] = {&slaac};
        AddressManager v4{v4l};
        AddressManager v6{v6l};
        NetworkAddress net{v4, &v6};

        int changes = 0;
        bool last_assigned = false;
        net.on_change([&](const AddressSnapshot& s) {
            ++changes;
            last_assigned = s.assigned;
        });

        net.begin();
        dhcp.bind({192u, 168u, 1u, 50u});
        slaac.bind_v6();
        net.poll(fake::now());

        check(changes == 1 && last_assigned, "on_change fires once when the interface comes up");
        const AddressSnapshot& s = net.snapshot();
        check(s.v4.has_address() && s.v6.has_address(), "both families present");
        check(s.v4.routable(), "v4 routable");
        check(s.v6.link_local(), "v6 link-local (fe80::)");
        check(fake::ipv4() == 0xC0A80132u, "IPv4 published to the platform");
        check(fake::ipv6()[0] == 0xFEu, "IPv6 published to the platform");
        check(net.stable() && net.has_address(), "stable + has_address");

        dhcp.set_phase(AddressPhase::failed);
        slaac.set_phase(AddressPhase::idle);
        net.poll(fake::now());  // exhausts v4 -> backoff; v6 gone
        net.poll(fake::now());
        check(!net.has_address() && !last_assigned, "on_change fires again on loss");
        check(fake::ipv4() == 0u, "IPv4 cleared on the platform");
    }

    // ── v4-only: wait_stable + has_address ─────────────────────────
    {
        fake::reset();
        MockProvider dhcp;
        AddressProvider* v4l[] = {&dhcp};
        AddressManager v4{v4l};
        NetworkAddress net{v4};
        net.begin();
        check(!net.wait(1000u), "times out while acquiring");
        check(!net.has_address(), "no address yet");

        dhcp.bind();
        check(net.wait(1000u), "returns once v4 binds");
        check(net.has_address(), "has_address() agrees");
    }

    // ── conflict surfaces through on_change (snapshot.v4.duplicate()) ─────
    {
        fake::reset();
        MockProvider dhcp;
        AddressProvider* v4l[] = {&dhcp};
        AddressManager v4{v4l};
        NetworkAddress net{v4};
        int saw_conflict = 0;
        net.on_change([&](const AddressSnapshot& s) {
            if (s.v4.duplicate()) ++saw_conflict;
        });
        net.begin();
        dhcp.bind();
        net.poll(fake::now());
        dhcp.signal_conflict();
        net.poll(fake::now());
        check(saw_conflict == 1, "conflict visible in on_change snapshot");
    }

    // ── link supervision: carrier drop releases, return re-acquires ──────
    {
        fake::reset();
        FakeInterface iface;
        MockProvider dhcp;
        AddressProvider* v4l[] = {&dhcp};
        AddressManager v4{v4l};
        NetworkAddress net{v4, nullptr, &iface};

        net.begin();
        dhcp.bind();
        net.poll(fake::now());
        check(net.has_address() && net.link_up(), "up with an address");
        const int begins_before = dhcp.begins;

        iface.link = false;
        net.poll(fake::now());
        check(!net.link_up() && !net.has_address(), "carrier drop releases the address");
        check(dhcp.started == false, "provider stopped while link is down");

        iface.link = true;
        net.poll(fake::now());
        check(dhcp.begins > begins_before, "carrier return kicks a fresh acquisition");
        dhcp.bind();
        net.poll(fake::now());
        check(net.has_address(), "re-acquires after the link returns");
    }

    return test_summary();
}
