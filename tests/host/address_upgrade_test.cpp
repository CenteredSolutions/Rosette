/**
 * @file address_upgrade_test.cpp
 * @brief The IPv4 provider list DHCP → link-local under one `AddressManager`:
 *        falling back, probing DHCP while link-local holds, and upgrading.
 *
 * The providers are the real `DhcpAddressProvider` and `Ipv4LinkLocalProvider`
 * over a `FakeInterface` (which records every address write and holds one
 * conflict latch, like the chip) and a scripted DHCP network. Each TC starts
 * from a board that found no DHCP server at boot and holds a 169.254 address.
 *
 * References: RFC 3927 §1.9, §2.5; RFC 2131 §4.4.1, §4.4.5.
 */

#include <array>
#include <cstdint>

#include "fake_dhcp_network.hpp"
#include "fake_interface.hpp"
#include "rosette/net/address_manager.hpp"
#include "rosette/net/ipv4ll_provider.hpp"
#include "rosette/net/wiznet/dhcp_provider.hpp"
#include "test_util.hpp"

namespace {

namespace net = rosette::net;
namespace wiz = rosette::net::wiznet;

constexpr std::array<uint8_t, 6> kMac{0x00u, 0x08u, 0xDCu, 0x54u, 0x75u, 0x30u};
constexpr uint32_t kProbeMs = 60000u;
constexpr uint8_t kDiscover = 1u;
constexpr uint8_t kDecline = 4u;
constexpr uint8_t kAck = 5u;

net::AddressManagerPolicy policy() {
    net::AddressManagerPolicy p{};
    p.upgrade_probe_ms = kProbeMs;
    return p;
}

std::array<uint8_t, 16> v4(std::array<uint8_t, 4> a) {
    return {a[0], a[1], a[2], a[3]};
}

/// A board with DHCP then IPv4 link-local, booted without a DHCP server.
struct Board {
    FakeInterface iface;
    fake::FakeDhcpNetwork dhcp_net;
    wiz::DhcpAddressProvider dhcp{dhcp_net, kMac, iface};
    net::Ipv4LinkLocalProvider zeroconf{kMac, iface};
    net::AddressProvider* list[2] = {&dhcp, &zeroconf};
    net::AddressManager mgr{list, policy()};
    uint32_t now = 0u;
    std::array<uint8_t, 16> link_local{};

    Board() {
        mgr.begin();
        run_until([this] { return mgr.stable(); }, 30000u);
        link_local = v4(zeroconf.candidate());
    }

    template <typename Done>
    bool run_until(Done done, uint32_t limit_ms) {
        for (const uint32_t end = now + limit_ms; now < end; now += 100u) {
            mgr.poll(now);
            if (done()) {
                return true;
            }
        }

        return false;
    }

    void run_for(uint32_t ms) {
        run_until([] { return false; }, ms);
    }

    std::size_t count_sent(uint8_t type) const {
        std::size_t n = 0u;
        for (const auto& m : dhcp_net.sent) {
            n += m.type == type ? 1u : 0u;
        }

        return n;
    }

    bool holds_link_local() const { return iface.has_address && iface.last_address == link_local; }

    /// Runs up to the first DISCOVER of the next upgrade probe.
    bool reach_probe() {
        const std::size_t before = count_sent(kDiscover);
        return run_until([&] { return count_sent(kDiscover) > before; }, kProbeMs + 1000u);
    }
};

// TC-NET-UPG-001: a DHCP probe while link-local holds the chip never writes
// the interface, so the link-local address stays in use.
void tc_probe_keeps_link_local() {
    Board b;
    check(b.mgr.lease().source == net::AddressSource::zeroconf && b.holds_link_local(),
          "001: no server at boot, the board holds its link-local address");
    const int applies = b.iface.applies;
    check(b.reach_probe() && b.mgr.probing(), "001: a DHCP probe starts while link-local holds");
    b.run_for(5000u);
    check(b.iface.applies == applies && b.holds_link_local(),
          "001: the probe made no interface write; link-local stays on the chip");
    check(b.mgr.stable() && b.mgr.lease().source == net::AddressSource::zeroconf,
          "001: the manager keeps reporting the link-local lease");
}

// TC-NET-UPG-002: DHCP holds a socket only inside an exchange window.
void tc_socket_only_while_probing() {
    Board b;
    check(b.dhcp_net.open_now == 0, "002: no DHCP socket while link-local holds between probes");
    (void)b.reach_probe();
    check(b.dhcp_net.open_now == 1, "002: one socket during the probe");
    b.run_for(11000u);
    check(b.dhcp_net.open_now == 0, "002: released when the probe window ends");
}

// TC-NET-UPG-003: a conflict during a probe goes to the link-local holder,
// which defends (RFC 3927 §2.5); DHCP sends no DECLINE.
void tc_conflict_goes_to_holder() {
    Board b;
    (void)b.reach_probe();
    b.iface.signal_conflict();
    b.run_for(100u);
    check(b.zeroconf.state() == net::Ipv4LinkLocalProvider::State::defending
              && b.zeroconf.conflict_count() == 1u,
          "003: link-local saw the conflict and defends");
    check(b.count_sent(kDecline) == 0u, "003: DHCP declined nothing");
    check(b.holds_link_local(), "003: the defended address stays on the chip");

    // Whatever order the providers are polled in, a prober leaves the latch alone.
    FakeInterface iface;
    fake::FakeDhcpNetwork dhcp_net;
    wiz::DhcpAddressProvider prober{dhcp_net, kMac, iface};
    (void)prober.begin_probe();
    iface.signal_conflict();
    (void)prober.poll(0u);
    check(iface.pending_conflict && iface.applies == 0,
          "003: a probing DHCP provider neither reads the conflict latch nor writes");
}

// TC-NET-UPG-004: a probe that binds takes over: link-local stops, the lease
// is written last, and the manager reports DHCP.
void tc_successful_probe_upgrades() {
    Board b;
    (void)b.reach_probe();
    const int clears = b.iface.clears;
    b.dhcp_net.answer({}, kMac);  // OFFER
    b.run_for(100u);
    fake::ServerReply ack{};
    ack.type = kAck;
    b.dhcp_net.answer(ack, kMac);
    b.run_for(100u);
    check(b.mgr.stable() && b.mgr.lease().source == net::AddressSource::dhcp,
          "004: the manager reports the DHCP lease");
    check(b.mgr.lease().address() == v4({192u, 168u, 32u, 79u}), "004: with the leased address");
    check(b.iface.has_address && b.iface.last_address == v4({192u, 168u, 32u, 79u}),
          "004: the chip holds the leased address");
    check(b.iface.clears == clears + 1, "004: one release of link-local before the write");
    check(b.zeroconf.state() == net::Ipv4LinkLocalProvider::State::idle,
          "004: link-local stopped (RFC 3927 §1.9)");
    check(!b.mgr.probing(), "004: nothing ahead of DHCP to probe");
}

// TC-NET-UPG-005: a probe that fails leaves link-local alone and the next one
// starts upgrade_probe_ms later.
void tc_failed_probe_retries() {
    Board b;
    (void)b.reach_probe();
    const int applies = b.iface.applies;
    check(b.run_until([&] { return !b.mgr.probing(); }, 16000u), "005: the probe gives up");
    check(b.iface.applies == applies && b.holds_link_local(), "005: link-local untouched");
    const uint32_t failed_at = b.now;
    check(b.reach_probe(), "005: another probe follows");
    check(b.now - failed_at >= kProbeMs, "005: no sooner than upgrade_probe_ms");
}

// TC-NET-UPG-006: DHCP as holder renews at T1 without touching the chip.
void tc_holder_renewal_keeps_address() {
    Board b;
    (void)b.reach_probe();
    b.dhcp_net.answer({}, kMac);
    b.run_for(100u);
    fake::ServerReply ack{};
    ack.type = kAck;
    b.dhcp_net.answer(ack, kMac);
    b.run_for(100u);
    const int applies = b.iface.applies;
    const std::size_t sent = b.dhcp_net.sent.size();
    b.run_until([&] { return b.dhcp_net.sent.size() > sent; }, 1801000u);
    check(b.dhcp_net.last().ciaddr == std::array<uint8_t, 4>{192u, 168u, 32u, 79u},
          "006: RENEWING REQUEST at T1");
    b.dhcp_net.answer(ack, kMac);
    b.run_for(200u);
    check(b.dhcp.client().state() == wiz::DhcpClientState::bound, "006: renewed");
    check(b.iface.applies == applies && b.iface.has_address, "006: no interface write, no clear");
    check(b.mgr.lease().source == net::AddressSource::dhcp, "006: still reported as DHCP");
}

}  // namespace

int main() {
    tc_probe_keeps_link_local();
    tc_socket_only_while_probing();
    tc_conflict_goes_to_holder();
    tc_successful_probe_upgrades();
    tc_failed_probe_retries();
    tc_holder_renewal_keeps_address();
    return test_summary();
}
