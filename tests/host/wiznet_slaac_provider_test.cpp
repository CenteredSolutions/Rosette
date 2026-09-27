/**
 * @file wiznet_slaac_provider_test.cpp
 * @brief `WiznetSlaacProvider` against a register-file model of the W6100.
 *
 * Scripts the chip's socket-less DAD outcomes, delivers router advertisements
 * to the IPRAW6 socket the provider claims, and checks the addresses written to
 * LLAR / GUAR / SUB6R / GA6R, the lease, and the socket's lifetime.
 *
 * Test cases: TC-NET-SLAAC-001 (EUI-64), -002 (link-local after DAD; global
 * and default router from the captured advertisement; socket released),
 * -003 (duplicate link-local stops IPv6), -004 (no router: three solicitations,
 * then RFC 7559 backoff), -005 (non-autonomous prefix: no global address),
 * -006 (refresh at half the router lifetime; router and valid lifetime
 * expiry), -007 (no free socket: addresses kept, solicitation retried).
 */

#include "fake_runtime.hpp"
#include "fake_wiznet_chip.hpp"
#include "test_util.hpp"

#include "rosette/net/wiznet/interface.hpp"
#include "rosette/net/wiznet/slaac_provider.hpp"

#include <array>
#include <cstdint>
#include <memory>
#include <vector>

// The header-only host-test target does not link `rosette_net`.
namespace rosette::net {
const char* socket_error_string(SocketError) noexcept { return "stub"; }
}  // namespace rosette::net

namespace {

namespace wiz = rosette::net::wiznet;
using Chip = wiz::W6100;
using Chipset = fake::FakeWiznetChip<Chip>;
using Device = wiz::WiznetDevice<Chip, Chipset>;
using Sockets = wiz::WiznetSocketProvider<Chip, Chipset>;
using Provider = wiz::WiznetSlaacProvider<Chip, Chipset>;
using rosette::net::AddressPhase;
using Bytes = std::vector<uint8_t>;

const std::array<uint8_t, 6> kMac{0x00, 0x1C, 0x42, 0x14, 0x87, 0x37};
const Bytes kLinkLocal{0xFE, 0x80, 0, 0, 0, 0, 0, 0, 0x02, 0x1C, 0x42, 0xFF, 0xFE, 0x14, 0x87, 0x37};
const Bytes kGlobal{0x26, 0x03, 0x80, 0x80, 0xF8, 0x06, 0xF4, 0x50, 0x02, 0x1C, 0x42, 0xFF, 0xFE, 0x14, 0x87, 0x37};
const Bytes kRouter{0xFE, 0x80, 0, 0, 0, 0, 0, 0, 0x9C, 0x05, 0xD6, 0xFF, 0xFE, 0xBB, 0xC9, 0x0F};
const Bytes kMask64{0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0, 0, 0, 0, 0, 0, 0, 0};
const Bytes kZero(16, 0);

/// The router advertisement captured on the test LAN (prefix option first).
Bytes captured_ra(uint16_t router_lifetime_s = 1800u, uint8_t prefix_flags = 0xC0u, uint32_t valid_s = 86400u)
{
    Bytes m{0x86, 0x00, 0x83, 0xcd, 0x40, 0x08, static_cast<uint8_t>(router_lifetime_s >> 8),
            static_cast<uint8_t>(router_lifetime_s), 0, 0, 0, 0, 0, 0, 0, 0,
            0x03, 0x04, 0x40, prefix_flags,
            static_cast<uint8_t>(valid_s >> 24), static_cast<uint8_t>(valid_s >> 16),
            static_cast<uint8_t>(valid_s >> 8), static_cast<uint8_t>(valid_s),
            static_cast<uint8_t>(valid_s >> 24), static_cast<uint8_t>(valid_s >> 16),
            static_cast<uint8_t>(valid_s >> 8), static_cast<uint8_t>(valid_s), 0, 0, 0, 0,
            0x26, 0x03, 0x80, 0x80, 0xf8, 0x06, 0xf4, 0x50, 0, 0, 0, 0, 0, 0, 0, 0,
            0x05, 0x01, 0, 0, 0, 0, 0x05, 0xdc,
            0x01, 0x01, 0x9e, 0x05, 0xd6, 0xbb, 0xc9, 0x0f};
    return m;
}

/// The octets a W6100 IPRAW6 socket stores for one received datagram.
Bytes ipraw6_record(const Bytes& source, const Bytes& payload)
{
    const auto len = static_cast<uint16_t>(payload.size());
    Bytes r{static_cast<uint8_t>(0x80u | ((len >> 8) & 0x07u)), static_cast<uint8_t>(len)};
    r.insert(r.end(), source.begin(), source.end());
    r.insert(r.end(), payload.begin(), payload.end());
    return r;
}

Bytes vec(const std::array<uint8_t, 16>& a) { return {a.begin(), a.end()}; }

struct Rig {
    Chipset chip;
    Device device{chip};
    Sockets sockets{device};
    wiz::WiznetInterface<Chip, Chipset> iface{device};
    Provider slaac{device, sockets, iface, kMac};
    uint32_t now = 0u;

    Rig() { device.init_socket_mode(kMac); }

    const rosette::net::AddressLease& step(uint32_t ms = 10u)
    {
        now += ms;
        fake::set_now(now);
        return slaac.poll(now);
    }

    /// Socket the provider holds in IPRAW6 mode, or socket_count() when none.
    uint8_t raw_socket() const
    {
        for (uint8_t s = 0; s < Chip::socket_count(); ++s) {
            if (chip.u8(Chip::socket_status(s)) == Chip::socket_status_ipraw6()) return s;
        }
        return Chip::socket_count();
    }

    std::size_t count(uint8_t command) const
    {
        std::size_t n = 0;
        for (const auto& c : chip.socketless) n += c.command == command ? 1u : 0u;
        return n;
    }

    /// Link-local DAD passes; returns once the provider is soliciting.
    void to_soliciting()
    {
        chip.socketless_reply = Chip::socketless_status_timeout();
        slaac.begin();
        (void)step();
    }
};

}  // namespace

int main()
{
    constexpr uint8_t kNs = 0x04u;
    constexpr uint8_t kRs = 0x02u;

    // TC-NET-SLAAC-001
    {
        const auto iid = wiz::modified_eui64(kMac);
        check(Bytes(iid.begin(), iid.end()) == Bytes(kLinkLocal.begin() + 8, kLinkLocal.end()),
              "EUI-64: U/L bit inverted, ff:fe inserted (matches Linux for the same MAC)");
    }

    // TC-NET-SLAAC-002
    {
        Rig r;
        r.chip.socketless_reply = Chip::socketless_status_timeout();    // nobody answers DAD
        r.slaac.begin();
        check(r.chip.socketless.size() == 1 && r.chip.socketless[0].command == kNs &&
                  r.chip.socketless[0].target == kLinkLocal && r.chip.socketless[0].retry_time == 10000u,
              "begin(): DAD for the EUI-64 link-local address, RetransTimer 1 s");
        check(r.chip.u8(Chip::common_icmp6_block()) == Chip::icmp6_block_router_advert(),
              "begin(): router advertisements diverted from the chip (ICMP6BLKR.RA)");
        check(r.chip.bytes(Chip::common_ip6_link_local(), 16) == kZero, "tentative address not in LLAR");

        const auto& l1 = r.step();
        check(l1.state == AddressPhase::bound && vec(l1.address) == kLinkLocal && l1.link_local() &&
                  r.chip.bytes(Chip::common_ip6_link_local(), 16) == kLinkLocal,
              "DAD silence: link-local bound and in LLAR");
        const uint8_t s = r.raw_socket();
        check(s < Chip::socket_count() && r.chip.u8(Chip::socket_next_header(s)) == 58u,
              "router discovery holds one IPRAW6 socket for ICMPv6");
        check(r.count(kRs) == 1u, "router solicitation sent");

        r.chip.deliver(s, ipraw6_record(kRouter, captured_ra()));
        (void)r.step();
        check(r.raw_socket() == Chip::socket_count(), "advertisement received: socket released");
        check(r.chip.socketless.back().command == kNs && r.chip.socketless.back().target == kGlobal,
              "advertised /64: DAD for prefix + interface identifier");
        const auto& l2 = r.step();
        check(l2.state == AddressPhase::bound && vec(l2.address) == kGlobal && l2.routable() &&
                  l2.lease_remaining_s == 86400u && vec(l2.gateway_v6) == kRouter,
              "global address reported with valid lifetime and default router");
        check(r.chip.bytes(Chip::common_ip6_global(), 16) == kGlobal &&
                  r.chip.bytes(Chip::common_ip6_prefix_mask(), 16) == kMask64 &&
                  r.chip.bytes(Chip::common_ip6_gateway(), 16) == kRouter &&
                  r.chip.bytes(Chip::common_ip6_link_local(), 16) == kLinkLocal,
              "GUAR, SUB6R, GA6R programmed; LLAR kept");

        r.slaac.stop();
        check(r.chip.bytes(Chip::common_ip6_global(), 16) == kZero &&
                  r.chip.bytes(Chip::common_ip6_link_local(), 16) == kZero &&
                  r.chip.bytes(Chip::common_ip6_gateway(), 16) == kZero,
              "stop(): addresses and router withdrawn");
    }

    // TC-NET-SLAAC-003
    {
        Rig r;
        r.chip.socketless_reply = Chip::socketless_status_neighbor_solicit();    // a neighbor defends it
        r.slaac.begin();
        const auto& l = r.step();
        check(l.state == AddressPhase::failed && l.duplicate() && !l.has_address &&
                  r.chip.bytes(Chip::common_ip6_link_local(), 16) == kZero && r.raw_socket() == Chip::socket_count(),
              "duplicate link-local: failed, flagged, never in LLAR, no socket held");
    }

    // TC-NET-SLAAC-004
    {
        Rig r;
        r.to_soliciting();
        for (int i = 0; i < 30; ++i) (void)r.step(500u);    // 15 s
        check(r.count(kRs) == 3u && r.raw_socket() == Chip::socket_count(),
              "no router: MAX_RTR_SOLICITATIONS (3) sent, then the socket is released");
        check(vec(r.step().address) == kLinkLocal, "no router: link-local stays bound");
        for (int i = 0; i < 55; ++i) (void)r.step(1000u);
        check(r.count(kRs) == 3u, "quiet during the first 60 s backoff");
        for (int i = 0; i < 5; ++i) (void)r.step(1000u);    // round ended ~12 s; +60 s backoff
        check(r.count(kRs) == 4u && r.raw_socket() < Chip::socket_count(),
              "after the backoff: a new round of solicitation");
    }

    // TC-NET-SLAAC-005
    {
        Rig r;
        r.to_soliciting();
        r.chip.deliver(r.raw_socket(), ipraw6_record(kRouter, captured_ra(1800u, 0x80u)));
        for (int i = 0; i < 5; ++i) (void)r.step(100u);
        check(vec(r.step().address) == kLinkLocal && r.chip.bytes(Chip::common_ip6_global(), 16) == kZero &&
                  r.raw_socket() == Chip::socket_count(),
              "on-link-only prefix: no global address, socket released");
    }

    // TC-NET-SLAAC-006
    {
        Rig r;
        r.to_soliciting();
        r.chip.deliver(r.raw_socket(), ipraw6_record(kRouter, captured_ra(60u, 0xC0u, 300u)));
        (void)r.step();
        (void)r.step();
        check(vec(r.step().address) == kGlobal, "short lifetimes: global address configured");
        const std::size_t rs_before = r.count(kRs);
        for (int i = 0; i < 31; ++i) (void)r.step(1000u);
        check(r.count(kRs) == rs_before + 1u && r.raw_socket() < Chip::socket_count(),
              "refresh solicitation at half the router lifetime (30 s)");
        for (int i = 0; i < 35; ++i) (void)r.step(1000u);    // no answer: router lifetime (60 s) ends
        check(r.chip.bytes(Chip::common_ip6_gateway(), 16) == kZero &&
                  r.chip.bytes(Chip::common_ip6_global(), 16) == kGlobal,
              "router lifetime over: GA6R withdrawn, global address kept");
        for (int i = 0; i < 240; ++i) (void)r.step(1000u);   // valid lifetime (300 s) ends
        check(vec(r.step().address) == kLinkLocal && r.chip.bytes(Chip::common_ip6_global(), 16) == kZero,
              "valid lifetime over: global withdrawn, link-local reported");
    }

    // TC-NET-SLAAC-007
    {
        Rig r;
        std::vector<std::unique_ptr<rosette::net::Socket>> hog;
        for (uint8_t i = 0; i < Chip::socket_count(); ++i) hog.push_back(r.sockets.open());
        r.to_soliciting();
        check(vec(r.step().address) == kLinkLocal && r.count(kRs) == 0u,
              "no free socket: link-local kept, no solicitation");
        hog.pop_back();
        for (int i = 0; i < 3; ++i) (void)r.step(500u);
        check(r.count(kRs) == 1u && r.raw_socket() < Chip::socket_count(),
              "socket freed: solicitation starts within a second");
    }

    return test_summary();
}
