// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file wiznet_arp_resolver_test.cpp
 * @brief IPv4 sends through the ARP cache on the W5100S and the W6100: no
 *        inline ARP, next-hop choice, group and broadcast MACs, the W6x00's
 *        Sn_MR2 mode, sharing the socket-less engine with SLAAC, and each
 *        socket's cached route.
 *
 * Each case opens sockets through WiznetSocketProvider with a
 * WiznetArpResolver attached and runs on `fake::FakeWiznetChip`, whose
 * `arp_hosts` decide which next hops answer the chip's socket-less ARP.
 * A datagram the resolver drops never reaches the chip's SEND, so
 * `chip.sent` shows exactly what went on the wire and to which MAC.
 *
 * Build and run: `cmake -S tests/host -B build/tests-host && ctest --test-dir build/tests-host`.
 */

#include "fake_clock.hpp"
#include "fake_wiznet_chip.hpp"
#include "test_util.hpp"

#include "ports/net/wiznet/arp_cache.hpp"
#include "ports/net/wiznet/arp_resolver.hpp"
#include "ports/net/wiznet/socket_provider.hpp"
#include "ports/net/wiznet/w5100s.hpp"
#include "ports/net/wiznet/w6x00.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace {

namespace net = rosette::net;
namespace wiz = rosette::net::wiznet;

const std::array<uint8_t, 4> kSelf{192, 168, 1, 2};
const std::array<uint8_t, 4> kMask{255, 255, 255, 0};
const std::array<uint8_t, 4> kGateway{192, 168, 1, 1};
const std::array<uint8_t, 4> kHost{192, 168, 1, 10};
const std::array<uint8_t, 4> kRemote{10, 0, 0, 5};
const std::array<uint8_t, 6> kHostMac{0x02, 0x00, 0x00, 0x00, 0x00, 0x10};
const std::array<uint8_t, 6> kGatewayMac{0x02, 0x00, 0x00, 0x00, 0x00, 0x01};
const std::vector<uint8_t> kPayload{'r', 't', 'p', 's'};

std::vector<uint8_t> v(const std::array<uint8_t, 4>& a) {
    return {a.begin(), a.end()};
}

std::vector<uint8_t> v(const std::array<uint8_t, 6>& a) {
    return {a.begin(), a.end()};
}

std::string tag(const char* chip, const char* what) {
    return std::string(chip) + ": " + what;
}

/// @brief A chip, its driver, an 8-entry ARP cache, and a provider routing through it.
template <typename Chip>
struct Rig {
    using Chipset = fake::FakeWiznetChip<Chip>;
    Chipset chip;
    wiz::WiznetDevice<Chip, Chipset> device{chip};
    wiz::ArpCacheStorage<8> table;
    wiz::WiznetArpResolver<Chip, Chipset> arp{device, table};
    wiz::WiznetSocketProvider<Chip, Chipset> sockets{device};

    /// `gateway` is configured too; its background resolution is then the first probe.
    explicit Rig(bool attach = true, bool gateway = false) {
        fake::reset();
        device.init_socket_mode({0x02, 0x08, 0xDC, 0x00, 0x00, 0x02});
        device.configure_ipv4(kSelf, kMask, gateway ? kGateway : std::array<uint8_t, 4>{});
        if (attach) {
            sockets.attach_arp_resolver(&arp);
        }

        chip.arp_hosts[kHost] = kHostMac;
        chip.arp_hosts[kGateway] = kGatewayMac;
    }

    std::unique_ptr<net::Socket> open(net::SocketDomain domain = net::SocketDomain::ipv4,
                                      uint16_t          port = 7400u) {
        auto socket = sockets.open(domain);
        net::SocketAddress local{};
        local.domain = domain;
        local.port = port;
        (void)socket->bind(local);
        return socket;
    }

    void send(net::Socket& socket, const std::array<uint8_t, 4>& to) {
        (void)socket.send_to(kPayload.data(), kPayload.size(),
                             net::make_socket_address_v4(to, 7411u));
    }

    void poll() { arp.poll(fake::now()); }
};

// TC-NET-WIZ-ARP-010: a first send is dropped and resolved in the background, reported as
// accepted with would_block; the next one goes to the cached MAC with no inline ARP.
template <typename Chip>
void first_contact(const char* name) {
    Rig<Chip> rig;
    auto socket = rig.open();
    const std::ptrdiff_t dropped = socket->send_to(kPayload.data(), kPayload.size(),
                                                   net::make_socket_address_v4(kHost, 7411u));
    check(rig.chip.sent.empty(),
          tag(name, "ARP-010: an unresolved send is dropped, not sent").c_str());
    check(dropped == static_cast<std::ptrdiff_t>(kPayload.size())
              && socket->last_error() == net::SocketError::would_block,
          tag(name, "ARP-010: the drop returns the length and reports would_block").c_str());
    check(rig.chip.socketless.size() == 1u
              && rig.chip.socketless[0].command == Chip::socketless_command_arp4()
              && rig.chip.socketless[0].target == v(kHost)
              && rig.chip.socketless[0].retry_time == 1000u,
          tag(name, "ARP-010: the miss starts a socket-less ARP for the host").c_str());
    check(rig.arp.drops() == 1u, tag(name, "ARP-010: the drop is counted").c_str());

    rig.poll();
    rig.send(*socket, kHost);
    const bool sent = rig.chip.sent.size() == 1u;
    check(sent && rig.chip.sent[0].dest_mac == v(kHostMac) && rig.chip.sent[0].dest_ip == v(kHost)
              && rig.chip.sent[0].payload == kPayload,
          tag(name, "ARP-010: the next send goes to the answered MAC").c_str());
    const uint8_t expected =
        Chip::dest_mac_per_socket() ? Chip::command_send() : Chip::command_send_mac();
    check(sent && rig.chip.sent[0].command == expected,
          tag(name, "ARP-010: sent without the chip's ARP (SEND_MAC / DHAM SEND)").c_str());
    check(socket->last_error() == net::SocketError::none,
          tag(name, "ARP-010: a send that reached the chip reports no error").c_str());
    check(rig.chip.socketless.size() == 1u, tag(name, "ARP-010: a hit starts no ARP").c_str());
}

// TC-NET-WIZ-ARP-011 / -012: off-subnet goes to the gateway's MAC; a 0.0.0.0 mask is all on-link.
template <typename Chip>
void next_hop(const char* name) {
    Rig<Chip> rig(true, true);
    auto socket = rig.open();
    rig.poll();
    check(rig.chip.socketless.size() == 1u && rig.chip.socketless[0].target == v(kGateway),
          tag(name, "ARP-011: a new configuration resolves its gateway before any send").c_str());
    rig.poll();
    rig.send(*socket, kRemote);
    check(rig.chip.sent.size() == 1u && rig.chip.sent[0].dest_mac == v(kGatewayMac)
              && rig.chip.sent[0].dest_ip == v(kRemote),
          tag(name, "ARP-011: the first off-subnet send goes to the gateway's MAC").c_str());

    rig.device.configure_ipv4({169, 254, 3, 4}, {}, {});
    rig.send(*socket, kHost);
    check(rig.chip.socketless.size() == 2u && rig.chip.socketless[1].target == v(kHost),
          tag(name, "ARP-012: with mask 0.0.0.0 every destination is on-link").c_str());

    rig.device.configure_ipv4(kSelf, kMask, {});
    const auto commands = rig.chip.socketless.size();
    rig.send(*socket, kRemote);
    check(rig.chip.socketless.size() == commands && rig.chip.sent.size() == 1u,
          tag(name, "ARP-011: off-subnet with no gateway is dropped without ARP").c_str());
}

// TC-NET-WIZ-ARP-013: multicast and broadcast need no ARP and use their fixed MACs.
template <typename Chip>
void group_and_broadcast(const char* name) {
    Rig<Chip> rig;
    auto socket = rig.open();
    rig.send(*socket, {239, 255, 0, 1});
    rig.send(*socket, {255, 255, 255, 255});
    rig.send(*socket, {192, 168, 1, 255});
    const std::vector<uint8_t> group{0x01, 0x00, 0x5e, 0x7f, 0x00, 0x01};
    const std::vector<uint8_t> all(6u, 0xFFu);
    check(rig.chip.sent.size() == 3u && rig.chip.sent[0].dest_mac == group,
          tag(name, "ARP-013: a multicast group goes to its group MAC").c_str());
    check(rig.chip.sent.size() == 3u && rig.chip.sent[1].dest_mac == all
              && rig.chip.sent[2].dest_mac == all,
          tag(name, "ARP-013: limited and directed broadcast go to ff:ff:ff:ff:ff:ff").c_str());
    check(rig.chip.socketless.empty(), tag(name, "ARP-013: no ARP for group or broadcast").c_str());
}

// TC-NET-WIZ-ARP-016: a dead host costs nothing per send; a new address empties the cache.
template <typename Chip>
void dead_host(const char* name) {
    Rig<Chip> rig;
    const std::array<uint8_t, 4> kDead{192, 168, 1, 66};
    auto socket = rig.open();
    rig.send(*socket, kDead);
    rig.poll();
    for (int i = 0; i < 20; ++i) {
        rig.send(*socket, kDead);
    }

    check(rig.chip.sent.empty() && rig.chip.socketless.size() == 1u,
          tag(name, "ARP-016: sends to an unreachable host neither SEND nor ARP").c_str());
    check(rig.device.send_timeouts() == 0u, tag(name, "ARP-016: no SEND ever times out").c_str());
    fake::advance(1000u);
    rig.poll();
    check(rig.chip.socketless.size() == 2u,
          tag(name, "ARP-016: retried after the backoff").c_str());

    rig.device.configure_ipv4({192, 168, 7, 2}, kMask, {192, 168, 7, 1});
    rig.poll();
    check(rig.table.find(kDead) == nullptr && rig.table.size() == 1u
              && rig.table.find({192, 168, 7, 1}) != nullptr,
          tag(name, "ARP-016: a new configuration empties the cache but for its gateway").c_str());
}

// TC-NET-WIZ-ARP-014: W6x00 sockets carry Sn_MR2.DHAM_MANUAL exactly when the cache supplies MACs.
template <typename Chip>
void w6x00_mode2(const char* name) {
    {
        Rig<Chip> rig;
        auto v4 = rig.open();
        int manual = 0;
        for (uint8_t s = 0; s < Chip::socket_count(); ++s) {
            manual += rig.chip.u8(Chip::socket_mode2(s)) == Chip::mode2_dest_mac_manual() ? 1 : 0;
        }

        check(manual == 1, tag(name, "ARP-014: an IPv4 socket opens with DHAM_MANUAL").c_str());

        auto v6 = rig.open(net::SocketDomain::ipv6);
        manual = 0;
        for (uint8_t s = 0; s < Chip::socket_count(); ++s) {
            manual += rig.chip.u8(Chip::socket_mode2(s)) != 0u ? 1 : 0;
        }

        check(manual == 1, tag(name, "ARP-014: an IPv6 socket keeps automatic MAC").c_str());
    }

    Rig<Chip> plain(false);
    auto socket = plain.open();
    bool any = false;
    for (uint8_t s = 0; s < Chip::socket_count(); ++s) {
        any = any || plain.chip.u8(Chip::socket_mode2(s)) != 0u;
    }

    check(!any, tag(name, "ARP-014: without a resolver no socket uses DHAM_MANUAL").c_str());
}

// TC-NET-WIZ-ARP-015: ARP probes and SLAAC commands share the socket-less engine.
template <typename Chip>
void shared_engine(const char* name) {
    {
        Rig<Chip> rig;
        auto socket = rig.open();
        rig.chip.arp_reply_reads = 3u;
        rig.send(*socket, kHost);     // probe in flight
        rig.send(*socket, kGateway);  // queued behind it
        rig.chip.socketless_reply = Chip::socketless_status_timeout();
        bool started = rig.device.start_socketless_command(
            Chip::socketless_command_neighbor_solicit(), {}, 1000u);
        check(!started && rig.chip.socketless.size() == 1u,
              tag(name, "ARP-015: SLAAC's command is refused, not waited for, during a probe")
                  .c_str());

        for (int i = 0; i < 4; ++i) {
            rig.poll();
        }

        check(rig.chip.socketless.size() == 1u,
              tag(name, "ARP-015: no new probe takes the engine SLAAC reserved").c_str());

        started = rig.device.start_socketless_command(Chip::socketless_command_neighbor_solicit(),
                                                      {}, 1000u);
        check(started && rig.chip.socketless.size() == 2u
                  && rig.chip.socketless[1].command == Chip::socketless_command_neighbor_solicit(),
              tag(name, "ARP-015: SLAAC's retry runs once the probe finished").c_str());
        const uint8_t status = rig.device.poll_socketless_status();
        check((status & Chip::socketless_status_arp4()) == 0u
                  && (status & Chip::socketless_status_timeout()) != 0u,
              tag(name, "ARP-015: SLAAC sees its own outcome, not the probe's").c_str());

        rig.poll();
        check(rig.chip.socketless.size() == 3u && rig.chip.socketless[2].target == v(kGateway),
              tag(name, "ARP-015: the queued probe starts after SLAAC's command").c_str());
        rig.send(*socket, kHost);
        check(rig.chip.sent.size() == 1u && rig.chip.sent[0].dest_mac == v(kHostMac),
              tag(name, "ARP-015: the earlier probe's answer reached the cache").c_str());
    }

    Rig<Chip> rig;
    auto socket = rig.open();
    rig.chip.socketless_reply = 0u;  // NS still in progress
    rig.device.start_socketless_command(Chip::socketless_command_neighbor_solicit(), {}, 1000u);
    rig.send(*socket, kHost);
    check(rig.chip.socketless.size() == 1u,
          tag(name, "ARP-015: no probe while SLAAC owns the engine").c_str());

    rig.chip.mem[Chip::common_socketless_status()] |= Chip::socketless_status_neighbor_solicit();
    rig.poll();
    check(rig.chip.socketless.size() == 2u
              && rig.chip.socketless[1].command == Chip::socketless_command_arp4(),
          tag(name, "ARP-015: the probe starts once SLAAC's command completed").c_str());
    check((rig.device.poll_socketless_status() & Chip::socketless_status_neighbor_solicit()) != 0u,
          tag(name, "ARP-015: SLAAC's completion is kept for it").c_str());
}

// TC-NET-WIZ-ARP-017: a datagram from off-link shortens the gateway's backoff, not its sender's.
template <typename Chip>
void heard_through_gateway(const char* name) {
    Rig<Chip> rig(true, true);
    rig.chip.arp_hosts.erase(kGateway);
    auto socket = rig.open();
    for (int failure = 0; failure < 3; ++failure) {
        rig.poll();  // start
        rig.poll();  // collect the timeout
        rig.send(*socket, kRemote);
        fake::advance(failure == 2 ? 0u : 1000u << failure);
    }

    check(rig.chip.socketless.size() == 3u,
          tag(name, "ARP-017: the gateway failed three probes").c_str());

    auto from = [](const std::array<uint8_t, 4>& ip) {
        wiz::UdpEndpoint endpoint{};
        std::copy(ip.begin(), ip.end(), endpoint.address.begin());
        endpoint.port = 7411u;
        return endpoint;
    };

    rig.arp.heard_from(from(kHost));
    fake::advance(1000u);
    rig.poll();
    check(rig.chip.socketless.size() == 3u,
          tag(name, "ARP-017: an on-link sender leaves the gateway's backoff alone").c_str());

    rig.arp.heard_from(from(kRemote));
    fake::advance(1000u);
    rig.poll();
    check(rig.chip.socketless.size() == 4u && rig.chip.socketless[3].target == v(kGateway),
          tag(name, "ARP-017: an off-link sender brings the gateway's retry forward").c_str());
}

// TC-NET-WIZ-ARP-011: a gateway with zero octets (192.168.0.1) is an ordinary gateway.
template <typename Chip>
void zero_octet_gateway(const char* name) {
    Rig<Chip> rig;
    const std::array<uint8_t, 4> gateway{192, 168, 0, 1};
    rig.chip.arp_hosts[gateway] = kGatewayMac;
    rig.device.configure_ipv4({192, 168, 0, 2}, kMask, gateway);
    auto socket = rig.open();
    rig.poll();
    rig.poll();
    rig.send(*socket, kRemote);
    check(rig.chip.sent.size() == 1u && rig.chip.sent[0].dest_mac == v(kGatewayMac),
          tag(name, "ARP-011: 192.168.0.1 routes off-subnet traffic").c_str());
}

// TC-NET-WIZ-ARP-018: a bound socket keeps its resolver through a detach; later sockets do not use
// it.
template <typename Chip>
void detach_after_bind(const char* name) {
    Rig<Chip> rig;
    auto bound = rig.open();
    rig.send(*bound, kHost);
    rig.poll();
    rig.sockets.attach_arp_resolver(nullptr);
    rig.send(*bound, kHost);
    check(rig.chip.sent.size() == 1u && rig.chip.sent[0].dest_mac == v(kHostMac),
          tag(name, "ARP-018: a socket bound before the detach still routes through the cache")
              .c_str());

    auto later = rig.open();
    const auto commands = rig.chip.socketless.size();
    rig.send(*later, {192, 168, 1, 66});
    check(rig.chip.sent.size() == 2u && rig.chip.socketless.size() == commands
              && rig.chip.sent[1].command == Chip::command_send(),
          tag(name, "ARP-018: a socket bound after the detach leaves ARP to the chip").c_str());

    if constexpr (Chip::dest_mac_per_socket()) {
        int manual = 0;
        for (uint8_t s = 0; s < Chip::socket_count(); ++s) {
            manual += rig.chip.u8(Chip::socket_mode2(s)) == Chip::mode2_dest_mac_manual() ? 1 : 0;
        }

        check(manual == 1, tag(name, "ARP-018: only the earlier socket keeps DHAM_MANUAL").c_str());
    }
}

// TC-NET-WIZ-ARP-019: a socket reuses its last route only while the cache entry and the board's
// address are unchanged.
template <typename Chip>
void cached_route_abandoned(const char* name) {
    const std::array<uint8_t, 6> kNewMac{0x02, 0x00, 0x00, 0x00, 0x00, 0x11};
    Rig<Chip>                    rig;
    auto                         socket = rig.open();
    rig.send(*socket, kHost);
    rig.poll();
    for (int i = 0; i < 3; ++i) {
        rig.send(*socket, kHost);
    }

    const bool all_to_host =
        rig.chip.sent.size() == 3u
        && std::all_of(rig.chip.sent.begin(), rig.chip.sent.end(),
                       [](const auto& d) { return d.dest_mac == v(kHostMac); });
    check(all_to_host, tag(name, "ARP-019: repeated sends go to the host's MAC").c_str());

    rig.chip.arp_hosts[kHost] = kNewMac;
    fake::advance(60000u);
    rig.poll();  // start the refresh
    rig.poll();  // collect the answer
    rig.send(*socket, kHost);
    check(rig.chip.sent.size() == 4u && rig.chip.sent.back().dest_mac == v(kNewMac),
          tag(name, "ARP-019: a refresh answered with a new MAC replaces the cached one").c_str());

    rig.chip.arp_hosts.erase(kHost);
    rig.send(*socket, kHost);  // a use, so the entry is refreshed when due
    fake::advance(60000u);
    rig.poll();
    rig.poll();
    rig.send(*socket, kHost);
    check(rig.chip.sent.size() == 6u && rig.chip.sent.back().dest_mac == v(kNewMac),
          tag(name, "ARP-019: a failing refresh keeps the MAC in use").c_str());

    fake::advance(60000u);
    rig.poll();
    rig.send(*socket, kHost);
    check(rig.chip.sent.size() == 6u,
          tag(name, "ARP-019: an entry settled unreachable ends the cached route").c_str());

    Rig<Chip> moved;
    auto      other = moved.open();
    moved.send(*other, kHost);
    moved.poll();
    moved.send(*other, kHost);
    moved.device.configure_ipv4({192, 168, 7, 2}, kMask, {});
    moved.send(*other, kHost);
    check(moved.chip.sent.size() == 1u && moved.arp.drops() == 2u,
          tag(name, "ARP-019: a new board address ends the cached route").c_str());
}

// TC-NET-WIZ-ARP-021: poll() settles an expired entry while another next hop's probe is still
// pending, so a cached route ends within one poll period of the entry's expiry.
template <typename Chip>
void cached_route_expires_during_probe(const char* name) {
    const std::array<uint8_t, 4> kOther{192, 168, 1, 20};
    Rig<Chip>                    rig;
    auto                         socket = rig.open();
    rig.send(*socket, kHost);
    rig.poll();  // the host answers; its MAC expires 120 s from now
    rig.chip.arp_hosts.erase(kHost);
    rig.send(*socket, kHost);  // a use, so the entry is refreshed when due
    fake::advance(60000u);
    rig.poll();  // start the refresh
    rig.poll();  // the refresh goes unanswered

    fake::advance(59950u);  // 50 ms before the host's MAC expires
    rig.chip.arp_reply_reads = 1000000u;
    rig.send(*socket, kOther);  // a probe that is still pending at the expiry
    rig.send(*socket, kHost);
    check(rig.chip.sent.size() == 2u && rig.chip.sent.back().dest_mac == v(kHostMac),
          tag(name, "ARP-021: a failing refresh keeps the MAC in use").c_str());

    fake::advance(100u);
    rig.poll();
    rig.send(*socket, kHost);
    check(rig.chip.sent.size() == 2u,
          tag(name, "ARP-021: an entry expiring under a pending probe ends the cached route")
              .c_str());
}

// TC-NET-WIZ-ARP-020: sends through a cached route count as uses, so a busy next hop is not
// the one a new next hop displaces.
template <typename Chip>
void cached_route_uses(const char* name) {
    Rig<Chip> rig;

    std::vector<std::array<uint8_t, 4>> others;
    for (uint8_t i = 0; i < 8u; ++i) {
        const std::array<uint8_t, 4> ip{192, 168, 1, static_cast<uint8_t>(20u + i)};
        others.push_back(ip);
        rig.chip.arp_hosts[ip] = {0x02, 0x00, 0x00, 0x00, 0x00, static_cast<uint8_t>(20u + i)};
    }

    auto busy = rig.open(net::SocketDomain::ipv4, 7400u);
    auto other = rig.open(net::SocketDomain::ipv4, 7401u);
    rig.send(*busy, kHost);
    rig.poll();
    for (std::size_t i = 0; i < 7u; ++i) {
        rig.send(*other, others[i]);
        rig.poll();
    }

    check(rig.table.size() == 8u, tag(name, "ARP-020: eight next hops fill the table").c_str());

    rig.send(*busy, kHost);  // caches the route
    for (std::size_t i = 0; i < 7u; ++i) {
        rig.send(*other, others[i]);  // each is now used more recently than the host
    }

    rig.chip.sent.clear();
    for (int i = 0; i < 3; ++i) {
        rig.send(*busy, kHost);
    }

    rig.poll();
    rig.send(*other, others[7]);
    check(rig.chip.sent.size() == 3u && rig.table.find(kHost) != nullptr
              && rig.table.find(others[0]) == nullptr,
          tag(name, "ARP-020: the host with a cached route stays; the least recently used goes")
              .c_str());
}

}  // namespace

int main() {
    first_contact<wiz::W5100S>("W5100S");
    first_contact<wiz::W6100>("W6100");
    next_hop<wiz::W5100S>("W5100S");
    next_hop<wiz::W6100>("W6100");
    group_and_broadcast<wiz::W5100S>("W5100S");
    group_and_broadcast<wiz::W6100>("W6100");
    dead_host<wiz::W5100S>("W5100S");
    dead_host<wiz::W6100>("W6100");
    w6x00_mode2<wiz::W6100>("W6100");
    shared_engine<wiz::W6100>("W6100");
    heard_through_gateway<wiz::W5100S>("W5100S");
    heard_through_gateway<wiz::W6100>("W6100");
    zero_octet_gateway<wiz::W5100S>("W5100S");
    zero_octet_gateway<wiz::W6100>("W6100");
    detach_after_bind<wiz::W5100S>("W5100S");
    detach_after_bind<wiz::W6100>("W6100");
    cached_route_abandoned<wiz::W5100S>("W5100S");
    cached_route_abandoned<wiz::W6100>("W6100");
    cached_route_uses<wiz::W5100S>("W5100S");
    cached_route_uses<wiz::W6100>("W6100");
    cached_route_expires_during_probe<wiz::W5100S>("W5100S");
    cached_route_expires_during_probe<wiz::W6100>("W6100");
    return test_summary();
}
