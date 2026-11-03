// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file dhcp_client_test.cpp
 * @brief The WIZnet DHCP client against a scripted socket: wire format, the
 *        RFC 2131 state machine, and that no `poll()` can wait.
 *
 * Each TC drives `DhcpClient::poll()` with an explicit clock and answers the
 * client's messages from the test, as a server would.
 */

#include <algorithm>
#include <cstdint>

#include "fake_dhcp_network.hpp"
#include "ports/net/wiznet/dhcp_client.hpp"
#include "test_util.hpp"

namespace {

namespace wiz = rosette::net::wiznet;

constexpr std::array<uint8_t, 6> kMac{0x00u, 0x08u, 0xDCu, 0x54u, 0x75u, 0x30u};
constexpr std::array<uint8_t, 4> kServer{192u, 168u, 32u, 99u};
constexpr std::array<uint8_t, 4> kLeased{192u, 168u, 32u, 79u};
constexpr std::array<uint8_t, 4> kZero{};
constexpr uint8_t kDiscover = 1u;
constexpr uint8_t kRequest = 3u;
constexpr uint8_t kDecline = 4u;
constexpr uint8_t kAck = 5u;
constexpr uint8_t kNak = 6u;

wiz::DhcpClientConfig config() {
    wiz::DhcpClientConfig c{};
    c.host_name = "rosette-test";
    return c;
}

/// Runs DISCOVER / OFFER / REQUEST / ACK at t = 0 with @p ack; returns the event of the ACK poll.
wiz::DhcpEvent acquire(fake::FakeDhcpNetwork& net, wiz::DhcpClient& client,
                       const fake::ServerReply& ack) {
    client.start();
    (void)client.poll(0u);
    net.answer({}, kMac);  // OFFER
    (void)client.poll(0u);
    net.answer(ack, kMac);
    return client.poll(0u);
}

fake::ServerReply ack_reply() {
    fake::ServerReply r{};
    r.type = kAck;
    return r;
}

// TC-NET-DHCP-001: DISCOVER and REQUEST carry what RFC 2131 Table 5 requires;
// replies for another transaction or another client are ignored.
void tc_wire_format() {
    fake::FakeDhcpNetwork net;
    wiz::DhcpClient client{net, kMac, config()};
    client.start();
    (void)client.poll(0u);
    check(net.sent.size() == 1u && net.last().type == kDiscover, "001: first poll sends DISCOVER");
    check(net.last().broadcast_to() && net.last().to_port == 67u, "001: to 255.255.255.255:67");
    check(net.last().flags == 0x8000u && net.last().ciaddr == kZero,
          "001: BROADCAST flag set, ciaddr zero");
    check(std::equal(kMac.begin(), kMac.end(), net.last().raw.begin() + 28), "001: chaddr = MAC");

    fake::ServerReply stranger{};
    stranger.xid = net.last().xid + 1u;
    stranger.mac = kMac;
    net.inbound.push_back(fake::server_reply(stranger));
    fake::ServerReply other_client{};
    other_client.xid = net.last().xid;
    other_client.mac = {1u, 2u, 3u, 4u, 5u, 6u};
    net.inbound.push_back(fake::server_reply(other_client));
    (void)client.poll(100u);
    check(client.state() == wiz::DhcpClientState::selecting, "001: foreign replies ignored");

    net.answer({}, kMac);
    (void)client.poll(200u);
    check(net.last().type == kRequest && net.last().requested_ip == kLeased
              && net.last().server_id == kServer && net.last().ciaddr == kZero
              && net.last().flags == 0x8000u,
          "001: REQUEST names the offer (options 50, 54), ciaddr zero, broadcast");
}

// TC-NET-DHCP-002: a full exchange binds, holds a socket only while waiting,
// and no poll sends more than one message or reads a blocking socket.
void tc_exchange_is_non_blocking() {
    fake::FakeDhcpNetwork net;
    wiz::DhcpClient client{net, kMac, config()};
    check(!client.socket_open() && net.open_now == 0, "002: an idle client holds no socket");
    client.start();
    (void)client.poll(0u);
    check(net.open_now == 1, "002: the exchange window holds one socket");

    std::size_t sends = net.sent.size();
    bool one_send_per_poll = true;
    for (uint32_t t = 0u; t < 1900u; t += 100u) {
        (void)client.poll(t);
        one_send_per_poll = one_send_per_poll && net.sent.size() <= sends + 1u;
        sends = net.sent.size();
    }

    check(one_send_per_poll && net.sent.size() == 1u, "002: waiting polls send nothing");
    net.answer({}, kMac);
    (void)client.poll(2500u);
    net.answer(ack_reply(), kMac);
    const wiz::DhcpEvent event = client.poll(2600u);
    check(event == wiz::DhcpEvent::bound && client.has_lease(), "002: ACK yields bound");
    check(client.lease().ip == kLeased && client.lease().server_id == kServer,
          "002: lease holds the address and server");
    check(client.lease().t1_s == 1800u && client.lease().t2_s == 3150u,
          "002: T1 / T2 default to 0.5 / 0.875 of the lease");
    check(net.open_now == 0, "002: bound client closed its socket");
}

// TC-NET-DHCP-003: without a server, DISCOVER is retransmitted at 2 s and 6 s,
// the window fails at 10 s, and the next DISCOVER waits retry_delay_ms.
void tc_timeout_and_retry() {
    fake::FakeDhcpNetwork net;
    wiz::DhcpClient client{net, kMac, config()};
    client.start();
    wiz::DhcpEvent event = wiz::DhcpEvent::none;
    uint32_t t = 0u;
    for (; t <= 10000u && event == wiz::DhcpEvent::none; t += 100u) {
        event = client.poll(t);
    }

    check(net.sent.size() == 3u, "003: DISCOVER at 0 s, 2 s, 6 s");
    check(event == wiz::DhcpEvent::failed && t - 100u == 10000u, "003: window fails at 10 s");
    check(net.open_now == 0 && client.state() == wiz::DhcpClientState::init_wait,
          "003: socket closed while waiting to retry");
    (void)client.poll(24900u);
    check(net.sent.size() == 3u, "003: no DISCOVER before retry_delay_ms");
    (void)client.poll(25000u);
    check(net.sent.size() == 4u && net.last().type == kDiscover, "003: DISCOVER after 15 s");

    fake::FakeDhcpNetwork busy;
    busy.refuse_open = true;
    wiz::DhcpClient starved{busy, kMac, config()};
    starved.start();
    check(starved.poll(0u) == wiz::DhcpEvent::failed, "003: no free socket fails the window");
}

// TC-NET-DHCP-004: at T1 the client unicasts a REQUEST with ciaddr to its
// server and an unchanged ACK extends the lease without a new address.
void tc_renewal_keeps_address() {
    fake::FakeDhcpNetwork net;
    wiz::DhcpClient client{net, kMac, config()};
    (void)acquire(net, client, ack_reply());
    (void)client.poll(1799999u);
    check(client.state() == wiz::DhcpClientState::bound && net.open_now == 0,
          "004: bound, no socket, before T1");
    (void)client.poll(1800000u);
    const fake::SentMessage& renew = net.last();
    check(client.state() == wiz::DhcpClientState::renewing && renew.type == kRequest,
          "004: REQUEST at T1");
    check(renew.to == kServer && renew.ciaddr == kLeased && renew.flags == 0u,
          "004: unicast to the server, ciaddr = lease, no BROADCAST flag");
    check(renew.requested_ip == kZero && renew.server_id == kZero,
          "004: RENEWING REQUEST omits options 50 and 54");
    net.answer(ack_reply(), kMac);
    check(client.poll(1800100u) == wiz::DhcpEvent::renewed, "004: unchanged ACK yields renewed");
    check(client.state() == wiz::DhcpClientState::bound && net.open_now == 0,
          "004: back to bound, socket closed");
    (void)client.poll(1800000u + 1799999u);
    check(client.state() == wiz::DhcpClientState::bound,
          "004: T1 restarts from the renewing REQUEST (RFC 2131 §4.4.1)");
    (void)client.poll(1800000u + 1800000u);
    check(client.state() == wiz::DhcpClientState::renewing, "004: and renews again at the new T1");

    wiz::DhcpClient moved{net, kMac, config()};
    (void)acquire(net, moved, ack_reply());
    (void)moved.poll(1800000u);
    fake::ServerReply other = ack_reply();
    other.router = {192u, 168u, 32u, 254u};
    net.answer(other, kMac);
    check(moved.poll(1800100u) == wiz::DhcpEvent::bound, "004: a changed ACK yields bound");
}

// TC-NET-DHCP-005: a decline broadcasts DHCPDECLINE and waits 10 s.
void tc_decline() {
    fake::FakeDhcpNetwork net;
    wiz::DhcpClient client{net, kMac, config()};
    (void)acquire(net, client, ack_reply());
    client.decline(5000u);
    const fake::SentMessage& decline = net.last();
    check(decline.type == kDecline && decline.broadcast_to(), "005: DECLINE broadcast");
    check(
        decline.requested_ip == kLeased && decline.server_id == kServer && decline.ciaddr == kZero,
        "005: options 50 and 54 name the address, ciaddr zero");
    check(!client.has_lease() && net.open_now == 0, "005: lease dropped, no socket held");
    (void)client.poll(19999u);
    check(net.last().type == kDecline, "005: no DISCOVER inside the wait");
    (void)client.poll(20000u);
    check(net.last().type == kDiscover && net.last().xid != decline.xid,
          "005: fresh DISCOVER with a new xid");
}

// TC-NET-DHCP-006: past T2 the client broadcasts REQUESTs with ciaddr, and
// retries no sooner than 60 s apart.
void tc_rebinding() {
    fake::FakeDhcpNetwork net;
    wiz::DhcpClient client{net, kMac, config()};
    (void)acquire(net, client, ack_reply());
    uint32_t t = 1800000u;
    for (; t < 3150000u; t += 1000u) {
        (void)client.poll(t);
    }

    const std::size_t renews = net.sent.size() - 3u;
    check(renews >= 2u && renews <= 20u, "006: RENEWING retries are spaced, not continuous");
    (void)client.poll(3150000u);
    check(client.state() == wiz::DhcpClientState::rebinding, "006: REBINDING at T2");
    check(net.last().broadcast_to() && net.last().ciaddr == kLeased && net.last().flags == 0u,
          "006: REBINDING REQUEST is broadcast with ciaddr");
    const std::size_t at_t2 = net.sent.size();
    for (t = 3150000u; t < 3150000u + 59000u; t += 1000u) {
        (void)client.poll(t);
    }

    check(net.sent.size() - at_t2 <= 3u, "006: in-window retransmits only, next attempt >= 60 s");
}

// TC-NET-DHCP-007: expiry or a NAK of a held lease yields lost and restarts INIT.
void tc_lease_lost() {
    fake::FakeDhcpNetwork net;
    wiz::DhcpClient client{net, kMac, config()};
    (void)acquire(net, client, ack_reply());
    wiz::DhcpEvent event = wiz::DhcpEvent::none;
    for (uint32_t t = 1800000u; t <= 3600000u && event != wiz::DhcpEvent::lost; t += 1000u) {
        event = client.poll(t);
    }

    check(event == wiz::DhcpEvent::lost && !client.has_lease(), "007: expiry yields lost");
    check(net.last().type == kDiscover, "007: and a fresh DISCOVER");

    wiz::DhcpClient nak{net, kMac, config()};
    (void)acquire(net, nak, ack_reply());
    (void)nak.poll(1800000u);
    fake::ServerReply reply{};
    reply.type = kNak;
    net.answer(reply, kMac);
    check(nak.poll(1800100u) == wiz::DhcpEvent::lost, "007: NAK while RENEWING yields lost");

    fake::ServerReply forever = ack_reply();
    forever.lease_s = 0xFFFFFFFFu;
    wiz::DhcpClient infinite{net, kMac, config()};
    (void)acquire(net, infinite, forever);
    (void)infinite.poll(0x7FFFFFF0u);
    check(infinite.state() == wiz::DhcpClientState::bound, "007: an infinite lease never renews");
}

}  // namespace

int main() {
    tc_wire_format();
    tc_exchange_is_non_blocking();
    tc_timeout_and_retry();
    tc_renewal_keeps_address();
    tc_decline();
    tc_rebinding();
    tc_lease_lost();
    return test_summary();
}
