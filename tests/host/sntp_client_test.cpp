/**
 * @file tests/host/sntp_client_test.cpp
 * @brief SntpClient against a simulated server and path: phase, server choice, filter, frequency,
 * pacing.
 *
 * The local tick and the disciplined offset are fake (fake_ntp_network.hpp),
 * so every expectation is exact arithmetic on the simulated clocks.
 *
 * References: RFC 4330 §5, §10; RFC 5905 §7.4, §8, §10.
 */

#include <array>
#include <cstdint>

#include "fake_ntp_network.hpp"
#include "rosette/net/sntp_client.hpp"
#include "test_util.hpp"

namespace net = rosette::net;
using fake::FakeNtpNetwork;
using fake::NtpBehavior;

namespace {

const std::array<uint8_t, 4> kDhcpServer{192u, 168u, 1u, 1u};
const std::array<uint8_t, 4> kConfigured{10u, 0u, 0u, 123u};

net::AddressSnapshot snapshot(bool with_address, bool with_dhcp_server) {
    net::AddressSnapshot s{};
    if (with_address) {
        (void)s.v4.addresses.add(net::AddressFamily::ipv4, {192u, 168u, 1u, 50u}, 24u);
    }

    if (with_dhcp_server) {
        s.ntp_servers_v4[0] = kDhcpServer;
        s.ntp_server_count = 1u;
    }

    return s;
}

/// @brief An enabled client with `kConfigured` as its configured server.
net::SntpConfig config() {
    net::SntpConfig c{};
    c.enabled = true;
    c.server = net::make_socket_address_v4(kConfigured, net::kNtpServerPort);
    return c;
}

uint32_t now_ms() {
    return static_cast<uint32_t>(FakeNtpNetwork::tick_us / 1000u);
}

void reset() {
    FakeNtpNetwork::tick_us = 1000000u;
    FakeNtpNetwork::clock_offset_us = 0;
}

/// @brief Advances the tick by `ms`, polling every 100 ms as a main loop would.
void run_for(net::SntpClient& client, const net::AddressSnapshot& s, uint32_t ms) {
    for (uint32_t t = 0; t < ms; t += 100u) {
        FakeNtpNetwork::tick_us += 100000u;
        client.poll(now_ms(), s);
    }
}

void first_exchange_sets_the_clock() {
    reset();
    FakeNtpNetwork server;
    net::SntpClient client(server, config(), FakeNtpNetwork::clock());

    client.poll(now_ms(), snapshot(false, true));
    check(server.requests == 0 && !client.synced(),
          "TC-NET-SNTP-CLI-001 no request without an IPv4 address");

    client.poll(now_ms(), snapshot(true, true));
    const int64_t error =
        FakeNtpNetwork::clock_offset_us - server.true_offset_at(FakeNtpNetwork::tick_us);
    check(client.synced() && error >= -1 && error <= 1,
          "TC-NET-SNTP-CLI-001 one exchange puts the clock on the server's time");
    check(server.open_sockets == 0,
          "TC-NET-SNTP-CLI-001 the request socket is closed after the exchange");
}

void servers_and_failover() {
    reset();
    FakeNtpNetwork server;
    server.behavior = NtpBehavior::silent;
    net::SntpClient client(server, config(), FakeNtpNetwork::clock());
    const net::AddressSnapshot s = snapshot(true, true);

    client.poll(now_ms(), s);
    check(
        server.requests == 1 && server.last_to.ipv4() == kConfigured && server.last_to.port == 123u,
        "TC-NET-SNTP-CLI-002 the configured server is asked before DHCP's");
    check(!client.synced() && server.open_sockets == 0,
          "TC-NET-SNTP-CLI-002 a silent server leaves the clock unset and no socket open");

    run_for(client, s, 15000u);
    check(server.requests == 1, "TC-NET-SNTP-CLI-002 no retry before retry_ms");

    server.behavior = NtpBehavior::answer;
    run_for(client, s, 1500u);
    check(server.requests == 2 && server.last_to.ipv4() == kDhcpServer && client.synced(),
          "TC-NET-SNTP-CLI-002 the retry goes to the next server (DHCP's) and syncs");

    net::SntpConfig dhcp_only{};
    dhcp_only.enabled = true;
    FakeNtpNetwork offered;
    net::SntpClient from_dhcp(offered, dhcp_only, FakeNtpNetwork::clock());
    from_dhcp.poll(now_ms(), snapshot(true, true));
    check(offered.requests == 1 && offered.last_to.ipv4() == kDhcpServer && from_dhcp.synced(),
          "TC-NET-SNTP-CLI-002 with no server configured, the DHCP-offered one is used");

    FakeNtpNetwork lonely;
    net::SntpClient idle(lonely, dhcp_only, FakeNtpNetwork::clock());
    run_for(idle, snapshot(true, false), 120000u);
    check(lonely.requests == 0 && !idle.synced(),
          "TC-NET-SNTP-CLI-002 with no server configured or offered, nothing is ever sent");
}

void filter_rejects_a_delayed_sample() {
    reset();
    FakeNtpNetwork server;
    net::SntpClient client(server, config(), FakeNtpNetwork::clock());
    const net::AddressSnapshot s = snapshot(true, true);
    client.poll(now_ms(), s);

    // One exchange whose request queues 40 ms on the way out: its offset reads 20 ms high.
    run_for(client, s, 63000u);
    server.out_delay_us = 42000u;
    run_for(client, s, 1100u);
    const int64_t error =
        FakeNtpNetwork::clock_offset_us - server.true_offset_at(FakeNtpNetwork::tick_us);
    check(client.samples() == 2 && error >= -5 && error <= 5,
          "TC-NET-SNTP-CLI-003 a sample with a long delay does not move the clock");
    check(client.frequency_ppm() > -0.01 && client.frequency_ppm() < 0.01,
          "TC-NET-SNTP-CLI-003 a sample that is not used leaves the frequency alone");

    server.behavior = NtpBehavior::stale_then_answer;
    server.out_delay_us = 2000u;
    run_for(client, s, 64000u);
    const int64_t after_stale =
        FakeNtpNetwork::clock_offset_us - server.true_offset_at(FakeNtpNetwork::tick_us);
    check(client.samples() == 3 && after_stale >= -20 && after_stale <= 20,
          "TC-NET-SNTP-CLI-003 a stale reply is skipped and the real one used");
}

void fresh_sample_beats_stale_one() {
    reset();
    FakeNtpNetwork server;
    net::SntpClient client(server, config(), FakeNtpNetwork::clock());
    const net::AddressSnapshot s = snapshot(true, true);
    client.poll(now_ms(), s);
    run_for(client, s, 64000u * 8u + 128000u + 256000u + 512000u + 1024000u + 1000u);

    // The server steps 5 ms while the path slows to 12 ms: every older sample is faster but stale.
    server.server_offset_us += 5000;
    server.out_delay_us = 6000u;
    server.back_delay_us = 6000u;
    run_for(client, s, client.poll_interval_ms() + 1000u);
    const int64_t error =
        FakeNtpNetwork::clock_offset_us - server.true_offset_at(FakeNtpNetwork::tick_us);
    check(error >= -50 && error <= 50,
          "TC-NET-SNTP-CLI-007 age outweighs delay: a fresh slower sample beats an old fast one");
}

void frequency_tracks_drift() {
    reset();
    FakeNtpNetwork server;
    server.drift_ppm = 20.0;
    net::SntpClient client(server, config(), FakeNtpNetwork::clock());
    const net::AddressSnapshot s = snapshot(true, true);
    run_for(client, s, 64000u * 60u);

    check(client.frequency_ppm() > 18.0 && client.frequency_ppm() < 22.0,
          "TC-NET-SNTP-CLI-004 the frequency estimate converges on the tick's drift");

    // Halfway between exchanges the clock has been slewed, not left to drift.
    run_for(client, s, client.poll_interval_ms() / 2u);
    const int64_t error =
        FakeNtpNetwork::clock_offset_us - server.true_offset_at(FakeNtpNetwork::tick_us);
    check(error > -2000 && error < 2000,
          "TC-NET-SNTP-CLI-004 between exchanges the offset follows the drift");
}

void poll_interval_backs_off() {
    reset();
    FakeNtpNetwork server;
    net::SntpClient client(server, config(), FakeNtpNetwork::clock());
    const net::AddressSnapshot s = snapshot(true, true);
    client.poll(now_ms(), s);
    run_for(client, s, 64000u * 7u + 500u);
    check(client.samples() == 8 && client.poll_interval_ms() == 64000u,
          "TC-NET-SNTP-CLI-005 the first eight exchanges are min_poll apart");

    run_for(client, s, 64000u + 1000u + 128000u + 256000u + 512000u + 1024000u * 2u);
    check(client.poll_interval_ms() == 1024000u,
          "TC-NET-SNTP-CLI-005 a settled clock backs off to max_poll");
}

void kiss_of_death() {
    reset();
    FakeNtpNetwork server;
    server.behavior = NtpBehavior::kiss_rate;
    net::SntpClient client(server, config(), FakeNtpNetwork::clock());
    const net::AddressSnapshot s = snapshot(true, true);
    client.poll(now_ms(), s);
    check(!client.synced() && client.poll_interval_ms() == 1024000u,
          "TC-NET-SNTP-CLI-006 a kiss-o'-death sets nothing and waits max_poll");

    server.behavior = NtpBehavior::answer;
    run_for(client, s, 1024000u + 200u);
    check(server.requests == 2 && server.last_to.ipv4() == kDhcpServer,
          "TC-NET-SNTP-CLI-006 the next request goes to a different server");
}

}  // namespace

int main() {
    first_exchange_sets_the_clock();
    servers_and_failover();
    filter_rejects_a_delayed_sample();
    frequency_tracks_drift();
    poll_interval_backs_off();
    kiss_of_death();
    fresh_sample_beats_stale_one();
    return test_summary();
}
