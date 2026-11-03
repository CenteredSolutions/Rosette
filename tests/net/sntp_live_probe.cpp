// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file tests/net/sntp_live_probe.cpp
 * @brief SntpClient over host sockets against a real NTP server, checked against the host's clock.
 *
 * The firmware's SNTP client with only the socket backend swapped
 * (`PosixSocketProvider` for the WIZnet chip). It disciplines the host's
 * `platform::WallClock`, first set back to 1970, then compares the result
 * with `CLOCK_REALTIME`, which the host keeps on UTC itself.
 *
 *   sntp_live_probe [server-ipv4 [exchanges]]   (default 216.239.35.0, 3)
 *
 * - TC-NET-SNTP-LIVE-001: the first exchange puts WallClock within 50 ms of
 *   the host clock, and later ones keep it there.
 *
 * Exit status 0 on a pass. The probe sets `min_poll_ms` to 16 s, just above
 * the RFC 4330 §10 floor, so three exchanges take about half a minute.
 *
 * References:
 * - RFC 4330 §5, §10; RFC 5905 §8
 */

#include <cstdio>
#include <cstdlib>
#include <ctime>

#include "posix_socket.hpp"
#include "rosette/net/inet.h"
#include "rosette/net/sntp_client.hpp"
#include "rosette/platform/platform.hpp"
#include "rosette/platform/wall_clock.hpp"

namespace {

namespace net = rosette::net;
namespace pf = rosette::platform;

int64_t host_unix_us() {
    timespec ts{};
    (void)clock_gettime(CLOCK_REALTIME, &ts);
    return static_cast<int64_t>(ts.tv_sec) * 1000000 + ts.tv_nsec / 1000;
}

}  // namespace

int main(int argc, char** argv) {
    const char* server_text = argc > 1 ? argv[1] : "216.239.35.0";
    const int exchanges = argc > 2 ? std::atoi(argv[2]) : 3;

    net::SntpConfig config{};
    config.enabled = true;
    if (!net::inet::make_socket_address_v4(server_text, net::kNtpServerPort, config.server)) {
        std::printf("[sntp-live] not an IPv4 address: %s\n", server_text);
        return 2;
    }

    config.local_port = 0u;
    config.min_poll_ms = 16000u;
    config.reply_timeout_ms = 1000u;

    // The client runs only while the board holds an IPv4 address; any will do on a host.
    net::AddressSnapshot snapshot{};
    (void)snapshot.v4.addresses.add(net::AddressFamily::ipv4, {127u, 0u, 0u, 1u}, 8u);

    net::posix::PosixSocketProvider sockets;
    net::SntpClient client(sockets, config, net::wall_clock_for_sntp());
    pf::WallClock::set_offset_us(0);

    bool passed = true;
    for (int done = 0; done < exchanges;) {
        const uint32_t before = client.samples();
        client.poll(pf::monotonic_ms(), snapshot);
        if (client.samples() == before) {
            pf::sleep_ms(100u);
            continue;
        }

        ++done;
        const int64_t skew_us = pf::WallClock::now_us() - host_unix_us();
        std::printf("[sntp-live] exchange %d: WallClock - host clock = %lld us\n", done,
                    static_cast<long long>(skew_us));
        passed = passed && skew_us > -50000 && skew_us < 50000;
    }

    std::printf("[sntp-live] TC-NET-SNTP-LIVE-001 %s: WallClock follows %s to within 50 ms\n",
                passed ? "PASS" : "FAIL", server_text);
    return passed ? 0 : 1;
}
