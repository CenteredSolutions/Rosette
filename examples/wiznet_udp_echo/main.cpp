/**
 * @file examples/wiznet_udp_echo/main.cpp
 * @brief RFC 862 UDP echo on a WIZnet board, over IPv4 and hardware IPv6.
 *
 * The smallest end-to-end check of the chip driver below every ROS layer: the
 * board brings its network up (DHCP or IPv4 link-local; on the W6100 / W6300
 * also IPv6 SLAAC), opens one UDP socket per family on port 7, and returns
 * every datagram to its sender. Both rmw_rosette and the micro-ROS agent
 * client sit on exactly this socket path, so what passes here passes for them.
 *
 *   - prints the IPv4 address, and the IPv6 link-local and global addresses as
 *     they are acquired;
 *   - echoes each datagram on the socket it arrived on;
 *   - reports its addresses, per-family echo counters, and wall clock every
 *     five seconds on the console, and to any datagram sent to UDP port 8
 *     (the datagram `bootsel` restarts the board into its USB bootloader for
 *     reflashing);
 *   - sets its wall clock by SNTP from the server named at configure time
 *     (`-DWIZNET_UDP_ECHO_NTP_SERVER=<IPv4>`), then from any server DHCP
 *     offers, so a host can compare the `clock` status line with its own.
 *
 * Sleeping between datagrams uses the provider's wait_readable() on the
 * receive event the chip's INT pin signals, the same wake path the RTPS
 * participant driver uses, so a round trip far below the one-second wait
 * shows that INT reaches the MCU.
 *
 * From a host on the same link: tools/wiznet_udp_echo_probe.py <board IPv4>
 * (ping, ping6, and echo round trips for every address the board prints).
 *
 * Verification: every probe datagram comes back unchanged to its sender over
 * each family and address, with sub-millisecond-scale chip latency; the
 * counters on the console match the probe's; with an NTP server known, the
 * `clock` line agrees with a host on NTP to within the status round trip.
 *
 * References: RFC 862 (echo); RFC 4862 (SLAAC); RFC 4330 (SNTP); W6100
 * datasheet §Sn_MR UDP6.
 */

#include <array>
#include <cstdio>
#include <cstring>
#include <memory>

#include "board.h"
#include "rosette/net/inet.h"
#include "rosette/net/wiznet/board_network.hpp"
#include "rosette/platform/platform.hpp"
#include "rosette/platform/wall_clock.hpp"

#include "pico/bootrom.h"
#include "pico/stdlib.h"

namespace {

namespace wiz = rosette::net::wiznet;
namespace net = rosette::net;
namespace pf = rosette::platform;

constexpr uint16_t kEchoPort = 7u;
/// Any datagram here is answered with the board's addresses and counters.
constexpr uint16_t kStatusPort = 8u;
constexpr uint32_t kWaitMs = 1000u;
constexpr uint32_t kReportPeriodMs = 5000u;
/// Longest wait for every family at boot; IPv6 DAD takes about a second.
constexpr uint32_t kBringUpTimeoutMs = 30000u;

/// One echo socket and its counters.
struct EchoSocket {
    const char* family = "";
    std::unique_ptr<net::Socket> socket;
    uint32_t echoed = 0u;
    uint32_t errors = 0u;
};

std::array<uint8_t, 1500> g_buffer;

/**
 * @brief Opens and binds a non-blocking UDP socket for `domain` on `port`.
 *
 * @param[in] sockets Board socket provider.
 * @param[in] domain  Address family.
 * @param[in] port    Local port.
 * @return The bound socket, non-blocking; null when the part lacks the family
 *   or binding fails.
 */
std::unique_ptr<net::Socket> open_echo_socket(net::SocketProvider& sockets,
                                              net::SocketDomain domain, uint16_t port) {
    auto s = sockets.open(domain);
    if (!s) {
        return nullptr;
    }

    net::SocketAddress local{};
    local.domain = domain;
    local.port = port;
    if (s->bind(local) != 0) {
        return nullptr;
    }

    (void)s->set_receive_timeout(0u);
    return s;
}

/// @brief Returns every waiting datagram to its sender.
void serve(EchoSocket& e) {
    if (!e.socket) {
        return;
    }

    for (;;) {
        net::SocketAddress from{};
        const auto n = e.socket->recv_from(g_buffer.data(), g_buffer.size(), &from);
        if (n < 0 && e.socket->last_error() == net::SocketError::would_block) {
            return;
        }

        if (n <= 0) {
            ++e.errors;
            return;
        }

        if (e.socket->send_to(g_buffer.data(), static_cast<std::size_t>(n), from) == n) {
            ++e.echoed;
        } else {
            ++e.errors;
        }
    }
}

/**
 * @brief Ready once IPv4 is up and, on parts with IPv6, an IPv6 address has passed DAD.
 *
 * The board's default config is ready on IPv4 alone; this example waits for
 * every family it echoes on, so the first status reply already lists them
 * all for tools/wiznet_udp_echo_probe.py.
 *
 * @param[in] snapshot The board's current addresses.
 * @return Whether every family the board runs holds an address.
 */
bool every_family_up(const net::AddressSnapshot& snapshot) {
    const bool runs_v6 = wiz::board_default_network().v6[0] != net::Method::none;
    return snapshot.v4.has_address() && (!runs_v6 || snapshot.v6.has_address());
}

/**
 * @brief Writes the board's addresses, echo counters, and wall clock as text lines.
 *
 * The same text goes to the console every kReportPeriodMs and back to any
 * datagram sent to kStatusPort, so a host can learn the board's addresses
 * even when the USB console is unavailable. The clock line is Unix time in
 * microseconds, or `unsynced` until a time source has set the wall clock.
 *
 * @param[out] out Text buffer.
 * @param[in]  cap Its size.
 * @param[in]  v4  IPv4 echo socket counters.
 * @param[in]  v6  IPv6 echo socket counters.
 * @return Length written, excluding the terminator.
 */
int format_status(char* out, int cap, const EchoSocket& v4, const EchoSocket& v6) {
    const uint32_t a = wiz::board_ipv4_address();
    char ll[48] = "none";
    char gl[48] = "none";
    for (const net::AddressEntry& entry : wiz::board_address_snapshot().v6.addresses) {
        char* text = entry.link_local() ? ll : gl;
        net::inet::ntop6(entry.address.data(), text, sizeof(ll));
    }

    int n = std::snprintf(
        out, static_cast<std::size_t>(cap),
        "ipv4 %lu.%lu.%lu.%lu\nipv6-link-local %s\nipv6-global %s\n"
        "echo ipv4=%lu/%lu ipv6=%lu/%lu arp_timeouts=%lu\n",
        static_cast<unsigned long>(a >> 24), static_cast<unsigned long>((a >> 16) & 0xFFu),
        static_cast<unsigned long>((a >> 8) & 0xFFu), static_cast<unsigned long>(a & 0xFFu), ll, gl,
        static_cast<unsigned long>(v4.echoed), static_cast<unsigned long>(v4.errors),
        static_cast<unsigned long>(v6.echoed), static_cast<unsigned long>(v6.errors),
        static_cast<unsigned long>(wiz::board_send_timeouts()));
    if (n <= 0 || n >= cap) {
        return n;
    }

    if (pf::WallClock::synchronized()) {
        n += std::snprintf(out + n, static_cast<std::size_t>(cap - n), "clock %lld\n",
                           static_cast<long long>(pf::WallClock::now_us()));
    } else {
        n += std::snprintf(out + n, static_cast<std::size_t>(cap - n), "clock unsynced\n");
    }

    return n;
}

/**
 * @brief Answers every datagram on a status socket with format_status().
 *
 * The datagram `bootsel` instead restarts the board into the RP2 USB
 * bootloader, so a host whose USB CDC link is unreliable (a VM's USB
 * passthrough) can still reach the BOOTSEL drive for the next flash. This is a
 * development convenience of this example: anyone on the link can send it.
 */
void serve_status(net::Socket* s, const EchoSocket& v4, const EchoSocket& v6) {
    if (s == nullptr) {
        return;
    }

    net::SocketAddress from{};
    std::ptrdiff_t got = 0;
    while ((got = s->recv_from(g_buffer.data(), g_buffer.size(), &from)) >= 0) {
        if (got == 7 && std::memcmp(g_buffer.data(), "bootsel", 7) == 0) {
            (void)s->send_to("rebooting to BOOTSEL\n", 21, from);
            pf::sleep_ms(50u);
            reset_usb_boot(0u, 0u);
        }

        char text[512];
        const int n = format_status(text, sizeof(text), v4, v6);
        if (n > 0) {
            (void)s->send_to(text, static_cast<std::size_t>(n), from);
        }
    }
}

}  // namespace

int main() {
    board_init();
    pf::sleep_ms(1500u);  // give the USB console time to enumerate
    std::printf("\n[echo] wiznet_udp_echo\n");

    net::NetworkConfig network = wiz::board_default_network();
    network.ready_when = every_family_up;
    network.ready_timeout_ms = kBringUpTimeoutMs;  // then echo on whatever is up
    // A configured server is static configuration, so SNTP runs on any build that names one.
    if (net::inet::make_socket_address_v4(WIZNET_UDP_ECHO_NTP_SERVER, net::kNtpServerPort,
                                          network.time.server)) {
        network.time.enabled = true;
    }

    (void)wiz::bring_up_board_network("rosette-echo", network);
    net::SocketProvider& sockets = wiz::board_socket_provider();

    EchoSocket v4{"ipv4", open_echo_socket(sockets, net::SocketDomain::ipv4, kEchoPort)};
    EchoSocket v6{"ipv6", open_echo_socket(sockets, net::SocketDomain::ipv6, kEchoPort)};
    auto status4 = open_echo_socket(sockets, net::SocketDomain::ipv4, kStatusPort);
    auto status6 = open_echo_socket(sockets, net::SocketDomain::ipv6, kStatusPort);
    std::printf("[echo] udp/%u ipv4 %s, ipv6 %s; status on udp/%u\n", kEchoPort,
                v4.socket ? "open" : "unavailable", v6.socket ? "open" : "unavailable",
                kStatusPort);

    pf::Event wake;
    sockets.set_receive_event(&wake);
    net::Socket* const watch[] = {v4.socket.get(), v6.socket.get(), status4.get(), status6.get()};

    uint32_t next_report = pf::monotonic_ms() + kReportPeriodMs;
    for (;;) {
        sockets.wait_readable(watch, 4u, wake, kWaitMs);
        serve(v4);
        serve(v6);
        serve_status(status4.get(), v4, v6);
        serve_status(status6.get(), v4, v6);

        const uint32_t now = pf::monotonic_ms();
        wiz::poll_board_network(now);
        if (static_cast<int32_t>(now - next_report) >= 0) {
            next_report += kReportPeriodMs;
            char text[512];
            if (format_status(text, sizeof(text), v4, v6) > 0) {
                std::printf("[echo] %s\n", text);
            }
        }
    }
}
