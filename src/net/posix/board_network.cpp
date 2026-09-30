/**
 * @file src/net/posix/board_network.cpp
 * @brief The board network on a host: BSD sockets and the operating system's address.
 *
 * See rosette/net/board_network.hpp. On a host the operating system owns the
 * interface and runs its addressing (DHCP, link-local), so this backend
 * starts nothing: it reports the host's IPv4 address as a snapshot and hands
 * out one process-wide BSD socket provider. Both are valid without bring-up,
 * because stock ROS 2 programs running on rmw_rosette never call it; a
 * program that does call it gets the same ready/timeout semantics as a
 * board, which lets firmware-shaped applications run on the host unchanged.
 *
 * The host address is the `ROSETTE_LOCAL_IP` override, else the address the
 * routing table picks for outbound traffic, else loopback.
 */

#include "rosette/net/board_network.hpp"

#include <array>
#include <cstdio>
#include <cstdlib>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include "rosette/net/posix/socket.hpp"
#include "rosette/platform/platform.hpp"

namespace rosette::net {
namespace {

namespace pf = rosette::platform;

/**
 * @brief Parses a dotted-quad string into a host-order IPv4 address.
 *
 * @param[in] text Candidate address string; may be null.
 * @return The parsed address in host byte order, or 0 when `text` is not a
 *   valid IPv4 literal.
 */
uint32_t ipv4_host_from_string(const char* text) noexcept {
    if (text == nullptr) {
        return 0u;
    }

    in_addr addr{};
    if (inet_pton(AF_INET, text, &addr) != 1) {
        return 0u;
    }

    return ntohl(addr.s_addr);
}

/**
 * @brief Asks the kernel which local IPv4 address would route to the internet.
 *
 * Opens an unconnected UDP socket, `connect()`s it toward a well-known public
 * address (no packets are sent), and reads the source address the routing
 * table selected.
 *
 * @return The selected local IPv4 address in host byte order, or 0 on failure.
 */
uint32_t detect_local_ipv4_host() noexcept {
    const int fd = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (fd < 0) {
        return 0u;
    }

    sockaddr_in remote{};
    remote.sin_family = AF_INET;
    remote.sin_port = htons(53u);
    (void)inet_pton(AF_INET, "8.8.8.8", &remote.sin_addr);
    (void)::connect(fd, reinterpret_cast<const sockaddr*>(&remote), sizeof(remote));

    sockaddr_in local{};
    socklen_t local_len = static_cast<socklen_t>(sizeof(local));
    const int rc = ::getsockname(fd, reinterpret_cast<sockaddr*>(&local), &local_len);
    (void)::close(fd);

    return (rc == 0) ? ntohl(local.sin_addr.s_addr) : 0u;
}

/**
 * @brief The host's IPv4 address, computed once per process.
 *
 * @return `ROSETTE_LOCAL_IP` if set and valid, else the routing table's
 *   choice, else 127.0.0.1; in host byte order.
 */
uint32_t host_ipv4() noexcept {
    static const uint32_t value = [] {
        if (const uint32_t forced = ipv4_host_from_string(std::getenv("ROSETTE_LOCAL_IP"))) {
            return forced;
        }

        if (const uint32_t detected = detect_local_ipv4_host()) {
            return detected;
        }

        return static_cast<uint32_t>(0x7F000001u);  // 127.0.0.1
    }();
    return value;
}

/// The config bring-up accepted; `ready_when` is null until then.
NetworkConfig g_config{};
bool g_started = false;

/**
 * @brief The host's address as a snapshot, rebuilt from the operating system's.
 *
 * @return The live snapshot: one `manual` IPv4 entry while the host holds an
 *   address (prefix length 0: the host's prefix is not read), none otherwise.
 */
AddressSnapshot& host_snapshot() noexcept {
    static AddressSnapshot snapshot{};
    const uint32_t host = host_ipv4();
    snapshot.active = true;
    snapshot.assigned = host != 0u;
    snapshot.v4.source = AddressSource::manual;
    snapshot.v4.state = host != 0u ? AddressPhase::bound : AddressPhase::acquiring;
    snapshot.v4.addresses.clear();
    if (host != 0u) {
        const std::array<uint8_t, 16> address{
            static_cast<uint8_t>(host >> 24), static_cast<uint8_t>(host >> 16),
            static_cast<uint8_t>(host >> 8), static_cast<uint8_t>(host)};
        (void)snapshot.v4.addresses.add(AddressFamily::ipv4, address, 0u);
    }

    return snapshot;
}

}  // namespace

NetworkCapabilities board_network_capabilities() noexcept {
    NetworkCapabilities capabilities{};
    capabilities.v4 = {Method::dhcp, Method::ipv4_link_local};
    return capabilities;
}

NetworkConfig board_default_network() noexcept {
    return NetworkConfig{};
}

/**
 * @brief Waits for the host to hold an IPv4 address that satisfies `config.ready_when`.
 *
 * @param[in] hostname Unused: the operating system names the host.
 * @param[in] config   Checked against board_network_capabilities(); copied.
 * @return `ready`, `timed_out`, or `unsupported_config`, as the header states.
 *
 * Verification:
 * - Correct means: the default config returns `ready` on a host with an
 *   address, and a config listing `static_ipv4` or SLAAC is refused.
 * - Covered by: tests/host/posix_board_network_test.cpp (TC-NET-BRD-001,
 *   TC-NET-BRD-002).
 */
BringUpResult bring_up_board_network(const char* hostname, const NetworkConfig& config) noexcept {
    (void)hostname;
    if (g_started) {
        return board_network_ready() ? BringUpResult::ready : BringUpResult::timed_out;
    }

    const ConfigError error = check_config(config, board_network_capabilities());
    if (error != ConfigError::none) {
        std::printf("[net] network config refused: %s\n", describe(error));
        return BringUpResult::unsupported_config;
    }

    g_config = config;
    g_started = true;
    const uint32_t start = pf::monotonic_ms();
    while (!board_network_ready()) {
        if (config.ready_timeout_ms != pf::kWaitForever
            && pf::monotonic_ms() - start >= config.ready_timeout_ms) {
            return BringUpResult::timed_out;
        }

        pf::sleep_ms(100u);
    }

    const auto& a = board_address_snapshot().v4.address();
    std::printf("[net] address %u.%u.%u.%u\n", a[0], a[1], a[2], a[3]);
    return BringUpResult::ready;
}

void poll_board_network(uint32_t now_ms) noexcept {
    (void)now_ms;
    (void)host_snapshot();
}

bool board_network_ready() noexcept {
    return g_started && g_config.ready_when(host_snapshot());
}

NetworkWatch board_network_watch() noexcept {
    return {poll_board_network, board_network_ready, board_address_snapshot};
}

rosette::net::SocketProvider& board_socket_provider() noexcept {
    static posix::PosixSocketProvider provider;
    return provider;
}

uint32_t board_ipv4_address() noexcept {
    return host_ipv4();
}

const AddressSnapshot& board_address_snapshot() noexcept {
    return host_snapshot();
}

uint32_t board_send_timeouts() noexcept {
    return 0u;
}

}  // namespace rosette::net
