/**
 * @file ports/net/posix/posix_board_network.cpp
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
 * The host address is that of the one interface the process is on:
 * `ROSETTE_INTERFACE`, by name or address, else the default route's, else
 * loopback (posix_host_interface.hpp). The socket backend joins and sends
 * discovery multicast on the same interface, because rmw_init() passes this
 * address to the transport.
 */

#include "rosette/net/board_network.hpp"

#include <array>
#include <cstdio>

#include "posix_host_interface.hpp"
#include "posix_socket.hpp"
#include "rosette/platform/platform.hpp"

namespace rosette::net {
namespace {

namespace pf = rosette::platform;

/// The config bring-up accepted; `ready_when` is null until then.
NetworkConfig g_config{};
bool g_started = false;

/**
 * @brief The host's address as a snapshot, rebuilt from the operating system's.
 *
 * @return The live snapshot: one `manual` IPv4 entry, the address and
 *   prefix length of the process's interface (posix_host_interface.hpp),
 *   while it has one; none when `ROSETTE_INTERFACE` names no usable interface.
 */
AddressSnapshot& host_snapshot() noexcept {
    static AddressSnapshot snapshot{};
    const posix::HostInterface& chosen = posix::host_interface();
    const uint32_t              host = chosen.address;
    snapshot.active = true;
    snapshot.assigned = host != 0u;
    snapshot.v4.source = AddressSource::manual;
    snapshot.v4.state = host != 0u ? AddressPhase::bound : AddressPhase::acquiring;
    snapshot.v4.addresses.clear();
    if (host != 0u) {
        const std::array<uint8_t, 16> address{
            static_cast<uint8_t>(host >> 24), static_cast<uint8_t>(host >> 16),
            static_cast<uint8_t>(host >> 8), static_cast<uint8_t>(host)};
        (void)snapshot.v4.addresses.add(AddressFamily::ipv4, address, chosen.prefix_len);
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

    const auto& v4 = board_address_snapshot().v4;
    const auto& a = v4.address();
    std::printf("[net] address %u.%u.%u.%u/%u\n", a[0], a[1], a[2], a[3], v4.primary().prefix_len);
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
    return posix::host_interface().address;
}

const AddressSnapshot& board_address_snapshot() noexcept {
    return host_snapshot();
}

uint32_t board_send_timeouts() noexcept {
    return 0u;
}

}  // namespace rosette::net
