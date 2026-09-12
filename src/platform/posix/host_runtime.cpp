/**
 * @file platform/posix/host_runtime.cpp
 * @brief POSIX implementation of the Rosette platform runtime accessors.
 *
 * Provides the millisecond clock and local-address queries declared in
 * `rosette/platform/host_runtime.hpp` for host builds (`ROSETTE_BOARD=posix`),
 * where they are used by examples and host tests that exercise the micro-ROS
 * client against a locally running `micro_ros_agent`.
 *
 * The local IPv4 address is discovered once by opening a throwaway UDP socket
 * toward a public address and reading back the kernel-selected source address
 * with `getsockname()`. `ROSETTE_LOCAL_IP` overrides the result when a specific
 * routable interface must be used (for example in Docker-backed CI).
 */

#include "rosette/platform/host_runtime.hpp"

#include <chrono>
#include <cstdlib>
#include <cstring>
#include <thread>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

namespace {

/**
 * @brief Parses a dotted-quad string into a host-order IPv4 address.
 *
 * @param[in] text Candidate address string; may be null.
 * @return The parsed address in host byte order, or 0 when @p text is not a
 * valid IPv4 literal.
 */
uint32_t ipv4_host_from_string(const char* text) noexcept
{
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
 * table selected. Returns 0 on any failure so callers fall back to loopback.
 *
 * @return The selected local IPv4 address in host byte order, or 0.
 */
uint32_t detect_local_ipv4_host() noexcept
{
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
 * @brief Returns the process-cached local IPv4 address.
 *
 * The result is computed once on first use: `ROSETTE_LOCAL_IP` if set and
 * valid, otherwise the routing-table selection, otherwise 127.0.0.1.
 *
 * @return Local IPv4 address in host byte order.
 */
uint32_t cached_ipv4_host() noexcept
{
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

uint32_t g_forced_ipv4_host = 0u;
uint8_t g_forced_ipv6[16] = {0};

}  // namespace

extern "C" uint32_t rosette_platform_get_ms(void)
{
    using clock = std::chrono::steady_clock;
    static const auto start = clock::now();
    return static_cast<uint32_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(clock::now() - start).count());
}

extern "C" uint32_t rosette_platform_get_ipv4_addr(void)
{
    return g_forced_ipv4_host != 0u ? g_forced_ipv4_host : cached_ipv4_host();
}

extern "C" void rosette_platform_set_ipv4_addr(uint32_t addr_host_order)
{
    g_forced_ipv4_host = addr_host_order;
}

extern "C" bool rosette_platform_get_ipv6_addr(uint8_t out[16])
{
    if (out == nullptr) {
        return false;
    }
    std::memcpy(out, g_forced_ipv6, 16u);
    uint8_t acc = 0u;
    for (unsigned i = 0; i < 16u; ++i) {
        acc |= g_forced_ipv6[i];
    }
    return acc != 0u;
}

extern "C" void rosette_platform_set_ipv6_addr(const uint8_t addr[16])
{
    if (addr != nullptr) {
        std::memcpy(g_forced_ipv6, addr, 16u);
    } else {
        std::memset(g_forced_ipv6, 0, 16u);
    }
}

extern "C" void rosette_platform_delay_ms(uint32_t ms)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(ms));
}

extern "C" void rosette_platform_wait_for_event(uint32_t max_wait_ms)
{
    // No hardware event source on the host; a short sleep keeps the poll loop
    // from spinning while staying responsive.
    std::this_thread::sleep_for(
        std::chrono::milliseconds(max_wait_ms < 2u ? max_wait_ms : 2u));
}
