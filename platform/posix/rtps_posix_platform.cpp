#include "rtps/types.hpp"

#include <array>
#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <cstring>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

namespace {

uint32_t ipv4_be_from_string(const char* text) noexcept
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

uint32_t detect_local_ipv4_be() noexcept
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

    if (rc != 0) {
        return 0u;
    }
    return ntohl(local.sin_addr.s_addr);
}

uint32_t cached_ipv4_be() noexcept
{
    static const uint32_t value = [] {
        if (const char* env = std::getenv("MILKDDS_LOCAL_IP")) {
            const uint32_t parsed = ipv4_be_from_string(env);
            if (parsed != 0u) {
                return parsed;
            }
        }

        const uint32_t detected = detect_local_ipv4_be();
        if (detected != 0u) {
            return detected;
        }

        return static_cast<uint32_t>(0x7F000001u);  // 127.0.0.1 fallback.
    }();
    return value;
}

}  // namespace

extern "C" uint32_t rtps_platform_get_ms(void)
{
    using clock = std::chrono::steady_clock;
    static const auto start = clock::now();
    const auto elapsed =
        std::chrono::duration_cast<std::chrono::milliseconds>(clock::now() - start);
    return static_cast<uint32_t>(elapsed.count());
}

extern "C" rtps::GuidPrefix rtps_platform_make_guid_prefix(uint8_t participant_id) noexcept
{
    const uint32_t ip = cached_ipv4_be();
    const uint32_t pid = static_cast<uint32_t>(::getpid());
    const uint32_t t = rtps_platform_get_ms();

    rtps::GuidPrefix gp{};
    gp.value[0] = 0x4Du;  // 'M'
    gp.value[1] = 0x44u;  // 'D'
    gp.value[2] = 0x53u;  // 'S'
    gp.value[3] = static_cast<uint8_t>((ip >> 24u) & 0xFFu);
    gp.value[4] = static_cast<uint8_t>((ip >> 16u) & 0xFFu);
    gp.value[5] = static_cast<uint8_t>((ip >> 8u) & 0xFFu);
    gp.value[6] = static_cast<uint8_t>(ip & 0xFFu);
    gp.value[7] = static_cast<uint8_t>((pid >> 8u) & 0xFFu);
    gp.value[8] = static_cast<uint8_t>(pid & 0xFFu);
    gp.value[9] = static_cast<uint8_t>((t >> 8u) & 0xFFu);
    gp.value[10] = static_cast<uint8_t>(t & 0xFFu);
    gp.value[11] = participant_id;
    return gp;
}

extern "C" uint32_t rtps_platform_get_ipv4_addr(void) noexcept
{
    return cached_ipv4_be();
}

extern "C" bool rtps_platform_get_ipv6_addr(uint8_t out[16]) noexcept
{
    if (out != nullptr) {
        std::memset(out, 0, 16u);
    }
    return false;
}
