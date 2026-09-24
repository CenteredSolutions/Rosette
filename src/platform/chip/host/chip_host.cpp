/**
 * @file chip_host.cpp
 * @brief Host (Linux, macOS) chip layer of the platform contract.
 *
 * The monotonic clock is CLOCK_MONOTONIC. A host runs many Rosette processes
 * at once, so the unique ID mixes the machine's identity with the process ID
 * and start time; wall-clock time comes from the OS, which keeps it
 * synchronized.
 */

#include "rosette/platform/platform.hpp"

#include <cstring>
#include <ctime>

#include <unistd.h>

namespace rosette::platform {

uint64_t monotonic_us() noexcept {
    timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1000000u + static_cast<uint64_t>(ts.tv_nsec) / 1000u;
}

namespace {

/// FNV-1a over `len` octets, continuing from `h`.
uint64_t fnv1a(uint64_t h, const void* data, size_t len) noexcept {
    const auto* p = static_cast<const uint8_t*>(data);
    for (size_t i = 0; i < len; ++i) {
        h ^= p[i];
        h *= 0x100000001B3ull;
    }
    return h;
}

}  // namespace

void device_unique_id(uint8_t out[8]) noexcept {
    // Computed once: the identity must stay stable for the process lifetime.
    static const uint64_t id = [] {
        uint64_t h = 0xCBF29CE484222325ull;
        char host[256] = {};
        gethostname(host, sizeof(host) - 1);
        h = fnv1a(h, host, std::strlen(host));
        const long hostid = gethostid();
        h = fnv1a(h, &hostid, sizeof(hostid));
        const pid_t pid = getpid();
        h = fnv1a(h, &pid, sizeof(pid));
        timespec start{};
        clock_gettime(CLOCK_REALTIME, &start);
        h = fnv1a(h, &start, sizeof(start));
        return h;
    }();
    for (int i = 0; i < 8; ++i) out[i] = static_cast<uint8_t>(id >> (8 * i));
}

bool realtime_hint_us(int64_t& unix_us) noexcept {
    timespec ts{};
    if (clock_gettime(CLOCK_REALTIME, &ts) != 0) return false;
    unix_us = static_cast<int64_t>(ts.tv_sec) * 1000000 + ts.tv_nsec / 1000;
    return true;
}

}  // namespace rosette::platform
