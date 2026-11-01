// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file ports/chip/host/chip_host.cpp
 * @brief Host (Linux, macOS) chip layer of the platform contract.
 *
 * The monotonic clock is CLOCK_MONOTONIC. A host runs many Rosette processes
 * at once, so the unique ID hashes (FNV-1a) the machine's identity with the
 * process ID and start time, and the boot nonce hashes the process ID and
 * start time alone. Wall-clock time comes from the OS, which keeps it
 * synchronized. A host has no GPIO interrupts.
 */

#include "rosette/platform/gpio_interrupt.hpp"
#include "rosette/platform/platform.hpp"
#include "rosette/support/fnv1a.hpp"

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

/// Continues the 64-bit FNV-1a hash @p h over the object representation of @p value.
template <typename T>
uint64_t fnv1a_object(uint64_t h, const T& value) noexcept {
    return support::fnv1a64(reinterpret_cast<const uint8_t*>(&value), sizeof(value), h);
}

}  // namespace

void device_unique_id(uint8_t out[8]) noexcept {
    // Computed once: the identity must stay stable for the process lifetime.
    static const uint64_t id = [] {
        char host[256] = {};
        gethostname(host, sizeof(host) - 1);
        uint64_t h = support::fnv1a64(reinterpret_cast<const uint8_t*>(host), std::strlen(host));
        h = fnv1a_object(h, gethostid());
        h = fnv1a_object(h, getpid());
        timespec start{};
        clock_gettime(CLOCK_REALTIME, &start);
        return fnv1a_object(h, start);
    }();
    for (int i = 0; i < 8; ++i) {
        out[i] = static_cast<uint8_t>(id >> (8 * i));
    }
}

uint32_t boot_nonce() noexcept {
    // Computed once: the process ID and start time differ at every process start.
    static const uint32_t nonce = [] {
        timespec start{};
        clock_gettime(CLOCK_REALTIME, &start);
        const pid_t pid = getpid();
        const uint32_t h = support::fnv1a32(reinterpret_cast<const uint8_t*>(&pid), sizeof(pid));
        return support::fnv1a32(reinterpret_cast<const uint8_t*>(&start), sizeof(start), h);
    }();
    return nonce;
}

bool realtime_hint_us(int64_t& unix_us) noexcept {
    timespec ts{};
    if (clock_gettime(CLOCK_REALTIME, &ts) != 0) return false;
    unix_us = static_cast<int64_t>(ts.tv_sec) * 1000000 + ts.tv_nsec / 1000;
    return true;
}

bool attach_falling_edge(uint32_t pin, GpioInterruptHandler handler, void* context) noexcept {
    (void)pin;
    (void)handler;
    (void)context;
    return false;
}

}  // namespace rosette::platform
