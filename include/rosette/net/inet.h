#pragma once

/**
 * @file net/inet.h
 * @brief Small network-byte-order and text/binary IP address utilities.
 *
 * Design goal
 * -----------
 * Give the project one shared place for the common "inet" style helpers that
 * appear across transports, protocol encoders, and board/platform code.
 *
 * Why this exists
 * ---------------
 * Rosette spans POSIX hosts, Pico-class MCUs, and custom socket backends. Some
 * environments expose BSD `inet_pton()` and friends directly, while others do
 * not.  These helpers provide a portable subset with standard names and
 * semantics so protocol code can stop open-coding byte swaps and IPv4 parsers.
 *
 * References
 * ----------
 * - RFC 791 / RFC 2460 / RFC 8200: Internet headers use network byte order
 *   (big-endian) for multi-octet integer fields.
 * - POSIX inet(3), inet_pton(3), inet_ntop(3): source model for the APIs
 *   mirrored here.
 */

#include "socket.hpp"

#include <stddef.h>
#include <stdint.h>

#include <array>
#include <cstdio>
#include <cstring>

namespace rosette::net::inet {

inline constexpr int kAfInet = 2;
inline constexpr int kAfInet6 = 10;

inline uint16_t bswap16(uint16_t value) noexcept
{
    return static_cast<uint16_t>((value << 8u) | (value >> 8u));
}

inline uint32_t bswap32(uint32_t value) noexcept
{
    return __builtin_bswap32(value);
}

inline bool host_is_little_endian() noexcept
{
    const uint16_t probe = 0x0102u;
    return *(reinterpret_cast<const uint8_t*>(&probe)) == 0x02u;
}

inline uint16_t to_network16(uint16_t value) noexcept
{
    return host_is_little_endian() ? bswap16(value) : value;
}

inline uint16_t from_network16(uint16_t value) noexcept
{
    return to_network16(value);
}

inline uint32_t to_network32(uint32_t value) noexcept
{
    return host_is_little_endian() ? bswap32(value) : value;
}

inline uint32_t from_network32(uint32_t value) noexcept
{
    return to_network32(value);
}

inline void store_be16(uint8_t* out, uint16_t value) noexcept
{
    if (out == nullptr) {
        return;
    }
    const uint16_t be = to_network16(value);
    std::memcpy(out, &be, sizeof(be));
}

inline void store_be32(uint8_t* out, uint32_t value) noexcept
{
    if (out == nullptr) {
        return;
    }
    const uint32_t be = to_network32(value);
    std::memcpy(out, &be, sizeof(be));
}

inline uint16_t load_be16(const uint8_t* in) noexcept
{
    if (in == nullptr) {
        return 0u;
    }
    uint16_t be = 0u;
    std::memcpy(&be, in, sizeof(be));
    return from_network16(be);
}

inline uint32_t load_be32(const uint8_t* in) noexcept
{
    if (in == nullptr) {
        return 0u;
    }
    uint32_t be = 0u;
    std::memcpy(&be, in, sizeof(be));
    return from_network32(be);
}

inline int pton4(const char* text, uint8_t out[4]) noexcept
{
    if ((text == nullptr) || (out == nullptr)) {
        return 0;
    }

    uint32_t value = 0u;
    uint8_t octet_index = 0u;
    bool have_digit = false;

    for (const char* p = text;; ++p) {
        const char ch = *p;
        if ((ch >= '0') && (ch <= '9')) {
            have_digit = true;
            value = (value * 10u) + static_cast<uint32_t>(ch - '0');
            if (value > 255u) {
                return 0;
            }
            continue;
        }

        if ((ch == '.') || (ch == '\0')) {
            if (!have_digit || (octet_index >= 4u)) {
                return 0;
            }
            out[octet_index++] = static_cast<uint8_t>(value);
            value = 0u;
            have_digit = false;
            if (ch == '\0') {
                break;
            }
            continue;
        }

        return 0;
    }

    return (octet_index == 4u) ? 1 : 0;
}

inline int pton(int af, const char* text, void* out) noexcept
{
    if (af == kAfInet) {
        return pton4(text, static_cast<uint8_t*>(out));
    }

    // IPv6 text parsing is not needed by current call sites.
    return 0;
}

inline const char* ntop4(const uint8_t in[4],
                         char* out,
                         size_t capacity) noexcept
{
    if ((in == nullptr) || (out == nullptr) || (capacity == 0u)) {
        return nullptr;
    }

    const int rc = std::snprintf(out,
                                 capacity,
                                 "%u.%u.%u.%u",
                                 static_cast<unsigned>(in[0]),
                                 static_cast<unsigned>(in[1]),
                                 static_cast<unsigned>(in[2]),
                                 static_cast<unsigned>(in[3]));
    if ((rc <= 0) || (static_cast<size_t>(rc) >= capacity)) {
        return nullptr;
    }
    return out;
}

inline const char* ntop(int af,
                        const void* in,
                        char* out,
                        size_t capacity) noexcept
{
    if (af == kAfInet) {
        return ntop4(static_cast<const uint8_t*>(in), out, capacity);
    }

    return nullptr;
}

/**
 * @brief Render a 16-byte IPv6 address as text.
 *
 * Produces the plain eight-group colon-hex form (e.g.
 * "fe80:0:0:0:202:b3ff:fe1e:8329"). This is not RFC 5952 canonical form (no
 * "::" zero-run compression), but it is valid IPv6 text that any conformant
 * parser accepts.
 */
inline const char* ntop6(const uint8_t in[16],
                         char* out,
                         size_t capacity) noexcept
{
    if ((in == nullptr) || (out == nullptr) || (capacity == 0u)) {
        return nullptr;
    }

    const int rc = std::snprintf(
        out, capacity, "%x:%x:%x:%x:%x:%x:%x:%x",
        (static_cast<unsigned>(in[0]) << 8u) | in[1],
        (static_cast<unsigned>(in[2]) << 8u) | in[3],
        (static_cast<unsigned>(in[4]) << 8u) | in[5],
        (static_cast<unsigned>(in[6]) << 8u) | in[7],
        (static_cast<unsigned>(in[8]) << 8u) | in[9],
        (static_cast<unsigned>(in[10]) << 8u) | in[11],
        (static_cast<unsigned>(in[12]) << 8u) | in[13],
        (static_cast<unsigned>(in[14]) << 8u) | in[15]);
    if ((rc <= 0) || (static_cast<size_t>(rc) >= capacity)) {
        return nullptr;
    }
    return out;
}

/// Renders a `SocketAddress` as text, dispatching on its address family.
inline const char* ntop(const SocketAddress& address,
                        char* out,
                        size_t capacity) noexcept
{
    if (address.is_ipv6()) {
        return ntop6(address.address.data(), out, capacity);
    }
    const auto v4 = address.ipv4();
    return ntop4(v4.data(), out, capacity);
}

inline bool make_socket_address_v4(const char* text,
                                   uint16_t port,
                                   SocketAddress& out) noexcept
{
    std::array<uint8_t, 4> bytes{};
    if (pton4(text, bytes.data()) != 1) {
        return false;
    }

    out = rosette::net::make_socket_address_v4(bytes, port);
    return true;
}

}  // namespace rosette::net::inet
