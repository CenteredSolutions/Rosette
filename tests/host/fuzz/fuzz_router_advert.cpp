// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file fuzz_router_advert.cpp
 * @brief Fuzz harness for the IPv6 Router Advertisement decoder.
 *
 * Any host on the link can send a Router Advertisement, and the decoder walks
 * its variable-length options. The input is a 16-octet source address followed
 * by the ICMPv6 message. Correct means no input reads outside the message.
 */

#include <stddef.h>
#include <stdint.h>

#include <array>

#include "rosette/net/ipv6_router_advert.hpp"

using namespace rosette::net;

extern "C" int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size) {
    constexpr size_t kPrefixOctets = 16u;
    if (data == nullptr || size < kPrefixOctets) {
        return 0;
    }

    std::array<uint8_t, 16> source{};
    for (size_t i = 0; i < source.size(); ++i) {
        source[i] = data[i];
    }

    RouterAdvert advert{};
    parse_router_advert(data + kPrefixOctets, size - kPrefixOctets, source, advert);
    return 0;
}
