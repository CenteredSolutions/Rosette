// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file fuzz_dhcp_wire.cpp
 * @brief Fuzz harness for the DHCP client's reply decoder.
 *
 * Any host on the LAN can send a datagram to the client's UDP port. The input
 * is a 4-octet transaction ID, a 6-octet hardware address, then the datagram.
 * The harness decodes it twice: once against the transaction the datagram
 * claims (so the option walk is reached) and once against an unrelated one.
 * Correct means no input reads outside the datagram or leaves a lease with a
 * count above its array.
 */

#include <stddef.h>
#include <stdint.h>

#include <array>

#include "ports/net/wiznet/dhcp_wire.hpp"

using namespace rosette::net::wiznet;

extern "C" int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size) {
    constexpr size_t kPrefixOctets = 10u;
    if (data == nullptr || size < kPrefixOctets) {
        return 0;
    }

    const uint32_t xid = (static_cast<uint32_t>(data[0]) << 24)
                         | (static_cast<uint32_t>(data[1]) << 16)
                         | (static_cast<uint32_t>(data[2]) << 8) | data[3];
    std::array<uint8_t, 6> mac{};
    for (size_t i = 0; i < mac.size(); ++i) {
        mac[i] = data[4 + i];
    }

    const uint8_t* datagram = data + kPrefixOctets;
    const size_t   length = size - kPrefixOctets;

    DhcpServerMessage message{};
    if (decode_dhcp_message(datagram, length, xid, mac, message)) {
        if (message.lease.dns_count > message.lease.dns.size()
            || message.lease.ntp_count > message.lease.ntp.size()) {
            __builtin_trap();
        }
    }

    decode_dhcp_message(datagram, length, xid ^ 1u, mac, message);
    return 0;
}
