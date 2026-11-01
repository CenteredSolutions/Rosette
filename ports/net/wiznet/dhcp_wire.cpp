// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file ports/net/wiznet/dhcp_wire.cpp
 * @brief DHCPv4 client message codec; see dhcp_wire.hpp.
 */

#include "dhcp_wire.hpp"

#include <cstring>

namespace rosette::net::wiznet {
namespace {

constexpr std::size_t kHeaderBytes = 240u;  // BOOTP fixed fields and the magic cookie
constexpr uint8_t kBootRequest = 1u;
constexpr uint8_t kBootReply = 2u;
constexpr uint8_t kHwTypeEthernet = 1u;
constexpr uint8_t kHwLengthEthernet = 6u;
constexpr uint16_t kBroadcastFlag = 0x8000u;
constexpr uint32_t kMagicCookie = 0x63825363u;
constexpr uint32_t kDefaultLeaseS = 86400u;
constexpr std::size_t kMaxHostNameBytes = 32u;

// Offsets of the BOOTP fields (RFC 2131 §2, Figure 1).
constexpr std::size_t kXidOffset = 4u;
constexpr std::size_t kFlagsOffset = 10u;
constexpr std::size_t kCiaddrOffset = 12u;
constexpr std::size_t kYiaddrOffset = 16u;
constexpr std::size_t kChaddrOffset = 28u;
constexpr std::size_t kCookieOffset = 236u;

enum Option : uint8_t {
    kPad = 0u,
    kSubnetMask = 1u,
    kRouter = 3u,
    kDnsServer = 6u,
    kHostName = 12u,
    kNtpServer = 42u,
    kRequestedIp = 50u,
    kLeaseTime = 51u,
    kMessageType = 53u,
    kServerId = 54u,
    kParameterList = 55u,
    kRenewalTime = 58u,
    kRebindingTime = 59u,
    kClientId = 61u,
    kEnd = 255u,
};

uint32_t read_be32(const uint8_t* p) noexcept {
    return (uint32_t{p[0]} << 24u) | (uint32_t{p[1]} << 16u) | (uint32_t{p[2]} << 8u)
           | uint32_t{p[3]};
}

void write_be32(uint8_t* p, uint32_t v) noexcept {
    p[0] = static_cast<uint8_t>(v >> 24u);
    p[1] = static_cast<uint8_t>(v >> 16u);
    p[2] = static_cast<uint8_t>(v >> 8u);
    p[3] = static_cast<uint8_t>(v);
}

bool is_zero(const std::array<uint8_t, 4>& a) noexcept {
    return (a[0] | a[1] | a[2] | a[3]) == 0u;
}

/// Appends one option; the encoder's fixed option set always fits a 548-byte message.
std::size_t put_option(std::array<uint8_t, kDhcpMaxMessageBytes>& out, std::size_t at, uint8_t code,
                       const uint8_t* value, std::size_t length) noexcept {
    out[at++] = code;
    out[at++] = static_cast<uint8_t>(length);
    std::memcpy(out.data() + at, value, length);
    return at + length;
}

/// Copies up to `out.size()` IPv4 addresses from a list option; returns how many.
template <std::size_t N>
uint8_t copy_address_list(const uint8_t* value, std::size_t length,
                          std::array<std::array<uint8_t, 4>, N>& out) noexcept {
    uint8_t count = 0u;
    for (std::size_t at = 0u; at + 4u <= length && count < N; at += 4u, ++count) {
        std::memcpy(out[count].data(), value + at, 4u);
    }

    return count;
}

/// Reads one option into the lease or the message type; unknown options are skipped.
void apply_option(uint8_t code, const uint8_t* value, std::size_t length, DhcpServerMessage& out) {
    DhcpLease& lease = out.lease;
    const bool has_address = length >= 4u;
    switch (code) {
        case kMessageType:
            if (length >= 1u) {
                out.type = static_cast<DhcpMessageType>(value[0]);
            }

            break;
        case kSubnetMask:
            if (has_address) {
                std::memcpy(lease.subnet.data(), value, 4u);
            }

            break;
        case kRouter:
            if (has_address) {
                std::memcpy(lease.gateway.data(), value, 4u);
            }

            break;
        case kServerId:
            if (has_address) {
                std::memcpy(lease.server_id.data(), value, 4u);
            }

            break;
        case kDnsServer:
            lease.dns_count = copy_address_list(value, length, lease.dns);
            break;
        case kNtpServer:
            lease.ntp_count = copy_address_list(value, length, lease.ntp);
            break;
        case kLeaseTime:
            lease.lease_s = has_address ? read_be32(value) : lease.lease_s;
            break;
        case kRenewalTime:
            lease.t1_s = has_address ? read_be32(value) : lease.t1_s;
            break;
        case kRebindingTime:
            lease.t2_s = has_address ? read_be32(value) : lease.t2_s;
            break;
        default:
            break;
    }
}

/// Fills lease timers a server left out, with the RFC 2131 §4.4.5 defaults.
void default_timers(DhcpLease& lease) noexcept {
    if (lease.lease_s == 0u) {
        lease.lease_s = kDefaultLeaseS;
    }

    if (lease.lease_s == kDhcpInfiniteLease) {
        lease.t1_s = kDhcpInfiniteLease;
        lease.t2_s = kDhcpInfiniteLease;
        return;
    }

    if (lease.t1_s == 0u) {
        lease.t1_s = lease.lease_s / 2u;
    }

    if (lease.t2_s == 0u) {
        lease.t2_s = static_cast<uint32_t>(uint64_t{lease.lease_s} * 7u / 8u);
    }
}

}  // namespace

std::size_t encode_dhcp_message(const DhcpClientMessage& message,
                                std::array<uint8_t, kDhcpMaxMessageBytes>& out) noexcept {
    out.fill(0u);
    out[0] = kBootRequest;
    out[1] = kHwTypeEthernet;
    out[2] = kHwLengthEthernet;
    write_be32(out.data() + kXidOffset, message.xid);
    if (is_zero(message.ciaddr)) {
        out[kFlagsOffset] = static_cast<uint8_t>(kBroadcastFlag >> 8u);
    }

    std::memcpy(out.data() + kCiaddrOffset, message.ciaddr.data(), 4u);
    std::memcpy(out.data() + kChaddrOffset, message.mac.data(), message.mac.size());
    write_be32(out.data() + kCookieOffset, kMagicCookie);

    const uint8_t type = static_cast<uint8_t>(message.type);
    std::size_t at = put_option(out, kHeaderBytes, kMessageType, &type, 1u);
    const uint8_t client_id[7] = {kHwTypeEthernet, message.mac[0], message.mac[1], message.mac[2],
                                  message.mac[3],  message.mac[4], message.mac[5]};
    at = put_option(out, at, kClientId, client_id, sizeof(client_id));
    if (!is_zero(message.requested_ip)) {
        at = put_option(out, at, kRequestedIp, message.requested_ip.data(), 4u);
    }

    if (!is_zero(message.server_id)) {
        at = put_option(out, at, kServerId, message.server_id.data(), 4u);
    }

    if (message.type != DhcpMessageType::decline && message.type != DhcpMessageType::release) {
        static constexpr uint8_t kWanted[] = {kSubnetMask, kRouter,   kDnsServer,   kNtpServer,
                                              kLeaseTime,  kServerId, kRenewalTime, kRebindingTime};
        at = put_option(out, at, kParameterList, kWanted, sizeof(kWanted));
        if (message.host_name != nullptr) {
            std::size_t length = 0u;
            while (length < kMaxHostNameBytes && message.host_name[length] != '\0') {
                ++length;
            }

            if (length > 0u) {
                at = put_option(out, at, kHostName,
                                reinterpret_cast<const uint8_t*>(message.host_name), length);
            }
        }
    }

    out[at++] = kEnd;
    return at;
}

bool decode_dhcp_message(const uint8_t* data, std::size_t length, uint32_t xid,
                         const std::array<uint8_t, 6>& mac, DhcpServerMessage& out) noexcept {
    if (data == nullptr || length < kHeaderBytes || data[0] != kBootReply
        || data[1] != kHwTypeEthernet || data[2] != kHwLengthEthernet
        || read_be32(data + kXidOffset) != xid
        || std::memcmp(data + kChaddrOffset, mac.data(), mac.size()) != 0
        || read_be32(data + kCookieOffset) != kMagicCookie) {
        return false;
    }

    DhcpServerMessage message{};
    std::memcpy(message.lease.ip.data(), data + kYiaddrOffset, 4u);
    std::size_t at = kHeaderBytes;
    while (at < length) {
        const uint8_t code = data[at++];
        if (code == kPad) {
            continue;
        }

        if (code == kEnd || at >= length) {
            break;
        }

        const std::size_t option_length = data[at++];
        if (at + option_length > length) {
            break;
        }

        apply_option(code, data + at, option_length, message);
        at += option_length;
    }

    if (message.type == DhcpMessageType::none) {
        return false;
    }

    if (message.type == DhcpMessageType::ack) {
        default_timers(message.lease);
    }

    out = message;
    return true;
}

}  // namespace rosette::net::wiznet
