// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file src/net/sntp_wire.cpp
 * @brief NTPv4 packet encoding, reply validation, and the on-wire equations; see net/sntp_wire.hpp.
 *
 * Fields are read and written octet by octet in network order rather than
 * through a packed struct, so the code is independent of the target's
 * endianness and alignment rules.
 */

#include "rosette/net/sntp_wire.hpp"

namespace rosette::net {
namespace {

/// Seconds from the NTP epoch (1900-01-01) to the Unix epoch (1970-01-01).
constexpr int64_t kNtpToUnixSeconds = 2208988800ll;

/// Seconds in one NTP era (2^32).
constexpr int64_t kEraSeconds = 4294967296ll;

/// Octet offsets of the timestamps in the packet header (RFC 5905 §7.3).
constexpr std::size_t kOriginOffset = 24u;
constexpr std::size_t kReceiveOffset = 32u;
constexpr std::size_t kTransmitOffset = 40u;
constexpr std::size_t kReferenceIdOffset = 12u;

uint32_t get_u32(const uint8_t* p) noexcept {
    return (uint32_t{p[0]} << 24) | (uint32_t{p[1]} << 16) | (uint32_t{p[2]} << 8) | uint32_t{p[3]};
}

void put_u32(uint8_t* p, uint32_t v) noexcept {
    p[0] = static_cast<uint8_t>(v >> 24);
    p[1] = static_cast<uint8_t>(v >> 16);
    p[2] = static_cast<uint8_t>(v >> 8);
    p[3] = static_cast<uint8_t>(v);
}

NtpTimestamp get_timestamp(const uint8_t* p) noexcept {
    return NtpTimestamp{get_u32(p), get_u32(p + 4)};
}

}  // namespace

NtpTimestamp unix_to_ntp(int64_t unix_us) noexcept {
    int64_t seconds = unix_us / 1000000;
    int64_t micros = unix_us % 1000000;
    if (micros < 0) {
        --seconds;
        micros += 1000000;
    }

    const uint64_t fraction = (static_cast<uint64_t>(micros) << 32) / 1000000u;
    return NtpTimestamp{static_cast<uint32_t>(seconds + kNtpToUnixSeconds),
                        static_cast<uint32_t>(fraction)};
}

int64_t ntp_to_unix_us(NtpTimestamp ts) noexcept {
    // RFC 4330 §3: with the most significant bit clear, the time is in era 1 (2036 onward).
    int64_t seconds = static_cast<int64_t>(ts.seconds);
    if ((ts.seconds & 0x80000000u) == 0u) {
        seconds += kEraSeconds;
    }

    const int64_t micros =
        static_cast<int64_t>((static_cast<uint64_t>(ts.fraction) * 1000000u + (1ull << 31)) >> 32);
    return (seconds - kNtpToUnixSeconds) * 1000000 + micros;
}

void encode_sntp_request(NtpTimestamp transmit,
                         std::array<uint8_t, kSntpPacketBytes>& out) noexcept {
    out.fill(0u);
    out[0] = 0x23u;  // LI 0, VN 4, mode 3 (client).
    put_u32(&out[kTransmitOffset], transmit.seconds);
    put_u32(&out[kTransmitOffset + 4], transmit.fraction);
}

SntpReplyError decode_sntp_reply(const uint8_t* data, std::size_t length, NtpTimestamp sent,
                                 SntpReply& reply) noexcept {
    if (data == nullptr || length < kSntpPacketBytes) {
        return SntpReplyError::truncated;
    }

    const uint8_t leap = data[0] >> 6;
    const uint8_t version = (data[0] >> 3) & 0x07u;
    const uint8_t mode = data[0] & 0x07u;
    if (mode != 4u) {
        return SntpReplyError::not_server;
    }

    if (version < 3u || version > 4u) {
        return SntpReplyError::bad_version;
    }

    if (!(get_timestamp(&data[kOriginOffset]) == sent)) {
        return SntpReplyError::wrong_origin;
    }

    reply.stratum = data[1];
    if (reply.stratum == 0u) {
        for (std::size_t i = 0u; i < reply.kiss_code.size(); ++i) {
            reply.kiss_code[i] = static_cast<char>(data[kReferenceIdOffset + i]);
        }

        return SntpReplyError::kiss_of_death;
    }

    if (leap == 3u || reply.stratum > 15u) {
        return SntpReplyError::unsynchronized;
    }

    const NtpTimestamp transmit = get_timestamp(&data[kTransmitOffset]);
    if (transmit.seconds == 0u && transmit.fraction == 0u) {
        return SntpReplyError::zero_transmit;
    }

    reply.receive_unix_us = ntp_to_unix_us(get_timestamp(&data[kReceiveOffset]));
    reply.transmit_unix_us = ntp_to_unix_us(transmit);
    return SntpReplyError::none;
}

SntpSample solve_sntp_exchange(int64_t t1_us, int64_t t2_us, int64_t t3_us,
                               int64_t t4_us) noexcept {
    SntpSample sample{};
    sample.offset_us = ((t2_us - t1_us) + (t3_us - t4_us)) / 2;
    sample.delay_us = (t4_us - t1_us) - (t3_us - t2_us);
    return sample;
}

}  // namespace rosette::net
