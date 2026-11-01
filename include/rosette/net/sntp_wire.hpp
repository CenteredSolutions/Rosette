// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

#pragma once

/**
 * @file net/sntp_wire.hpp
 * @brief The NTPv4 client/server packet and the on-wire offset and delay calculation.
 *
 * An SNTP client sends a 48-octet mode-3 request whose transmit timestamp
 * the server echoes back as the originate timestamp; the server fills in
 * when it received the request (T2) and when it sent the reply (T3). With
 * the client's own send (T1) and receive (T4) instants, the four timestamps
 * give the clock offset and the round-trip delay (RFC 5905 §8):
 *
 *     offset = ((T2 - T1) + (T3 - T4)) / 2
 *     delay  =  (T4 - T1) - (T3 - T2)
 *
 * The functions here are the pure part of that exchange: encode, validate
 * and decode, convert timestamps, and solve the two equations. The client
 * that drives them over a socket is `SntpClient` (net/sntp_client.hpp).
 *
 * Timestamps in this interface are signed microseconds. T1 and T4 are read
 * from the local monotonic clock, T2 and T3 are Unix time, so the offset is
 * "Unix time minus monotonic time": exactly the value `platform::WallClock`
 * adds to the monotonic clock.
 */

#include <array>
#include <cstddef>
#include <cstdint>

namespace rosette::net {

/// Octets in an NTPv4 packet without extension fields or MAC (RFC 5905 §7.3).
inline constexpr std::size_t kSntpPacketBytes = 48u;

/// The UDP port NTP servers listen on (RFC 5905 §7.2).
inline constexpr uint16_t kNtpServerPort = 123u;

/// An NTP 64-bit timestamp: seconds since 1900-01-01 and a binary fraction of a second.
struct NtpTimestamp {
    uint32_t seconds = 0u;
    uint32_t fraction = 0u;

    friend bool operator==(const NtpTimestamp& a, const NtpTimestamp& b) noexcept {
        return a.seconds == b.seconds && a.fraction == b.fraction;
    }
};

/// Why a reply was not accepted, or `none` when it was.
enum class SntpReplyError : uint8_t {
    none = 0u,
    truncated,       ///< Fewer than 48 octets.
    not_server,      ///< Mode is not 4 (server).
    bad_version,     ///< Version outside 3..4.
    kiss_of_death,   ///< Stratum 0: the server refuses service (RFC 5905 §7.4).
    unsynchronized,  ///< Leap indicator 3 or stratum above 15: the server has no time to give.
    wrong_origin,    ///< Originate timestamp is not the request's transmit timestamp.
    zero_transmit,   ///< The server's transmit timestamp is zero.
};

/// The fields of a server reply that an SNTP client uses.
struct SntpReply {
    uint8_t stratum = 0u;
    int64_t receive_unix_us = 0;   ///< T2: when the server received the request.
    int64_t transmit_unix_us = 0;  ///< T3: when the server sent the reply.
    std::array<char, 4>
        kiss_code{};  ///< Reference ID as ASCII when stratum is 0 (for example "RATE").
};

/// One exchange's result: the clock offset and the round-trip delay.
struct SntpSample {
    int64_t offset_us = 0;  ///< Unix time minus local monotonic time.
    int64_t delay_us = 0;   ///< Round-trip network delay, the server's hold time excluded.
};

/**
 * @brief Converts Unix microseconds to an NTP timestamp (era 0, 1900 to 2036).
 *
 * @param[in] unix_us Microseconds since 1970-01-01; negative values reach back before 1970.
 * @return The timestamp; the fraction is `usec * 2^32 / 10^6`, truncated.
 *
 * Verification:
 * - Correct means: `ntp_to_unix_us(unix_to_ntp(t)) == t` for every t in era 0.
 * - Covered by: tests/host/sntp_wire_test.cpp (TC-NET-SNTP-6-001).
 *
 * References:
 * - RFC 5905 §6 (timestamp format, 1900 epoch, 2208988800 s to the Unix epoch)
 */
NtpTimestamp unix_to_ntp(int64_t unix_us) noexcept;

/**
 * @brief Converts an NTP timestamp to Unix microseconds.
 *
 * Seconds below 2^31 are read as era 1 (2036-02-07 onward), per the RFC 4330
 * §3 rule, so the conversion stays correct past the era-0 rollover.
 *
 * @param[in] ts The timestamp.
 * @return Microseconds since 1970-01-01, with the fraction rounded to the nearest microsecond.
 *
 * Verification:
 * - Correct means: the Unix epoch maps to 0; an era-1 timestamp maps past 2036.
 * - Covered by: tests/host/sntp_wire_test.cpp (TC-NET-SNTP-6-001, TC-NET-SNTP-6-002).
 *
 * References:
 * - RFC 5905 §6; RFC 4330 §3 (the MSB rule for era 1)
 */
int64_t ntp_to_unix_us(NtpTimestamp ts) noexcept;

/**
 * @brief Writes a client (mode 3, version 4) request carrying `transmit` as its transmit timestamp.
 *
 * Every other field is zero, as RFC 4330 §5 recommends for a client request;
 * the server copies `transmit` into its reply's originate timestamp.
 *
 * @param[in]  transmit The request's transmit timestamp; the caller keeps it to validate the reply.
 * @param[out] out      The 48-octet request.
 *
 * Verification:
 * - Correct means: octet 0 is 0x23 (LI 0, VN 4, mode 3); octets 40..47 hold
 *   `transmit` in network order; all other octets are zero.
 * - Covered by: tests/host/sntp_wire_test.cpp (TC-NET-SNTP-5-001).
 *
 * References:
 * - RFC 4330 §5; RFC 5905 §7.3 (packet header layout)
 */
void encode_sntp_request(NtpTimestamp transmit,
                         std::array<uint8_t, kSntpPacketBytes>& out) noexcept;

/**
 * @brief Validates a server reply to the request whose transmit timestamp was `sent` and decodes
 * it.
 *
 * A reply is accepted only if it is a version-3 or -4 server-mode packet
 * from a synchronized server (leap indicator not 3, stratum 1..15) whose
 * originate timestamp equals `sent` (which rejects stale, duplicated, and
 * off-path replies) and whose transmit timestamp is nonzero. A stratum-0
 * reply is a kiss-o'-death; its reference ID is returned as `kiss_code`.
 *
 * @param[in]  data   The received datagram.
 * @param[in]  length Its length in octets.
 * @param[in]  sent   The transmit timestamp of the request being answered.
 * @param[out] reply  The decoded reply; filled for `none` and for `kiss_of_death`.
 * @return `SntpReplyError::none` when the reply may be used, else why not.
 *
 * Verification:
 * - Correct means: each RFC 4330 §5 sanity check rejects exactly its fault.
 * - Covered by: tests/host/sntp_wire_test.cpp (TC-NET-SNTP-5-002 to TC-NET-SNTP-5-004).
 *
 * References:
 * - RFC 4330 §5 (client checks on a server reply); RFC 5905 §7.3, §7.4 (kiss codes), §8
 */
SntpReplyError decode_sntp_reply(const uint8_t* data, std::size_t length, NtpTimestamp sent,
                                 SntpReply& reply) noexcept;

/**
 * @brief Solves the on-wire equations for one exchange.
 *
 * @param[in] t1_us Local monotonic time the request was sent.
 * @param[in] t2_us Unix time the server received it.
 * @param[in] t3_us Unix time the server sent the reply.
 * @param[in] t4_us Local monotonic time the reply was received.
 * @return offset = ((T2 - T1) + (T3 - T4)) / 2 and delay = (T4 - T1) - (T3 - T2).
 *
 * Verification:
 * - Correct means: with a symmetric path, the offset equals the true
 *   difference between the clocks and the delay equals the two path legs.
 * - Covered by: tests/host/sntp_wire_test.cpp (TC-NET-SNTP-8-001).
 *
 * References:
 * - RFC 5905 §8 (on-wire protocol, offset and delay)
 */
SntpSample solve_sntp_exchange(int64_t t1_us, int64_t t2_us, int64_t t3_us, int64_t t4_us) noexcept;

}  // namespace rosette::net
