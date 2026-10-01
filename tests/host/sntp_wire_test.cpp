/**
 * @file tests/host/sntp_wire_test.cpp
 * @brief The NTPv4 packet encoding, reply checks, timestamp conversion, and on-wire equations.
 *
 * References: RFC 5905 §6, §7.3, §7.4, §8; RFC 4330 §3, §5.
 */

#include <array>
#include <cstdint>

#include "rosette/net/sntp_wire.hpp"
#include "test_util.hpp"

namespace net = rosette::net;

namespace {

/// A valid server reply to `sent`: stratum 2, T2 and T3 at the given Unix times.
std::array<uint8_t, 48> reply(net::NtpTimestamp sent, int64_t t2_us, int64_t t3_us) {
    std::array<uint8_t, 48> r{};
    r[0] = 0x24u;
    r[1] = 2u;
    auto put = [&r](int at, net::NtpTimestamp ts) {
        const uint32_t words[2] = {ts.seconds, ts.fraction};
        for (int w = 0; w < 2; ++w) {
            for (int b = 0; b < 4; ++b) {
                r[at + w * 4 + b] = static_cast<uint8_t>(words[w] >> (24 - 8 * b));
            }
        }
    };
    put(24, sent);
    put(32, net::unix_to_ntp(t2_us));
    put(40, net::unix_to_ntp(t3_us));
    return r;
}

void timestamps() {
    const int64_t samples[] = {0, 1, 999999, 1790000000123456, -1000000, 2085978495999999};
    bool round_trips = true;
    for (int64_t t : samples) {
        round_trips = round_trips && net::ntp_to_unix_us(net::unix_to_ntp(t)) == t;
    }

    check(round_trips, "TC-NET-SNTP-6-001 Unix -> NTP -> Unix is exact to the microsecond");
    check(net::unix_to_ntp(0).seconds == 2208988800u,
          "TC-NET-SNTP-6-001 the Unix epoch is NTP second 2208988800");

    // RFC 4330 §3: seconds with the MSB clear are era 1, after 2036-02-07T06:28:16Z.
    const int64_t era1 = net::ntp_to_unix_us(net::NtpTimestamp{1u, 0u});
    check(era1 == (4294967296ll + 1 - 2208988800ll) * 1000000,
          "TC-NET-SNTP-6-002 an era-1 timestamp decodes past 2036");
}

void request() {
    std::array<uint8_t, net::kSntpPacketBytes> out{};
    out.fill(0xAAu);
    net::encode_sntp_request(net::NtpTimestamp{0x01020304u, 0x05060708u}, out);
    bool others_zero = true;
    for (std::size_t i = 1; i < 40; ++i) {
        others_zero = others_zero && out[i] == 0u;
    }

    check(out[0] == 0x23u, "TC-NET-SNTP-5-001 request is LI 0, VN 4, mode 3");
    check(out[40] == 1u && out[43] == 4u && out[44] == 5u && out[47] == 8u,
          "TC-NET-SNTP-5-001 transmit timestamp is in network order at octet 40");
    check(others_zero, "TC-NET-SNTP-5-001 every other field is zero");
}

void reply_checks() {
    const net::NtpTimestamp sent{3999999999u, 42u};
    net::SntpReply out{};
    auto good = reply(sent, 1790000000000000, 1790000000000100);
    check(net::decode_sntp_reply(good.data(), good.size(), sent, out) == net::SntpReplyError::none
              && out.receive_unix_us == 1790000000000000
              && out.transmit_unix_us == 1790000000000100,
          "TC-NET-SNTP-5-002 a valid reply decodes T2 and T3");
    check(net::decode_sntp_reply(good.data(), 47u, sent, out) == net::SntpReplyError::truncated,
          "TC-NET-SNTP-5-002 a short datagram is rejected");
    check(net::decode_sntp_reply(good.data(), good.size(), net::NtpTimestamp{3999999999u, 43u}, out)
              == net::SntpReplyError::wrong_origin,
          "TC-NET-SNTP-5-002 a reply to another request is rejected");

    auto bad = good;
    bad[0] = 0x23u;
    check(net::decode_sntp_reply(bad.data(), bad.size(), sent, out)
              == net::SntpReplyError::not_server,
          "TC-NET-SNTP-5-003 a non-server mode is rejected");
    bad = good;
    bad[0] = 0x14u;  // VN 2.
    check(net::decode_sntp_reply(bad.data(), bad.size(), sent, out)
              == net::SntpReplyError::bad_version,
          "TC-NET-SNTP-5-003 version 2 is rejected");
    bad = good;
    bad[0] = 0xE4u;  // LI 3.
    check(net::decode_sntp_reply(bad.data(), bad.size(), sent, out)
              == net::SntpReplyError::unsynchronized,
          "TC-NET-SNTP-5-003 an unsynchronized server (LI 3) is rejected");
    bad = good;
    bad[1] = 16u;
    check(net::decode_sntp_reply(bad.data(), bad.size(), sent, out)
              == net::SntpReplyError::unsynchronized,
          "TC-NET-SNTP-5-003 stratum 16 is rejected");
    bad = good;
    for (int i = 40; i < 48; ++i) {
        bad[i] = 0u;
    }

    check(net::decode_sntp_reply(bad.data(), bad.size(), sent, out)
              == net::SntpReplyError::zero_transmit,
          "TC-NET-SNTP-5-003 a zero transmit timestamp is rejected");

    bad = good;
    bad[1] = 0u;
    bad[12] = 'R';
    bad[13] = 'A';
    bad[14] = 'T';
    bad[15] = 'E';
    check(net::decode_sntp_reply(bad.data(), bad.size(), sent, out)
                  == net::SntpReplyError::kiss_of_death
              && out.kiss_code == std::array<char, 4>{'R', 'A', 'T', 'E'},
          "TC-NET-SNTP-5-004 stratum 0 is a kiss-o'-death carrying its code");
}

void equations() {
    // Server 5 s ahead; 3 ms each way; 100 us hold.
    const int64_t t1 = 1000000;
    const int64_t t2 = t1 + 3000 + 5000000;
    const int64_t t3 = t2 + 100;
    const int64_t t4 = t1 + 3000 + 100 + 3000;
    const net::SntpSample s = net::solve_sntp_exchange(t1, t2, t3, t4);
    check(s.offset_us == 5000000 && s.delay_us == 6000,
          "TC-NET-SNTP-8-001 symmetric path: offset is the clock difference, delay the two legs");
}

}  // namespace

int main() {
    timestamps();
    request();
    reply_checks();
    equations();
    return test_summary();
}
