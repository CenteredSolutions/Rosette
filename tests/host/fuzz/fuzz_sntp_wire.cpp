/**
 * @file fuzz_sntp_wire.cpp
 * @brief Fuzz harness for the SNTP reply decoder and the on-wire solver.
 *
 * A reply arrives from the network and the client trusts its four timestamps
 * to discipline the clock. The input is an 8-octet request timestamp followed
 * by the datagram. Correct means no input reads outside the datagram and the
 * exchange solver, fed any decoded timestamps, does not overflow.
 */

#include <stddef.h>
#include <stdint.h>

#include "rosette/net/sntp_wire.hpp"

using namespace rosette::net;

namespace {

uint32_t be32(const uint8_t* p) {
    return (static_cast<uint32_t>(p[0]) << 24) | (static_cast<uint32_t>(p[1]) << 16)
           | (static_cast<uint32_t>(p[2]) << 8) | p[3];
}

}  // namespace

extern "C" int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size) {
    constexpr size_t kPrefixOctets = 8u;
    if (data == nullptr || size < kPrefixOctets) {
        return 0;
    }

    NtpTimestamp sent{};
    sent.seconds = be32(data);
    sent.fraction = be32(data + 4);

    SntpReply reply{};
    if (decode_sntp_reply(data + kPrefixOctets, size - kPrefixOctets, sent, reply)
        == SntpReplyError::none) {
        solve_sntp_exchange(0, reply.receive_unix_us, reply.transmit_unix_us, 1000);
    }

    return 0;
}
