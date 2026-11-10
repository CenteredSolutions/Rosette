// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file test_participant_8_5_3_lease.cpp
 * @brief A discovered participant's lease: what renews it and when it
 *        expires (RTPSParticipant::parse_received(), SpdpReader::renew_lease()).
 *
 * A peer is kept while it is heard from. Cyclone DDS announces SPDP every
 * 8 s against a 10 s lease, so a board that renewed leases from SPDP alone
 * dropped a Cyclone peer whenever one announcement was lost, although the
 * peer was exchanging ACKNACKs with it the whole time.
 */

#include "participant_test_support.hpp"

namespace spec_test {

extern uint32_t g_participant_test_now_ms;

#define CHECK(COND, MSG)  \
    do {                  \
        if (!(COND)) {    \
            err = (MSG);  \
            return false; \
        }                 \
    } while (0)

using namespace participant_support;

namespace {

/// An ACKNACK from `prefix`'s subscriptions detector to the local announcer.
bool build_sedp_acknack(const GuidPrefix& prefix, std::vector<Octet>& out) {
    SequenceNumberSet state{};
    state.bitmapBase = SequenceNumber::from_int64(1);
    state.numBits = 0u;
    return build_acknack_message(prefix, entity_id::SEDP_BUILTIN_SUBSCRIPTIONS_DETECTOR,
                                 entity_id::SEDP_BUILTIN_SUBSCRIPTIONS_ANNOUNCER, state, 1, true,
                                 out);
}

/// Delivers `msg` to the participant at `now_ms`.
ParseResult deliver(RTPSParticipant& p, const std::vector<Octet>& msg, uint32_t now_ms) {
    g_participant_test_now_ms = now_ms;
    return p.parse_received(msg.data(), static_cast<uint32_t>(msg.size()));
}

}  // namespace

// §8.5.3.2: a peer is alive while messages from it arrive. A well-formed
// non-SPDP message from a known peer renews its lease, so a lost SPDP
// announcement does not unmatch a peer that is still talking; a message from
// an unknown prefix admits nothing, and a malformed one renews nothing.
static bool tc_part_8_5_3_015(std::string& err) {
    CaptureTransport tx{};
    RTPSParticipant  p(0u, 1u, tx);
    CHECK(p.add_ipv4_address(0xC0A8010Au), "local address setup should succeed");

    const GuidPrefix   a = make_prefix(0xA1u, 0xA2u, 0xA3u);
    std::vector<Octet> spdp{};
    CHECK(build_spdp_payload(a, 30u, make_udpv4_locator(0xC0A80111u, 7410u), spdp),
          "SPDP payload build failed");
    g_participant_test_now_ms = 1000u;
    CHECK(p.spdp_reader().on_data(a, spdp.data(), static_cast<uint32_t>(spdp.size()), 1000u),
          "peer A is discovered with a 30 s lease");
    CHECK(p.spdp_reader().find_peer(a)->leaseExpiryMs == 31000u, "its lease runs to 31 s");

    std::vector<Octet> acknack{};
    CHECK(build_sedp_acknack(a, acknack), "ACKNACK build failed");
    CHECK(deliver(p, acknack, 20000u) == ParseResult::OK, "A's ACKNACK parses");
    CHECK(p.spdp_reader().find_peer(a)->leaseExpiryMs == 50000u,
          "an ACKNACK renews A's lease to now + 30 s");

    g_participant_test_now_ms = 40000u;
    p.tick(40000u);
    CHECK(p.spdp_reader().find_peer(a) != nullptr,
          "A outlives the expiry its last SPDP set, because it kept talking");

    const GuidPrefix   b = make_prefix(0xB1u, 0xB2u, 0xB3u);
    std::vector<Octet> stranger{};
    CHECK(build_sedp_acknack(b, stranger), "ACKNACK build failed");
    CHECK(deliver(p, stranger, 40000u) == ParseResult::OK, "B's ACKNACK parses");
    CHECK(p.spdp_reader().find_peer(b) == nullptr, "a message without SPDP admits no peer");

    std::vector<Octet> truncated = acknack;
    truncated.resize(truncated.size() - 4u);
    CHECK(deliver(p, truncated, 45000u) != ParseResult::OK, "a truncated ACKNACK is rejected");
    CHECK(p.spdp_reader().find_peer(a)->leaseExpiryMs == 50000u, "and does not renew A's lease");

    g_participant_test_now_ms = 50001u;
    p.tick(50001u);
    CHECK(p.spdp_reader().find_peer(a) == nullptr, "A expires once it falls silent for 30 s");
    return true;
}

void append_participant_lease_tests(std::vector<TestCase>& tests) {
    tests.push_back({"TC-PART-8.5.3-015",
                     "§8.5.3.2: any well-formed message from a known peer renews its lease; an "
                     "unknown prefix admits no peer and a malformed message renews nothing.",
                     tc_part_8_5_3_015});
}

}  // namespace spec_test
