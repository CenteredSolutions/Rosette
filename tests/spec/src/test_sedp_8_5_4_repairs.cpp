// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file test_sedp_8_5_4_repairs.cpp
 * @brief SEDP builtin reader reliability: holes, GAPs, and narrow repairs.
 *
 * A remote SEDP writer announces endpoints in bursts. On a lossy link, or one
 * that cannot deliver IP-fragmented datagrams, part of a burst is lost while
 * later announcements arrive. The builtin readers are ordinary reliable
 * StatefulReaders; these tests pin down that, fed through SEDP, they keep
 * requesting every hole until DATA or GAP resolves it, and that on a
 * datagram-limited link each request names a single announcement so that
 * the writer's repair fits one datagram.
 */

#include "spec_test_common.hpp"
#include "sedp_peer_driver.hpp"

#include "rosette/rtps/sedp.hpp"
#include "rosette/rtps/sn_window.hpp"
#include "rosette/rtps/transport.hpp"

namespace spec_test {

#define CHECK(COND, MSG) do { if (!(COND)) { err = (MSG); return false; } } while (0)

namespace {

struct LimitedTransport : ITransport {
    uint32_t limit = kMaxUdpDatagram;
    std::vector<std::vector<Octet>> sent{};

    bool send_unicast(const BufRef& buf, const Locator&) noexcept override {
        sent.emplace_back(buf.data, buf.data + buf.size);
        return true;
    }
    bool send_multicast(const BufRef&, const Locator&) noexcept override { return false; }
    uint32_t max_datagram_size() const noexcept override { return limit; }
};

/// Parses the single ACKNACK in a sent message.
bool parse_acknack(const std::vector<Octet>& msg, ParsedAckNack& out) {
    ParseOps ops{};
    ops.on_acknack = [](const ParseContext& c, const ParsedAckNack& a) noexcept {
        *static_cast<ParsedAckNack*>(c.user_data) = a;
    };
    return parse_message(msg.data(), static_cast<uint32_t>(msg.size()), GuidPrefix{}, ops,
                         &out) == ParseResult::OK;
}

GuidPrefix make_prefix(Octet p0) {
    GuidPrefix prefix{};
    prefix.value[0] = p0;
    prefix.value[1] = 0x5Eu;
    return prefix;
}

}  // namespace

// §8.4.12.1: the window reports each change new once, closes holes in any
// order, and a NACK always starts at the lowest unresolved change.
static bool tc_sedp_8_5_4_010(std::string& err) {
    SequenceNumberWindow w{};
    CHECK(w.resolve(SequenceNumber::from_int64(3)), "SN 3 is new");
    CHECK(!w.resolve(SequenceNumber::from_int64(3)), "SN 3 again is a duplicate");
    CHECK(w.base() == SequenceNumber::from_int64(1), "holes 1-2 keep base at 1");

    SequenceNumberSet set{};
    w.unresolved(SequenceNumber::from_int64(5), 256u, set);
    CHECK(set.bitmapBase == SequenceNumber::from_int64(1), "NACK starts at the lowest hole");
    CHECK(set.test(0) && set.test(1) && !set.test(2) && set.test(3) && set.test(4),
          "NACK names 1, 2, 4, 5 but not the received 3");

    w.unresolved(SequenceNumber::from_int64(5), 1u, set);
    CHECK(set.numBits == 1u && set.test(0), "a one-change NACK names only SN 1");

    CHECK(w.resolve(SequenceNumber::from_int64(1)), "SN 1 is new");
    CHECK(w.resolve(SequenceNumber::from_int64(2)), "SN 2 is new");
    CHECK(w.base() == SequenceNumber::from_int64(4), "base advances over the closed prefix 1-3");

    w.resolve_below(SequenceNumber::from_int64(6));
    CHECK(w.base() == SequenceNumber::from_int64(6), "firstSN resolves what the writer dropped");
    w.unresolved(SequenceNumber::from_int64(5), 256u, set);
    CHECK(set.empty() && set.bitmapBase == SequenceNumber::from_int64(6),
          "nothing missing: a pure ACK of everything below 6");

    CHECK(!w.covers(SequenceNumber::from_int64(6 + 100)) &&
          !w.resolve(SequenceNumber::from_int64(6 + 100)), "beyond the window cannot be recorded");
    CHECK(!w.is_resolved(SequenceNumber::from_int64(6 + 100)), "so it is still unresolved");
    return true;
}

// §8.4.12.1, §8.3.7.4 on a 1472-octet link: a burst of lost announcements is
// requested one change at a time; each repair (DATA or GAP) requests the next
// immediately; once nothing is missing the next HEARTBEAT gets a pure ACK. On
// an unrestricted link the whole hole set is requested at once.
static bool tc_sedp_8_5_4_011(std::string& err) {
    const GuidPrefix local = make_prefix(0x01u);
    const EntityId pubs = entity_id::SEDP_BUILTIN_PUBLICATIONS_ANNOUNCER;
    const SedpPeer peer{make_prefix(0x02u), make_udpv4_locator(0xC0A80102u, 7410u)};
    ParsedAckNack ack{};

    {
        LimitedTransport wide{};
        EndpointMatchTable table(nullptr, nullptr, nullptr);
        SedpReader reader(local, wide, table);
        peer.discover(reader);
        wide.sent.clear();  // the preemptive ACKNACKs (TC-SRR-8.4.7.4-002)
        peer.heartbeat(reader, pubs, 1, 4, 1);
        CHECK(wide.sent.size() == 1u && parse_acknack(wide.sent[0], ack), "one ACKNACK");
        CHECK(ack.readerSNState.numBits == 4u, "unrestricted link: all four requested at once");
    }

    LimitedTransport tx{};
    tx.limit = 1472u;
    EndpointMatchTable table(nullptr, nullptr, nullptr);
    SedpReader reader(local, tx, table);
    peer.discover(reader);
    tx.sent.clear();  // the preemptive ACKNACKs (TC-SRR-8.4.7.4-002)

    // Announcement 4 arrives; 1-3 were in a datagram the link dropped.
    peer.account(reader, pubs, 4);
    peer.heartbeat(reader, pubs, 1, 4, 1);
    CHECK(tx.sent.size() == 1u && parse_acknack(tx.sent.back(), ack), "HEARTBEAT is answered");
    CHECK(ack.readerSNState.bitmapBase == SequenceNumber::from_int64(1) &&
          ack.readerSNState.numBits == 1u && ack.readerSNState.test(0),
          "narrow link: only SN 1 is requested");
    CHECK(!ack.finalFlag, "a request is not final");

    peer.account(reader, pubs, 1);
    CHECK(tx.sent.size() == 2u && parse_acknack(tx.sent.back(), ack),
          "the repair of SN 1 requests the next hole at once");
    CHECK(ack.readerSNState.bitmapBase == SequenceNumber::from_int64(2) &&
          ack.readerSNState.numBits == 1u, "next request names SN 2 alone");

    peer.gap(reader, pubs, 2, 3);
    CHECK(tx.sent.size() == 3u && parse_acknack(tx.sent.back(), ack),
          "a GAP for the requested SN requests the next hole");
    CHECK(ack.readerSNState.bitmapBase == SequenceNumber::from_int64(3) &&
          ack.readerSNState.numBits == 1u, "next request names SN 3 alone");

    peer.account(reader, pubs, 3);
    CHECK(tx.sent.size() == 3u, "the last repair leaves nothing to request");
    peer.heartbeat(reader, pubs, 1, 4, 2);
    CHECK(tx.sent.size() == 4u && parse_acknack(tx.sent.back(), ack), "HEARTBEAT is answered");
    CHECK(ack.readerSNState.empty() && ack.finalFlag &&
          ack.readerSNState.bitmapBase == SequenceNumber::from_int64(5),
          "nothing missing: pure ACK of 1-4");

    peer.account(reader, pubs, 3);
    CHECK(tx.sent.size() == 4u, "a duplicate announcement sends nothing");
    return true;
}

// §8.3.7.4.5: a GAP run with an empty bitmap resolves every announcement in
// it, whether the run starts at the window base or inside the window.
static bool tc_sedp_8_5_4_012(std::string& err) {
    SequenceNumberWindow w{};
    w.resolve_range(SequenceNumber::from_int64(1), SequenceNumber::from_int64(4));
    CHECK(w.base() == SequenceNumber::from_int64(4), "run 1-3 at the base advances it");
    w.resolve_range(SequenceNumber::from_int64(6), SequenceNumber::from_int64(8));
    CHECK(w.base() == SequenceNumber::from_int64(4) && !w.is_resolved(SequenceNumber::from_int64(5)) &&
          w.is_resolved(SequenceNumber::from_int64(6)) && w.is_resolved(SequenceNumber::from_int64(7)),
          "run 6-7 inside the window leaves 4-5 missing");
    w.resolve_range(SequenceNumber::from_int64(4), SequenceNumber::from_int64(100000));
    CHECK(w.base() == SequenceNumber::from_int64(100000), "a long run from the base jumps it");

    const EntityId pubs = entity_id::SEDP_BUILTIN_PUBLICATIONS_ANNOUNCER;
    const SedpPeer peer{make_prefix(0x04u), make_udpv4_locator(0xC0A80103u, 7410u)};
    LimitedTransport tx{};
    EndpointMatchTable table(nullptr, nullptr, nullptr);
    SedpReader reader(make_prefix(0x03u), tx, table);
    peer.discover(reader);
    peer.account(reader, pubs, 4);
    peer.gap(reader, pubs, 1, 4);
    peer.heartbeat(reader, pubs, 1, 4, 1);
    ParsedAckNack ack{};
    CHECK(!tx.sent.empty() && parse_acknack(tx.sent.back(), ack), "HEARTBEAT is answered");
    CHECK(ack.readerSNState.empty() && ack.readerSNState.bitmapBase == SequenceNumber::from_int64(5),
          "GAP {1, base 4} resolved 1-3: pure ACK of 1-4");
    return true;
}

void append_sedp_tests_8_5_4_repairs(std::vector<TestCase>& tests) {
    tests.push_back({"TC-SEDP-8.5.4-010",
                     "§8.4.10.4: SequenceNumberWindow reports each SN resolved once, tracks holes in any order, honors HEARTBEAT firstSN, and NACKs from the lowest hole.",
                     tc_sedp_8_5_4_010});
    tests.push_back({"TC-SEDP-8.5.4-011",
                     "§8.4.12.1/§8.3.7.4: SEDP readers request lost announcements until DATA or GAP resolves them; on a datagram-limited link one per ACKNACK, the next requested as each repair lands.",
                     tc_sedp_8_5_4_011});
    tests.push_back({"TC-SEDP-8.5.4-012",
                     "§8.3.7.4.5: a GAP run with an empty bitmap resolves every announcement in [gapStart, bitmapBase).",
                     tc_sedp_8_5_4_012});
}

}  // namespace spec_test
