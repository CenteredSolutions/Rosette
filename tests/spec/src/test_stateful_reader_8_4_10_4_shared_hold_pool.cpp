// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file test_stateful_reader_8_4_10_4_shared_hold_pool.cpp
 * @brief Readers that share one hold pool still deliver in order and never stall each other.
 *
 * A participant's user readers hold changes behind a gap in one
 * HeldChangePool, each up to its limit (held_changes.hpp). These tests run
 * two SharedPoolStatefulReaders over one small pool and check the rules
 * tests/formal/RosetteReaderDelivery.tla verifies: a reader that finds the
 * pool full is slowed, not stopped, since the change that closes its gap
 * needs no slot; a change beyond a reader's window costs neither a block
 * nor an assembly slot; and when the buffer pool is exhausted, the change
 * that closes a gap takes another reader's held block, as it must when that
 * reader's writer has stopped.
 */

#include <vector>

#include "spec_test_common.hpp"

#include "rosette/rtps/stateful_reader.hpp"
#include "rosette/rtps/transport.hpp"

namespace spec_test {

#define CHECK(COND, MSG)  \
    do {                  \
        if (!(COND)) {    \
            err = (MSG);  \
            return false; \
        }                 \
    } while (0)

namespace {

struct CaptureTransport : ITransport {
    std::vector<std::vector<Octet>> unicast_msgs{};

    bool send_unicast(const BufRef& buf, const Locator&) noexcept override {
        unicast_msgs.emplace_back(buf.data, buf.data + buf.size);
        return true;
    }

    bool send_multicast(const BufRef&, const Locator&) noexcept override { return true; }
};

Guid make_guid(Octet p0, const EntityId& eid) {
    Guid g{};
    g.prefix.value[0] = p0;
    g.prefix.value[1] = 0x4Du;
    g.entityId = eid;
    return g;
}

const Locator kLoc = make_udpv4_locator(0xC0A801A3u, 7411u);

/// One reader of the pair, the writer matched to it, and what it delivered.
struct Side {
    Guid                 reader;
    Guid                 writer;
    std::vector<int64_t> order{};
    int32_t              hb_count = 0;

    static void on_change(const CacheChange& c, void* user) noexcept {
        static_cast<Side*>(user)->order.push_back(c.sequenceNumber.to_int64());
    }

    ParseContext ctx() const {
        ParseContext c{};
        c.senderGuidPrefix = writer.prefix;
        return c;
    }
};

/// Two reliable readers sharing a hold pool of two slots, each with a limit of two.
struct Pair {
    CaptureTransport                 tx{};
    ProxyPoolStorage<WriterProxy, 2> proxies;
    FragmentAssemblyPoolStorage<2>   assembly;
    HeldChangePoolStorage<2>         held;
    Side                             a{make_guid(0x31u, make_entity(1u, 1u, 1u, 0x04u)),
           make_guid(0x41u, make_entity(2u, 2u, 2u, 0x03u))};
    Side                             b{make_guid(0x32u, make_entity(1u, 1u, 3u, 0x04u)),
           make_guid(0x42u, make_entity(2u, 2u, 4u, 0x03u))};
    SharedPoolStatefulReader         ra{
        a.reader, ReliabilityKind::RELIABLE, tx, proxies, 1u, 1u, assembly, 1u, held,
        2u,       Side::on_change,           &a};
    SharedPoolStatefulReader rb{
        b.reader, ReliabilityKind::RELIABLE, tx, proxies, 1u, 1u, assembly, 1u, held,
        2u,       Side::on_change,           &b};

    Pair() {
        (void)proxies.reserve(2u);
        (void)ra.matched_writer_add(a.writer, &kLoc, 1u);
        (void)rb.matched_writer_add(b.writer, &kLoc, 1u);
    }

    static void data(StatefulReader& r, const Side& s, int64_t sn) {
        static const Octet payload[5] = {0x00u, 0x01u, 0x00u, 0x00u, 0x42u};
        ParsedData         d{};
        d.readerId = s.reader.entityId;
        d.writerId = s.writer.entityId;
        d.writerSN = SequenceNumber::from_int64(sn);
        d.dataFlag = true;
        d.serializedPayload = payload;
        d.serializedPayloadSize = sizeof(payload);
        r.on_data(s.ctx(), d);
    }

    /// The ACKNACK `r` sends in answer to HEARTBEAT(1, last) from `s`'s writer.
    bool acknack_for(StatefulReader& r, Side& s, int64_t last, ParsedAckNack& out) {
        ParsedHeartbeat hb{};
        hb.readerId = s.reader.entityId;
        hb.writerId = s.writer.entityId;
        hb.firstSN = SequenceNumber::from_int64(1);
        hb.lastSN = SequenceNumber::from_int64(last);
        hb.count = ++s.hb_count;
        const size_t before = tx.unicast_msgs.size();
        r.on_heartbeat(s.ctx(), hb);
        if (tx.unicast_msgs.size() == before) {
            return false;
        }

        ParseOps ops{};
        ops.on_acknack = [](const ParseContext& c, const ParsedAckNack& ack) noexcept {
            *static_cast<ParsedAckNack*>(c.user_data) = ack;
        };
        const std::vector<Octet>& msg = tx.unicast_msgs.back();
        return parse_message(msg.data(), static_cast<uint32_t>(msg.size()), GuidPrefix{}, ops, &out)
               == ParseResult::OK;
    }

    static bool requests(const ParsedAckNack& ack, int64_t sn) {
        const int64_t off = sn - ack.readerSNState.bitmapBase.to_int64();
        return off >= 0 && off < static_cast<int64_t>(ack.readerSNState.numBits)
               && ack.readerSNState.test(static_cast<uint32_t>(off));
    }
};

}  // namespace

// §8.4.10.4: a reader that finds the shared pool full withdraws the change
// it cannot hold and is slowed, not stopped: the change that closes its gap
// needs no slot, and the withdrawn change follows in order.
static bool tc_srr_8_4_10_4_017(std::string& err) {
    Pair p{};
    Pair::data(p.ra, p.a, 1);
    Pair::data(p.ra, p.a, 3);
    Pair::data(p.ra, p.a, 4);
    CHECK(p.held.count() == 2u, "reader A fills the shared pool");
    Pair::data(p.rb, p.b, 1);
    Pair::data(p.rb, p.b, 3);
    ParsedAckNack ack{};
    CHECK(p.acknack_for(p.rb, p.b, 3, ack) && Pair::requests(ack, 2) && Pair::requests(ack, 3),
          "reader B could not hold SN 3 and requests it with SN 2");
    Pair::data(p.rb, p.b, 2);
    Pair::data(p.rb, p.b, 3);
    CHECK(p.b.order == (std::vector<int64_t>{1, 2, 3}), "reader B delivers every change in order");
    Pair::data(p.ra, p.a, 2);
    CHECK(p.a.order == (std::vector<int64_t>{1, 2, 3, 4}) && p.held.count() == 0u,
          "reader A releases its held run and frees the pool");
    return true;
}

// §8.4.10.4, §8.4.14: a fragmented change beyond the hold window takes no
// assembly slot, and a whole one is refused before it is copied.
static bool tc_srr_8_4_10_4_018(std::string& err) {
    static const Octet bytes[4] = {0xA0u, 0xA1u, 0xA2u, 0xA3u};
    Pair               p{};
    Pair::data(p.ra, p.a, 1);
    ParsedDataFrag df{};
    df.readerId = p.a.reader.entityId;
    df.writerId = p.a.writer.entityId;
    df.writerSN = SequenceNumber::from_int64(5);  // SN 2 missing; the window is 3..4
    df.fragmentStartingNum = 1u;
    df.fragmentsInSubmessage = 1u;
    df.fragmentSize = 4u;
    df.dataSize = 12u;
    df.fragmentPayload = bytes;
    df.fragmentPayloadSize = 4u;
    p.ra.on_data_frag(p.a.ctx(), df);
    CHECK(p.assembly.active_count() == 0u, "a fragment beyond the window takes no assembly slot");
    df.writerSN = SequenceNumber::from_int64(4);
    p.ra.on_data_frag(p.a.ctx(), df);
    CHECK(p.assembly.active_count() == 1u, "a fragment inside the window is assembled");
    Pair::data(p.ra, p.a, 6);
    CHECK(p.held.count() == 0u, "a whole change beyond the window is not held");
    ParsedAckNack ack{};
    CHECK(p.acknack_for(p.ra, p.a, 6, ack) && Pair::requests(ack, 2) && Pair::requests(ack, 5)
              && Pair::requests(ack, 6),
          "the refused changes are MISSING and requested with the rest");
    return true;
}

// §8.4.10.4: with the buffer pool exhausted, the change that closes one
// reader's gap takes the block of another reader's held change, as it must
// when that reader's writer has stopped; the evicted change is MISSING again.
static bool tc_srr_8_4_10_4_019(std::string& err) {
    Pair p{};
    Pair::data(p.ra, p.a, 1);
    Pair::data(p.ra, p.a, 3);  // writer A stops here, SN 3 held behind its lost SN 2
    Pair::data(p.rb, p.b, 1);
    std::vector<BufRef> hog{};
    for (BufRef blk = buf_alloc(5u); blk.valid(); blk = buf_alloc(5u)) {
        hog.push_back(static_cast<BufRef&&>(blk));
    }

    Pair::data(p.rb, p.b, 2);
    hog.clear();
    CHECK(p.b.order == (std::vector<int64_t>{1, 2}),
          "reader B's next change took reader A's held block");
    CHECK(p.held.count() == 0u && p.a.order == std::vector<int64_t>{1},
          "reader A's held change was given up, not delivered");
    ParsedAckNack ack{};
    CHECK(p.acknack_for(p.ra, p.a, 3, ack) && Pair::requests(ack, 2) && Pair::requests(ack, 3),
          "reader A requests its evicted SN 3 again");
    Pair::data(p.ra, p.a, 2);
    Pair::data(p.ra, p.a, 3);
    CHECK(p.a.order == (std::vector<int64_t>{1, 2, 3}), "and delivers it in order once repaired");
    return true;
}

void append_stateful_reader_tests_8_4_10_4_shared_hold_pool(std::vector<TestCase>& tests) {
    tests.push_back({"TC-SRR-8.4.10.4-017",
                     "§8.4.10.4: a reader that finds the shared hold pool full is slowed, not "
                     "stopped.",
                     tc_srr_8_4_10_4_017});
    tests.push_back({"TC-SRR-8.4.10.4-018",
                     "§8.4.10.4/§8.4.14: a change beyond the hold window takes no assembly slot "
                     "and no block, and stays MISSING.",
                     tc_srr_8_4_10_4_018});
    tests.push_back({"TC-SRR-8.4.10.4-019",
                     "§8.4.10.4: with the buffer pool exhausted, a closing change takes another "
                     "reader's held block; that change is requested again.",
                     tc_srr_8_4_10_4_019});
}

}  // namespace spec_test
