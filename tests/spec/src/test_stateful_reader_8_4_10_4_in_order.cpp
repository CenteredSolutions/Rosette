// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file test_stateful_reader_8_4_10_4_in_order.cpp
 * @brief A reliable reader delivers each writer's changes in the order written.
 *
 * Under loss, repairs arrive after later changes. A reliable StatefulReader
 * holds a change that arrives behind a MISSING one (HeldChanges) and
 * delivers the held run once the gap closes, whether by the repair, by a
 * GAP, or by a HEARTBEAT whose firstSN passes it. The hold store and the
 * buffer pool are bounded; when either is full the reader gives up its
 * highest held change, which is requested again, so the change that closes
 * the gap always gets in (tests/formal/RosetteReaderDelivery.tla). Its
 * ACKNACKs never acknowledge a held change before it is delivered.
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
    g.prefix.value[1] = 0x5Au;
    g.entityId = eid;
    return g;
}

struct Fixture {
    CaptureTransport      tx{};
    std::vector<int64_t>  order{};  ///< SNs in the order delivered.
    Guid                  reader_guid = make_guid(0x61u, make_entity(4u, 4u, 4u, 0x04u));
    Guid                  writer_guid = make_guid(0x62u, make_entity(5u, 5u, 5u, 0x03u));
    Locator               loc = make_udpv4_locator(0xC0A801A1u, 7411u);
    DefaultStatefulReader r;
    int32_t               hb_count = 0;

    Fixture() : r(reader_guid, ReliabilityKind::RELIABLE, tx, on_change, this, 0u) {
        (void)r.matched_writer_add(writer_guid, &loc, 1u);
    }

    static void on_change(const CacheChange& c, void* user) noexcept {
        static_cast<Fixture*>(user)->order.push_back(c.sequenceNumber.to_int64());
    }

    ParseContext ctx() const {
        ParseContext c{};
        c.senderGuidPrefix = writer_guid.prefix;
        return c;
    }

    void data(int64_t sn) {
        static const Octet payload[5] = {0x00u, 0x01u, 0x00u, 0x00u, 0x42u};
        ParsedData         d{};
        d.readerId = reader_guid.entityId;
        d.writerId = writer_guid.entityId;
        d.writerSN = SequenceNumber::from_int64(sn);
        d.dataFlag = true;
        d.serializedPayload = payload;
        d.serializedPayloadSize = sizeof(payload);
        r.on_data(ctx(), d);
    }

    void heartbeat(int64_t first, int64_t last) {
        ParsedHeartbeat hb{};
        hb.readerId = reader_guid.entityId;
        hb.writerId = writer_guid.entityId;
        hb.firstSN = SequenceNumber::from_int64(first);
        hb.lastSN = SequenceNumber::from_int64(last);
        hb.count = ++hb_count;
        r.on_heartbeat(ctx(), hb);
    }

    void gap(int64_t sn) {
        ParsedGap g{};
        g.readerId = reader_guid.entityId;
        g.writerId = writer_guid.entityId;
        g.gapStart = SequenceNumber::from_int64(sn);
        g.gapList.bitmapBase = SequenceNumber::from_int64(sn + 1);
        r.on_gap(ctx(), g);
    }

    /// The ACKNACK that answers HEARTBEAT(1, last).
    bool acknack_for(int64_t last, ParsedAckNack& out) {
        heartbeat(1, last);
        if (tx.unicast_msgs.empty()) {
            return false;
        }

        ParseOps ops{};
        ops.on_acknack = [](const ParseContext& c, const ParsedAckNack& a) noexcept {
            *static_cast<ParsedAckNack*>(c.user_data) = a;
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

    bool delivered_exactly(std::initializer_list<int64_t> want) const {
        return order == std::vector<int64_t>(want);
    }
};

}  // namespace

// §8.4.10.4: changes that arrive out of order are delivered in the order written.
static bool tc_srr_8_4_10_4_010(std::string& err) {
    Fixture f{};
    f.data(3);
    f.data(2);
    CHECK(f.order.empty(), "SN 2 and 3 wait while SN 1 is missing");
    f.data(1);
    CHECK(f.delivered_exactly({1, 2, 3}), "SN 1 releases 2 and 3, in order");
    f.data(2);
    CHECK(f.order.size() == 3u, "a duplicate of a released change is dropped");
    return true;
}

// §8.3.7.4.5, §8.4.10.4: a GAP for the missing change releases the changes behind it.
static bool tc_srr_8_4_10_4_011(std::string& err) {
    Fixture f{};
    f.data(1);
    f.data(3);
    f.data(4);
    CHECK(f.delivered_exactly({1}), "SN 3 and 4 wait behind SN 2");
    f.gap(2);
    CHECK(f.delivered_exactly({1, 3, 4}), "the GAP for SN 2 releases 3 and 4");
    return true;
}

// §8.4.10.4 (lost_changes_update): a HEARTBEAT whose firstSN passes the
// missing change releases the changes behind it.
static bool tc_srr_8_4_10_4_012(std::string& err) {
    Fixture f{};
    f.data(1);
    f.data(4);
    f.data(3);
    f.heartbeat(3, 4);
    CHECK(f.delivered_exactly({1, 3, 4}), "firstSN 3 gives up SN 2 and releases 3 and 4");
    return true;
}

// §8.4.10.4: with every hold slot in use, a change above every held one is
// requested again rather than held, and the held ones are not acknowledged.
static bool tc_srr_8_4_10_4_013(std::string& err) {
    constexpr int64_t kHeld = HeldChanges::kCapacity;
    const int64_t     extra = 3 + kHeld;
    Fixture           f{};
    f.data(1);
    for (int64_t sn = 3; sn <= extra; ++sn) {
        f.data(sn);
    }

    ParsedAckNack ack{};
    CHECK(f.acknack_for(extra, ack), "HEARTBEAT is answered");
    CHECK(ack.readerSNState.bitmapBase == SequenceNumber::from_int64(2),
          "nothing past SN 1 is acknowledged while SN 2 is missing");
    CHECK(Fixture::requests(ack, 2) && Fixture::requests(ack, extra) && !Fixture::requests(ack, 3)
              && !Fixture::requests(ack, 2 + kHeld),
          "SN 2 and the change that found no slot are requested; held ones are not");
    f.data(2);
    f.data(extra);
    CHECK(f.order.size() == static_cast<size_t>(extra) && f.order.back() == extra,
          "the repairs deliver every change in order");
    return true;
}

// §8.4.10.4: with every hold slot in use, a change below the highest held
// one evicts it; the evicted change is requested again and delivered in its turn.
static bool tc_srr_8_4_10_4_014(std::string& err) {
    constexpr int64_t kHeld = HeldChanges::kCapacity;
    const int64_t     top = 3 + kHeld;  // SN 2 and SN 3 missing; 4..top held
    Fixture           f{};
    f.data(1);
    for (int64_t sn = 4; sn <= top; ++sn) {
        f.data(sn);
    }

    f.data(3);
    ParsedAckNack ack{};
    CHECK(f.acknack_for(top, ack), "HEARTBEAT is answered");
    CHECK(ack.readerSNState.bitmapBase == SequenceNumber::from_int64(2) && Fixture::requests(ack, 2)
              && Fixture::requests(ack, top) && !Fixture::requests(ack, 3),
          "SN 3 took the slot of the highest held change, which is requested again");
    f.data(2);
    CHECK(f.order.back() == top - 1, "SN 2 releases the held run up to the evicted change");
    f.data(top);
    CHECK(f.order.size() == static_cast<size_t>(top) && f.order.back() == top,
          "the evicted change is delivered last, every change in order");
    return true;
}

// §8.4.7.4 (matched_writer_remove): unmatching a writer drops what was held
// for it, and a rematch starts clean.
static bool tc_srr_8_4_10_4_015(std::string& err) {
    Fixture f{};
    f.data(2);
    f.data(3);
    f.r.matched_writer_remove(f.writer_guid);
    CHECK(f.order.empty(), "held changes of an unmatched writer are never delivered");
    CHECK(f.r.matched_writer_add(f.writer_guid, &f.loc, 1u), "the writer matches again");
    f.data(1);
    CHECK(f.delivered_exactly({1}), "nothing held before the unmatch is released after it");
    return true;
}

// §8.4.10.4: when the buffer pool is exhausted, the change that closes a gap
// takes the block of the writer's highest held change, which is requested again.
static bool tc_srr_8_4_10_4_016(std::string& err) {
    Fixture f{};
    f.data(1);
    f.data(3);
    f.data(4);
    std::vector<BufRef> hog{};
    for (BufRef b = buf_alloc(5u); b.valid(); b = buf_alloc(5u)) {
        hog.push_back(static_cast<BufRef&&>(b));
    }

    f.data(2);
    hog.clear();
    CHECK(f.delivered_exactly({1, 2, 3}), "SN 2 got the block of SN 4 and released SN 3");
    ParsedAckNack ack{};
    CHECK(f.acknack_for(4, ack) && ack.readerSNState.bitmapBase == SequenceNumber::from_int64(4)
              && Fixture::requests(ack, 4),
          "the evicted SN 4 is requested again");
    f.data(4);
    CHECK(f.delivered_exactly({1, 2, 3, 4}), "and delivered in its turn");
    return true;
}

void append_stateful_reader_tests_8_4_10_4_in_order(std::vector<TestCase>& tests) {
    tests.push_back(
        {"TC-SRR-8.4.10.4-010",
         "§8.4.10.4: changes that arrive out of order are delivered in the order written.",
         tc_srr_8_4_10_4_010});
    tests.push_back(
        {"TC-SRR-8.4.10.4-011",
         "§8.3.7.4.5/§8.4.10.4: a GAP for the missing change releases the changes held behind it.",
         tc_srr_8_4_10_4_011});
    tests.push_back({"TC-SRR-8.4.10.4-012",
                     "§8.4.10.4: a HEARTBEAT whose firstSN passes the missing change releases the "
                     "changes held behind it.",
                     tc_srr_8_4_10_4_012});
    tests.push_back({"TC-SRR-8.4.10.4-013",
                     "§8.4.10.4: with the hold store full, a change above every held one is "
                     "requested again; held changes are never acknowledged early.",
                     tc_srr_8_4_10_4_013});
    tests.push_back({"TC-SRR-8.4.10.4-014",
                     "§8.4.10.4: with the hold store full, a lower change evicts the highest held "
                     "one, which is requested again.",
                     tc_srr_8_4_10_4_014});
    tests.push_back({"TC-SRR-8.4.10.4-015",
                     "§8.4.7.4: unmatching a writer drops the changes held for it.",
                     tc_srr_8_4_10_4_015});
    tests.push_back({"TC-SRR-8.4.10.4-016",
                     "§8.4.10.4: with the buffer pool exhausted, the change closing a gap takes "
                     "the block of the highest held change.",
                     tc_srr_8_4_10_4_016});
}

}  // namespace spec_test
