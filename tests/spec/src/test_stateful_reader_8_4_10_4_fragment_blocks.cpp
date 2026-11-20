// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file test_stateful_reader_8_4_10_4_fragment_blocks.cpp
 * @brief A fragmented change takes its buffer-pool block under the rules a whole DATA follows.
 *
 * A reliable reader that delivers in order holds changes behind a gap, and
 * each held change pins one buffer-pool block. Two rules keep that from
 * stopping delivery (StatefulReader::take_sample_block()): the change that
 * closes the gap may take a held change's block, and a change that would
 * only be held must leave a block for the ACKNACK that requests the gap.
 * A fragmented change takes its block when its first fragment starts the
 * assembly, so both rules apply there. These tests drain the pool's
 * classes with blocks of their own, so they hold for the host pool and for
 * the firmware profile (tests/spec/CMakeLists.txt) alike.
 */

#include <vector>

#include "spec_test_common.hpp"

#include "rosette/rtps/message_builder.hpp"
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

constexpr uint32_t kControl = MessageBuilder::kControlMessageCapacity;

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
    g.prefix.value[1] = 0x7Cu;
    g.entityId = eid;
    return g;
}

struct Fixture {
    CaptureTransport      tx{};
    std::vector<int64_t>  order{};  ///< SNs in the order delivered.
    Guid                  reader_guid = make_guid(0x71u, make_entity(8u, 8u, 8u, 0x04u));
    Guid                  writer_guid = make_guid(0x72u, make_entity(9u, 9u, 9u, 0x03u));
    Locator               loc = make_udpv4_locator(0xC0A801A3u, 7411u);
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

    /// Fragments `first` .. `first + count - 1` of a `data_size`-octet sample in 512-octet
    /// fragments.
    void frags(int64_t sn, uint32_t data_size, uint32_t first, uint16_t count) {
        constexpr uint16_t frag_size = 512u;
        std::vector<Octet> payload(static_cast<size_t>(count) * frag_size, 0x42u);
        ParsedDataFrag     d{};
        d.readerId = reader_guid.entityId;
        d.writerId = writer_guid.entityId;
        d.writerSN = SequenceNumber::from_int64(sn);
        d.fragmentStartingNum = first;
        d.fragmentsInSubmessage = count;
        d.fragmentSize = frag_size;
        d.dataSize = data_size;
        d.fragmentPayload = payload.data();
        d.fragmentPayloadSize = static_cast<uint32_t>(payload.size());
        r.on_data_frag(ctx(), d);
    }

    /// Every fragment of a `data_size`-octet sample, in one submessage.
    void sample(int64_t sn, uint32_t data_size) {
        frags(sn, data_size, 1u, static_cast<uint16_t>((data_size + 511u) / 512u));
    }

    /// The ACKNACK that answers HEARTBEAT(1, last); false when none went out.
    bool acknack_for(int64_t last, ParsedAckNack& out) {
        const size_t    before = tx.unicast_msgs.size();
        ParsedHeartbeat hb{};
        hb.readerId = reader_guid.entityId;
        hb.writerId = writer_guid.entityId;
        hb.firstSN = SequenceNumber::from_int64(1);
        hb.lastSN = SequenceNumber::from_int64(last);
        hb.count = ++hb_count;
        r.on_heartbeat(ctx(), hb);
        if (tx.unicast_msgs.size() == before) {
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

// §8.4.10.4: with every block of the largest class pinned by held
// fragmented changes, the fragmented change that closes the gap takes the
// block of the highest held change, which is requested again.
static bool tc_srr_8_4_10_4_024(std::string& err) {
    Fixture        f{};
    const uint8_t  below_largest = static_cast<uint8_t>(g_buf_pool.class_count() - 2u);
    const uint32_t size =
        static_cast<uint32_t>(g_buf_pool.class_info(below_largest).block_size) + 1u;
    f.data(1);

    std::vector<BufRef> hog{};
    for (BufRef b = buf_alloc_received(size); b.valid(); b = buf_alloc_received(size)) {
        hog.push_back(static_cast<BufRef&&>(b));
    }

    CHECK(hog.size() >= 2u, "the largest class has two blocks to leave free");
    hog.pop_back();
    hog.pop_back();
    f.sample(3, size);
    f.sample(4, size);
    CHECK(f.delivered_exactly({1}), "SN 3 and SN 4 are held, pinning the last two large blocks");

    f.sample(2, size);
    CHECK(f.delivered_exactly({1, 2, 3}),
          "the fragmented SN 2 took SN 4's block and released SN 3");
    hog.clear();
    ParsedAckNack ack{};
    CHECK(f.acknack_for(4, ack) && ack.readerSNState.bitmapBase == SequenceNumber::from_int64(4)
              && Fixture::requests(ack, 4),
          "the evicted SN 4 is requested again");
    f.sample(4, size);
    CHECK(f.delivered_exactly({1, 2, 3, 4}), "and delivered in its turn");
    return true;
}

// §8.4.10.4: a fragmented change that arrives behind a gap does not start
// its assembly in the last block an ACKNACK fits in; it stays MISSING, the
// ACKNACK requesting the gap goes out, and the change is delivered once
// requested again.
static bool tc_srr_8_4_10_4_025(std::string& err) {
    Fixture f{};
    f.data(1);

    // Drain every class a control message may take; the last block taken
    // is of the largest class it is promoted into, and is given back.
    std::vector<BufRef> hog{};
    for (BufRef b = buf_alloc(kControl); b.valid(); b = buf_alloc(kControl)) {
        hog.push_back(static_cast<BufRef&&>(b));
    }

    CHECK(!hog.empty(), "the pool has blocks for control messages");
    const uint32_t size = hog.back().capacity();
    hog.pop_back();
    CHECK(g_buf_pool.can_alloc(kControl), "one block is left for an ACKNACK");

    f.frags(3, size, 1u, 1u);
    CHECK(g_buf_pool.can_alloc(kControl), "SN 3's first fragment did not take that block");
    ParsedAckNack ack{};
    CHECK(f.acknack_for(3, ack) && Fixture::requests(ack, 2) && Fixture::requests(ack, 3),
          "the ACKNACK goes out and requests SN 2 and SN 3");

    hog.clear();
    f.data(2);
    f.sample(3, size);
    CHECK(f.delivered_exactly({1, 2, 3}), "the resent SN 3 is delivered in order");
    return true;
}

void append_stateful_reader_tests_8_4_10_4_fragment_blocks(std::vector<TestCase>& tests) {
    tests.push_back({"TC-SRR-8.4.10.4-024",
                     "§8.4.10.4: with the large blocks pinned by held changes, the fragmented "
                     "change closing a gap takes the block of the highest held change.",
                     tc_srr_8_4_10_4_024});
    tests.push_back({"TC-SRR-8.4.10.4-025",
                     "§8.4.10.4: a fragmented change held behind a gap never starts its assembly "
                     "in the last block an ACKNACK fits in.",
                     tc_srr_8_4_10_4_025});
}

}  // namespace spec_test
