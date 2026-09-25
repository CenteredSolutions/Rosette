/**
 * @file test_stateful_reader_8_4_10_4_changes_from_writer.cpp
 * @brief WriterProxy changes_from_writer: which changes a reader still expects.
 *
 * A reliable reader must remember every change it is missing for as long as
 * the writer may still send it, however many later changes arrive meanwhile,
 * and must deliver each change exactly once. A best-effort reader delivers
 * only changes newer than any it has seen. These tests drive a StatefulReader
 * through bursts, holes, and repairs and check the ACKNACKs it sends and the
 * changes it delivers.
 */

#include <map>

#include "spec_test_common.hpp"

#include "rosette/rtps/stateful_reader.hpp"
#include "rosette/rtps/transport.hpp"

namespace spec_test {

#define CHECK(COND, MSG) do { if (!(COND)) { err = (MSG); return false; } } while (0)

namespace {

struct CaptureTransport : ITransport {
    std::vector<std::vector<Octet>> unicast_msgs{};

    bool send_unicast(const BufRef& buf, const Locator&) noexcept override {
        unicast_msgs.emplace_back(buf.data, buf.data + buf.size);
        return true;
    }
    bool send_multicast(const BufRef&, const Locator&) noexcept override { return true; }
};

struct Fixture {
    CaptureTransport tx{};
    std::map<int64_t, int> delivered{};  ///< SN -> times delivered.
    Guid reader_guid{};
    Guid writer_guid{};
    Locator loc = make_udpv4_locator(0xC0A801A0u, 7411u);
    StatefulReader r;
    int32_t hb_count = 0;

    explicit Fixture(ReliabilityKind reliability = ReliabilityKind::RELIABLE)
        : reader_guid(make_guid(0x71u, make_entity(9u, 9u, 9u, 0x04u))),
          writer_guid(make_guid(0x81u, make_entity(10u, 10u, 10u, 0x03u))),
          r(reader_guid, reliability, tx, on_change, this, 0u) {
        (void)r.matched_writer_add(writer_guid, &loc, 1u);
    }

    static Guid make_guid(Octet p0, const EntityId& eid) {
        Guid g{};
        g.prefix.value[0] = p0;
        g.prefix.value[1] = 0x7Cu;
        g.entityId = eid;
        return g;
    }

    static void on_change(const CacheChange& c, void* user) noexcept {
        ++static_cast<Fixture*>(user)->delivered[c.sequenceNumber.to_int64()];
    }

    ParseContext ctx() const {
        ParseContext c{};
        c.senderGuidPrefix = writer_guid.prefix;
        return c;
    }

    void deliver(int64_t sn) {
        static const Octet payload[5] = {0x00u, 0x01u, 0x00u, 0x00u, 0x42u};
        ParsedData data{};
        data.readerId = reader_guid.entityId;
        data.writerId = writer_guid.entityId;
        data.writerSN = SequenceNumber::from_int64(sn);
        data.dataFlag = true;
        data.serializedPayload = payload;
        data.serializedPayloadSize = sizeof(payload);
        r.on_data(ctx(), data);
    }

    void deliver_range(int64_t first, int64_t last) {
        for (int64_t sn = first; sn <= last; ++sn) deliver(sn);
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

    bool last_acknack(ParsedAckNack& out) const {
        if (tx.unicast_msgs.empty()) return false;
        ParseOps ops{};
        ops.on_acknack = [](const ParseContext& c, const ParsedAckNack& a) noexcept {
            *static_cast<ParsedAckNack*>(c.user_data) = a;
        };
        const std::vector<Octet>& msg = tx.unicast_msgs.back();
        return parse_message(msg.data(), static_cast<uint32_t>(msg.size()), GuidPrefix{}, ops,
                             &out) == ParseResult::OK;
    }

    /// True when the ACKNACK requests exactly the sequence numbers in `want`.
    static bool requests_exactly(const ParsedAckNack& ack, std::initializer_list<int64_t> want) {
        uint32_t expected_bits = 0u;
        for (int64_t sn : want) {
            const int64_t off = sn - ack.readerSNState.bitmapBase.to_int64();
            if (off < 0 || !ack.readerSNState.test(static_cast<uint32_t>(off))) return false;
            ++expected_bits;
        }
        uint32_t set_bits = 0u;
        for (uint32_t i = 0; i < ack.readerSNState.numBits; ++i) {
            if (ack.readerSNState.test(i)) ++set_bits;
        }
        return set_bits == expected_bits;
    }

    bool each_delivered_once(int64_t first, int64_t last) const {
        for (int64_t sn = first; sn <= last; ++sn) {
            auto it = delivered.find(sn);
            if (it == delivered.end() || it->second != 1) return false;
        }
        return delivered.size() == static_cast<size_t>(last - first + 1);
    }
};

}  // namespace

// §8.4.10.4, §8.4.12.2: a change lost in transit stays MISSING however many
// later changes arrive before the writer's next HEARTBEAT; the ACKNACK
// requests it and its repair is delivered.
static bool tc_srr_8_4_10_001(std::string& err) {
    Fixture f{};
    f.deliver(1);
    f.deliver_range(3, 11);  // SN 2 lost; nine later changes arrive
    f.heartbeat(1, 11);
    ParsedAckNack ack{};
    CHECK(f.last_acknack(ack), "HEARTBEAT is answered");
    CHECK(!ack.finalFlag && Fixture::requests_exactly(ack, {2}),
          "the lost SN 2 is still requested after nine later changes");
    f.deliver(2);
    CHECK(f.each_delivered_once(1, 11), "the repair of SN 2 is delivered; every change exactly once");
    return true;
}

// §8.4.10.4, §8.4.12.2: changes too far past a hole to be recorded are not
// delivered on arrival; they are requested once the hole closes and each
// change is delivered exactly once in the end.
static bool tc_srr_8_4_10_002(std::string& err) {
    Fixture f{};
    f.deliver(1);
    f.deliver_range(3, 80);  // SN 2 lost; 78 later changes arrive
    f.heartbeat(1, 80);
    ParsedAckNack ack{};
    CHECK(f.last_acknack(ack) && Fixture::requests_exactly(ack, {2}),
          "the ACKNACK requests the hole at SN 2");
    f.deliver(2);
    f.heartbeat(1, 80);
    CHECK(f.last_acknack(ack), "the second HEARTBEAT is answered");
    for (int64_t sn = 3; sn <= 80; ++sn) {
        const int64_t off = sn - ack.readerSNState.bitmapBase.to_int64();
        if (off >= 0 && ack.readerSNState.test(static_cast<uint32_t>(off))) f.deliver(sn);
    }
    f.heartbeat(1, 80);
    CHECK(f.last_acknack(ack) && ack.readerSNState.empty() && ack.finalFlag &&
          ack.readerSNState.bitmapBase == SequenceNumber::from_int64(81),
          "after the repairs nothing is missing: a pure ACK of 1-80");
    CHECK(f.each_delivered_once(1, 80), "every change 1-80 is delivered exactly once");
    return true;
}

// §8.3.7.2, §8.4.12.2: a DATA carrying neither payload nor key (only inline
// QoS) still accounts for its sequence number, so it is not requested again.
static bool tc_srr_8_4_10_003(std::string& err) {
    Fixture f{};
    f.deliver(1);
    ParsedData bare{};
    bare.readerId = f.reader_guid.entityId;
    bare.writerId = f.writer_guid.entityId;
    bare.writerSN = SequenceNumber::from_int64(2);
    bare.inlineQosPresent = true;
    bare.inlineQos.hasStatusInfo = true;
    bare.inlineQos.statusInfo = status_info::UNREGISTERED;
    f.r.on_data(f.ctx(), bare);
    f.heartbeat(1, 2);
    ParsedAckNack ack{};
    CHECK(f.last_acknack(ack), "HEARTBEAT is answered");
    CHECK(ack.readerSNState.empty() && ack.finalFlag &&
          ack.readerSNState.bitmapBase == SequenceNumber::from_int64(3),
          "SN 2 is accounted for: a pure ACK of 1-2");
    return true;
}

// §8.4.12.1: a best-effort reader delivers only changes newer than every
// change it has seen; a late, out-of-order change is dropped, and a jump
// ahead is delivered at once.
static bool tc_srr_8_4_12_001(std::string& err) {
    Fixture f{ReliabilityKind::BEST_EFFORT};
    f.deliver(1);
    f.deliver(3);
    f.deliver(2);
    CHECK(f.delivered.size() == 2u && f.delivered.count(2) == 0u,
          "the late SN 2 is not delivered");
    f.deliver(1000);
    f.deliver(3);
    CHECK(f.delivered.size() == 3u && f.delivered[1000] == 1 && f.delivered[3] == 1,
          "a jump to SN 1000 is delivered once and older changes are dropped");
    return true;
}

void append_stateful_reader_tests_8_4_10_4(std::vector<TestCase>& tests) {
    tests.push_back({"TC-SRR-8.4.10-001",
                     "§8.4.10.4/§8.4.12.2: a lost change stays MISSING however many later changes arrive; the ACKNACK requests it and its repair is delivered.",
                     tc_srr_8_4_10_001});
    tests.push_back({"TC-SRR-8.4.10-002",
                     "§8.4.10.4/§8.4.12.2: changes too far past a hole are requested once it closes; every change is delivered exactly once.",
                     tc_srr_8_4_10_002});
    tests.push_back({"TC-SRR-8.4.10-003",
                     "§8.3.7.2/§8.4.12.2: a DATA with neither payload nor key accounts for its sequence number.",
                     tc_srr_8_4_10_003});
    tests.push_back({"TC-SRR-8.4.12-001",
                     "§8.4.12.1: a best-effort reader delivers only changes newer than every change it has seen.",
                     tc_srr_8_4_12_001});
}

}  // namespace spec_test
