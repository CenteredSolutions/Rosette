/**
 * @file test_stateful_reader_8_4_9_gap_ranges.cpp
 * @brief Reliable StatefulReader: every change a GAP names becomes irrelevant.
 *
 * A GAP names a contiguous run [gapStart, gapList.bitmapBase) plus the
 * bitmap's set bits. Fast DDS sends runs with an empty bitmap (changes
 * 1-10 as {1, base 11}), and a run may be very long. These tests pin down
 * that the reader stops requesting every named change, delivers none of
 * them if they arrive later, and still requests the changes the GAP did not
 * name.
 */

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
    int delivered = 0;
    Guid reader_guid{};
    Guid writer_guid{};
    Locator loc = make_udpv4_locator(0xC0A80188u, 7411u);
    DefaultStatefulReader r;

    Fixture()
        : reader_guid(make_guid(0x51u, make_entity(7u, 7u, 7u, 0x04u))),
          writer_guid(make_guid(0x61u, make_entity(8u, 8u, 8u, 0x03u))),
          r(reader_guid, ReliabilityKind::RELIABLE, tx, on_change, this, 0u) {
        (void)r.matched_writer_add(writer_guid, &loc, 1u);
    }

    static Guid make_guid(Octet p0, const EntityId& eid) {
        Guid g{};
        g.prefix.value[0] = p0;
        g.prefix.value[1] = 0x6Bu;
        g.entityId = eid;
        return g;
    }

    static void on_change(const CacheChange&, void* user) noexcept {
        ++static_cast<Fixture*>(user)->delivered;
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

    void heartbeat(int64_t first, int64_t last, int32_t count) {
        ParsedHeartbeat hb{};
        hb.readerId = reader_guid.entityId;
        hb.writerId = writer_guid.entityId;
        hb.firstSN = SequenceNumber::from_int64(first);
        hb.lastSN = SequenceNumber::from_int64(last);
        hb.count = count;
        r.on_heartbeat(ctx(), hb);
    }

    void gap(int64_t start, int64_t base, std::initializer_list<int64_t> bits) {
        ParsedGap g{};
        g.readerId = reader_guid.entityId;
        g.writerId = writer_guid.entityId;
        g.gapStart = SequenceNumber::from_int64(start);
        g.gapList.clear();
        g.gapList.bitmapBase = SequenceNumber::from_int64(base);
        for (int64_t sn : bits) g.gapList.set(static_cast<uint32_t>(sn - base));
        r.on_gap(ctx(), g);
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
};

}  // namespace

// §8.3.7.4.5: a run with an empty bitmap ({1, base 11}, as Fast DDS sends
// for changes 1-10) makes all ten irrelevant, and late DATA for any of them
// is not delivered.
static bool tc_srr_8_4_9_003(std::string& err) {
    Fixture f{};
    f.heartbeat(1, 10, 1);
    f.gap(1, 11, {});
    f.heartbeat(1, 10, 2);
    ParsedAckNack ack{};
    CHECK(f.last_acknack(ack), "HEARTBEAT is answered");
    CHECK(ack.readerSNState.empty(), "nothing is requested after the GAP");

    f.deliver(5);
    CHECK(f.delivered == 0, "DATA for a GAPped change is not delivered");
    f.deliver(11);
    CHECK(f.delivered == 1, "the first change after the run is delivered");
    return true;
}

// §8.3.7.4.5: a run plus bitmap names exactly its changes; the rest stay
// requested. A run far past the frontier moves the frontier without
// tracking each change.
static bool tc_srr_8_4_9_004(std::string& err) {
    Fixture f{};
    f.deliver(1);
    f.heartbeat(1, 6, 1);
    f.gap(2, 4, {5});  // irrelevant: 2, 3 (run) and 5 (bitmap)
    f.heartbeat(1, 6, 2);
    ParsedAckNack ack{};
    CHECK(f.last_acknack(ack), "HEARTBEAT is answered");
    bool only_4_and_6 = true;
    for (int64_t sn = 1; sn <= 6; ++sn) {
        const int64_t off = sn - ack.readerSNState.bitmapBase.to_int64();
        const bool requested = off >= 0 && ack.readerSNState.test(static_cast<uint32_t>(off));
        if (requested != (sn == 4 || sn == 6)) only_4_and_6 = false;
    }
    CHECK(only_4_and_6, "only 4 and 6, which the GAP did not name, are requested");

    f.deliver(4);
    f.deliver(6);
    const int before = f.delivered;
    f.gap(7, 100007, {});
    f.deliver(100007);
    CHECK(f.delivered == before + 1, "a 100000-change run is applied and the next change delivered");
    return true;
}

void append_stateful_reader_tests_8_4_9_gap_ranges(std::vector<TestCase>& tests) {
    tests.push_back({"TC-SRR-8.4.9-003",
                     "§8.3.7.4.5: a GAP run with an empty bitmap makes every change in [gapStart, bitmapBase) irrelevant.",
                     tc_srr_8_4_9_003});
    tests.push_back({"TC-SRR-8.4.9-004",
                     "§8.3.7.4.5: a GAP run plus bitmap names exactly its changes; long runs move the frontier.",
                     tc_srr_8_4_9_004});
}

}  // namespace spec_test
