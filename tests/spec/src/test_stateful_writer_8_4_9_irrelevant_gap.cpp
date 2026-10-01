/**
 * @file test_stateful_writer_8_4_9_irrelevant_gap.cpp
 * @brief Reliable StatefulWriter: requested changes a reader can never get are sent as GAP.
 *
 * A VOLATILE writer's HEARTBEAT advertises its whole history, so a reader
 * that matched mid-stream requests changes written before it matched, and
 * a KEEP_LAST history may evict a change before its repair. The writer
 * must answer such requests with a GAP whose run and bitmap name exactly
 * those changes; otherwise the reader keeps requesting them and finally
 * reports them as lost samples.
 */

#include "spec_test_common.hpp"

#include "rosette/rtps/gap_encoding.hpp"
#include "rosette/rtps/stateful_writer.hpp"

namespace spec_test {

extern uint32_t g_stateful_writer_test_now_ms;

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

struct GapCapture {
    int gaps = 0;
    ParsedGap gap{};
};

/// Counts the GAPs in every captured message and keeps the last one.
GapCapture collect_gaps(const CaptureTransport& tx) {
    GapCapture cap{};
    ParseOps ops{};
    ops.on_gap = [](const ParseContext& c, const ParsedGap& g) noexcept {
        auto* out = static_cast<GapCapture*>(c.user_data);
        ++out->gaps;
        out->gap = g;
    };
    for (const auto& msg : tx.unicast_msgs) {
        (void)parse_message(msg.data(), static_cast<uint32_t>(msg.size()), GuidPrefix{}, ops, &cap);
    }
    return cap;
}

Guid make_guid(Octet p0, const EntityId& eid) {
    Guid g{};
    g.prefix.value[0] = p0;
    g.prefix.value[1] = 0x9Au;
    g.entityId = eid;
    return g;
}

ParsedAckNack nack(const Guid& reader, const Guid& writer, int64_t base,
                   std::initializer_list<int64_t> sns, int32_t count) {
    ParsedAckNack ack{};
    ack.readerId = reader.entityId;
    ack.writerId = writer.entityId;
    ack.readerSNState.clear();
    ack.readerSNState.bitmapBase = SequenceNumber::from_int64(base);
    for (int64_t sn : sns) ack.readerSNState.set(static_cast<uint32_t>(sn - base));
    ack.count = count;
    return ack;
}

}  // namespace

// §8.4.9.2.12, §8.7.2.2.1: a VOLATILE writer answers a request for changes
// written before the reader matched with one GAP naming exactly those, and
// leaves changes written after the match to the repair path.
static bool tc_swr_8_4_9_001(std::string& err) {
    CaptureTransport tx{};
    const Guid writer = make_guid(0x11u, make_entity(1u, 2u, 3u, 0x02u));
    const Guid reader = make_guid(0x22u, make_entity(4u, 5u, 6u, 0x07u));
    const Locator loc = make_udpv4_locator(0xC0A80177u, 7411u);
    StatefulWriterStorage<4, 1> w(writer, ReliabilityKind::RELIABLE, DurabilityKind::VOLATILE, tx, 5u, 1000u);

    const Octet payload[1] = {0x42u};
    for (int i = 0; i < 3; ++i) CHECK(w.write_async(ChangeKind::ALIVE, payload, 1u), "write before match");
    CHECK(w.matched_reader_add(reader, &loc, 1u, false), "match");
    CHECK(w.write_async(ChangeKind::ALIVE, payload, 1u), "write SN 4 after match");
    tx.unicast_msgs.clear();

    ParseContext ctx{};
    ctx.senderGuidPrefix = reader.prefix;
    g_stateful_writer_test_now_ms = 100u;
    w.on_acknack(ctx, nack(reader, writer, 1, {1, 2, 3, 4, 9}, 1));

    GapCapture cap = collect_gaps(tx);
    CHECK(cap.gaps == 1, "one GAP answers the pre-match changes at once");
    CHECK(cap.gap.readerId == reader.entityId, "the GAP is addressed to the requesting reader");
    CHECK(cap.gap.gapStart == SequenceNumber::from_int64(1) &&
          gap_run_end(cap.gap.gapStart, cap.gap.gapList) == SequenceNumber::from_int64(4) &&
          cap.gap.gapList.empty(),
          "GAP names exactly 1-3 as a run: not 4 (written after the match), not 9 (never written)");

    auto* proxy = w.matched_reader_lookup(reader);
    CHECK(proxy != nullptr && proxy->has_requested_changes(), "SN 4 is queued for repair, not GAPped");
    return true;
}

// §8.4.9.2.12: a change the KEEP_LAST history evicted before its repair is
// answered with a GAP that names it, not with an empty GAP.
static bool tc_swr_8_4_9_002(std::string& err) {
    CaptureTransport tx{};
    const Guid writer = make_guid(0x33u, make_entity(1u, 2u, 3u, 0x02u));
    const Guid reader = make_guid(0x44u, make_entity(4u, 5u, 6u, 0x07u));
    const Locator loc = make_udpv4_locator(0xC0A80178u, 7411u);
    StatefulWriterStorage<1, 1> w(writer, ReliabilityKind::RELIABLE, DurabilityKind::VOLATILE, tx, 5u, 1000u);
    CHECK(w.matched_reader_add(reader, &loc, 1u, false), "match");

    const Octet payload[1] = {0x42u};
    CHECK(w.write_async(ChangeKind::ALIVE, payload, 1u), "write SN 1");
    CHECK(w.write_async(ChangeKind::ALIVE, payload, 1u), "write SN 2 evicts SN 1");
    tx.unicast_msgs.clear();

    ParseContext ctx{};
    ctx.senderGuidPrefix = reader.prefix;
    g_stateful_writer_test_now_ms = 200u;
    w.on_acknack(ctx, nack(reader, writer, 1, {1}, 1));
    w.tick(300u);

    GapCapture cap = collect_gaps(tx);
    CHECK(cap.gaps == 1, "the evicted change is answered with one GAP");
    CHECK(cap.gap.gapStart == SequenceNumber::from_int64(1) &&
          gap_run_end(cap.gap.gapStart, cap.gap.gapList) == SequenceNumber::from_int64(2),
          "the GAP's run is exactly SN 1");
    return true;
}

// §8.3.7.4.5: encode_gap() emits a contiguous run followed by a bitmap
// relative to the first number after the run.
static bool tc_swr_8_4_9_003(std::string& err) {
    SequenceNumberSet in{};
    in.clear();
    in.bitmapBase = SequenceNumber::from_int64(10);
    for (uint32_t off : {1u, 2u, 3u, 6u, 9u}) in.set(off);  // 11, 12, 13, 16, 19

    SequenceNumber start{};
    SequenceNumberSet list{};
    CHECK(encode_gap(in, start, list), "a non-empty set encodes");
    CHECK(start == SequenceNumber::from_int64(11), "run starts at the lowest SN");
    CHECK(list.bitmapBase == SequenceNumber::from_int64(14), "run is 11-13");
    CHECK(list.test(2) && list.test(5) && list.numBits == 6u && !list.test(0),
          "bitmap names 16 and 19 relative to 14");

    SequenceNumberSet none{};
    none.clear();
    CHECK(!encode_gap(none, start, list), "an empty set encodes no GAP");
    return true;
}

void append_stateful_writer_tests_8_4_9_irrelevant_gap(std::vector<TestCase>& tests) {
    tests.push_back({"TC-SWR-8.4.9-001",
                     "§8.4.9.2.12/§8.7.2.2.1: a VOLATILE writer GAPs requested pre-match changes at once and repairs post-match ones.",
                     tc_swr_8_4_9_001});
    tests.push_back({"TC-SWR-8.4.9-002",
                     "§8.4.9.2.12: a requested change evicted from a KEEP_LAST history is named in a GAP.",
                     tc_swr_8_4_9_002});
    tests.push_back({"TC-SWR-8.4.9-003",
                     "§8.3.7.4.5: encode_gap emits a contiguous run plus a bitmap relative to the end of the run.",
                     tc_swr_8_4_9_003});
}

}  // namespace spec_test
