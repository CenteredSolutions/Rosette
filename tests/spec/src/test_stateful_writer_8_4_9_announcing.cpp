/**
 * @file test_stateful_writer_8_4_9_announcing.cpp
 * @brief Reliable StatefulWriter: periodic HEARTBEATs only while a reader is announcing.
 *
 * §8.4.9.2 (transitions T5-T7): a ReaderProxy enters "announcing" when it
 * has unacknowledged changes and returns to "idle" once they are all
 * acknowledged; the periodic HEARTBEAT is sent only in "announcing". A writer
 * that heartbeats acknowledged readers draws an ACKNACK from each of them
 * every period, forever.
 */

#include "spec_test_common.hpp"

#include "rosette/rtps/stateful_writer.hpp"

namespace spec_test {

extern uint32_t g_stateful_writer_test_now_ms;

#define CHECK(COND, MSG) do { if (!(COND)) { err = (MSG); return false; } } while (0)

namespace {

struct CountingTransport : ITransport {
    int heartbeats = 0;

    bool send_unicast(const BufRef& buf, const Locator&) noexcept override {
        ParseOps ops{};
        ops.on_heartbeat = [](const ParseContext& c, const ParsedHeartbeat&) noexcept {
            ++*static_cast<int*>(c.user_data);
        };
        (void)parse_message(buf.data, buf.size, GuidPrefix{}, ops, &heartbeats);
        return true;
    }
    bool send_multicast(const BufRef&, const Locator&) noexcept override { return true; }
};

}  // namespace

// §8.4.9.2 T5-T7: no periodic HEARTBEAT to an idle reader; one per period
// while a change is unacknowledged; silence again after the ACK.
static bool tc_swr_8_4_9_004(std::string& err) {
    CountingTransport tx{};
    Guid writer{};
    writer.prefix.value[0] = 0x55u;
    writer.entityId = make_entity(1u, 2u, 3u, 0x02u);
    Guid reader{};
    reader.prefix.value[0] = 0x66u;
    reader.entityId = make_entity(4u, 5u, 6u, 0x07u);
    const Locator loc = make_udpv4_locator(0xC0A80179u, 7411u);
    StatefulWriterStorage<2, 1> w(writer, ReliabilityKind::RELIABLE, DurabilityKind::VOLATILE, tx, 5u, 1000u);
    CHECK(w.matched_reader_add(reader, &loc, 1u, false), "match");

    w.tick(1000u);
    w.tick(2000u);
    CHECK(tx.heartbeats == 0, "an idle reader receives no periodic HEARTBEAT");

    const Octet payload[1] = {0x42u};
    CHECK(w.write_async(ChangeKind::ALIVE, payload, 1u), "write SN 1");
    const int after_write = tx.heartbeats;
    w.tick(3000u);
    CHECK(tx.heartbeats == after_write + 1, "an unacknowledged change is announced each period");

    ParsedAckNack ack{};
    ack.readerId = reader.entityId;
    ack.writerId = writer.entityId;
    ack.readerSNState.clear();
    ack.readerSNState.bitmapBase = SequenceNumber::from_int64(2);
    ack.count = 1;
    ParseContext ctx{};
    ctx.senderGuidPrefix = reader.prefix;
    g_stateful_writer_test_now_ms = 3100u;
    w.on_acknack(ctx, ack);
    const int after_ack = tx.heartbeats;
    w.tick(4000u);
    w.tick(5000u);
    CHECK(tx.heartbeats == after_ack, "once acknowledged the reader is idle again");
    CHECK(w.ms_until_next_timer(5000u) == UINT32_MAX, "and the writer has no timer to wake for");
    return true;
}

void append_stateful_writer_tests_8_4_9_announcing(std::vector<TestCase>& tests) {
    tests.push_back({"TC-SWR-8.4.9-004",
                     "§8.4.9.2 T5-T7: periodic HEARTBEATs go only to readers with unacknowledged changes.",
                     tc_swr_8_4_9_004});
}

}  // namespace spec_test
