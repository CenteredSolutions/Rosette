// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file test_stateful_writer_8_4_1_4_replay.cpp
 * @brief Spec tests: a TRANSIENT_LOCAL writer replays its history to a new
 *        reader from tick(), a budget at a time, packing rebuilt payloads.
 *
 * matched_reader_add() only marks the reader replaying; tick() sends at most
 * RTPS_REPLAY_DATAGRAMS_PER_TICK datagrams of the replay per call
 * (StatefulWriter::drain_replays()). A writer that builds its payloads on
 * demand, as the SEDP announcers do, packs each datagram with as many
 * changes as fit in the block the first payload was built in.
 *
 * References: OMG DDSI-RTPS 2.5 §8.7.2.2 (TRANSIENT_LOCAL), §8.4.1.4
 * (matched_reader_add), §8.3.3 and §9.4.1 (several submessages per message),
 * §9.4.5.3 (DATA), §8.4.15.4 (piggybacked HEARTBEAT).
 */

#include "spec_test_common.hpp"

#include "rosette/rtps/stateful_writer.hpp"

#include <string.h>

namespace spec_test {

extern uint32_t g_stateful_writer_test_now_ms;

#define CHECK(COND, MSG)  \
    do {                  \
        if (!(COND)) {    \
            err = (MSG);  \
            return false; \
        }                 \
    } while (0)

namespace {

/// One datagram the writer sent: its size and the submessages in it.
struct Datagram {
    uint32_t             size = 0u;
    Locator              dest{};
    std::vector<int64_t> data_sns{};
    int                  heartbeats = 0;
};

/// Records every datagram, parsed.
struct RecordingTransport : ITransport {
    std::vector<Datagram> sent{};
    uint32_t              limit = 65507u;  ///< max_datagram_size() reported to the writer.

    uint32_t max_datagram_size() const noexcept override { return limit; }

    bool send_unicast(const BufRef& buf, const Locator& dest) noexcept override {
        Datagram d{};
        d.size = buf.size;
        d.dest = dest;
        ParseOps ops{};
        ops.on_data = [](const ParseContext& c, const ParsedData& data) noexcept {
            static_cast<Datagram*>(c.user_data)->data_sns.push_back(data.writerSN.to_int64());
        };
        ops.on_heartbeat = [](const ParseContext& c, const ParsedHeartbeat&) noexcept {
            ++static_cast<Datagram*>(c.user_data)->heartbeats;
        };
        if (parse_message(buf.data, buf.size, GuidPrefix{}, ops, &d) == ParseResult::OK) {
            sent.push_back(d);
        }

        return true;
    }

    bool send_multicast(const BufRef&, const Locator&) noexcept override { return true; }

    /// Every DATA sequence number sent, in order.
    std::vector<int64_t> all_data_sns() const {
        std::vector<int64_t> out{};
        for (const Datagram& d : sent) {
            out.insert(out.end(), d.data_sns.begin(), d.data_sns.end());
        }

        return out;
    }

    int all_heartbeats() const {
        int n = 0;
        for (const Datagram& d : sent) {
            n += d.heartbeats;
        }

        return n;
    }
};

Guid make_guid(Octet p0, const EntityId& eid) {
    Guid g{};
    g.prefix.value[0] = p0;
    g.entityId = eid;
    return g;
}

/// Delivers an ACKNACK from @p reader acknowledging everything below @p base
/// and NACKing @p nacks.
void acknack(StatefulWriter& w, const Guid& reader, const Guid& writer, int64_t base, int32_t count,
             std::initializer_list<int64_t> nacks) {
    ParsedAckNack ack{};
    ack.readerId = reader.entityId;
    ack.writerId = writer.entityId;
    ack.readerSNState.clear();
    ack.readerSNState.bitmapBase = SequenceNumber::from_int64(base);
    for (const int64_t sn : nacks) {
        (void)ack.readerSNState.add(SequenceNumber::from_int64(sn));
    }

    ack.count = count;
    ParseContext ctx{};
    ctx.senderGuidPrefix = reader.prefix;
    w.on_acknack(ctx, ack);
}

int count_of(const std::vector<int64_t>& sns, int64_t sn) {
    int n = 0;
    for (const int64_t s : sns) {
        n += s == sn ? 1 : 0;
    }

    return n;
}

/// A payload builder that rebuilds a preframed payload of kBuiltPayload
/// octets for each change, as SEDP rebuilds its announcements (sedp.hpp),
/// in a block with the same headroom and tailroom.
constexpr uint32_t kBuiltPayload = 100u;

BufRef build_payload(void* /*user*/, const HistoryChange& change) noexcept {
    BufRef payload = buf_alloc_with_headroom(kBuiltPayload + kRtpsHeartbeatTailroom,
                                             kRtpsDataHeadroom + kMaxInlineQosSize);
    if (!payload.valid()) {
        return payload;
    }

    memset(payload.data, static_cast<int>(change.sequenceNumber.low & 0xFFu), kBuiltPayload);
    payload.size = kBuiltPayload;
    return payload;
}

uint32_t size_payload(void* /*user*/, const HistoryChange& /*change*/) noexcept {
    return kBuiltPayload;
}

}  // namespace

// TC-SWR-8.4.1.4-010: matching sends nothing; each tick() sends at most the
// budget; a change written during the replay waits its turn in it; a NACK
// for changes not yet replayed draws no second copy of them.
static bool tc_swr_8_4_1_4_010(std::string& err) {
    RecordingTransport          tx{};
    const Guid                  writer = make_guid(0x10u, make_entity(1u, 2u, 3u, 0x02u));
    const Guid                  reader = make_guid(0x20u, make_entity(4u, 5u, 6u, 0x07u));
    const Locator               loc = make_udpv4_locator(0xC0A80190u, 7411u);
    StatefulWriterStorage<8, 1> w(writer, ReliabilityKind::RELIABLE,
                                  DurabilityKind::TRANSIENT_LOCAL, tx, 5u, 1000u);
    g_stateful_writer_test_now_ms = 0u;  // the NACK below is timed from here
    const Octet    payload[4] = {1u, 2u, 3u, 4u};
    const uint32_t history = RTPS_REPLAY_DATAGRAMS_PER_TICK + 2u;
    for (uint32_t i = 0; i < history; ++i) {
        CHECK(w.write_async(ChangeKind::ALIVE, payload, 4u), "write before the reader matches");
    }

    CHECK(w.matched_reader_add(reader, &loc, 1u, false), "match");
    CHECK(tx.sent.empty() && w.replay_pending() && w.ms_until_next_timer(0u) == 0u,
          "matching sends nothing and asks for an immediate tick()");

    w.tick(0u);
    CHECK(tx.sent.size() == RTPS_REPLAY_DATAGRAMS_PER_TICK,
          "one tick() sends exactly the budget of datagrams");
    CHECK(w.replay_pending(), "and the replay goes on");

    CHECK(w.write_async(ChangeKind::ALIVE, payload, 4u), "write during the replay");
    CHECK(tx.sent.size() == RTPS_REPLAY_DATAGRAMS_PER_TICK,
          "the new change is not pushed past the replay to a replaying reader");

    // The reader NACKs everything up to the newest change; only what it was
    // already sent is repaired, the rest is the replay's to send.
    acknack(w, reader, writer, 1, 1, {1, 2, 3, 4, 5, 6, 7});
    finish_replay(w, 0u);
    CHECK(!w.replay_pending(), "the replay ends once nothing is UNSENT");
    std::vector<int64_t> sns = tx.all_data_sns();
    for (int64_t sn = 1; sn <= static_cast<int64_t>(history) + 1; ++sn) {
        CHECK(count_of(sns, sn) == 1, "the replay sends each change once, in order");
    }

    CHECK(tx.sent.back().heartbeats == 1, "the newest change carries the replay's HEARTBEAT");

    w.tick(10u);  // nackResponseDelay (5 ms) has passed
    sns = tx.all_data_sns();
    for (int64_t sn = 1; sn <= static_cast<int64_t>(history) + 1; ++sn) {
        const int expected = sn <= static_cast<int64_t>(RTPS_REPLAY_DATAGRAMS_PER_TICK) ? 2 : 1;
        CHECK(count_of(sns, sn) == expected,
              "only changes already sent when NACKed are repaired; the replay covered the rest");
    }

    return true;
}

// TC-SWR-8.4.1.4-011: a writer that builds payloads on demand packs the
// replay: each datagram holds its first change and as many following ones
// as fit in that change's block, in order, never more than the block holds;
// the last datagram carries the HEARTBEAT.
static bool tc_swr_8_4_1_4_011(std::string& err) {
    RecordingTransport          tx{};
    const Guid                  writer = make_guid(0x30u, make_entity(0u, 0u, 3u, 0xC2u));
    const Guid                  reader = make_guid(0x40u, make_entity(0u, 0u, 3u, 0xC7u));
    const Locator               loc = make_udpv4_locator(0xC0A80191u, 7410u);
    StatefulWriterStorage<8, 1> w(writer, ReliabilityKind::RELIABLE,
                                  DurabilityKind::TRANSIENT_LOCAL, tx, 5u, 1000u);
    w.set_payload_builder(&build_payload, nullptr, &size_payload);

    const uint32_t changes = 5u;
    for (uint32_t i = 0; i < changes; ++i) {
        InlineQos qos{};
        qos.set_key_hash(make_guid(0x30u, make_entity(0u, 0u, static_cast<Octet>(i), 0x03u)));
        CHECK(w.write_on_demand_async(ChangeKind::ALIVE, qos), "announce an endpoint");
    }

    CHECK(w.matched_reader_add(reader, &loc, 1u, false), "match a peer's detector");
    finish_replay(w, 0u);

    const std::vector<int64_t> sns = tx.all_data_sns();
    CHECK(sns == (std::vector<int64_t>{1, 2, 3, 4, 5}), "every change is replayed once, in order");
    CHECK(tx.sent.size() < changes, "datagrams carry several changes each");

    // The first payload's block is the smallest pool class that holds the
    // payload with its headroom and tailroom; no datagram outgrows it.
    const BufRef probe = buf_alloc_with_headroom(kBuiltPayload + kRtpsHeartbeatTailroom,
                                                 kRtpsDataHeadroom + kMaxInlineQosSize);
    CHECK(probe.valid(), "a block of the replay's class is free");
    const uint32_t block = probe.capacity() + kRtpsDataHeadroom + kMaxInlineQosSize;
    for (const Datagram& d : tx.sent) {
        CHECK(d.size <= block, "a packed datagram never exceeds its first payload's block");
        CHECK(!d.data_sns.empty() || d.heartbeats == 1,
              "every datagram carries a change or the HEARTBEAT");
    }

    CHECK(tx.sent.size() >= 2u && tx.sent[0].data_sns.size() >= 2u,
          "the first datagram packs the change that fits after its own");
    CHECK(tx.all_heartbeats() == 1, "the replay carries exactly one HEARTBEAT");
    const Datagram& last = tx.sent.back();
    CHECK(last.heartbeats == 1
              && (last.data_sns.empty() || last.data_sns.back() == static_cast<int64_t>(changes)),
          "the HEARTBEAT follows the newest change");
    return true;
}

// TC-SWR-8.4.1.4-012: the replay budget is charged in datagrams sent, not in
// changes: a fragmented sample costs one datagram per fragment, so a tick()
// stops starting changes once the budget is spent and overshoots by at most
// the rest of the change in progress.
static bool tc_swr_8_4_1_4_012(std::string& err) {
    RecordingTransport tx{};
    tx.limit = 200u;
    const Guid                  writer = make_guid(0x50u, make_entity(1u, 2u, 3u, 0x02u));
    const Guid                  reader = make_guid(0x60u, make_entity(4u, 5u, 6u, 0x07u));
    const Locator               loc = make_udpv4_locator(0xC0A80192u, 7411u);
    StatefulWriterStorage<8, 1> w(writer, ReliabilityKind::RELIABLE,
                                  DurabilityKind::TRANSIENT_LOCAL, tx, 5u, 1000u);
    const std::vector<Octet>    payload(400u, 0x5Au);
    const uint32_t              samples = 3u;
    for (uint32_t i = 0; i < samples; ++i) {
        CHECK(w.write_async(ChangeKind::ALIVE, payload.data(), 400u), "write a fragmented sample");
    }

    CHECK(tx.sent.empty(), "no reader is matched, so nothing is sent");
    CHECK(w.matched_reader_add(reader, &loc, 1u, false), "match");

    w.tick(0u);
    const uint32_t fragments_per_sample = 3u;
    CHECK(tx.sent.size() >= RTPS_REPLAY_DATAGRAMS_PER_TICK
              && tx.sent.size() < RTPS_REPLAY_DATAGRAMS_PER_TICK + fragments_per_sample,
          "a tick() sends the budget in datagrams, plus the rest of the sample in progress");
    CHECK(tx.sent.size() < samples * fragments_per_sample,
          "the budget stops the replay before every fragment has gone out");
    CHECK(w.replay_pending(), "the rest of the replay waits for the next tick()");

    const size_t first_tick = tx.sent.size();
    finish_replay(w, 0u);
    CHECK(tx.sent.size() >= samples * fragments_per_sample && tx.sent.size() > first_tick,
          "later ticks send every fragment of every sample");
    return true;
}

// TC-SWR-8.4.1.4-013: replaying readers share the budget: a pass starts at
// the reader after the last one served, so a reader that keeps receiving new
// changes cannot keep the budget from the readers matched after it.
static bool tc_swr_8_4_1_4_013(std::string& err) {
    RecordingTransport           tx{};
    const Guid                   writer = make_guid(0x70u, make_entity(1u, 2u, 3u, 0x02u));
    const Guid                   first = make_guid(0x80u, make_entity(4u, 5u, 6u, 0x07u));
    const Guid                   second = make_guid(0x90u, make_entity(4u, 5u, 6u, 0x07u));
    const Locator                first_loc = make_udpv4_locator(0xC0A80193u, 7411u);
    const Locator                second_loc = make_udpv4_locator(0xC0A80194u, 7411u);
    StatefulWriterStorage<32, 2> w(writer, ReliabilityKind::RELIABLE,
                                   DurabilityKind::TRANSIENT_LOCAL, tx, 5u, 1000u);
    const Octet                  payload[4] = {1u, 2u, 3u, 4u};
    const uint32_t               budget = RTPS_REPLAY_DATAGRAMS_PER_TICK;
    for (uint32_t i = 0; i < budget + 2u; ++i) {
        CHECK(w.write_async(ChangeKind::ALIVE, payload, 4u), "write before the readers match");
    }

    CHECK(w.matched_reader_add(first, &first_loc, 1u, false), "match the first reader");
    CHECK(w.matched_reader_add(second, &second_loc, 1u, false), "match the second reader");

    // As many new changes arrive between ticks as a tick() can send, so the
    // first reader always has changes UNSENT.
    for (uint32_t tick = 0u; tick < 2u; ++tick) {
        for (uint32_t i = 0; i < budget; ++i) {
            CHECK(w.write_async(ChangeKind::ALIVE, payload, 4u), "write during the replay");
        }

        w.tick(0u);
    }

    uint32_t to_second = 0u;
    for (const Datagram& d : tx.sent) {
        to_second += d.dest == second_loc ? 1u : 0u;
    }

    CHECK(to_second > 0u, "the reader matched second is served while the first has more to send");
    CHECK(w.replay_pending(), "both readers still have changes to replay");
    return true;
}

// TC-SWR-8.4.1.4-014: a NACK for a change that is UNSENT to a reader that is
// not replaying is repaired; only a replaying reader's NACK is left to the
// replay.
static bool tc_swr_8_4_1_4_014(std::string& err) {
    RecordingTransport          tx{};
    const Guid                  writer = make_guid(0xA0u, make_entity(1u, 2u, 3u, 0x02u));
    const Guid                  reader = make_guid(0xB0u, make_entity(4u, 5u, 6u, 0x07u));
    const Locator               loc = make_udpv4_locator(0xC0A80195u, 7411u);
    StatefulWriterStorage<8, 1> w(writer, ReliabilityKind::RELIABLE, DurabilityKind::VOLATILE, tx,
                                  5u, 1000u);
    g_stateful_writer_test_now_ms = 0u;
    CHECK(w.matched_reader_add(reader, &loc, 1u, false), "match");
    CHECK(!w.replay_pending(), "a VOLATILE writer has nothing to replay");

    // add_change() alone sends nothing: the change stays UNSENT with no send due.
    BufRef       buf{};
    HistoryChange change = w.new_change(ChangeKind::ALIVE, buf, 4u);
    CHECK(change.payload.valid(), "stage a change");
    memset(buf.data, 0x11, 4u);
    buf.size = 4u;
    change.payload.size = 4u;
    CHECK(w.add_change(static_cast<HistoryChange&&>(change)), "add the change without sending it");
    CHECK(tx.sent.empty(), "add_change() sends nothing");

    acknack(w, reader, writer, 1, 1, {1});
    w.tick(10u);  // nackResponseDelay (5 ms) has passed
    CHECK(count_of(tx.all_data_sns(), 1) == 1, "the NACKed change is repaired");
    return true;
}

/// A payload builder for an exhausted pool: it counts the attempt and builds nothing.
static BufRef starved_payload(void* attempts, const HistoryChange& /*change*/) noexcept {
    ++*static_cast<uint32_t*>(attempts);
    return BufRef{};
}

// TC-SWR-8.4.1.4-015: an attempt that sends nothing, because its payload
// could not be built, still spends the replay budget: with the pool
// exhausted a tick() attempts the budget's worth of changes, not the whole
// history, and the changes it could not build are not attempted twice.
static bool tc_swr_8_4_1_4_015(std::string& err) {
    RecordingTransport           tx{};
    const Guid                   writer = make_guid(0xC0u, make_entity(0u, 0u, 3u, 0xC2u));
    const Guid                   reader = make_guid(0xD0u, make_entity(0u, 0u, 3u, 0xC7u));
    const Locator                loc = make_udpv4_locator(0xC0A80196u, 7410u);
    StatefulWriterStorage<32, 1> w(writer, ReliabilityKind::RELIABLE,
                                   DurabilityKind::TRANSIENT_LOCAL, tx, 5u, 1000u);
    uint32_t                     attempts = 0u;
    w.set_payload_builder(&starved_payload, &attempts, &size_payload);

    const uint32_t history = 3u * RTPS_REPLAY_DATAGRAMS_PER_TICK;
    for (uint32_t i = 0; i < history; ++i) {
        InlineQos qos{};
        qos.set_key_hash(make_guid(0xC0u, make_entity(0u, 0u, static_cast<Octet>(i), 0x03u)));
        CHECK(w.write_on_demand_async(ChangeKind::ALIVE, qos), "announce an endpoint");
    }

    CHECK(w.matched_reader_add(reader, &loc, 1u, false), "match a peer's detector");

    w.tick(0u);
    CHECK(attempts == RTPS_REPLAY_DATAGRAMS_PER_TICK,
          "a tick() with no payload to send attempts only the budget's worth of changes");
    CHECK(w.replay_pending(), "the rest of the history waits for the next tick()");

    finish_replay(w, 0u);
    CHECK(attempts == history && !w.replay_pending(),
          "each change is attempted once and the replay then ends");
    return true;
}

void append_stateful_writer_tests_8_4_1_4_replay(std::vector<TestCase>& tests) {
    tests.push_back({"TC-SWR-8.4.1.4-010",
                     "§8.7.2.2/§8.4.1.4: a TRANSIENT_LOCAL replay goes out from tick(), at most "
                     "RTPS_REPLAY_DATAGRAMS_PER_TICK datagrams per call, each change once, a "
                     "write during it in order, and a NACK of unreplayed changes draws no repair.",
                     tc_swr_8_4_1_4_010});
    tests.push_back({"TC-SWR-8.4.1.4-011",
                     "§8.3.3/§9.4.1/§8.4.15.4: a writer that builds payloads on demand packs its "
                     "replay into the first payload's block, several DATA per datagram in order, "
                     "with one HEARTBEAT after the newest.",
                     tc_swr_8_4_1_4_011});
    tests.push_back({"TC-SWR-8.4.1.4-012",
                     "§8.7.2.2: a replay tick() charges its budget in datagrams sent, fragments "
                     "included, and overshoots by at most the rest of the change in progress.",
                     tc_swr_8_4_1_4_012});
    tests.push_back({"TC-SWR-8.4.1.4-013",
                     "§8.7.2.2: replaying readers share the per-tick budget; a reader with a "
                     "steady supply of new changes does not keep it from the readers after it.",
                     tc_swr_8_4_1_4_013});
    tests.push_back({"TC-SWR-8.4.1.4-014",
                     "§8.4.7.1/§8.4.9.2: a NACK for a change UNSENT to a reader that is not "
                     "replaying is repaired; only a replaying reader's NACK is left to the replay.",
                     tc_swr_8_4_1_4_014});
    tests.push_back({"TC-SWR-8.4.1.4-015",
                     "§8.7.2.2: a replay attempt that sends nothing because its payload could not "
                     "be built still spends the per-tick budget, so an exhausted pool cannot make "
                     "one tick() attempt the whole history.",
                     tc_swr_8_4_1_4_015});
}

}  // namespace spec_test
