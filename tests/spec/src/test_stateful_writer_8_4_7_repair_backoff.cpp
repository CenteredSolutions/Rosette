// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file test_stateful_writer_8_4_7_repair_backoff.cpp
 * @brief Reliable StatefulWriter: how long each repair waits after its NACK.
 *
 * A reader's first repair waits RTPS_REPAIR_FIRST_DELAY_MS; each repair sent
 * to it before its cumulative acknowledgment advances doubles the next wait,
 * up to the writer's nackResponseDelay (§8.4.1.1), which stays the ceiling.
 * NACK and NACK_FRAG requests share the one backoff. See "Repair backoff" in
 * stateful_writer.hpp.
 */

#include <algorithm>

#include "spec_test_common.hpp"

#include "rosette/rtps/stateful_writer.hpp"

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

/// Counts the DATA and DATA_FRAG submessages the writer sends.
struct RepairTransport : ITransport {
    int data = 0;
    uint32_t limit = 65507u;

    bool send_unicast(const BufRef& buf, const Locator&) noexcept override {
        ParseOps ops{};
        ops.on_data = [](const ParseContext& c, const ParsedData&) noexcept {
            ++static_cast<RepairTransport*>(c.user_data)->data;
        };
        ops.on_data_frag = [](const ParseContext& c, const ParsedDataFrag&) noexcept {
            ++static_cast<RepairTransport*>(c.user_data)->data;
        };
        (void)parse_message(buf.data, buf.size, GuidPrefix{}, ops, this);
        return true;
    }

    bool send_multicast(const BufRef&, const Locator&) noexcept override { return true; }

    uint32_t max_datagram_size() const noexcept override { return limit; }
};

Guid make_guid(Octet p0, const EntityId& eid) {
    Guid g{};
    g.prefix.value[0] = p0;
    g.prefix.value[1] = 0x7Bu;
    g.entityId = eid;
    return g;
}

const Guid kWriter = make_guid(0x51u, make_entity(1u, 2u, 3u, 0x02u));
const Guid kReader = make_guid(0x61u, make_entity(4u, 5u, 6u, 0x07u));
const Locator kLoc = make_udpv4_locator(0xC0A801B1u, 7411u);

/// Long enough that no periodic HEARTBEAT falls inside a test.
constexpr uint32_t kPeriod = 60000u;

using Writer = StatefulWriterStorage<4, 1>;

/// Delivers an ACKNACK acknowledging every SN below `base` and requesting `nack_sn` (0: none).
void acknack(StatefulWriter& w, int64_t base, int64_t nack_sn, int32_t count, uint32_t now_ms) {
    ParsedAckNack msg{};
    msg.readerId = kReader.entityId;
    msg.writerId = kWriter.entityId;
    msg.readerSNState.clear();
    msg.readerSNState.bitmapBase = SequenceNumber::from_int64(base);
    if (nack_sn != 0) {
        msg.readerSNState.numBits = static_cast<uint32_t>(nack_sn - base + 1);
        msg.readerSNState.set(static_cast<uint32_t>(nack_sn - base));
    }

    msg.count = count;
    ParseContext ctx{};
    ctx.senderGuidPrefix = kReader.prefix;
    g_stateful_writer_test_now_ms = now_ms;
    w.on_acknack(ctx, msg);
}

/// Delivers a NACK_FRAG for fragment 2 of `sn`.
void nack_frag(StatefulWriter& w, int64_t sn, int32_t count, uint32_t now_ms) {
    ParsedNackFrag msg{};
    msg.readerId = kReader.entityId;
    msg.writerId = kWriter.entityId;
    msg.writerSN = SequenceNumber::from_int64(sn);
    msg.fragmentNumberState.clear();
    (void)msg.fragmentNumberState.add(2u);
    msg.count = count;
    ParseContext ctx{};
    ctx.senderGuidPrefix = kReader.prefix;
    g_stateful_writer_test_now_ms = now_ms;
    w.on_nack_frag(ctx, msg);
}

/// Ticks from `from_ms` a millisecond at a time until a repair goes out; returns its wait.
uint32_t repair_wait(StatefulWriter& w, RepairTransport& tx, uint32_t from_ms) {
    const int before = tx.data;
    for (uint32_t t = from_ms; t <= from_ms + 1000u; ++t) {
        g_stateful_writer_test_now_ms = t;
        (void)w.tick(t);
        if (tx.data != before) {
            return t - from_ms;
        }
    }

    return UINT32_MAX;
}

bool write_at(StatefulWriter& w, const Octet* payload, uint32_t size, uint32_t now_ms) {
    g_stateful_writer_test_now_ms = now_ms;
    return w.write_async(ChangeKind::ALIVE, payload, size);
}

}  // namespace

// §8.4.1.1: a reader that NACKs the same change after every repair is
// answered after 5, 10, 20, 40, 80 and 160 ms, then at nackResponseDelay.
static bool tc_swr_8_4_7_016(std::string& err) {
    RepairTransport tx{};
    Writer w(kWriter, ReliabilityKind::RELIABLE, DurabilityKind::VOLATILE, tx, 200u, kPeriod);
    CHECK(w.matched_reader_add(kReader, &kLoc, 1u, false), "match");
    const Octet payload[2] = {0x12u, 0x34u};
    CHECK(write_at(w, payload, 2u, 1000u), "write SN 1");

    uint32_t now = 2000u;
    uint32_t want = RTPS_REPAIR_FIRST_DELAY_MS;
    for (int32_t count = 1; count <= 8; ++count) {
        acknack(w, 1, 1, count, now);
        const uint32_t waited = repair_wait(w, tx, now);
        CHECK(waited == std::min(want, 200u), "each repeated NACK doubles the wait, up to 200 ms");
        now += waited + 1u;
        want *= 2u;
    }

    return true;
}

// §8.4.1.1: once the reader acknowledges the repaired change, its next NACK
// is answered after the first wait again.
static bool tc_swr_8_4_7_017(std::string& err) {
    RepairTransport tx{};
    Writer w(kWriter, ReliabilityKind::RELIABLE, DurabilityKind::VOLATILE, tx, 200u, kPeriod);
    CHECK(w.matched_reader_add(kReader, &kLoc, 1u, false), "match");
    const Octet payload[2] = {0x12u, 0x34u};
    CHECK(write_at(w, payload, 2u, 1000u), "write SN 1");
    CHECK(write_at(w, payload, 2u, 1001u), "write SN 2");

    acknack(w, 1, 1, 1, 2000u);
    CHECK(repair_wait(w, tx, 2000u) == RTPS_REPAIR_FIRST_DELAY_MS, "first repair");
    acknack(w, 1, 1, 2, 2100u);
    CHECK(repair_wait(w, tx, 2100u) == RTPS_REPAIR_FIRST_DELAY_MS * 2u,
          "the same change NACKed again waits twice as long");

    acknack(w, 2, 2, 3, 2200u);
    CHECK(repair_wait(w, tx, 2200u) == RTPS_REPAIR_FIRST_DELAY_MS,
          "an acknowledgment past SN 1 resets the wait for the NACK of SN 2");
    return true;
}

// §8.4.1.1: a NACK_FRAG backs off like a NACK, and nackResponseDelay below
// the first wait is used as configured, a zero delay repairing at once.
static bool tc_swr_8_4_7_018(std::string& err) {
    RepairTransport tx{};
    tx.limit = 200u;
    Writer w(kWriter, ReliabilityKind::RELIABLE, DurabilityKind::VOLATILE, tx, 200u, kPeriod);
    CHECK(w.matched_reader_add(kReader, &kLoc, 1u, false), "match");
    std::vector<Octet> big(400u, 0x5Au);
    CHECK(write_at(w, big.data(), 400u, 1000u), "write a fragmented SN 1");

    nack_frag(w, 1, 1, 2000u);
    CHECK(repair_wait(w, tx, 2000u) == RTPS_REPAIR_FIRST_DELAY_MS, "first fragment repair");
    nack_frag(w, 1, 2, 2100u);
    CHECK(repair_wait(w, tx, 2100u) == RTPS_REPAIR_FIRST_DELAY_MS * 2u,
          "a repeated NACK_FRAG waits twice as long");

    RepairTransport tx0{};
    Writer w0(kWriter, ReliabilityKind::RELIABLE, DurabilityKind::VOLATILE, tx0, 0u, kPeriod);
    CHECK(w0.matched_reader_add(kReader, &kLoc, 1u, false), "match the zero-delay writer");
    const Octet payload[2] = {0x12u, 0x34u};
    CHECK(write_at(w0, payload, 2u, 1000u), "write SN 1");
    for (int32_t count = 1; count <= 3; ++count) {
        const uint32_t now = 2000u + 10u * static_cast<uint32_t>(count);
        acknack(w0, 1, 1, count, now);
        CHECK(repair_wait(w0, tx0, now) == 0u, "a zero nackResponseDelay repairs at once");
    }

    return true;
}

void append_stateful_writer_tests_8_4_7_repair_backoff(std::vector<TestCase>& tests) {
    tests.push_back({"TC-SWR-8.4.7-016",
                     "§8.4.1.1: a change NACKed after every repair waits 5, 10, 20, 40, 80, 160 "
                     "ms, then nackResponseDelay.",
                     tc_swr_8_4_7_016});
    tests.push_back({"TC-SWR-8.4.7-017",
                     "§8.4.1.1: an advancing acknowledgment resets the repair wait.",
                     tc_swr_8_4_7_017});
    tests.push_back({"TC-SWR-8.4.7-018",
                     "§8.4.1.1: NACK_FRAG shares the backoff; a nackResponseDelay below the first "
                     "wait, including 0, is used as configured.",
                     tc_swr_8_4_7_018});
}

}  // namespace spec_test
