// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

#include "spec_test_common.hpp"

#include "rosette/rtps/stateful_reader.hpp"
#include "rosette/rtps/transport.hpp"

namespace spec_test {

#define CHECK(COND, MSG) do { if (!(COND)) { err = (MSG); return false; } } while (0)

namespace {

struct NullTransport : ITransport {
    bool send_unicast(const BufRef&, const Locator&) noexcept override { return true; }
    bool send_multicast(const BufRef&, const Locator&) noexcept override { return true; }
};

/// Records every unicast datagram with its destination.
struct CaptureTransport : ITransport {
    std::vector<std::vector<Octet>> sent{};
    std::vector<Locator> dests{};

    bool send_unicast(const BufRef& buf, const Locator& dest) noexcept override {
        sent.emplace_back(buf.data, buf.data + buf.size);
        dests.push_back(dest);
        return true;
    }

    bool send_multicast(const BufRef&, const Locator&) noexcept override { return true; }
};

/// @brief Parses the ACKNACK in @p msg into @p out; false when it has none or does not parse.
bool parse_acknack(const std::vector<Octet>& msg, ParsedAckNack& out) {
    struct Found {
        ParsedAckNack* ack;
        bool seen;
    } found{&out, false};

    ParseOps ops{};
    ops.on_acknack = [](const ParseContext& ctx, const ParsedAckNack& ack) noexcept {
        auto* f = static_cast<Found*>(ctx.user_data);
        *f->ack = ack;
        f->seen = true;
    };
    const ParseResult result =
        parse_message(msg.data(), static_cast<uint32_t>(msg.size()), GuidPrefix{}, ops, &found);
    return result == ParseResult::OK && found.seen;
}

Guid make_guid(Octet p0, Octet p1, Octet p2, const EntityId& eid) {
    Guid g{};
    g.prefix.value[0] = p0;
    g.prefix.value[1] = p1;
    g.prefix.value[2] = p2;
    g.entityId = eid;
    return g;
}

} // namespace

static bool tc_srr_8_4_7_4_001(std::string& err) {
    NullTransport tx{};
    const Guid reader_guid = make_guid(0x11u, 0x12u, 0x13u, make_entity(1u, 2u, 3u, 0xC7u));
    const Guid writer_guid = make_guid(0x21u, 0x22u, 0x23u, make_entity(4u, 5u, 6u, 0xC2u));
    const Locator loc = make_udpv4_locator(0xC0A80114u, 7411u);

    DefaultStatefulReader r(reader_guid, ReliabilityKind::RELIABLE, tx);
    CHECK(r.matched_writer_add(writer_guid, &loc, 1u), "first matched_writer_add should succeed");
    CHECK(r.matched_writer_count() == 1u, "matched writer count should be one");
    CHECK(r.matched_writer_lookup(writer_guid) != nullptr, "matched writer should be discoverable");

    CHECK(!r.matched_writer_add(writer_guid, &loc, 1u),
          "duplicate matched_writer_add should be rejected");

    r.matched_writer_remove(writer_guid);
    CHECK(r.matched_writer_count() == 0u, "matched writer count should drop after remove");
    CHECK(r.matched_writer_lookup(writer_guid) == nullptr,
          "removed writer should no longer be discoverable");
    return true;
}

static bool tc_srr_8_4_7_4_002(std::string& err) {
    CaptureTransport tx{};
    const Guid reader_guid = make_guid(0x11u, 0x12u, 0x13u, make_entity(1u, 2u, 3u, 0xC7u));
    const Guid writer_guid = make_guid(0x21u, 0x22u, 0x23u, make_entity(4u, 5u, 6u, 0xC2u));
    const Locator loc = make_udpv4_locator(0xC0A80114u, 7411u);

    DefaultStatefulReader r(reader_guid, ReliabilityKind::RELIABLE, tx);
    CHECK(r.matched_writer_add(writer_guid, &loc, 1u), "matched_writer_add should succeed");
    CHECK(tx.sent.size() == 1u, "a RELIABLE match sends exactly one ACKNACK");
    CHECK(tx.dests[0] == loc, "the ACKNACK goes to the writer's unicast locator");

    ParsedAckNack ack{};
    CHECK(parse_acknack(tx.sent[0], ack), "the datagram carries an ACKNACK");
    CHECK(ack.readerId == reader_guid.entityId && ack.writerId == writer_guid.entityId,
          "the ACKNACK names this reader and the matched writer");
    CHECK(ack.readerSNState.bitmapBase == SequenceNumber::from_int64(1),
          "nothing is acknowledged: bitmapBase is 1");
    CHECK(ack.readerSNState.empty(), "nothing is requested");
    CHECK(!ack.finalFlag, "the F-flag is clear, so the writer answers with a HEARTBEAT");
    CHECK(ack.count == 1, "the preemptive ACKNACK is the first, count 1");

    CHECK(!r.matched_writer_add(writer_guid, &loc, 1u), "a duplicate match is rejected");
    CHECK(tx.sent.size() == 1u, "a rejected duplicate match sends nothing");
    return true;
}

static bool tc_srr_8_4_7_4_003(std::string& err) {
    CaptureTransport tx{};
    const Guid reader_guid = make_guid(0x11u, 0x12u, 0x13u, make_entity(1u, 2u, 3u, 0xC4u));
    const Guid writer_guid = make_guid(0x21u, 0x22u, 0x23u, make_entity(4u, 5u, 6u, 0xC3u));
    const Locator loc = make_udpv4_locator(0xC0A80114u, 7411u);

    DefaultStatefulReader r(reader_guid, ReliabilityKind::BEST_EFFORT, tx);
    CHECK(r.matched_writer_add(writer_guid, &loc, 1u), "matched_writer_add should succeed");
    CHECK(tx.sent.empty(), "a BEST_EFFORT reader sends no ACKNACK on a match");
    return true;
}

static bool tc_srr_8_4_7_4_004(std::string& err) {
    const Guid reader_guid = make_guid(0x11u, 0x12u, 0x13u, make_entity(1u, 2u, 3u, 0xC7u));
    const Guid writer_guid = make_guid(0x21u, 0x22u, 0x23u, make_entity(4u, 5u, 6u, 0xC2u));
    const Locator loc = make_udpv4_locator(0xC0A80114u, 7411u);
    const VendorId vendors[] = {VENDORID_EPROSIMA, VENDORID_RTI_CONNEXT, VENDORID_THIS_IMPL};
    const int64_t bases[] = {0, 0, 1};

    for (std::size_t i = 0; i < 3u; ++i) {
        CaptureTransport tx{};
        DefaultStatefulReader r(reader_guid, ReliabilityKind::RELIABLE, tx);
        CHECK(r.matched_writer_add(writer_guid, &loc, 1u, vendors[i]),
              "matched_writer_add should succeed");
        ParsedAckNack ack{};
        CHECK(tx.sent.size() == 1u && parse_acknack(tx.sent[0], ack),
              "one preemptive ACKNACK is sent");
        CHECK(ack.readerSNState.bitmapBase == SequenceNumber::from_int64(bases[i]),
              "bitmapBase 0 for eProsima and RTI writers, 1 for other vendors");
        CHECK(ack.readerSNState.empty() && !ack.finalFlag && ack.count == 1,
              "every form requests nothing, has the F-flag clear and count 1");
    }

    return true;
}

// §8.3.7.1: a writer ignores an ACKNACK whose count is not above the last it
// processed from that reader, and keeps that record across the reader's
// unmatch (a participant parking and reactivating a peer does exactly
// this). A rematched writer's preemptive ACKNACK therefore counts on from the
// highest count the reader sent before.
static bool tc_srr_8_4_7_4_005(std::string& err) {
    CaptureTransport tx{};
    const Guid reader_guid = make_guid(0x11u, 0x12u, 0x13u, make_entity(1u, 2u, 3u, 0xC7u));
    const Guid writer_guid = make_guid(0x21u, 0x22u, 0x23u, make_entity(4u, 5u, 6u, 0xC2u));
    const Locator loc = make_udpv4_locator(0xC0A80114u, 7411u);

    DefaultStatefulReader r(reader_guid, ReliabilityKind::RELIABLE, tx);
    r.heartbeat_response_delay(0u);
    CHECK(r.matched_writer_add(writer_guid, &loc, 1u), "matched_writer_add should succeed");
    ParsedHeartbeat hb{};
    hb.writerId = writer_guid.entityId;
    hb.readerId = reader_guid.entityId;
    hb.firstSN = SequenceNumber::from_int64(1);
    hb.lastSN = SequenceNumber::from_int64(1);
    hb.count = 1;
    ParseContext ctx{};
    ctx.senderGuidPrefix = writer_guid.prefix;
    r.on_heartbeat(ctx, hb);
    r.tick(0u);
    ParsedAckNack ack{};
    CHECK(tx.sent.size() == 2u && parse_acknack(tx.sent[1], ack) && ack.count == 2,
          "the HEARTBEAT is answered with the second ACKNACK, count 2");

    r.matched_writer_remove(writer_guid);
    tx.sent.clear();
    CHECK(r.matched_writer_add(writer_guid, &loc, 1u), "the writer is matched again");
    CHECK(tx.sent.size() == 1u && parse_acknack(tx.sent[0], ack),
          "the new match sends a preemptive ACKNACK");
    CHECK(ack.count == 3, "its count continues above the earlier match's, at 3");
    return true;
}

// TC-SRR-8.4.7.4-006: readers sharing a WriterProxy pool bind by its
// admission rule; a refused match sends no ACKNACK and leaves no proxy; an
// unmatch or a destroyed reader returns proxies to the pool.
bool tc_srr_8_4_7_4_006(std::string& err) {
    CaptureTransport tx{};
    ProxyPoolStorage<WriterProxy, 3> pool;
    FragmentAssemblyPoolStorage<2> assembly;
    CHECK(pool.reserve(2u), "two floors of 1 must fit a pool of 3");

    const Guid busy_guid = make_guid(0x01u, 0x02u, 0x03u, make_entity(0u, 0u, 1u, 0x04u));
    const Guid quiet_guid = make_guid(0x01u, 0x02u, 0x03u, make_entity(0u, 0u, 2u, 0x04u));
    const Locator loc = make_udpv4_locator(0xC0A80160u, 7411u);
    Guid writers[4]{};
    for (Octet i = 0u; i < 4u; ++i) {
        writers[i] = make_guid(0x50u, 0x51u, i, make_entity(8u, 8u, i, 0x02u));
    }

    SharedPoolStatefulReader quiet(quiet_guid, ReliabilityKind::RELIABLE, tx, pool, 1u, 3u,
                                   assembly, 1u);
    {
        SharedPoolStatefulReader busy(busy_guid, ReliabilityKind::RELIABLE, tx, pool, 1u, 3u,
                                      assembly, 1u);
        CHECK(busy.matched_writer_add(writers[0], &loc, 1u), "busy binds its floor");
        CHECK(busy.matched_writer_add(writers[1], &loc, 1u), "busy borrows the unpromised proxy");
        tx.sent.clear();
        CHECK(!busy.matched_writer_add(writers[2], &loc, 1u),
              "busy must not take the proxy promised to quiet's floor");
        CHECK(tx.sent.empty(), "a refused match sends no preemptive ACKNACK");
        CHECK(busy.matched_writer_lookup(writers[2]) == nullptr, "a refused match has no proxy");
        CHECK(quiet.matched_writer_add(writers[3], &loc, 1u), "quiet binds its floor");

        busy.matched_writer_remove(writers[0]);
        CHECK(busy.matched_writer_count() == 1u && pool.free_count() == 1u,
              "an unmatch returns the proxy to the pool");
        CHECK(busy.matched_writer_add(writers[2], &loc, 1u), "busy rebinds within its floor");
        CHECK(busy.matched_writer_lookup(writers[3]) == nullptr, "busy never sees quiet's writer");
    }

    CHECK(pool.free_count() == 2u && pool.reserved() == 1u,
          "a destroyed reader returns every proxy, and its floor stays promised to its slot");
    CHECK(quiet.matched_writer_add(writers[0], &loc, 1u), "quiet borrows what busy returned");
    CHECK(quiet.matched_writer_count() == 2u && quiet.max_matched_writers() == 3u,
          "quiet reports its bound writers and its cap");
    return true;
}

extern uint32_t g_stateful_reader_test_now_ms;

// TC-SRR-8.4.7.4-007: with a heartbeatResponseDelay, HEARTBEATs that arrive
// while an ACKNACK is owed fold into it: the first one sets the deadline,
// later ones do not push it back, and one ACKNACK answers them all.
bool tc_srr_8_4_7_4_007(std::string& err) {
    CaptureTransport tx{};
    const Guid       reader_guid = make_guid(0x11u, 0x12u, 0x13u, make_entity(1u, 2u, 3u, 0xC7u));
    const Guid       writer_guid = make_guid(0x21u, 0x22u, 0x23u, make_entity(4u, 5u, 6u, 0xC2u));
    const Locator    loc = make_udpv4_locator(0xC0A80114u, 7411u);

    g_stateful_reader_test_now_ms = 1000u;
    DefaultStatefulReader r(reader_guid, ReliabilityKind::RELIABLE, tx);
    r.heartbeat_response_delay(2u);
    CHECK(r.matched_writer_add(writer_guid, &loc, 1u), "matched_writer_add should succeed");
    tx.sent.clear();

    ParsedHeartbeat hb{};
    hb.writerId = writer_guid.entityId;
    hb.readerId = reader_guid.entityId;
    hb.firstSN = SequenceNumber::from_int64(1);
    hb.lastSN = SequenceNumber::from_int64(0);
    hb.finalFlag = false;
    ParseContext ctx{};
    ctx.senderGuidPrefix = writer_guid.prefix;
    hb.count = 1;
    r.on_heartbeat(ctx, hb);
    r.tick(g_stateful_reader_test_now_ms);
    CHECK(tx.sent.empty(), "the first HEARTBEAT is not answered at once");

    g_stateful_reader_test_now_ms = 1001u;
    hb.count = 2;
    r.on_heartbeat(ctx, hb);
    r.tick(g_stateful_reader_test_now_ms);
    CHECK(tx.sent.empty(), "nor is the second, 1 ms later");

    g_stateful_reader_test_now_ms = 1002u;
    CHECK(r.ms_until_next_timer(g_stateful_reader_test_now_ms) == 0u,
          "the ACKNACK is due 2 ms after the first HEARTBEAT, not after the last");
    r.tick(g_stateful_reader_test_now_ms);
    ParsedAckNack ack{};
    CHECK(tx.sent.size() == 1u && parse_acknack(tx.sent[0], ack),
          "one ACKNACK answers both HEARTBEATs");
    CHECK(ack.finalFlag, "an up-to-date reader's ACKNACK carries the F-flag");

    r.tick(g_stateful_reader_test_now_ms + 10u);
    CHECK(tx.sent.size() == 1u, "nothing more is owed once it is sent");
    return true;
}

// TC-SRR-8.4.7.4-008: a writer whose proxy for this reader holds everything
// acknowledged sends nothing on its own, so a lost preemptive ACKNACK must
// be sent again. It is resent, in the writer's vendor's form, every
// RTPS_PREEMPTIVE_ACKNACK_RETRY_MS until the writer is heard, and never
// after; a final HEARTBEAT that owes no answer still counts as heard.
bool tc_srr_8_4_7_4_008(std::string& err) {
    CaptureTransport   tx{};
    const Guid         reader_guid = make_guid(0x11u, 0x12u, 0x13u, make_entity(1u, 2u, 3u, 0xC7u));
    const Guid         writer_guid = make_guid(0x21u, 0x22u, 0x23u, make_entity(4u, 5u, 6u, 0xC2u));
    const Locator      loc = make_udpv4_locator(0xC0A80114u, 7411u);
    constexpr uint32_t kRetry = RTPS_PREEMPTIVE_ACKNACK_RETRY_MS;

    g_stateful_reader_test_now_ms = 1000u;
    DefaultStatefulReader r(reader_guid, ReliabilityKind::RELIABLE, tx);
    CHECK(r.matched_writer_add(writer_guid, &loc, 1u, VENDORID_EPROSIMA),
          "matched_writer_add should succeed");
    CHECK(tx.sent.size() == 1u, "the match sends the preemptive ACKNACK");
    CHECK(r.ms_until_next_timer(1000u) == kRetry, "its retry is the next timer");

    r.tick(1000u + kRetry - 1u);
    CHECK(tx.sent.size() == 1u, "nothing is resent before the retry period");
    r.tick(1000u + kRetry);
    ParsedAckNack ack{};
    CHECK(tx.sent.size() == 2u && parse_acknack(tx.sent[1], ack),
          "an unheard writer is sent the preemptive ACKNACK again");
    CHECK(tx.dests[1] == loc && !ack.finalFlag && ack.readerSNState.empty()
              && ack.readerSNState.bitmapBase == SEQUENCENUMBER_ZERO && ack.count == 2,
          "in the same form, F-flag clear, with the next count");
    r.tick(1000u + 2u * kRetry);
    CHECK(tx.sent.size() == 3u, "and again every retry period");

    ParsedHeartbeat hb{};
    hb.writerId = writer_guid.entityId;
    hb.readerId = reader_guid.entityId;
    hb.firstSN = SequenceNumber::from_int64(1);
    hb.lastSN = SequenceNumber::from_int64(0);
    hb.finalFlag = true;
    hb.count = 1;
    ParseContext ctx{};
    ctx.senderGuidPrefix = writer_guid.prefix;
    g_stateful_reader_test_now_ms = 1000u + 2u * kRetry + 5u;
    r.on_heartbeat(ctx, hb);
    CHECK(r.matched_writer_lookup(writer_guid)->heard, "a HEARTBEAT marks the writer heard");
    CHECK(r.ms_until_next_timer(g_stateful_reader_test_now_ms) == UINT32_MAX,
          "and a final HEARTBEAT to an up-to-date reader leaves nothing owed");
    r.tick(1000u + 5u * kRetry);
    CHECK(tx.sent.size() == 3u, "no preemptive ACKNACK follows once the writer is heard");
    return true;
}

bool tc_srr_8_4_7_4_009(std::string& err) {
    CaptureTransport   tx{};
    const Guid         reader_guid = make_guid(0x11u, 0x12u, 0x13u, make_entity(1u, 2u, 3u, 0xC7u));
    const Guid         writer_guid = make_guid(0x21u, 0x22u, 0x23u, make_entity(4u, 5u, 6u, 0xC2u));
    const Locator      loc = make_udpv4_locator(0xC0A80114u, 7411u);
    constexpr uint32_t kRetry = RTPS_PREEMPTIVE_ACKNACK_RETRY_MS;

    g_stateful_reader_test_now_ms = 1000u;
    DefaultStatefulReader r(reader_guid, ReliabilityKind::RELIABLE, tx);
    CHECK(r.matched_writer_add(writer_guid, &loc, 1u, VENDORID_EPROSIMA),
          "matched_writer_add should succeed");
    CHECK(tx.sent.size() == 1u, "the match sends the preemptive ACKNACK");

    // The first of three fragments of SN 1: the sample stays partial.
    const Octet    bytes[4] = {0x00u, 0x01u, 0x00u, 0x00u};
    ParsedDataFrag df{};
    df.readerId = reader_guid.entityId;
    df.writerId = writer_guid.entityId;
    df.writerSN = SequenceNumber::from_int64(1);
    df.fragmentStartingNum = 1u;
    df.fragmentsInSubmessage = 1u;
    df.fragmentSize = 4u;
    df.dataSize = 12u;
    df.fragmentPayload = bytes;
    df.fragmentPayloadSize = sizeof(bytes);
    ParseContext ctx{};
    ctx.senderGuidPrefix = writer_guid.prefix;
    r.on_data_frag(ctx, df);
    CHECK(r.matched_writer_lookup(writer_guid)->heard, "a DATA_FRAG marks the writer heard");

    r.tick(1000u + kRetry);
    r.tick(1000u + 2u * kRetry);
    for (size_t i = 1u; i < tx.sent.size(); ++i) {
        ParsedAckNack ack{};
        CHECK(!parse_acknack(tx.sent[i], ack), "no preemptive ACKNACK follows a DATA_FRAG");
    }

    return true;
}

// A writer that never answers is sent only so many preemptive ACKNACKs:
// RTPS_PREEMPTIVE_ACKNACK_ATTEMPTS from a user reader, and
// RTPS_BUILTIN_PREEMPTIVE_ACKNACK_ATTEMPTS from a builtin one, after which
// nothing is owed. A HEARTBEAT that arrives later is still answered.
static bool tc_srr_8_4_7_4_010(std::string& err) {
    constexpr uint32_t kRetry = RTPS_PREEMPTIVE_ACKNACK_RETRY_MS;
    const Guid         writer_guid = make_guid(0x21u, 0x22u, 0x23u, make_entity(4u, 5u, 6u, 0x02u));
    const Locator      loc = make_udpv4_locator(0xC0A80114u, 7411u);

    const struct {
        Octet    reader_kind;
        uint32_t attempts;
    } cases[] = {{0x07u, RTPS_PREEMPTIVE_ACKNACK_ATTEMPTS},
                 {0xC7u, RTPS_BUILTIN_PREEMPTIVE_ACKNACK_ATTEMPTS}};

    for (const auto& c : cases) {
        CaptureTransport tx{};
        const Guid       reader_guid =
            make_guid(0x11u, 0x12u, 0x13u, make_entity(1u, 2u, 3u, c.reader_kind));
        g_stateful_reader_test_now_ms = 1000u;
        DefaultStatefulReader r(reader_guid, ReliabilityKind::RELIABLE, tx);
        CHECK(r.matched_writer_add(writer_guid, &loc, 1u), "matched_writer_add should succeed");

        for (uint32_t i = 1u; i <= c.attempts + 2u; ++i) {
            r.tick(1000u + i * kRetry);
        }

        CHECK(tx.sent.size() == c.attempts, "the preemptive ACKNACK stops after its attempts");
        CHECK(r.ms_until_next_timer(1000u + (c.attempts + 2u) * kRetry) == UINT32_MAX,
              "and nothing is owed after the last");

        ParsedHeartbeat hb{};
        hb.writerId = writer_guid.entityId;
        hb.readerId = reader_guid.entityId;
        hb.firstSN = SequenceNumber::from_int64(1);
        hb.lastSN = SequenceNumber::from_int64(1);
        hb.count = 1;
        ParseContext ctx{};
        ctx.senderGuidPrefix = writer_guid.prefix;
        const uint32_t later = 1000u + (c.attempts + 3u) * kRetry;
        g_stateful_reader_test_now_ms = later;
        r.on_heartbeat(ctx, hb);
        r.tick(later + RTPS_HEARTBEAT_RESPONSE_DELAY_MS);
        ParsedAckNack ack{};
        CHECK(tx.sent.size() == c.attempts + 1u && parse_acknack(tx.sent.back(), ack)
                  && !ack.readerSNState.empty(),
              "a later HEARTBEAT is answered with an ACKNACK requesting SN 1");
    }

    return true;
}

void append_stateful_reader_tests_8_4_7_4(std::vector<TestCase>& tests) {
    tests.push_back({"TC-SRR-8.4.7.4-001",
                     "§8.4.7.4: matched_writer_add creates one WriterProxy for a newly matched writer, and matched_writer_remove removes it from the matched set.",
                     tc_srr_8_4_7_4_001});
    tests.push_back(
        {"TC-SRR-8.4.7.4-002",
         "§8.4.7.4/§8.3.7.1: a RELIABLE reader sends one preemptive ACKNACK on a new match "
         "(bitmapBase 1, empty set, F-flag clear, count 1) to the writer's locator.",
         tc_srr_8_4_7_4_002});
    tests.push_back({"TC-SRR-8.4.7.4-003",
                     "§8.4.7.4: a BEST_EFFORT reader sends no ACKNACK on a new match.",
                     tc_srr_8_4_7_4_003});
    tests.push_back(
        {"TC-SRR-8.4.7.4-004",
         "§8.4.7.4/§8.3.7.1: the preemptive ACKNACK has bitmapBase 0 for an eProsima or "
         "RTI writer (the form Fast DDS answers) and bitmapBase 1 for any other vendor.",
         tc_srr_8_4_7_4_004});
    tests.push_back(
        {"TC-SRR-8.4.7.4-005",
         "§8.3.7.1/§8.4.7.4: a reader that rematches a writer counts its ACKNACKs on from the "
         "highest count it sent before, so a writer that remembers it accepts them.",
         tc_srr_8_4_7_4_005});
    tests.push_back(
        {"TC-SRR-8.4.7.4-006",
         "§8.4.7.1/§8.4.7.4: readers sharing a WriterProxy pool bind by its admission rule; a "
         "refused match sends no ACKNACK; an unmatch or a destroyed reader returns proxies.",
         tc_srr_8_4_7_4_006});
    tests.push_back({"TC-SRR-8.4.7.4-007",
                     "§8.4.7.1: HEARTBEATs that arrive while an ACKNACK is owed keep its "
                     "heartbeatResponseDelay deadline, so one ACKNACK answers them all.",
                     tc_srr_8_4_7_4_007});
    tests.push_back({"TC-SRR-8.4.7.4-008",
                     "§8.4.7.4/§8.3.7.1: the preemptive ACKNACK is resent every "
                     "RTPS_PREEMPTIVE_ACKNACK_RETRY_MS until the writer is heard, then never.",
                     tc_srr_8_4_7_4_008});
    tests.push_back({"TC-SRR-8.4.7.4-009",
                     "§8.4.7.4/§8.3.7.4: a DATA_FRAG marks its writer heard, so a writer sending "
                     "a fragmented sample is not sent the preemptive ACKNACK again.",
                     tc_srr_8_4_7_4_009});
    tests.push_back({"TC-SRR-8.4.7.4-010",
                     "§8.4.7.4: the preemptive ACKNACK is sent at most "
                     "RTPS_PREEMPTIVE_ACKNACK_ATTEMPTS times by a user reader and "
                     "RTPS_BUILTIN_PREEMPTIVE_ACKNACK_ATTEMPTS by a builtin one; a later HEARTBEAT "
                     "is still answered.",
                     tc_srr_8_4_7_4_010});
}

} // namespace spec_test
