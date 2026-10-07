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
}

} // namespace spec_test
