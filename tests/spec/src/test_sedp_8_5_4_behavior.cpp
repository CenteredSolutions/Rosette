#include "spec_test_common.hpp"

#include "rtps/sedp.hpp"
#include "rtps/transport.hpp"

namespace spec_test {

#define CHECK(COND, MSG) do { if (!(COND)) { err = (MSG); return false; } } while (0)

namespace {

struct FakeTransport : ITransport {
    std::vector<std::vector<Octet>> sent{};
    std::vector<Locator> dests{};

    bool send_unicast(const BufRef& buf, const Locator& dest) noexcept override {
        sent.emplace_back(buf.data, buf.data + buf.size);
        dests.push_back(dest);
        return true;
    }

    bool send_multicast(const BufRef&, const Locator&) noexcept override {
        return false;
    }
};

struct DataCapture {
    int calls = 0;
    ParsedData data{};
    std::vector<Octet> payload{};
};

static void on_data_capture(const ParseContext& ctx, const ParsedData& d) noexcept {
    auto* cap = static_cast<DataCapture*>(ctx.user_data);
    cap->calls++;
    cap->data = d;
    cap->payload.assign(d.serializedPayload, d.serializedPayload + d.serializedPayloadSize);
}

static bool parse_data_from_message(const std::vector<Octet>& msg,
                                    DataCapture&              cap,
                                    std::string&              err) {
    ParseOps ops{};
    ops.on_data = on_data_capture;
    CHECK(parse_message(msg.data(),
                        static_cast<uint32_t>(msg.size()),
                        GuidPrefix{},
                        ops,
                        &cap) == ParseResult::OK,
          "sent SEDP message should parse");
    CHECK(cap.calls == 1, "each SEDP send should contain one DATA");
    return true;
}

static bool try_parse_data_from_message(const std::vector<Octet>& msg,
                                        DataCapture&              cap) {
    ParseOps ops{};
    ops.on_data = on_data_capture;
    if (parse_message(msg.data(),
                      static_cast<uint32_t>(msg.size()),
                      GuidPrefix{},
                      ops,
                      &cap) != ParseResult::OK) {
        return false;
    }
    return cap.calls == 1;
}

static Guid make_guid(Octet p0, Octet p1, Octet p2, const EntityId& eid) {
    Guid g{};
    g.prefix.value[0] = p0;
    g.prefix.value[1] = p1;
    g.prefix.value[2] = p2;
    g.entityId = eid;
    return g;
}

static DiscoveredWriterData make_writer_data(const Guid& g, const char* topic, const char* type) {
    DiscoveredWriterData wd{};
    wd.writerGuid = g;
    wd.topicName.assign(topic);
    wd.typeName.assign(type);
    wd.reliabilityKind = ReliabilityKind::RELIABLE;
    wd.durabilityKind = DurabilityKind::TRANSIENT_LOCAL;
    wd.unicastLocatorCount = 1u;
    wd.unicastLocatorList[0] = make_udpv4_locator(0xC0A80110u, 7411u);
    return wd;
}

static DiscoveredReaderData make_reader_data(const Guid& g, const char* topic, const char* type) {
    DiscoveredReaderData rd{};
    rd.readerGuid = g;
    rd.topicName.assign(topic);
    rd.typeName.assign(type);
    rd.reliabilityKind = ReliabilityKind::RELIABLE;
    rd.durabilityKind = DurabilityKind::VOLATILE;
    rd.unicastLocatorCount = 1u;
    rd.unicastLocatorList[0] = make_udpv4_locator(0xC0A80120u, 7412u);
    return rd;
}

struct MatchObs {
    int calls = 0;
    Guid last_w = GUID_UNKNOWN;
    Guid last_r = GUID_UNKNOWN;
};

static void on_match(const DiscoveredWriterData& wd,
                     const DiscoveredReaderData& rd,
                     void* user) noexcept {
    auto* o = static_cast<MatchObs*>(user);
    o->calls++;
    o->last_w = wd.writerGuid;
    o->last_r = rd.readerGuid;
}

static bool build_publication_payload(const DiscoveredWriterData& wd, std::vector<Octet>& out) {
    Octet buf[512] = {};
    uint32_t off = 0;

    const uint16_t rep_id = 0x0003u;
    const uint16_t rep_opt = 0x0000u;
    memcpy(buf + off, &rep_id, 2u); off += 2u;
    memcpy(buf + off, &rep_opt, 2u); off += 2u;

    Octet g[16] = {};
    memcpy(g, wd.writerGuid.prefix.value, 12u);
    memcpy(g + 12u, &wd.writerGuid.entityId, 4u);
    if (!plcdr_write_param(buf, sizeof(buf), off, pid::ENDPOINT_GUID, g, 16u)) return false;
    if (!plcdr_write_string(buf, sizeof(buf), off, pid::TOPIC_NAME, wd.topicName.str)) return false;
    if (!plcdr_write_string(buf, sizeof(buf), off, pid::TYPE_NAME, wd.typeName.str)) return false;
    {
        const uint32_t rel = static_cast<uint32_t>(wd.reliabilityKind);
        if (!plcdr_write_param(buf, sizeof(buf), off, pid::RELIABILITY, &rel, 4u)) return false;
    }
    {
        const uint32_t dur = static_cast<uint32_t>(wd.durabilityKind);
        if (!plcdr_write_param(buf, sizeof(buf), off, pid::DURABILITY, &dur, 4u)) return false;
    }
    for (uint8_t i = 0; i < wd.unicastLocatorCount; ++i) {
        if (!plcdr_write_param(buf, sizeof(buf), off, pid::UNICAST_LOCATOR,
                               &wd.unicastLocatorList[i], 24u)) return false;
    }
    if (!plcdr_write_sentinel(buf, sizeof(buf), off)) return false;
    out.assign(buf, buf + off);
    return true;
}

static bool build_subscription_payload(const DiscoveredReaderData& rd, std::vector<Octet>& out) {
    Octet buf[512] = {};
    uint32_t off = 0;

    const uint16_t rep_id = 0x0003u;
    const uint16_t rep_opt = 0x0000u;
    memcpy(buf + off, &rep_id, 2u); off += 2u;
    memcpy(buf + off, &rep_opt, 2u); off += 2u;

    Octet g[16] = {};
    memcpy(g, rd.readerGuid.prefix.value, 12u);
    memcpy(g + 12u, &rd.readerGuid.entityId, 4u);
    if (!plcdr_write_param(buf, sizeof(buf), off, pid::ENDPOINT_GUID, g, 16u)) return false;
    if (!plcdr_write_string(buf, sizeof(buf), off, pid::TOPIC_NAME, rd.topicName.str)) return false;
    if (!plcdr_write_string(buf, sizeof(buf), off, pid::TYPE_NAME, rd.typeName.str)) return false;
    {
        const uint32_t rel = static_cast<uint32_t>(rd.reliabilityKind);
        if (!plcdr_write_param(buf, sizeof(buf), off, pid::RELIABILITY, &rel, 4u)) return false;
    }
    {
        const uint32_t dur = static_cast<uint32_t>(rd.durabilityKind);
        if (!plcdr_write_param(buf, sizeof(buf), off, pid::DURABILITY, &dur, 4u)) return false;
    }
    for (uint8_t i = 0; i < rd.unicastLocatorCount; ++i) {
        if (!plcdr_write_param(buf, sizeof(buf), off, pid::UNICAST_LOCATOR,
                               &rd.unicastLocatorList[i], 24u)) return false;
    }
    if (!plcdr_write_sentinel(buf, sizeof(buf), off)) return false;
    out.assign(buf, buf + off);
    return true;
}

} // namespace

static bool tc_sedp_8_5_4_001(std::string& err) {
    FakeTransport tx{};
    const GuidPrefix local_prefix = make_guid(
        0x11u, 0x12u, 0x13u, entity_id::PARTICIPANT).prefix;
    SedpWriter w(local_prefix, tx);

    const Guid local_writer = make_guid(0x11u, 0x12u, 0x13u, make_entity(1u, 2u, 3u, 0xC2u));
    const Guid local_reader = make_guid(0x11u, 0x12u, 0x13u, make_entity(4u, 5u, 6u, 0xC7u));
    CHECK(w.register_writer(make_writer_data(local_writer, "chatter", "std_msgs::msg::String")),
          "register_writer failed");
    CHECK(w.register_reader(make_reader_data(local_reader, "chatter", "std_msgs::msg::String")),
          "register_reader failed");

    const Locator dest = make_udpv4_locator(0xEFFF0001u, 7400u);
    const GuidPrefix peer_prefix = make_guid(
        0x21u, 0x22u, 0x23u, entity_id::PARTICIPANT).prefix;
    w.announce_to_peer(peer_prefix, dest);

    CHECK(tx.sent.size() >= 2u, "SEDP announce should emit at least publications + subscriptions DATA");
    CHECK(tx.dests.size() == tx.sent.size(), "destination record count mismatch");
    for (const auto& d : tx.dests) {
        CHECK(d == dest, "dest locator mismatch");
    }

    bool saw_pub = false;
    bool saw_sub = false;
    for (const auto& msg : tx.sent) {
        DataCapture cap{};
        if (!try_parse_data_from_message(msg, cap)) {
            continue;
        }
        CHECK(cap.payload.size() >= 4u, "SEDP payload should contain PL_CDR header");
        CHECK(cap.payload[0] == 0x00u && cap.payload[1] == 0x03u &&
              cap.payload[2] == 0x00u && cap.payload[3] == 0x00u,
              "SEDP payload should start with PL_CDR_LE header");
        if (cap.data.writerId == entity_id::SEDP_BUILTIN_PUBLICATIONS_ANNOUNCER &&
            cap.data.readerId == entity_id::SEDP_BUILTIN_PUBLICATIONS_DETECTOR) {
            saw_pub = true;
        }
        if (cap.data.writerId == entity_id::SEDP_BUILTIN_SUBSCRIPTIONS_ANNOUNCER &&
            cap.data.readerId == entity_id::SEDP_BUILTIN_SUBSCRIPTIONS_DETECTOR) {
            saw_sub = true;
        }
    }
    CHECK(saw_pub, "missing SEDP publications DATA announcement");
    CHECK(saw_sub, "missing SEDP subscriptions DATA announcement");
    return true;
}

static bool tc_sedp_8_5_4_002(std::string& err) {
    MatchObs obs{};
    FakeTransport tx{};
    EndpointMatchTable table(on_match, nullptr, &obs);
    const GuidPrefix local_prefix = make_guid(0x11u, 0x12u, 0x13u, entity_id::PARTICIPANT).prefix;
    SedpReader reader(local_prefix, tx, table);

    const Guid local_reader = make_guid(0x21u, 0x22u, 0x23u, make_entity(8u, 8u, 8u, 0xC7u));
    table.add_reader(make_reader_data(local_reader, "scan", "sensor_msgs::msg::LaserScan"));

    const Guid remote_writer = make_guid(0x31u, 0x32u, 0x33u, make_entity(9u, 9u, 9u, 0xC2u));
    DiscoveredWriterData wd = make_writer_data(remote_writer, "scan", "sensor_msgs::msg::LaserScan");
    std::vector<Octet> payload{};
    CHECK(build_publication_payload(wd, payload), "publication payload build failed");

    reader.on_publication_data(remote_writer.prefix, payload.data(),
                               static_cast<uint32_t>(payload.size()),
                               SequenceNumber::from_int64(1));
    CHECK(obs.calls == 1, "publication data should trigger endpoint match");
    CHECK(obs.last_w == remote_writer && obs.last_r == local_reader, "match GUIDs mismatch");
    return true;
}

static bool tc_sedp_8_5_4_003(std::string& err) {
    MatchObs obs{};
    FakeTransport tx{};
    EndpointMatchTable table(on_match, nullptr, &obs);
    const GuidPrefix local_prefix = make_guid(0x41u, 0x42u, 0x43u, entity_id::PARTICIPANT).prefix;
    SedpReader reader(local_prefix, tx, table);

    const Guid local_writer = make_guid(0x41u, 0x42u, 0x43u, make_entity(7u, 7u, 7u, 0xC2u));
    table.add_writer(make_writer_data(local_writer, "odom", "nav_msgs::msg::Odometry"));

    const Guid remote_reader = make_guid(0x51u, 0x52u, 0x53u, make_entity(6u, 6u, 6u, 0xC7u));
    DiscoveredReaderData rd = make_reader_data(remote_reader, "odom", "nav_msgs::msg::Odometry");
    std::vector<Octet> payload{};
    CHECK(build_subscription_payload(rd, payload), "subscription payload build failed");

    reader.on_subscription_data(remote_reader.prefix, payload.data(),
                                static_cast<uint32_t>(payload.size()),
                                SequenceNumber::from_int64(1));
    CHECK(obs.calls == 1, "subscription data should trigger endpoint match");
    CHECK(obs.last_w == local_writer && obs.last_r == remote_reader, "match GUIDs mismatch");
    return true;
}

static bool tc_sedp_8_5_4_004(std::string& err) {
    FakeTransport tx{};
    const GuidPrefix local_prefix = make_guid(
        0x61u, 0x62u, 0x63u, entity_id::PARTICIPANT).prefix;
    SedpWriter w(local_prefix, tx);

    const Guid local_writer_a = make_guid(0x61u, 0x62u, 0x63u, make_entity(1u, 0u, 1u, 0xC2u));
    const Guid local_writer_b = make_guid(0x61u, 0x62u, 0x63u, make_entity(1u, 0u, 2u, 0xC2u));
    CHECK(w.register_writer(make_writer_data(local_writer_a, "a", "std_msgs::msg::String")),
          "register_writer a failed");
    CHECK(w.register_writer(make_writer_data(local_writer_b, "b", "std_msgs::msg::String")),
          "register_writer b failed");

    const Locator dest = make_udpv4_locator(0xEFFF0001u, 7400u);
    const GuidPrefix peer_prefix = make_guid(
        0x81u, 0x82u, 0x83u, entity_id::PARTICIPANT).prefix;
    w.announce_to_peer(peer_prefix, dest);

    CHECK(tx.sent.size() >= 2u, "expected at least two SEDP publications DATA messages");
    int64_t previous = 0;
    uint32_t pub_count = 0u;
    for (const auto& msg : tx.sent) {
        DataCapture cap{};
        if (!try_parse_data_from_message(msg, cap)) {
            continue;
        }
        if (cap.data.writerId != entity_id::SEDP_BUILTIN_PUBLICATIONS_ANNOUNCER ||
            cap.data.readerId != entity_id::SEDP_BUILTIN_PUBLICATIONS_DETECTOR) {
            continue;
        }
        pub_count++;
        const int64_t sn = cap.data.writerSN.to_int64();
        CHECK(sn > previous, "publication writerSN must increase for each announced endpoint");
        previous = sn;
    }
    CHECK(pub_count == 2u, "expected two publication DATA announcements");
    return true;
}

static bool tc_sedp_8_5_4_005(std::string& err) {
    FakeTransport tx{};
    const GuidPrefix local_prefix = make_guid(
        0x71u, 0x72u, 0x73u, entity_id::PARTICIPANT).prefix;
    SedpWriter w(local_prefix, tx);

    const Guid local_writer = make_guid(0x71u, 0x72u, 0x73u, make_entity(1u, 1u, 1u, 0xC2u));
    const Guid local_reader = make_guid(0x71u, 0x72u, 0x73u, make_entity(2u, 2u, 2u, 0xC7u));
    CHECK(w.register_writer(make_writer_data(local_writer, "chatter", "std_msgs::msg::String")),
          "register_writer failed");
    CHECK(w.register_reader(make_reader_data(local_reader, "chatter", "std_msgs::msg::String")),
          "register_reader failed");

    const Locator dest = make_udpv4_locator(0xEFFF0001u, 7400u);
    const GuidPrefix peer_prefix = make_guid(
        0x91u, 0x92u, 0x93u, entity_id::PARTICIPANT).prefix;
    w.announce_to_peer(peer_prefix, dest);
    w.announce_to_peer(peer_prefix, dest);

    CHECK(tx.sent.size() >= 4u, "two announce_to_peer() calls should produce at least four DATA messages");

    int64_t last_pub_sn = 0;
    int64_t last_sub_sn = 0;
    uint32_t pub_data = 0u;
    uint32_t sub_data = 0u;
    for (const auto& msg : tx.sent) {
        DataCapture cap{};
        if (!try_parse_data_from_message(msg, cap)) {
            continue;
        }
        const int64_t sn = cap.data.writerSN.to_int64();
        if (cap.data.writerId == entity_id::SEDP_BUILTIN_PUBLICATIONS_ANNOUNCER) {
            CHECK(sn > last_pub_sn, "publications writerSN must increase across announce_to_peer calls");
            last_pub_sn = sn;
            pub_data++;
            continue;
        }
        if (cap.data.writerId == entity_id::SEDP_BUILTIN_SUBSCRIPTIONS_ANNOUNCER) {
            CHECK(sn > last_sub_sn, "subscriptions writerSN must increase across announce_to_peer calls");
            last_sub_sn = sn;
            sub_data++;
            continue;
        }
    }

    CHECK(last_pub_sn == 2, "expected two monotonically increasing publication sequence numbers");
    CHECK(last_sub_sn == 2, "expected two monotonically increasing subscription sequence numbers");
    CHECK(pub_data == 2u, "expected two publication DATA samples");
    CHECK(sub_data == 2u, "expected two subscription DATA samples");
    return true;
}

// ============================================================
// tc_sedp_8_5_4_006
//
// §8.3.7.5, p. 91: A HEARTBEAT with F=0 MUST produce an ACKNACK.
// §8.5.4.1, p. 165: SEDP builtin endpoints use RELIABLE QoS.
//
// Verify that SedpReader::on_heartbeat() sends:
//   1. Exactly one unicast ACKNACK to the writer_locator.
//   2. An ACKNACK NACKing the full range [firstSN..lastSN] when no
//      DATA has been received yet (highest_sn = 0).
//   3. A pure ACK (numBits=0) after data up to lastSN is recorded
//      via on_publication_data().
//   4. A monotonically increasing ACKNACK count on repeated HBs.
// ============================================================

static bool tc_sedp_8_5_4_006(std::string& err) {
    FakeTransport tx{};
    EndpointMatchTable table(nullptr, nullptr, nullptr);
    const GuidPrefix local_prefix = make_guid(0xA1u, 0xA2u, 0xA3u, entity_id::PARTICIPANT).prefix;
    SedpReader reader(local_prefix, tx, table);

    const GuidPrefix remote_prefix = make_guid(0xB1u, 0xB2u, 0xB3u, entity_id::PARTICIPANT).prefix;
    const Locator remote_meta = make_udpv4_locator(0xC0A820C8u, 7410u);

    // Simulate a HEARTBEAT from the remote publications writer (seq 1-3, F=0).
    ParseContext ctx{};
    ctx.senderGuidPrefix = remote_prefix;
    ctx.user_data        = nullptr;

    ParsedHeartbeat hb{};
    hb.writerId   = entity_id::SEDP_BUILTIN_PUBLICATIONS_ANNOUNCER;
    hb.readerId   = entity_id::SEDP_BUILTIN_PUBLICATIONS_DETECTOR;
    hb.firstSN    = SequenceNumber::from_int64(1);
    hb.lastSN     = SequenceNumber::from_int64(3);
    hb.count      = 1;
    hb.finalFlag  = false;

    // Before any DATA is received, HEARTBEAT should produce a NACK for 1..3.
    reader.on_heartbeat(ctx, hb, remote_meta);
    CHECK(tx.sent.size() == 1u, "HEARTBEAT F=0 must elicit exactly one ACKNACK");
    CHECK(tx.dests[0] == remote_meta, "ACKNACK must be sent to writer_locator");

    // Parse the ACKNACK and verify it requests SNs 1-3.
    {
        ParseOps ops{};
        struct AckCap { int calls = 0; ParsedAckNack ack{}; };
        AckCap cap{};
        ops.on_acknack = [](const ParseContext& /*c*/, const ParsedAckNack& a) noexcept {
            // capture via user_data trick below
            (void)a;
        };
        // Direct struct inspection of the parsed ACKNACK.
        ParsedAckNack parsed_ack{};
        bool got_ack = false;
        ParseOps ops2{};
        ops2.on_acknack = [](const ParseContext& c, const ParsedAckNack& a) noexcept {
            auto* out = static_cast<ParsedAckNack*>(c.user_data);
            *out = a;
        };
        ParseContext rctx{};
        rctx.user_data = &parsed_ack;
        parse_message(tx.sent[0].data(),
                      static_cast<uint32_t>(tx.sent[0].size()),
                      GuidPrefix{}, ops2, &parsed_ack);

        CHECK(parsed_ack.writerId == entity_id::SEDP_BUILTIN_PUBLICATIONS_ANNOUNCER,
              "ACKNACK writerId must match SEDP publications announcer");
        CHECK(parsed_ack.readerId == entity_id::SEDP_BUILTIN_PUBLICATIONS_DETECTOR,
              "ACKNACK readerId must match SEDP publications detector");
        CHECK(parsed_ack.readerSNState.bitmapBase == SequenceNumber::from_int64(1),
              "ACKNACK bitmapBase must be firstSN when no data received");
        CHECK(parsed_ack.readerSNState.numBits >= 3u,
              "ACKNACK must cover SNs 1-3 in the NACK bitmap");
        CHECK(parsed_ack.count == 1, "first ACKNACK count must be 1");
    }

    // Record that we received publication data with SN 1, 2, 3.
    reader.on_publication_data(remote_prefix, nullptr, 0u, SequenceNumber::from_int64(1));
    reader.on_publication_data(remote_prefix, nullptr, 0u, SequenceNumber::from_int64(2));
    reader.on_publication_data(remote_prefix, nullptr, 0u, SequenceNumber::from_int64(3));

    // Now a second HEARTBEAT for the same range should yield a pure ACK.
    hb.count = 2;
    tx.sent.clear();
    tx.dests.clear();
    reader.on_heartbeat(ctx, hb, remote_meta);
    CHECK(tx.sent.size() == 1u, "second HEARTBEAT must still elicit ACKNACK");

    {
        ParsedAckNack parsed_ack2{};
        ParseOps ops3{};
        ops3.on_acknack = [](const ParseContext& c, const ParsedAckNack& a) noexcept {
            *static_cast<ParsedAckNack*>(c.user_data) = a;
        };
        parse_message(tx.sent[0].data(),
                      static_cast<uint32_t>(tx.sent[0].size()),
                      GuidPrefix{}, ops3, &parsed_ack2);
        CHECK(parsed_ack2.readerSNState.numBits == 0u,
              "after receiving all data, ACKNACK must be a pure ACK (numBits=0)");
        CHECK(parsed_ack2.readerSNState.bitmapBase == SequenceNumber::from_int64(4),
              "pure ACK bitmapBase must be lastSN+1");
        CHECK(parsed_ack2.count == 2, "second ACKNACK count must be 2");
    }
    return true;
}

void append_sedp_tests_8_5_4(std::vector<TestCase>& tests) {
    tests.push_back({"TC-SEDP-8.5.4-001",
                     "§8.5.4/§8.5.4.3: SEDP writer announces registered local publications and subscriptions as DATA submessages with PL_CDR payloads.",
                     tc_sedp_8_5_4_001});
    tests.push_back({"TC-SEDP-8.5.4-002",
                     "§8.5.4: SEDP publications reader processes DiscoveredWriterData and updates endpoint discovery state.",
                     tc_sedp_8_5_4_002});
    tests.push_back({"TC-SEDP-8.5.4-003",
                     "§8.5.4: SEDP subscriptions reader processes DiscoveredReaderData and updates endpoint discovery state.",
                     tc_sedp_8_5_4_003});
    tests.push_back({"TC-SEDP-8.5.4-004",
                     "§8.5.4.3: SEDP builtin publications writer uses monotonically increasing writerSN for distinct endpoint DATA changes.",
                     tc_sedp_8_5_4_004});
    tests.push_back({"TC-SEDP-8.5.4-005",
                     "§8.5.4.3: SEDP builtin writer sequence numbers continue monotonically across repeated announce_to_peer() calls.",
                     tc_sedp_8_5_4_005});
    tests.push_back({"TC-SEDP-8.5.4-006",
                     "§8.3.7.5/§8.5.4.1: SEDP reader responds to HEARTBEAT F=0 with NACK for missing SNs, and with pure ACK once all DATA received.",
                     tc_sedp_8_5_4_006});
}

} // namespace spec_test
