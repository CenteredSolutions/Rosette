#include "sedp_peer_driver.hpp"
#include "sedp_recorded_endpoints.hpp"
#include "spec_test_common.hpp"

#include "rosette/rtps/sedp.hpp"
#include "rosette/rtps/transport.hpp"

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

static void on_match(const MatchedEndpoint& writer,
                     const MatchedEndpoint& reader,
                     const TopicName&,
                     void* user) noexcept {
    auto* o = static_cast<MatchObs*>(user);
    o->calls++;
    o->last_w = writer.guid;
    o->last_r = reader.guid;
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
    SpecSedpWriter w(local_prefix, tx);

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
    table.set_local_prefix(local_prefix);

    const Guid local_reader = make_guid(0x11u, 0x12u, 0x13u, make_entity(8u, 8u, 8u, 0xC7u));
    table.add_reader(make_reader_data(local_reader, "scan", "sensor_msgs::msg::LaserScan"));

    const Guid remote_writer = make_guid(0x31u, 0x32u, 0x33u, make_entity(9u, 9u, 9u, 0xC2u));
    DiscoveredWriterData wd = make_writer_data(remote_writer, "scan", "sensor_msgs::msg::LaserScan");
    std::vector<Octet> payload{};
    CHECK(build_publication_payload(wd, payload), "publication payload build failed");

    const SedpPeer peer{remote_writer.prefix, make_udpv4_locator(0xC0A80131u, 7410u)};
    peer.discover(reader);
    peer.announce(reader, entity_id::SEDP_BUILTIN_PUBLICATIONS_ANNOUNCER, 1, payload);
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
    table.set_local_prefix(local_prefix);

    const Guid local_writer = make_guid(0x41u, 0x42u, 0x43u, make_entity(7u, 7u, 7u, 0xC2u));
    table.add_writer(make_writer_data(local_writer, "odom", "nav_msgs::msg::Odometry"));

    const Guid remote_reader = make_guid(0x51u, 0x52u, 0x53u, make_entity(6u, 6u, 6u, 0xC7u));
    DiscoveredReaderData rd = make_reader_data(remote_reader, "odom", "nav_msgs::msg::Odometry");
    std::vector<Octet> payload{};
    CHECK(build_subscription_payload(rd, payload), "subscription payload build failed");

    const SedpPeer peer{remote_reader.prefix, make_udpv4_locator(0xC0A80151u, 7410u)};
    peer.discover(reader);
    peer.announce(reader, entity_id::SEDP_BUILTIN_SUBSCRIPTIONS_ANNOUNCER, 1, payload);
    CHECK(obs.calls == 1, "subscription data should trigger endpoint match");
    CHECK(obs.last_w == local_writer && obs.last_r == remote_reader, "match GUIDs mismatch");
    return true;
}

static bool tc_sedp_8_5_4_004(std::string& err) {
    FakeTransport tx{};
    const GuidPrefix local_prefix = make_guid(
        0x61u, 0x62u, 0x63u, entity_id::PARTICIPANT).prefix;
    SpecSedpWriter w(local_prefix, tx);

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

// Parses every captured message and tallies SEDP DATA per builtin writer.
struct SedpTally {
    uint32_t pub_alive = 0u, sub_alive = 0u, pub_disposed = 0u, sub_disposed = 0u;
    int64_t  last_pub_sn = 0, last_sub_sn = 0;
    Guid     last_disposed_key = GUID_UNKNOWN;
};

static SedpTally tally_sedp(const FakeTransport& tx) {
    SedpTally t{};
    ParseOps ops{};
    ops.on_data = [](const ParseContext& ctx, const ParsedData& d) noexcept {
        auto* out = static_cast<SedpTally*>(ctx.user_data);
        const bool disposed = d.inlineQos.withdraws_instance();
        if (disposed && d.inlineQos.hasKeyHash) out->last_disposed_key = d.inlineQos.key_hash_guid();
        if (d.writerId == entity_id::SEDP_BUILTIN_PUBLICATIONS_ANNOUNCER) {
            (disposed ? out->pub_disposed : out->pub_alive)++;
            out->last_pub_sn = d.writerSN.to_int64();
        } else if (d.writerId == entity_id::SEDP_BUILTIN_SUBSCRIPTIONS_ANNOUNCER) {
            (disposed ? out->sub_disposed : out->sub_alive)++;
            out->last_sub_sn = d.writerSN.to_int64();
        }
    };
    for (const auto& msg : tx.sent) {
        (void)parse_message(msg.data(), static_cast<uint32_t>(msg.size()), GuidPrefix{}, ops, &t);
    }
    return t;
}

// §8.5.4.1: each local endpoint is one TRANSIENT_LOCAL instance written
// once. Rediscovering a peer must not write new changes.
static bool tc_sedp_8_5_4_005(std::string& err) {
    FakeTransport tx{};
    const GuidPrefix local_prefix = make_guid(
        0x71u, 0x72u, 0x73u, entity_id::PARTICIPANT).prefix;
    SpecSedpWriter w(local_prefix, tx);

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

    const SedpTally t = tally_sedp(tx);
    CHECK(t.pub_alive == 1u && t.sub_alive == 1u,
          "rediscovering a matched peer must not re-send or re-write announcements");
    CHECK(t.last_pub_sn == 1 && t.last_sub_sn == 1,
          "each endpoint occupies a single SN in its builtin writer's history");
    return true;
}

// §8.5.4.1: an endpoint created after a peer was matched is announced to it.
static bool tc_sedp_8_5_4_007(std::string& err) {
    FakeTransport tx{};
    const GuidPrefix local_prefix = make_guid(0xC1u, 0xC2u, 0xC3u, entity_id::PARTICIPANT).prefix;
    SpecSedpWriter w(local_prefix, tx);
    const GuidPrefix peer_prefix = make_guid(0xD1u, 0xD2u, 0xD3u, entity_id::PARTICIPANT).prefix;
    const Locator dest = make_udpv4_locator(0xC0A80110u, 7410u);
    w.announce_to_peer(peer_prefix, dest);
    CHECK(tx.sent.empty(), "nothing to announce before any endpoint exists");

    const Guid late_writer = make_guid(0xC1u, 0xC2u, 0xC3u, make_entity(3u, 3u, 3u, 0x03u));
    CHECK(w.register_writer(make_writer_data(late_writer, "late", "std_msgs::msg::String")),
          "register_writer failed");
    const SedpTally t = tally_sedp(tx);
    CHECK(t.pub_alive == 1u, "an endpoint created after matching must be announced to the peer");
    CHECK(tx.dests.back() == dest, "late announcement must go to the peer's metatraffic locator");
    return true;
}

// §8.5.4.1 / §9.6.3.3: deleting an endpoint publishes a disposal keyed by
// its GUID, and late joiners no longer receive its announcement.
static bool tc_sedp_8_5_4_008(std::string& err) {
    FakeTransport tx{};
    const GuidPrefix local_prefix = make_guid(0xE1u, 0xE2u, 0xE3u, entity_id::PARTICIPANT).prefix;
    SpecSedpWriter w(local_prefix, tx);
    const Guid writer_guid = make_guid(0xE1u, 0xE2u, 0xE3u, make_entity(4u, 4u, 4u, 0x03u));
    CHECK(w.register_writer(make_writer_data(writer_guid, "gone", "std_msgs::msg::String")),
          "register_writer failed");
    const Locator dest = make_udpv4_locator(0xC0A80111u, 7410u);
    w.announce_to_peer(make_guid(0xF1u, 0xF2u, 0xF3u, entity_id::PARTICIPANT).prefix, dest);

    CHECK(w.unregister_writer(writer_guid), "unregister_writer should find the endpoint");
    SedpTally t = tally_sedp(tx);
    CHECK(t.pub_disposed == 1u, "deleting an endpoint must send one disposal");
    CHECK(t.last_disposed_key == writer_guid, "disposal key hash must be the endpoint GUID");
    CHECK(!w.unregister_writer(writer_guid), "a second unregister must report unknown GUID");

    // A peer discovered afterwards receives no live announcement for it.
    tx.sent.clear();
    tx.dests.clear();
    w.announce_to_peer(make_guid(0xF4u, 0xF5u, 0xF6u, entity_id::PARTICIPANT).prefix, dest);
    t = tally_sedp(tx);
    CHECK(t.pub_alive == 0u, "a deleted endpoint must not be replayed to late joiners");

    // Registering and deleting many endpoints never evicts a live one.
    const Guid keeper = make_guid(0xE1u, 0xE2u, 0xE3u, make_entity(9u, 9u, 9u, 0x03u));
    CHECK(w.register_writer(make_writer_data(keeper, "keep", "std_msgs::msg::String")),
          "register keeper failed");
    for (uint8_t i = 0; i < 3u * SedpWriter::kSedpHistoryDepth; ++i) {
        const Guid g = make_guid(0xE1u, 0xE2u, 0xE3u, make_entity(5u, 0u, i, 0x03u));
        CHECK(w.register_writer(make_writer_data(g, "churn", "std_msgs::msg::String")),
              "churn register failed");
        CHECK(w.unregister_writer(g), "churn unregister failed");
    }
    tx.sent.clear();
    tx.dests.clear();
    w.announce_to_peer(make_guid(0xF7u, 0xF8u, 0xF9u, entity_id::PARTICIPANT).prefix, dest);
    t = tally_sedp(tx);
    CHECK(t.pub_alive == 1u, "the live endpoint must survive disposal churn in the history");
    return true;
}

// §8.5.4.1 / §9.6.3.3: a remote disposal unmatches the withdrawn endpoint.
static bool tc_sedp_8_5_4_009(std::string& err) {
    struct UnmatchObs { int matches = 0; int unmatches = 0; };
    UnmatchObs obs{};
    FakeTransport tx{};
    EndpointMatchTable table(
        [](const MatchedEndpoint&, const MatchedEndpoint&, const TopicName&, void* u) noexcept {
            static_cast<UnmatchObs*>(u)->matches++;
        },
        [](const Guid&, const Guid&, void* u) noexcept {
            static_cast<UnmatchObs*>(u)->unmatches++;
        },
        &obs);
    const GuidPrefix local_prefix = make_guid(0x11u, 0x22u, 0x33u, entity_id::PARTICIPANT).prefix;
    SedpReader reader(local_prefix, tx, table);
    table.set_local_prefix(local_prefix);
    const Guid local_reader = make_guid(0x11u, 0x22u, 0x33u, make_entity(1u, 1u, 1u, 0x04u));
    table.add_reader(make_reader_data(local_reader, "scan", "sensor_msgs::msg::LaserScan"));

    const Guid remote_writer = make_guid(0x44u, 0x55u, 0x66u, make_entity(2u, 2u, 2u, 0x03u));
    std::vector<Octet> payload{};
    CHECK(build_publication_payload(
              make_writer_data(remote_writer, "scan", "sensor_msgs::msg::LaserScan"), payload),
          "publication payload build failed");
    const EntityId pubs = entity_id::SEDP_BUILTIN_PUBLICATIONS_ANNOUNCER;
    const SedpPeer owner{remote_writer.prefix, make_udpv4_locator(0xC0A80144u, 7410u)};
    const SedpPeer other{make_guid(0x77u, 0x88u, 0x99u, entity_id::PARTICIPANT).prefix,
                         make_udpv4_locator(0xC0A80177u, 7410u)};
    owner.discover(reader);
    other.discover(reader);
    owner.announce(reader, pubs, 1, payload);
    CHECK(obs.matches == 1, "remote writer should match the local reader");

    // A disposal from a different participant must be ignored.
    other.dispose(reader, pubs, 1, remote_writer);
    CHECK(obs.unmatches == 0, "only the owning participant may withdraw an endpoint");

    // Inline QoS alone (no payload, no key) withdraws the endpoint.
    owner.dispose(reader, pubs, 2, remote_writer);
    CHECK(obs.unmatches == 1, "disposal must unmatch the withdrawn remote writer");
    return true;
}

// ============================================================
// tc_sedp_8_5_4_006
//
// §8.3.7.5, p. 91: A HEARTBEAT with F=0 MUST produce an ACKNACK.
// §8.5.4.1, p. 165: SEDP builtin endpoints use RELIABLE QoS.
//
// Verify that the builtin publications reader, once SPDP has matched the
// peer's announcer, sends:
//   1. Exactly one unicast ACKNACK to the peer's metatraffic locator.
//   2. An ACKNACK NACKing the full range [firstSN..lastSN] when no
//      DATA has been received yet.
//   3. A pure ACK (numBits=0) after every change up to lastSN arrived.
//   4. A monotonically increasing ACKNACK count on repeated HBs.
// ============================================================

static bool parse_single_acknack(const std::vector<Octet>& msg, ParsedAckNack& out) {
    ParseOps ops{};
    ops.on_acknack = [](const ParseContext& c, const ParsedAckNack& a) noexcept {
        *static_cast<ParsedAckNack*>(c.user_data) = a;
    };
    return parse_message(msg.data(), static_cast<uint32_t>(msg.size()), GuidPrefix{}, ops,
                         &out) == ParseResult::OK;
}

static bool tc_sedp_8_5_4_006(std::string& err) {
    FakeTransport tx{};
    EndpointMatchTable table(nullptr, nullptr, nullptr);
    const GuidPrefix local_prefix = make_guid(0xA1u, 0xA2u, 0xA3u, entity_id::PARTICIPANT).prefix;
    SedpReader reader(local_prefix, tx, table);
    table.set_local_prefix(local_prefix);

    const EntityId pubs = entity_id::SEDP_BUILTIN_PUBLICATIONS_ANNOUNCER;
    const SedpPeer peer{make_guid(0xB1u, 0xB2u, 0xB3u, entity_id::PARTICIPANT).prefix,
                        make_udpv4_locator(0xC0A820C8u, 7410u)};

    peer.heartbeat(reader, pubs, 1, 3, 1);
    CHECK(tx.sent.empty(), "a HEARTBEAT from an undiscovered participant is ignored");
    peer.discover(reader);

    // Before any DATA is received, HEARTBEAT should produce a NACK for 1..3.
    peer.heartbeat(reader, pubs, 1, 3, 1);
    CHECK(tx.sent.size() == 1u, "HEARTBEAT F=0 must elicit exactly one ACKNACK");
    CHECK(tx.dests[0] == peer.meta, "ACKNACK must go to the peer's metatraffic locator");

    ParsedAckNack ack{};
    CHECK(parse_single_acknack(tx.sent[0], ack), "ACKNACK must parse");
    CHECK(ack.writerId == pubs, "ACKNACK writerId must match SEDP publications announcer");
    CHECK(ack.readerId == entity_id::SEDP_BUILTIN_PUBLICATIONS_DETECTOR,
          "ACKNACK readerId must match SEDP publications detector");
    CHECK(ack.readerSNState.bitmapBase == SequenceNumber::from_int64(1),
          "ACKNACK bitmapBase must be firstSN when no data received");
    CHECK(ack.readerSNState.numBits >= 3u, "ACKNACK must cover SNs 1-3 in the NACK bitmap");
    CHECK(ack.count == 1, "first ACKNACK count must be 1");

    peer.account(reader, pubs, 1);
    peer.account(reader, pubs, 2);
    peer.account(reader, pubs, 3);

    // Now a second HEARTBEAT for the same range should yield a pure ACK.
    tx.sent.clear();
    tx.dests.clear();
    peer.heartbeat(reader, pubs, 1, 3, 2);
    CHECK(tx.sent.size() == 1u, "second HEARTBEAT must still elicit ACKNACK");
    CHECK(parse_single_acknack(tx.sent[0], ack), "second ACKNACK must parse");
    CHECK(ack.readerSNState.numBits == 0u,
          "after receiving all data, ACKNACK must be a pure ACK (numBits=0)");
    CHECK(ack.readerSNState.bitmapBase == SequenceNumber::from_int64(4),
          "pure ACK bitmapBase must be lastSN+1");
    CHECK(ack.count == 2, "second ACKNACK count must be 2");
    return true;
}

// §8.4.14 / §8.5.4.1: an announcement too large for one datagram arrives as
// DATA_FRAGs; the builtin reader reassembles it and the endpoint matches.
static bool tc_sedp_8_5_4_013(std::string& err) {
    MatchObs obs{};
    FakeTransport tx{};
    EndpointMatchTable table(on_match, nullptr, &obs);
    const GuidPrefix local_prefix = make_guid(0x13u, 0x14u, 0x15u, entity_id::PARTICIPANT).prefix;
    SedpReader reader(local_prefix, tx, table);
    table.set_local_prefix(local_prefix);

    const Guid local_reader = make_guid(0x13u, 0x14u, 0x15u, make_entity(8u, 8u, 8u, 0x04u));
    table.add_reader(make_reader_data(local_reader, "scan", "sensor_msgs::msg::LaserScan"));

    const Guid remote_writer = make_guid(0x35u, 0x36u, 0x37u, make_entity(9u, 9u, 9u, 0x03u));
    std::vector<Octet> payload{};
    CHECK(build_publication_payload(
              make_writer_data(remote_writer, "scan", "sensor_msgs::msg::LaserScan"), payload),
          "publication payload build failed");
    CHECK(payload.size() > 64u, "the announcement spans several fragments");

    const SedpPeer peer{remote_writer.prefix, make_udpv4_locator(0xC0A80135u, 7410u)};
    peer.discover(reader);
    peer.announce_fragmented(reader, entity_id::SEDP_BUILTIN_PUBLICATIONS_ANNOUNCER, 1,
                             payload, 32u);
    CHECK(obs.calls == 1, "the reassembled announcement matches the endpoint");
    CHECK(obs.last_w == remote_writer && obs.last_r == local_reader, "match GUIDs mismatch");
    return true;
}

// count() answers the graph's per-topic queries: local endpoints by name,
// remote endpoints by topic key, whatever their type; on_change fires once
// per recorded or forgotten endpoint and not for a repeated announcement.
static bool tc_sedp_8_5_4_014(std::string& err) {
    int changes = 0;
    EndpointMatchTable t(nullptr, nullptr, &changes,
                         [](void* u) noexcept { ++*static_cast<int*>(u); });
    GuidPrefix local{};
    local.value[0] = 0x01u;
    GuidPrefix remote{};
    remote.value[0] = 0x02u;
    t.set_local_prefix(local);

    auto writer = [](const GuidPrefix& prefix, uint8_t key, const char* topic,
                     const char* type) {
        DiscoveredWriterData wd{};
        wd.writerGuid.prefix = prefix;
        wd.writerGuid.entityId = make_entity(0u, 0u, key, 0x03u);
        wd.topicName.assign(topic);
        wd.typeName.assign(type);
        return wd;
    };
    t.add_writer(writer(local, 1u, "rt/chatter", "std_msgs::msg::dds_::String_"));
    t.add_writer(writer(remote, 2u, "rt/chatter", "std_msgs::msg::dds_::String_"));
    t.add_writer(writer(remote, 3u, "rt/chatter", "std_msgs::msg::dds_::Int32_"));
    t.add_writer(writer(remote, 4u, "rt/other", "std_msgs::msg::dds_::String_"));
    CHECK(changes == 4, "each new endpoint fires on_change once");
    t.add_writer(writer(remote, 2u, "rt/chatter", "std_msgs::msg::dds_::String_"));
    CHECK(changes == 4, "a repeated identical announcement does not fire on_change");

    CHECK(t.count("rt/chatter", true) == 3u, "local and remote writers of any type count");
    CHECK(t.count("rt/chatter", false) == 0u, "no readers on the topic");
    CHECK(t.count("rt/other", true) == 1u, "another topic counts separately");
    CHECK(t.count("rt/none", true) == 0u, "an unknown topic counts zero");

    DiscoveredWriterData gone = writer(remote, 2u, "rt/chatter", "std_msgs::msg::dds_::String_");
    t.remove_writer(gone.writerGuid);
    CHECK(changes == 5, "a forgotten endpoint fires on_change");
    t.remove_writer(gone.writerGuid);
    CHECK(changes == 5, "forgetting an unknown endpoint does not");
    CHECK(t.count("rt/chatter", true) == 2u, "a forgotten endpoint no longer counts");

    t.remove_participant(remote);
    CHECK(t.count("rt/chatter", true) == 1u && t.count("rt/other", true) == 0u,
          "a departed participant's endpoints no longer count; local ones still do");
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
                     "§8.5.4.1: each local endpoint is written once; rediscovering a matched peer writes and sends nothing new.",
                     tc_sedp_8_5_4_005});
    tests.push_back({"TC-SEDP-8.5.4-006",
                     "§8.3.7.5/§8.5.4.1: once SPDP matches a peer's announcer, the SEDP reader answers HEARTBEAT F=0 with a NACK for missing SNs, and with a pure ACK once all DATA received.",
                     tc_sedp_8_5_4_006});
    tests.push_back({"TC-SEDP-8.5.4-007",
                     "§8.5.4.1: an endpoint registered after a peer's builtin readers were matched is announced to that peer.",
                     tc_sedp_8_5_4_007});
    tests.push_back({"TC-SEDP-8.5.4-008",
                     "§8.5.4.1/§9.6.3.3: unregistering an endpoint sends a disposal keyed by its GUID, stops replay to late joiners, and disposal churn never evicts a live announcement.",
                     tc_sedp_8_5_4_008});
    tests.push_back({"TC-SEDP-8.5.4-009",
                     "§8.5.4.1/§9.6.3.3: an inline-QoS-only disposal from the owning participant unmatches the withdrawn remote endpoint; disposals from other participants are ignored.",
                     tc_sedp_8_5_4_009});
    tests.push_back({"TC-SEDP-8.5.4-013",
                     "§8.4.14/§8.5.4.1: a fragmented endpoint announcement is reassembled by the builtin reader and matches the endpoint.",
                     tc_sedp_8_5_4_013});
    tests.push_back({"TC-SEDP-8.5.4-014",
                     "§8.5.4: count() counts local and remote endpoints per topic whatever their type; on_change fires once per recorded or forgotten endpoint.",
                     tc_sedp_8_5_4_014});
}

} // namespace spec_test
