#include "spec_test_common.hpp"

#include "rtps/participant.hpp"
#include "rtps/spdp.hpp"
#include "rtps/transport.hpp"

#include <string.h>

namespace spec_test {

extern uint32_t g_participant_test_now_ms;

#define CHECK(COND, MSG) do { if (!(COND)) { err = (MSG); return false; } } while (0)

namespace {

struct CaptureTransport : ITransport {
    struct Packet {
        std::vector<Octet> bytes{};
        Locator dest = LOCATOR_INVALID;
        bool multicast = false;
    };

    std::vector<Packet> packets{};

    bool send_unicast(const BufRef& buf, const Locator& dest) noexcept override {
        Packet p{};
        p.bytes.assign(buf.data, buf.data + buf.size);
        p.dest = dest;
        p.multicast = false;
        packets.push_back(static_cast<Packet&&>(p));
        return true;
    }

    bool send_multicast(const BufRef& buf, const Locator& dest) noexcept override {
        Packet p{};
        p.bytes.assign(buf.data, buf.data + buf.size);
        p.dest = dest;
        p.multicast = true;
        packets.push_back(static_cast<Packet&&>(p));
        return true;
    }
};

struct ReaderChangeObs {
    int calls = 0;
};

void on_reader_change(const CacheChange&, void* user_data) noexcept {
    auto* obs = static_cast<ReaderChangeObs*>(user_data);
    obs->calls++;
}

Guid make_guid(Octet p0, Octet p1, Octet p2, const EntityId& eid) {
    Guid g{};
    g.prefix.value[0] = p0;
    g.prefix.value[1] = p1;
    g.prefix.value[2] = p2;
    g.entityId = eid;
    return g;
}

GuidPrefix make_prefix(Octet p0, Octet p1, Octet p2) {
    GuidPrefix gp{};
    gp.value[0] = p0;
    gp.value[1] = p1;
    gp.value[2] = p2;
    return gp;
}

bool build_data_message(const GuidPrefix& sender_prefix,
                        const EntityId& reader_id,
                        const EntityId& writer_id,
                        const std::vector<Octet>& payload,
                        std::vector<Octet>& out,
                        const SequenceNumber& writer_sn = SequenceNumber::from_int64(1)) {
    BufRef payload_buf = buf_alloc_with_headroom(
        static_cast<uint32_t>(payload.size()),
        kRtpsDataHeadroom);
    if (!payload_buf.valid()) {
        return false;
    }

    if (!payload.empty()) {
        memcpy(payload_buf.data, payload.data(), payload.size());
    }
    payload_buf.size = static_cast<uint32_t>(payload.size());

    MessageBuilder builder{};
    const bool ok = builder.begin_with_payload(static_cast<BufRef&&>(payload_buf)) &&
                    builder.prepend_data_message(
                        sender_prefix,
                        reader_id,
                        writer_id,
                        writer_sn);
    if (!ok) {
        builder.abort();
        return false;
    }

    BufRef msg = builder.finish();
    out.assign(msg.data, msg.data + msg.size);
    msg.reset();
    return true;
}

bool build_acknack_message(const GuidPrefix& sender_prefix,
                           const EntityId& reader_id,
                           const EntityId& writer_id,
                           const SequenceNumberSet& sn_state,
                           int32_t count,
                           bool final_flag,
                           std::vector<Octet>& out) {
    MessageBuilder builder{};
    if (!builder.begin(sender_prefix)) {
        return false;
    }
    if (!builder.append_acknack(reader_id, writer_id, sn_state, count, final_flag)) {
        builder.abort();
        return false;
    }
    BufRef msg = builder.finish();
    out.assign(msg.data, msg.data + msg.size);
    msg.reset();
    return true;
}

bool build_nack_frag_message(const GuidPrefix& sender_prefix,
                             const EntityId& reader_id,
                             const EntityId& writer_id,
                             const SequenceNumber& writer_sn,
                             const FragmentNumberSet& missing_frags,
                             int32_t count,
                             std::vector<Octet>& out) {
    MessageBuilder builder{};
    if (!builder.begin(sender_prefix)) {
        return false;
    }
    if (!builder.append_nack_frag(reader_id, writer_id, writer_sn, missing_frags, count)) {
        builder.abort();
        return false;
    }
    BufRef msg = builder.finish();
    out.assign(msg.data, msg.data + msg.size);
    msg.reset();
    return true;
}

bool build_spdp_payload(const GuidPrefix& guid_prefix,
                        uint32_t lease_seconds,
                        const Locator& metatraffic_unicast,
                        std::vector<Octet>& out) {
    Octet buf[512] = {};
    uint32_t off = 0;

    const uint16_t rep_id = 0x0003u;
    const uint16_t rep_opt = 0x0000u;
    memcpy(buf + off, &rep_id, 2u); off += 2u;
    memcpy(buf + off, &rep_opt, 2u); off += 2u;

    Octet guid[16] = {};
    memcpy(guid, guid_prefix.value, 12u);
    guid[12] = 0x00u;
    guid[13] = 0x00u;
    guid[14] = 0x01u;
    guid[15] = 0xC1u;
    if (!plcdr_write_param(buf, sizeof(buf), off, pid::PARTICIPANT_GUID, guid, 16u)) return false;

    const Octet pv[2] = {PROTOCOLVERSION_2_5.major, PROTOCOLVERSION_2_5.minor};
    if (!plcdr_write_param(buf, sizeof(buf), off, pid::PROTOCOL_VERSION, pv, 2u)) return false;
    if (!plcdr_write_param(buf, sizeof(buf), off, pid::VENDOR_ID, VENDORID_THIS_IMPL.vendorId, 2u)) return false;

    const uint32_t lease[2] = {lease_seconds, 0u};
    if (!plcdr_write_param(buf, sizeof(buf), off, pid::PARTICIPANT_LEASE_DURATION, lease, 8u)) return false;
    if (!plcdr_write_param(buf, sizeof(buf), off, pid::METATRAFFIC_UNICAST_LOCATOR, &metatraffic_unicast, 24u)) return false;
    if (!plcdr_write_sentinel(buf, sizeof(buf), off)) return false;

    out.assign(buf, buf + off);
    return true;
}

bool build_publication_payload(const DiscoveredWriterData& wd, std::vector<Octet>& out) {
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

bool build_subscription_payload(const DiscoveredReaderData& rd, std::vector<Octet>& out) {
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

size_t count_sedp_data_announcements(const CaptureTransport& tx) {
    struct CountCap {
        size_t announces = 0u;
    } cap{};

    ParseOps ops{};
    ops.on_data = [](const ParseContext& ctx, const ParsedData& d) noexcept {
        auto* c = static_cast<CountCap*>(ctx.user_data);
        const bool pub = d.writerId == entity_id::SEDP_BUILTIN_PUBLICATIONS_ANNOUNCER &&
                         d.readerId == entity_id::SEDP_BUILTIN_PUBLICATIONS_DETECTOR;
        const bool sub = d.writerId == entity_id::SEDP_BUILTIN_SUBSCRIPTIONS_ANNOUNCER &&
                         d.readerId == entity_id::SEDP_BUILTIN_SUBSCRIPTIONS_DETECTOR;
        if (pub || sub) {
            c->announces++;
        }
    };

    for (const auto& packet : tx.packets) {
        (void)parse_message(packet.bytes.data(),
                            static_cast<uint32_t>(packet.bytes.size()),
                            GuidPrefix{},
                            ops,
                            &cap);
    }
    return cap.announces;
}

} // namespace

static bool tc_part_8_5_1_001(std::string& err) {
    CaptureTransport tx{};
    RTPSParticipant p(0u, 1u, tx);

    p.announce();
    CHECK(tx.packets.empty(), "announce should be gated until participant has local addresses");

    CHECK(p.add_ipv4_address(0xC0A8010Au), "add_ipv4_address should succeed");
    CHECK(p.has_addresses(), "participant should report configured addresses");

    p.announce();
    CHECK(tx.packets.size() == 1u, "announce should send one SPDP packet once addresses exist");
    CHECK(tx.packets[0].multicast, "SPDP announce should use multicast transport path");
    CHECK(tx.packets[0].dest == spdp_multicast_locator_v4(0u),
          "SPDP announce should target the well-known multicast locator");

    struct DataCap {
        int calls = 0;
        ParsedData data{};
    } cap{};
    ParseOps ops{};
    ops.on_data = [](const ParseContext& ctx, const ParsedData& d) noexcept {
        auto* c = static_cast<DataCap*>(ctx.user_data);
        c->calls++;
        c->data = d;
    };
    CHECK(parse_message(tx.packets[0].bytes.data(),
                        static_cast<uint32_t>(tx.packets[0].bytes.size()),
                        GuidPrefix{},
                        ops,
                        &cap) == ParseResult::OK,
          "generated SPDP announce packet should parse");
    CHECK(cap.calls == 1, "SPDP announce should include a DATA submessage");
    CHECK(cap.data.writerId == entity_id::SPDP_BUILTIN_PARTICIPANT_WRITER,
          "SPDP announce DATA writerId mismatch");
    CHECK(cap.data.readerId == entity_id::SPDP_BUILTIN_PARTICIPANT_READER,
          "SPDP announce DATA readerId mismatch");
    return true;
}

static bool tc_part_8_5_1_002(std::string& err) {
    CaptureTransport tx{};
    RTPSParticipant p(0u, 1u, tx);
    CHECK(p.add_ipv4_address(0xC0A8010Au), "local address setup should succeed");

    WriterConfig wcfg{};
    wcfg.entityId = make_entity(0x10u, 0x20u, 0x30u, 0xC2u);
    wcfg.topicName.assign("writer_topic");
    wcfg.typeName.assign("pkg::msg::WriterType");
    wcfg.reliability = ReliabilityKind::RELIABLE;
    wcfg.durability = DurabilityKind::VOLATILE;

    ReaderConfig rcfg{};
    rcfg.entityId = make_entity(0x10u, 0x20u, 0x31u, 0xC7u);
    rcfg.topicName.assign("reader_topic");
    rcfg.typeName.assign("pkg::msg::ReaderType");
    rcfg.reliability = ReliabilityKind::RELIABLE;
    rcfg.durability = DurabilityKind::VOLATILE;

    auto* local_writer = p.create_writer<1, 2>(wcfg);
    auto* local_reader = p.create_reader(rcfg, nullptr, nullptr);
    CHECK(local_writer != nullptr, "create_writer should succeed");
    CHECK(local_reader != nullptr, "create_reader should succeed");
    CHECK(local_writer->matched_reader_count() == 0u,
          "local writer should start unmatched in this test setup");
    CHECK(local_reader->matched_writer_count() == 0u,
          "local reader should start unmatched in this test setup");

    const GuidPrefix remote_prefix = make_prefix(0x91u, 0x92u, 0x93u);
    const Locator remote_meta = make_udpv4_locator(0xC0A80164u, 7410u);

    std::vector<Octet> spdp_payload{};
    CHECK(build_spdp_payload(remote_prefix, 1u, remote_meta, spdp_payload),
          "SPDP payload build failed");
    g_participant_test_now_ms = 100u;
    CHECK(p.spdp_reader().on_data(remote_prefix,
                                  spdp_payload.data(),
                                  static_cast<uint32_t>(spdp_payload.size()),
                                  g_participant_test_now_ms),
          "participant should accept SPDP participant announcement");
    {
        const size_t announces = count_sedp_data_announcements(tx);
        if (announces != 2u) {
            err = "new peer discovery SEDP announce count mismatch: got " +
                  std::to_string(announces) + " expected 2";
            return false;
        }
    }
    CHECK(!tx.packets[0].multicast && !tx.packets[1].multicast,
          "SEDP announcements should be unicast to peer metatraffic locator");
    CHECK(tx.packets[0].dest == remote_meta && tx.packets[1].dest == remote_meta,
          "SEDP announcement destination should be peer metatraffic locator");

    g_participant_test_now_ms = 200u;
    CHECK(p.spdp_reader().on_data(remote_prefix,
                                  spdp_payload.data(),
                                  static_cast<uint32_t>(spdp_payload.size()),
                                  g_participant_test_now_ms),
          "participant should accept SPDP refresh");
    {
        const size_t announces = count_sedp_data_announcements(tx);
        if (announces != 2u) {
            err = "SPDP refresh SEDP announce count mismatch: got " +
                  std::to_string(announces) + " expected 2";
            return false;
        }
    }

    const Guid remote_writer_guid =
        make_guid(0x91u, 0x92u, 0x93u, make_entity(0x21u, 0x22u, 0x23u, 0xC2u));
    const Guid remote_reader_guid =
        make_guid(0x91u, 0x92u, 0x93u, make_entity(0x24u, 0x25u, 0x26u, 0xC7u));

    DiscoveredWriterData wd{};
    wd.writerGuid = remote_writer_guid;
    wd.topicName.assign("reader_topic");
    wd.typeName.assign("pkg::msg::ReaderType");
    wd.reliabilityKind = ReliabilityKind::RELIABLE;
    wd.durabilityKind = DurabilityKind::VOLATILE;
    wd.unicastLocatorCount = 1u;
    wd.unicastLocatorList[0] = make_udpv4_locator(0xC0A80166u, 7411u);

    DiscoveredReaderData rd{};
    rd.readerGuid = remote_reader_guid;
    rd.topicName.assign("writer_topic");
    rd.typeName.assign("pkg::msg::WriterType");
    rd.reliabilityKind = ReliabilityKind::RELIABLE;
    rd.durabilityKind = DurabilityKind::VOLATILE;
    rd.unicastLocatorCount = 1u;
    rd.unicastLocatorList[0] = make_udpv4_locator(0xC0A80167u, 7412u);

    p.match_table().add_writer(wd);
    p.match_table().add_reader(rd);

    CHECK(local_writer->matched_reader_count() == 1u,
          "local writer should match remote reader after SEDP discovery");
    CHECK(local_reader->matched_writer_count() == 1u,
          "local reader should match remote writer after SEDP discovery");

    p.tick(1301u); // lease expiry for refresh at t=200 with leaseDuration=1s.
    CHECK(local_writer->matched_reader_count() == 0u,
          "expired participant should be removed and writer matches dissolved");
    CHECK(local_reader->matched_writer_count() == 0u,
          "expired participant should be removed and reader matches dissolved");

    g_participant_test_now_ms = 1500u;
    CHECK(p.spdp_reader().on_data(remote_prefix,
                                  spdp_payload.data(),
                                  static_cast<uint32_t>(spdp_payload.size()),
                                  g_participant_test_now_ms),
          "participant should accept rediscovery after lease expiry");
    {
        const size_t announces = count_sedp_data_announcements(tx);
        if (announces != 6u) {
            err = "rediscovery SEDP announce count mismatch: got " +
                  std::to_string(announces) + " expected 6";
            return false;
        }
    }
    return true;
}

static bool tc_part_8_5_1_003(std::string& err) {
    CaptureTransport tx{};
    const uint8_t domain = 3u;
    const uint8_t participant_id = 7u;
    RTPSParticipant p(domain, participant_id, tx);

    CHECK(p.participant_guid().entityId == entity_id::PARTICIPANT,
          "participant GUID entityId should be PARTICIPANT");
    CHECK(p.guid_prefix().value[11] == participant_id,
          "participant GUID prefix should come from platform prefix hook");
    CHECK(!p.add_ipv4_address(0u), "unspecified IPv4 address should be rejected");

    const uint8_t ipv6[16] = {
        0x20u, 0x01u, 0x0Du, 0xB8u, 0x00u, 0x00u, 0x00u, 0x00u,
        0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x42u
    };
    CHECK(p.add_ipv6_address(ipv6), "add_ipv6_address should accept global IPv6");
    CHECK(p.has_addresses(), "participant should have configured addresses after IPv6 add");

    p.announce();
    CHECK(tx.packets.size() == 1u, "announce should send after IPv6 address configured");
    CHECK(tx.packets[0].dest == spdp_multicast_locator_v6(domain),
          "IPv6-only participant should use IPv6 SPDP multicast locator");

    CHECK(p.add_ipv4_address(0xC0A80177u), "add_ipv4_address should succeed");
    tx.packets.clear();
    p.announce();
    CHECK(tx.packets.size() == 1u, "announce should still send with dual-stack addresses");
    CHECK(tx.packets[0].dest == spdp_multicast_locator_v4(domain),
          "dual-stack participant should prefer IPv4 SPDP multicast locator");

    CHECK(p.remove_ipv4_address(0xC0A80177u), "remove_ipv4_address should remove configured address");
    tx.packets.clear();
    p.announce();
    CHECK(tx.packets.size() == 1u, "announce should continue with remaining IPv6 address");
    CHECK(tx.packets[0].dest == spdp_multicast_locator_v6(domain),
          "removing IPv4 should fall back to IPv6 SPDP multicast locator");

    CHECK(p.remove_ipv6_address(ipv6), "remove_ipv6_address should remove configured IPv6");
    CHECK(!p.has_addresses(), "participant should report no addresses after removing all");
    tx.packets.clear();
    p.announce();
    CHECK(tx.packets.empty(), "announce should be gated once all addresses are removed");
    return true;
}

static bool tc_part_8_3_4_001(std::string& err) {
    CaptureTransport tx{};
    RTPSParticipant p(0u, 1u, tx);
    CHECK(p.add_ipv4_address(0xC0A8010Au), "local address setup should succeed");

    ReaderChangeObs obs1{};
    ReaderChangeObs obs2{};
    ReaderConfig r1{};
    r1.entityId = make_entity(0x20u, 0x20u, 0x01u, 0xC7u);
    r1.topicName.assign("r1");
    r1.typeName.assign("T");
    r1.reliability = ReliabilityKind::BEST_EFFORT;
    ReaderConfig r2{};
    r2.entityId = make_entity(0x20u, 0x20u, 0x02u, 0xC7u);
    r2.topicName.assign("r2");
    r2.typeName.assign("T");
    r2.reliability = ReliabilityKind::BEST_EFFORT;

    auto* rr1 = p.create_reader(r1, on_reader_change, &obs1);
    auto* rr2 = p.create_reader(r2, on_reader_change, &obs2);
    CHECK(rr1 != nullptr && rr2 != nullptr, "create_reader should succeed");

    const GuidPrefix remote_prefix = make_prefix(0x99u, 0x10u, 0x10u);
    const EntityId remote_writer_eid = make_entity(0x11u, 0x22u, 0x33u, 0xC2u);
    const Guid remote_writer_guid{remote_prefix, remote_writer_eid};
    const Locator remote_loc = make_udpv4_locator(0xC0A80166u, 7411u);
    CHECK(rr1->matched_writer_add(remote_writer_guid, &remote_loc, 1u),
          "reader1 should match remote writer for callback delivery");
    CHECK(rr2->matched_writer_add(remote_writer_guid, &remote_loc, 1u),
          "reader2 should match remote writer for callback delivery");

    const std::vector<Octet> payload = {0x00u, 0x01u, 0x00u, 0x00u, 0xA5u};

    std::vector<Octet> msg_broadcast{};
    CHECK(build_data_message(remote_prefix,
                             entity_id::UNKNOWN,
                             remote_writer_eid,
                             payload,
                             msg_broadcast,
                             SequenceNumber::from_int64(1)),
          "broadcast DATA message build failed");
    CHECK(p.parse_received(msg_broadcast.data(), static_cast<uint32_t>(msg_broadcast.size())) == ParseResult::OK,
          "participant should parse broadcast DATA");
    if (!(obs1.calls == 1 && obs2.calls == 1)) {
        err = "broadcast DATA dispatch mismatch: obs1=" + std::to_string(obs1.calls) +
              " obs2=" + std::to_string(obs2.calls);
        return false;
    }

    std::vector<Octet> msg_targeted{};
    CHECK(build_data_message(remote_prefix,
                             r1.entityId,
                             remote_writer_eid,
                             payload,
                             msg_targeted,
                             SequenceNumber::from_int64(2)),
          "targeted DATA message build failed");
    CHECK(p.parse_received(msg_targeted.data(), static_cast<uint32_t>(msg_targeted.size())) == ParseResult::OK,
          "participant should parse targeted DATA");
    if (!(obs1.calls == 2 && obs2.calls == 1)) {
        err = "targeted DATA dispatch mismatch: obs1=" + std::to_string(obs1.calls) +
              " obs2=" + std::to_string(obs2.calls);
        return false;
    }
    return true;
}

static bool tc_part_8_3_4_002(std::string& err) {
    CaptureTransport tx{};
    RTPSParticipant p(0u, 1u, tx);
    CHECK(p.add_ipv4_address(0xC0A8010Au), "local address setup should succeed");

    WriterConfig wcfg{};
    wcfg.entityId = make_entity(0x31u, 0x32u, 0x33u, 0xC2u);
    wcfg.topicName.assign("writer");
    wcfg.typeName.assign("T");
    wcfg.reliability = ReliabilityKind::RELIABLE;
    wcfg.durability = DurabilityKind::VOLATILE;
    wcfg.nackDelayMs = 0u;
    wcfg.hbPeriodMs = 100000u;
    auto* w = p.create_writer<2, 2>(wcfg);
    CHECK(w != nullptr, "create_writer should succeed");

    const Guid remote_reader = make_guid(0xA1u, 0xA2u, 0xA3u, make_entity(0x44u, 0x55u, 0x66u, 0xC7u));
    const Locator remote_loc = make_udpv4_locator(0xC0A80155u, 7412u);
    CHECK(w->matched_reader_add(remote_reader, &remote_loc, 1u, false),
          "matched_reader_add should succeed");

    const Octet user_payload[1] = {0x42u};
    CHECK(w->write_reliable(ChangeKind::ALIVE, user_payload, 1u), "write_reliable should succeed");
    tx.packets.clear();

    SequenceNumberSet sn_state{};
    sn_state.clear();
    sn_state.bitmapBase = SequenceNumber::from_int64(2);
    sn_state.numBits = 0u;

    std::vector<Octet> ack_msg{};
    CHECK(build_acknack_message(remote_reader.prefix,
                                remote_reader.entityId,
                                wcfg.entityId,
                                sn_state,
                                1,
                                true,
                                ack_msg),
          "ACKNACK message build failed");
    CHECK(p.parse_received(ack_msg.data(), static_cast<uint32_t>(ack_msg.size())) == ParseResult::OK,
          "participant should parse ACKNACK");

    auto* proxy = w->matched_reader_lookup(remote_reader);
    CHECK(proxy != nullptr, "matched reader proxy should exist");
    CHECK(proxy->highestAckedSN.to_int64() == 1,
          "ACKNACK should be routed to local writer and update highestAckedSN");

    const Octet user_payload2[1] = {0x43u};
    CHECK(w->write_reliable(ChangeKind::ALIVE, user_payload2, 1u),
          "second reliable write should succeed");
    tx.packets.clear();

    FragmentNumberSet frags{};
    frags.clear();
    CHECK(frags.add(1u), "fragment set add should succeed");

    std::vector<Octet> nack_frag_msg{};
    CHECK(build_nack_frag_message(remote_reader.prefix,
                                  remote_reader.entityId,
                                  wcfg.entityId,
                                  SequenceNumber::from_int64(2),
                                  frags,
                                  1,
                                  nack_frag_msg),
          "NACK_FRAG message build failed");

    g_participant_test_now_ms = 100u;
    CHECK(p.parse_received(nack_frag_msg.data(), static_cast<uint32_t>(nack_frag_msg.size())) == ParseResult::OK,
          "participant should parse NACK_FRAG");
    p.tick(100u);

    struct PacketCounts {
        int data_calls = 0;
        int hb_calls = 0;
    };
    PacketCounts totals{};
    for (const auto& pkt : tx.packets) {
        PacketCounts pkt_counts{};
        ParseOps ops{};
        ops.on_data = [](const ParseContext& ctx, const ParsedData& d) noexcept {
            auto* counts = static_cast<PacketCounts*>(ctx.user_data);
            counts->data_calls++;
        };
        ops.on_heartbeat = [](const ParseContext& ctx, const ParsedHeartbeat& hb) noexcept {
            auto* counts = static_cast<PacketCounts*>(ctx.user_data);
            counts->hb_calls++;
        };

        CHECK(parse_message(pkt.bytes.data(),
                            static_cast<uint32_t>(pkt.bytes.size()),
                            GuidPrefix{},
                            ops,
                            &pkt_counts) == ParseResult::OK,
              "writer retransmit packet should parse");
        totals.data_calls += pkt_counts.data_calls;
        totals.hb_calls += pkt_counts.hb_calls;
    }
    if (totals.data_calls < 1) {
        err = "NACK_FRAG retransmit DATA count=" + std::to_string(totals.data_calls) +
              " heartbeat count=" + std::to_string(totals.hb_calls) +
              " packets=" + std::to_string(tx.packets.size());
        return false;
    }
    CHECK(totals.hb_calls >= 1, "NACK_FRAG route should also drive directed HEARTBEAT after retransmit");
    return true;
}

static bool tc_part_8_5_1_004(std::string& err) {
    CaptureTransport tx{};
    RTPSParticipant p(0u, 1u, tx);
    CHECK(p.add_ipv4_address(0xC0A8010Au), "local address setup should succeed");

    const GuidPrefix remote_prefix = make_prefix(0x77u, 0x88u, 0x99u);
    const Locator remote_meta = make_udpv4_locator(0xC0A80164u, 7412u);
    std::vector<Octet> spdp_payload{};
    CHECK(build_spdp_payload(remote_prefix, 20u, remote_meta, spdp_payload),
          "SPDP payload build failed");

    g_participant_test_now_ms = 100u;
    CHECK(p.spdp_reader().on_data(remote_prefix,
                                  spdp_payload.data(),
                                  static_cast<uint32_t>(spdp_payload.size()),
                                  g_participant_test_now_ms),
          "participant should accept SPDP announcement");

    bool found_unicast_spdp = false;
    Locator spdp_dest = LOCATOR_INVALID;
    for (const auto& pkt : tx.packets) {
        struct Cap { bool spdp = false; } cap{};
        ParseOps ops{};
        ops.on_data = [](const ParseContext& ctx, const ParsedData& d) noexcept {
            auto* c = static_cast<Cap*>(ctx.user_data);
            if (d.writerId == entity_id::SPDP_BUILTIN_PARTICIPANT_WRITER) c->spdp = true;
        };
        (void)parse_message(pkt.bytes.data(),
                            static_cast<uint32_t>(pkt.bytes.size()),
                            GuidPrefix{}, ops, &cap);
        if (cap.spdp && !pkt.multicast) {
            found_unicast_spdp = true;
            spdp_dest = pkt.dest;
            break;
        }
    }
    CHECK(found_unicast_spdp,
          "on new peer discovery, participant must send unicast SPDP to peer metatraffic locator");
    CHECK(spdp_dest == remote_meta,
          "unicast SPDP destination must equal the discovered peer's metatraffic unicast locator");

    // A second SPDP refresh must not retrigger the unicast SPDP (sedp_announced guards it).
    const size_t packets_after_first = tx.packets.size();
    g_participant_test_now_ms = 200u;
    CHECK(p.spdp_reader().on_data(remote_prefix,
                                  spdp_payload.data(),
                                  static_cast<uint32_t>(spdp_payload.size()),
                                  g_participant_test_now_ms),
          "SPDP refresh should be accepted");
    CHECK(tx.packets.size() == packets_after_first,
          "SPDP refresh must not resend unicast SPDP (sedp_announced already set)");
    return true;
}

void append_participant_tests_8_5_1(std::vector<TestCase>& tests) {
    tests.push_back({"TC-PART-8.5.1-001",
                     "§8.5.1/§8.5.3.3: participant announce path is gated on configured local locators and sends SPDP DATA to the well-known multicast locator.",
                     tc_part_8_5_1_001});
    tests.push_back({"TC-PART-8.5.1-002",
                     "§8.5.1/§8.5.3.2/§8.5.5.1: participant SPDP callbacks avoid duplicate SEDP re-announce on refresh, and endpoint-match wiring/unwiring tracks lease-expiry participant removal.",
                     tc_part_8_5_1_002});
    tests.push_back({"TC-PART-8.5.1-003",
                     "§8.5.1.2/§9.6.1: participant GUID is initialized as a participant EntityId, and local address management drives IPv4/IPv6 SPDP multicast locator selection and announce gating.",
                     tc_part_8_5_1_003});
    tests.push_back({"TC-PART-8.5.1-004",
                     "§8.5.3.2 directed SPDP: on new peer discovery the participant unicasts its own SPDP DATA to the peer's metatraffic locator and suppresses retransmit on subsequent refreshes.",
                     tc_part_8_5_1_004});
    tests.push_back({"TC-PART-8.3.4-001",
                     "§8.3.4: parse_received dispatches DATA to local readers by readerId, and readerId=UNKNOWN fans out to all local readers.",
                     tc_part_8_3_4_001});
    tests.push_back({"TC-PART-8.3.4-002",
                     "§8.3.4/§8.4.3/§8.4.4: parse_received routes ACKNACK and NACK_FRAG to the addressed local writer instance for reliability state updates and retransmit workflow.",
                     tc_part_8_3_4_002});
}

} // namespace spec_test
