#include "spec_test_common.hpp"

#include "rtps/spdp.hpp"
#include "rtps/transport.hpp"

namespace spec_test {

#define CHECK(COND, MSG) do { if (!(COND)) { err = (MSG); return false; } } while (0)

namespace {

struct FakeTransport : ITransport {
    int unicast_calls = 0;
    int multicast_calls = 0;
    std::vector<Octet> last_msg{};
    Locator last_dest = LOCATOR_INVALID;

    bool send_unicast(const BufRef& buf, const Locator& dest) noexcept override {
        unicast_calls++;
        last_dest = dest;
        last_msg.assign(buf.data, buf.data + buf.size);
        return true;
    }
    bool send_multicast(const BufRef& buf, const Locator& dest) noexcept override {
        multicast_calls++;
        last_dest = dest;
        last_msg.assign(buf.data, buf.data + buf.size);
        return true;
    }
};

struct DataCap {
    int calls = 0;
    ParsedData data{};
    std::vector<Octet> payload{};
};

static void on_data_capture(const ParseContext& ctx, const ParsedData& d) noexcept {
    auto* cap = static_cast<DataCap*>(ctx.user_data);
    cap->calls++;
    cap->data = d;
    cap->payload.assign(d.serializedPayload, d.serializedPayload + d.serializedPayloadSize);
}

static bool payload_contains_pid(const std::vector<Octet>& payload,
                                 uint16_t                  pid_value) noexcept {
    if (payload.size() < 8u) return false;
    uint32_t off = 4u; // RepresentationHeader
    while (off + 4u <= payload.size()) {
        const uint16_t pid = static_cast<uint16_t>(payload[off]) |
                             static_cast<uint16_t>(payload[off + 1u] << 8u);
        const uint16_t len = static_cast<uint16_t>(payload[off + 2u]) |
                             static_cast<uint16_t>(payload[off + 3u] << 8u);
        if (pid == pid::SENTINEL) return false;
        if (pid == pid_value) return true;
        const uint32_t padded = static_cast<uint32_t>((len + 3u) & ~3u);
        off += 4u + padded;
    }
    return false;
}

struct PeerObs {
    int new_calls = 0;
    int removed_calls = 0;
    GuidPrefix last_new = {};
};

static void on_new_peer(const PeerEntry& p, void* user) noexcept {
    auto* obs = static_cast<PeerObs*>(user);
    obs->new_calls++;
    obs->last_new = p.data.guidPrefix;
}

static void on_peer_removed(const PeerEntry&, void* user) noexcept {
    auto* obs = static_cast<PeerObs*>(user);
    obs->removed_calls++;
}

static GuidPrefix make_prefix(Octet a, Octet b, Octet c) {
    GuidPrefix gp{};
    gp.value[0] = a;
    gp.value[1] = b;
    gp.value[2] = c;
    return gp;
}

static bool build_payload_for_peer(const GuidPrefix& guid_prefix,
                                   uint32_t lease_seconds,
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

    const Locator meta = make_udpv4_locator(0xC0A80164u, 7410u);
    if (!plcdr_write_param(buf, sizeof(buf), off, pid::METATRAFFIC_UNICAST_LOCATOR, &meta, 24u)) return false;

    if (!plcdr_write_sentinel(buf, sizeof(buf), off)) return false;
    out.assign(buf, buf + off);
    return true;
}

} // namespace

static bool tc_spdp_8_5_3_001(std::string& err) {
    SPDPdiscoveredParticipantData local{};
    local.guidPrefix = make_prefix(0x31u, 0x32u, 0x33u);
    local.protocolVersion = PROTOCOLVERSION_2_5;
    local.vendorId = VENDORID_THIS_IMPL;
    local.leaseDuration = Duration::from_ms(20000u);
    local.availableBuiltinEndpoints =
        builtin_endpoint::DISC_BUILTIN_ENDPOINT_PARTICIPANT_ANNOUNCER |
        builtin_endpoint::DISC_BUILTIN_ENDPOINT_PARTICIPANT_DETECTOR;

    local.metatrafficUnicastLocatorCount = 1u;
    local.metatrafficUnicastLocatorList[0] = make_udpv4_locator(0xC0A80132u, 7410u);
    local.defaultUnicastLocatorCount = 1u;
    local.defaultUnicastLocatorList[0] = make_udpv4_locator(0xC0A80132u, 7411u);

    FakeTransport tx{};
    const Locator dest = spdp_multicast_locator_v4(0u);
    SpdpWriter w(local, tx, dest, 1000u);
    w.announce();

    CHECK(tx.multicast_calls == 1, "SPDP writer should send one multicast DATA announcement");
    CHECK(!tx.last_msg.empty(), "SPDP writer should produce a message");

    DataCap cap{};
    ParseOps ops{};
    ops.on_data = on_data_capture;
    CHECK(parse_message(tx.last_msg.data(),
                        static_cast<uint32_t>(tx.last_msg.size()),
                        GuidPrefix{},
                        ops,
                        &cap) == ParseResult::OK,
          "SPDP announcement message should be parseable");
    CHECK(cap.calls == 1, "SPDP announcement should contain one DATA submessage");
    CHECK(cap.data.writerId == entity_id::SPDP_BUILTIN_PARTICIPANT_WRITER,
          "SPDP DATA writerId should be SPDP builtin participant writer");
    CHECK(cap.data.readerId == entity_id::SPDP_BUILTIN_PARTICIPANT_READER,
          "SPDP DATA readerId should be SPDP builtin participant reader");
    CHECK(cap.payload.size() >= 8u, "SPDP serialized payload should include PL_CDR header and params");
    CHECK(cap.payload[0] == 0x00u && cap.payload[1] == 0x03u &&
          cap.payload[2] == 0x00u && cap.payload[3] == 0x00u,
          "SPDP payload should begin with PL_CDR_LE RepresentationHeader");
    CHECK(payload_contains_pid(cap.payload, pid::DEFAULT_UNICAST_LOCATOR),
          "SPDP payload should include PID_DEFAULT_UNICAST_LOCATOR (0x0031)");
    return true;
}

static bool tc_spdp_8_5_3_002(std::string& err) {
    PeerObs obs{};
    SpdpReader r(on_new_peer, on_peer_removed, &obs);
    const GuidPrefix gp = make_prefix(0x41u, 0x42u, 0x43u);

    std::vector<Octet> payload{};
    CHECK(build_payload_for_peer(gp, 1u, payload), "payload construction failed");

    CHECK(r.on_data(gp, payload.data(), static_cast<uint32_t>(payload.size()), 0u),
          "first participant DATA should be accepted");
    CHECK(obs.new_calls == 1, "first sighting should trigger new peer callback");
    CHECK(r.peer_count() == 1u, "expected one live peer");

    r.mark_sedp_announced(gp);

    CHECK(r.on_data(gp, payload.data(), static_cast<uint32_t>(payload.size()), 200u),
          "refresh DATA should be accepted");
    CHECK(obs.new_calls == 1, "refresh should not retrigger new peer callback");
    CHECK(r.peer_count() == 1u, "refresh should keep exactly one live peer");

    bool sedp_announced_seen = false;
    r.for_each_peer([&](const PeerEntry& peer) {
        if (peer.data.guidPrefix == gp) {
            sedp_announced_seen = peer.sedp_announced;
        }
    });
    CHECK(sedp_announced_seen, "sedp_announced flag should persist across refresh");

    r.tick(1300u);
    CHECK(obs.removed_calls == 1, "lease expiry should trigger peer removed callback");
    CHECK(r.peer_count() == 0u, "expired peer should be removed");

    CHECK(r.on_data(gp, payload.data(), static_cast<uint32_t>(payload.size()), 1500u),
          "rediscovery after expiry should be accepted");
    CHECK(obs.new_calls == 2, "rediscovery after removal should trigger new callback again");

    bool sedp_reset = false;
    r.for_each_peer([&](const PeerEntry& peer) {
        if (peer.data.guidPrefix == gp) {
            sedp_reset = !peer.sedp_announced;
        }
    });
    CHECK(sedp_reset, "rediscovered peer should reset sedp_announced until re-marked");
    return true;
}

static bool tc_spdp_8_5_3_003(std::string& err) {
    PeerObs obs{};
    SpdpReader r(on_new_peer, on_peer_removed, &obs);

    std::vector<std::vector<Octet>> payloads(RTPS_MAX_PEERS);
    for (uint8_t i = 0; i < RTPS_MAX_PEERS; ++i) {
        GuidPrefix gp = make_prefix(static_cast<Octet>(0x60u + i), 0x01u, 0x02u);
        CHECK(build_payload_for_peer(gp, 2u, payloads[i]), "payload construction failed");
        CHECK(r.on_data(gp,
                        payloads[i].data(),
                        static_cast<uint32_t>(payloads[i].size()),
                        10u + i),
              "filling peer table should succeed");
    }

    CHECK(r.peer_count() == RTPS_MAX_PEERS, "peer table should be full");

    std::vector<Octet> overflow{};
    GuidPrefix extra = make_prefix(0xF1u, 0xF2u, 0xF3u);
    CHECK(build_payload_for_peer(extra, 2u, overflow), "overflow payload construction failed");
    CHECK(!r.on_data(extra, overflow.data(), static_cast<uint32_t>(overflow.size()), 99u),
          "new peer should be rejected when peer table is full");
    CHECK(r.peer_count() == RTPS_MAX_PEERS, "peer count should remain capped at RTPS_MAX_PEERS");
    return true;
}

static bool tc_spdp_8_5_3_004(std::string& err) {
    SPDPdiscoveredParticipantData local{};
    local.guidPrefix = make_prefix(0x31u, 0x32u, 0x33u);
    local.protocolVersion = PROTOCOLVERSION_2_5;
    local.vendorId = VENDORID_THIS_IMPL;
    local.leaseDuration = Duration::from_ms(20000u);
    local.availableBuiltinEndpoints =
        builtin_endpoint::DISC_BUILTIN_ENDPOINT_PARTICIPANT_ANNOUNCER |
        builtin_endpoint::DISC_BUILTIN_ENDPOINT_PARTICIPANT_DETECTOR;
    local.metatrafficUnicastLocatorCount = 1u;
    local.metatrafficUnicastLocatorList[0] = make_udpv4_locator(0xC0A80132u, 7410u);

    FakeTransport tx{};
    const Locator multicast_dest = spdp_multicast_locator_v4(0u);
    SpdpWriter w(local, tx, multicast_dest, 1000u);

    const Locator peer_meta = make_udpv4_locator(0xC0A80164u, 7412u);
    w.announce_unicast(peer_meta);

    CHECK(tx.unicast_calls == 1,
          "announce_unicast should invoke send_unicast once");
    CHECK(tx.multicast_calls == 0,
          "announce_unicast must not invoke send_multicast");
    CHECK(tx.last_dest == peer_meta,
          "announce_unicast destination must match the supplied peer locator");

    DataCap cap{};
    ParseOps ops{};
    ops.on_data = on_data_capture;
    CHECK(parse_message(tx.last_msg.data(),
                        static_cast<uint32_t>(tx.last_msg.size()),
                        GuidPrefix{},
                        ops,
                        &cap) == ParseResult::OK,
          "unicast SPDP message should be parseable");
    CHECK(cap.calls == 1,
          "unicast SPDP message should contain exactly one DATA submessage");
    CHECK(cap.data.writerId == entity_id::SPDP_BUILTIN_PARTICIPANT_WRITER,
          "unicast SPDP DATA writerId must be SPDP_BUILTIN_PARTICIPANT_WRITER");
    CHECK(cap.data.readerId == entity_id::SPDP_BUILTIN_PARTICIPANT_READER,
          "unicast SPDP DATA readerId must be SPDP_BUILTIN_PARTICIPANT_READER");
    return true;
}

void append_spdp_tests_8_5_3(std::vector<TestCase>& tests) {
    tests.push_back({"TC-SPDP-8.5.3-001",
                     "§8.5.3.3: SPDPbuiltinParticipantWriter sends a DATA Submessage carrying local SPDPdiscoveredParticipantData PL_CDR payload.",
                     tc_spdp_8_5_3_001});
    tests.push_back({"TC-SPDP-8.5.3-002",
                     "§8.5.3.2/§8.5.3.3: SPDP reader discovers, refreshes, expires, and re-discovers a participant (restart path) with deterministic callbacks.",
                     tc_spdp_8_5_3_002});
    tests.push_back({"TC-SPDP-8.5.3-003",
                     "§8.5.3.3 implementation limit: SPDP peer table accepts up to RTPS_MAX_PEERS and rejects additional undiscovered participants safely.",
                     tc_spdp_8_5_3_003});
    tests.push_back({"TC-SPDP-8.5.3-004",
                     "§8.5.3.2 directed SPDP: announce_unicast() sends the local participant data via send_unicast to the specified peer locator with correct SPDP entity IDs.",
                     tc_spdp_8_5_3_004});
}

} // namespace spec_test
