#include "spec_test_common.hpp"
#include "sedp_peer_driver.hpp"

#include "rosette/rtps/sedp.hpp"
#include "rosette/rtps/transport.hpp"

namespace spec_test {

#define CHECK(COND, MSG) do { if (!(COND)) { err = (MSG); return false; } } while (0)

namespace {

struct FakeTransport : ITransport {
    bool send_unicast(const BufRef&, const Locator&) noexcept override { return true; }
    bool send_multicast(const BufRef&, const Locator&) noexcept override { return false; }
};

struct Obs {
    int match_calls = 0;
};

static void on_match(const DiscoveredWriterData&,
                     const DiscoveredReaderData&,
                     void* user) noexcept {
    auto* o = static_cast<Obs*>(user);
    o->match_calls++;
}

static Guid make_guid(Octet p0, Octet p1, Octet p2, const EntityId& eid) {
    Guid g{};
    g.prefix.value[0] = p0;
    g.prefix.value[1] = p1;
    g.prefix.value[2] = p2;
    g.entityId = eid;
    return g;
}

static DiscoveredReaderData make_local_reader(const Guid& g) {
    DiscoveredReaderData rd{};
    rd.readerGuid = g;
    rd.topicName.assign("imu");
    rd.typeName.assign("sensor_msgs::msg::Imu");
    rd.reliabilityKind = ReliabilityKind::RELIABLE;
    rd.durabilityKind = DurabilityKind::VOLATILE;
    return rd;
}

static bool build_writer_payload(bool include_guid,
                                 bool include_unknown_pid,
                                 std::vector<Octet>& out) {
    Octet buf[512] = {};
    uint32_t off = 0;

    const uint16_t rep_id = 0x0003u;
    const uint16_t rep_opt = 0x0000u;
    memcpy(buf + off, &rep_id, 2u); off += 2u;
    memcpy(buf + off, &rep_opt, 2u); off += 2u;

    if (include_unknown_pid) {
        const Octet junk[3] = {0xFAu, 0xFBu, 0xFCu};
        if (!plcdr_write_param(buf, sizeof(buf), off, 0x7777u, junk, 3u)) return false;
    }

    if (include_guid) {
        const Guid wg = make_guid(0x91u, 0x92u, 0x93u, make_entity(9u, 9u, 9u, 0xC2u));
        Octet g[16] = {};
        memcpy(g, wg.prefix.value, 12u);
        memcpy(g + 12u, &wg.entityId, 4u);
        if (!plcdr_write_param(buf, sizeof(buf), off, pid::ENDPOINT_GUID, g, 16u)) return false;
    }

    if (!plcdr_write_string(buf, sizeof(buf), off, pid::TOPIC_NAME, "imu")) return false;
    if (!plcdr_write_string(buf, sizeof(buf), off, pid::TYPE_NAME, "sensor_msgs::msg::Imu")) return false;
    {
        const uint32_t rel = static_cast<uint32_t>(ReliabilityKind::RELIABLE);
        if (!plcdr_write_param(buf, sizeof(buf), off, pid::RELIABILITY, &rel, 4u)) return false;
    }
    {
        const uint32_t dur = static_cast<uint32_t>(DurabilityKind::VOLATILE);
        if (!plcdr_write_param(buf, sizeof(buf), off, pid::DURABILITY, &dur, 4u)) return false;
    }
    if (!plcdr_write_sentinel(buf, sizeof(buf), off)) return false;
    out.assign(buf, buf + off);
    return true;
}

} // namespace

static bool tc_sedp_9_6_3_2_001(std::string& err) {
    Obs obs{};
    FakeTransport tx{};
    EndpointMatchTable table(on_match, nullptr, &obs);
    SedpReader reader(GuidPrefix{}, tx, table);

    const Guid local_reader = make_guid(0x10u, 0x11u, 0x12u, make_entity(1u, 1u, 1u, 0xC7u));
    table.add_reader(make_local_reader(local_reader));

    std::vector<Octet> payload{};
    CHECK(build_writer_payload(true, true, payload), "payload build failed");

    const SedpPeer peer{GuidPrefix{}, make_udpv4_locator(0xC0A80101u, 7410u)};
    peer.discover(reader);
    peer.announce(reader, entity_id::SEDP_BUILTIN_PUBLICATIONS_ANNOUNCER, 1, payload);
    CHECK(obs.match_calls == 1, "unknown PIDs should be skipped without breaking required field parse");
    return true;
}

static bool tc_sedp_9_6_3_2_002(std::string& err) {
    Obs obs{};
    FakeTransport tx{};
    EndpointMatchTable table(on_match, nullptr, &obs);
    SedpReader reader(GuidPrefix{}, tx, table);

    const Guid local_reader = make_guid(0x20u, 0x21u, 0x22u, make_entity(2u, 2u, 2u, 0xC7u));
    table.add_reader(make_local_reader(local_reader));

    std::vector<Octet> payload{};
    CHECK(build_writer_payload(false, false, payload), "payload build failed");

    const SedpPeer peer{GuidPrefix{}, make_udpv4_locator(0xC0A80101u, 7410u)};
    peer.discover(reader);
    peer.announce(reader, entity_id::SEDP_BUILTIN_PUBLICATIONS_ANNOUNCER, 1, payload);
    CHECK(obs.match_calls == 0, "missing PID_ENDPOINT_GUID should reject discovered endpoint");
    return true;
}

void append_sedp_tests_9_6_3_2(std::vector<TestCase>& tests) {
    tests.push_back({"TC-SEDP-9.6.3.2-001",
                     "§9.6.3.2: SEDP PL_CDR endpoint data parsing skips unknown PID entries while preserving mandatory-field processing.",
                     tc_sedp_9_6_3_2_001});
    tests.push_back({"TC-SEDP-9.6.3.2-002",
                     "§9.6.3.2: endpoint discovery payload without PID_ENDPOINT_GUID is rejected.",
                     tc_sedp_9_6_3_2_002});
}

} // namespace spec_test
