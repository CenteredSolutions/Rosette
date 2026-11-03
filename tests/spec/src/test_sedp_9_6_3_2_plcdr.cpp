// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

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

static bool on_match(const MatchedEndpoint&, const MatchedEndpoint&, const char*,
                     void* user) noexcept {
    auto* o = static_cast<Obs*>(user);
    o->match_calls++;
    return true;
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

/// A writer announcement for the "imu" topic listing `locators` as
/// PID_UNICAST_LOCATOR parameters, in that order.
static bool build_writer_payload_with_locators(std::initializer_list<Locator> locators,
                                               std::vector<Octet>& out) {
    std::vector<Octet> head{};
    if (!build_writer_payload(true, false, head)) {
        return false;
    }

    // Drop the sentinel, append the locators, then close the list again.
    Octet buf[768] = {};
    uint32_t off = static_cast<uint32_t>(head.size()) - 4u;
    memcpy(buf, head.data(), off);
    for (const Locator& l : locators) {
        if (!plcdr_write_param(buf, sizeof(buf), off, pid::UNICAST_LOCATOR, &l, 24u)) {
            return false;
        }
    }

    if (!plcdr_write_sentinel(buf, sizeof(buf), off)) {
        return false;
    }

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
    table.set_local_prefix(local_reader.prefix);
    table.add_reader(make_local_reader(local_reader));

    std::vector<Octet> payload{};
    CHECK(build_writer_payload(true, true, payload), "payload build failed");

    const SedpPeer peer{GuidPrefix{}, make_udpv4_locator(0xC0A80101u, 7410u)};
    peer.discover(reader);
    peer.announce(reader, entity_id::SEDP_BUILTIN_PUBLICATIONS_ANNOUNCER, 1, payload);
    CHECK(obs.match_calls == 1, "unknown PIDs should be skipped without breaking required field parse");
    return true;
}

/// An unknown parameter declaring length 0xFFFD (padded 0x10000) ahead of a
/// complete endpoint announcement must end parsing: it runs past the payload.
/// A reader that wrapped the padded length to 0 would treat it as empty and
/// parse the announcement behind it, matching an endpoint the payload does not
/// legitimately describe.
static bool tc_sedp_9_6_3_2_004(std::string& err) {
    Obs                obs{};
    FakeTransport      tx{};
    EndpointMatchTable table(on_match, nullptr, &obs);
    SedpReader         reader(GuidPrefix{}, tx, table);

    const Guid local_reader = make_guid(0x10u, 0x11u, 0x12u, make_entity(1u, 1u, 1u, 0xC7u));
    table.set_local_prefix(local_reader.prefix);
    table.add_reader(make_local_reader(local_reader));

    std::vector<Octet> valid{};
    CHECK(build_writer_payload(true, false, valid), "payload build failed");
    CHECK(valid.size() > 4u, "payload must hold a representation header and a parameter list");

    // Representation header, then the oversized parameter, then the list.
    const Octet        oversized[4] = {0x77u, 0x77u, 0xFDu, 0xFFu};
    std::vector<Octet> payload(valid.size() + sizeof(oversized));
    CHECK(payload.data() != nullptr, "payload allocation failed");
    memcpy(payload.data(), valid.data(), 4u);
    memcpy(payload.data() + 4u, oversized, sizeof(oversized));
    memcpy(payload.data() + 4u + sizeof(oversized), valid.data() + 4u, valid.size() - 4u);

    const SedpPeer peer{GuidPrefix{}, make_udpv4_locator(0xC0A80101u, 7410u)};
    peer.discover(reader);
    peer.announce(reader, entity_id::SEDP_BUILTIN_PUBLICATIONS_ANNOUNCER, 1, payload);
    CHECK(obs.match_calls == 0, "a parameter longer than the payload must end parsing");
    return true;
}

static bool tc_sedp_9_6_3_2_002(std::string& err) {
    Obs obs{};
    FakeTransport tx{};
    EndpointMatchTable table(on_match, nullptr, &obs);
    SedpReader reader(GuidPrefix{}, tx, table);

    const Guid local_reader = make_guid(0x20u, 0x21u, 0x22u, make_entity(2u, 2u, 2u, 0xC7u));
    table.set_local_prefix(local_reader.prefix);
    table.add_reader(make_local_reader(local_reader));

    std::vector<Octet> payload{};
    CHECK(build_writer_payload(false, false, payload), "payload build failed");

    const SedpPeer peer{GuidPrefix{}, make_udpv4_locator(0xC0A80101u, 7410u)};
    peer.discover(reader);
    peer.announce(reader, entity_id::SEDP_BUILTIN_PUBLICATIONS_ANNOUNCER, 1, payload);
    CHECK(obs.match_calls == 0, "missing PID_ENDPOINT_GUID should reject discovered endpoint");
    return true;
}

// §8.5.4.2 / §9.6.2: an endpoint's locator list is ranked as it is decoded,
// so the reachable locator is kept even when the endpoint announces it after
// more locators than the decoded list holds: here a shared-memory locator and
// four Docker bridges come first.
static bool tc_sedp_9_6_3_2_003(std::string& err) {
    struct Reported {
        int calls = 0;
        uint8_t count = 0u;
        Locator first = LOCATOR_INVALID;
    } obs{};

    FakeTransport tx{};
    EndpointMatchTable table(
        [](const MatchedEndpoint& w, const MatchedEndpoint&, const char*, void* u) noexcept {
            auto* o = static_cast<Reported*>(u);
            o->calls++;
            o->count = w.locatorCount;
            o->first = w.locatorCount > 0u ? w.locators[0] : LOCATOR_INVALID;
            return true;
        },
        nullptr, &obs);
    LocalSubnets subnets{};
    CHECK(subnets.add(make_udpv4_locator(0xC0A80105u, 0u), 24u), "add 192.168.1.5/24");
    table.set_local_subnets(subnets);
    SedpReader reader(GuidPrefix{}, tx, table);

    const Guid local_reader = make_guid(0x30u, 0x31u, 0x32u, make_entity(3u, 3u, 3u, 0xC7u));
    table.set_local_prefix(local_reader.prefix);
    table.add_reader(make_local_reader(local_reader));

    Locator shm{};
    shm.kind = 16;
    shm.port = 7411u;
    const Locator lan = make_udpv4_locator(0xC0A80164u, 7411u);  // 192.168.1.100
    std::vector<Octet> payload{};
    CHECK(build_writer_payload_with_locators(
              {shm, make_udpv4_locator(0xAC110001u, 7411u), make_udpv4_locator(0xAC120001u, 7411u),
               make_udpv4_locator(0xAC130001u, 7411u), make_udpv4_locator(0xAC140001u, 7411u), lan},
              payload),
          "payload build failed");

    const SedpPeer peer{GuidPrefix{}, make_udpv4_locator(0xC0A80101u, 7410u)};
    peer.discover(reader);
    peer.announce(reader, entity_id::SEDP_BUILTIN_PUBLICATIONS_ANNOUNCER, 1, payload);
    CHECK(obs.calls == 1, "the remote writer matches");
    CHECK(obs.count > 0u && obs.first == lan, "the sixth announced locator is reported first");
    return true;
}

void append_sedp_tests_9_6_3_2(std::vector<TestCase>& tests) {
    tests.push_back({"TC-SEDP-9.6.3.2-001",
                     "§9.6.3.2: SEDP PL_CDR endpoint data parsing skips unknown PID entries while preserving mandatory-field processing.",
                     tc_sedp_9_6_3_2_001});
    tests.push_back({"TC-SEDP-9.6.3.2-004",
                     "§9.6.2: a parameter whose padded length reaches 0x10000 is bounded by the "
                     "payload; endpoint data parsing never reads past it.",
                     tc_sedp_9_6_3_2_004});
    tests.push_back({"TC-SEDP-9.6.3.2-002",
                     "§9.6.3.2: endpoint discovery payload without PID_ENDPOINT_GUID is rejected.",
                     tc_sedp_9_6_3_2_002});
    tests.push_back({"TC-SEDP-9.6.3.2-003",
                     "§8.5.4.2 / §9.6.2: an endpoint's most reachable unicast locator is kept "
                     "however many locators it announces before it.",
                     tc_sedp_9_6_3_2_003});
}

} // namespace spec_test
