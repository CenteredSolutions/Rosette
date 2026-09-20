#include "spec_test_common.hpp"

#include "rosette/rtps/endpoint_match_table.hpp"

namespace spec_test {

#define CHECK(COND, MSG) do { if (!(COND)) { err = (MSG); return false; } } while (0)

namespace {

struct Obs {
    int match_calls = 0;
    int unmatch_calls = 0;
};

static void on_match(const DiscoveredWriterData&,
                     const DiscoveredReaderData&,
                     void* user) noexcept {
    auto* o = static_cast<Obs*>(user);
    o->match_calls++;
}

static void on_unmatch(const Guid&,
                       const Guid&,
                       void* user) noexcept {
    auto* o = static_cast<Obs*>(user);
    o->unmatch_calls++;
}

static Guid make_guid(Octet p0, Octet p1, Octet p2, const EntityId& eid) {
    Guid g{};
    g.prefix.value[0] = p0;
    g.prefix.value[1] = p1;
    g.prefix.value[2] = p2;
    g.entityId = eid;
    return g;
}

static DiscoveredWriterData make_writer(const Guid& g,
                                        const char* topic,
                                        const char* type,
                                        ReliabilityKind rel,
                                        DurabilityKind dur) {
    DiscoveredWriterData wd{};
    wd.writerGuid = g;
    wd.topicName.assign(topic);
    wd.typeName.assign(type);
    wd.reliabilityKind = rel;
    wd.durabilityKind = dur;
    return wd;
}

static DiscoveredReaderData make_reader(const Guid& g,
                                        const char* topic,
                                        const char* type,
                                        ReliabilityKind rel,
                                        DurabilityKind dur) {
    DiscoveredReaderData rd{};
    rd.readerGuid = g;
    rd.topicName.assign(topic);
    rd.typeName.assign(type);
    rd.reliabilityKind = rel;
    rd.durabilityKind = dur;
    return rd;
}

} // namespace

static bool tc_sedp_8_5_5_1_001(std::string& err) {
    Obs obs{};
    EndpointMatchTable t(on_match, on_unmatch, &obs);

    const Guid wg = make_guid(0x11u, 0x00u, 0x00u, make_entity(1u, 1u, 1u, 0xC2u));
    const Guid rg = make_guid(0x22u, 0x00u, 0x00u, make_entity(2u, 2u, 2u, 0xC7u));

    t.add_writer(make_writer(wg, "camera", "sensor_msgs::msg::Image",
                             ReliabilityKind::RELIABLE, DurabilityKind::VOLATILE));
    t.add_reader(make_reader(rg, "camera", "sensor_msgs::msg::CameraInfo",
                             ReliabilityKind::RELIABLE, DurabilityKind::VOLATILE));

    CHECK(obs.match_calls == 0, "topic/type mismatch should not match endpoints");
    return true;
}

static bool tc_sedp_8_5_5_1_002(std::string& err) {
    Obs obs{};
    EndpointMatchTable t(on_match, on_unmatch, &obs);

    const Guid wg = make_guid(0x33u, 0x00u, 0x00u, make_entity(3u, 3u, 3u, 0xC2u));
    const Guid rg = make_guid(0x44u, 0x00u, 0x00u, make_entity(4u, 4u, 4u, 0xC7u));

    t.add_writer(make_writer(wg, "scan", "sensor_msgs::msg::LaserScan",
                             ReliabilityKind::BEST_EFFORT, DurabilityKind::VOLATILE));
    t.add_reader(make_reader(rg, "scan", "sensor_msgs::msg::LaserScan",
                             ReliabilityKind::RELIABLE, DurabilityKind::VOLATILE));

    CHECK(obs.match_calls == 0, "BEST_EFFORT writer must not match RELIABLE reader");
    return true;
}

static bool tc_sedp_8_5_5_1_003(std::string& err) {
    Obs obs{};
    EndpointMatchTable t(on_match, on_unmatch, &obs);

    const Guid wg = make_guid(0x55u, 0x00u, 0x00u, make_entity(5u, 5u, 5u, 0xC2u));
    const Guid rg = make_guid(0x66u, 0x00u, 0x00u, make_entity(6u, 6u, 6u, 0xC7u));

    t.add_writer(make_writer(wg, "tf", "tf2_msgs::msg::TFMessage",
                             ReliabilityKind::RELIABLE, DurabilityKind::VOLATILE));
    t.add_reader(make_reader(rg, "tf", "tf2_msgs::msg::TFMessage",
                             ReliabilityKind::BEST_EFFORT, DurabilityKind::TRANSIENT_LOCAL));
    CHECK(obs.match_calls == 0, "writer durability lower than reader should be incompatible");

    t.add_writer(make_writer(wg, "tf", "tf2_msgs::msg::TFMessage",
                             ReliabilityKind::RELIABLE, DurabilityKind::TRANSIENT_LOCAL));
    CHECK(obs.match_calls == 1, "writer durability >= reader should become compatible");
    return true;
}

static bool tc_sedp_8_5_5_1_004(std::string& err) {
    Obs obs{};
    EndpointMatchTable t(on_match, on_unmatch, &obs);

    const GuidPrefix remote_prefix = make_guid(0x77u, 0x88u, 0x99u, entity_id::UNKNOWN).prefix;
    const Guid local_writer = make_guid(0x10u, 0x20u, 0x30u, make_entity(7u, 7u, 7u, 0xC2u));
    const Guid remote_reader = make_guid(remote_prefix.value[0], remote_prefix.value[1], remote_prefix.value[2],
                                         make_entity(8u, 8u, 8u, 0xC7u));

    t.add_writer(make_writer(local_writer, "cmd_vel", "geometry_msgs::msg::Twist",
                             ReliabilityKind::RELIABLE, DurabilityKind::VOLATILE));
    t.add_reader(make_reader(remote_reader, "cmd_vel", "geometry_msgs::msg::Twist",
                             ReliabilityKind::RELIABLE, DurabilityKind::VOLATILE));
    CHECK(obs.match_calls == 1, "initial discovery should match endpoints");

    t.remove_participant(remote_prefix);
    CHECK(obs.unmatch_calls == 1, "participant removal should dissolve endpoint matches");

    t.add_reader(make_reader(remote_reader, "cmd_vel", "geometry_msgs::msg::Twist",
                             ReliabilityKind::RELIABLE, DurabilityKind::VOLATILE));
    CHECK(obs.match_calls == 2, "re-discovered endpoint after participant removal should rematch");
    return true;
}

void append_sedp_tests_8_5_5_1(std::vector<TestCase>& tests) {
    tests.push_back({"TC-SEDP-8.5.5.1-001",
                     "§8.5.5.1: writer and reader match only when both topicName and typeName are equal.",
                     tc_sedp_8_5_5_1_001});
    tests.push_back({"TC-SEDP-8.5.5.1-002",
                     "§8.5.5.1/§7.6.3.3: RELIABLE reader is incompatible with BEST_EFFORT writer.",
                     tc_sedp_8_5_5_1_002});
    tests.push_back({"TC-SEDP-8.5.5.1-003",
                     "§8.5.5.1/§7.6.3.4: writer durability must be greater than or equal to reader durability for compatibility.",
                     tc_sedp_8_5_5_1_003});
    tests.push_back({"TC-SEDP-8.5.5.1-004",
                     "§8.5.5.1 with restart path: remove_participant dissolves matches and rediscovered endpoints rematch cleanly.",
                     tc_sedp_8_5_5_1_004});
}

} // namespace spec_test
