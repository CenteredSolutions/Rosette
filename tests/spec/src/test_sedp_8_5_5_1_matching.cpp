#include "spec_test_common.hpp"

#include <string.h>

#include "rosette/rtps/endpoint_match_table.hpp"

namespace spec_test {

#define CHECK(COND, MSG) do { if (!(COND)) { err = (MSG); return false; } } while (0)

namespace {

struct Obs {
    int match_calls = 0;
    int unmatch_calls = 0;
};

static void on_match(const MatchedEndpoint&,
                     const MatchedEndpoint&,
                     const TopicName&,
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
    t.set_local_prefix(wg.prefix);

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
    t.set_local_prefix(wg.prefix);

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
    t.set_local_prefix(wg.prefix);

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
    t.set_local_prefix(local_writer.prefix);

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

// §8.5.5.1 with compact remote records: a remote reader announced before
// the local writer exists is matched later by its topic/type key and
// reported with the locators it announced; two remote endpoints are never
// matched with each other; a remote reader on another topic is not matched.
struct PairObs {
    int calls = 0;
    Guid last_w = GUID_UNKNOWN;
    Guid last_r = GUID_UNKNOWN;
    uint8_t reader_locators = 0;
    Locator first_locator = LOCATOR_INVALID;
    TopicName topic{};
};

static bool tc_sedp_8_5_5_1_005(std::string& err) {
    PairObs obs{};
    EndpointMatchTable t(
        [](const MatchedEndpoint& w, const MatchedEndpoint& r, const TopicName& topic,
           void* u) noexcept {
            auto* o = static_cast<PairObs*>(u);
            o->calls++;
            o->last_w = w.guid;
            o->last_r = r.guid;
            o->reader_locators = r.locatorCount;
            o->first_locator = r.locatorCount ? r.locators[0] : LOCATOR_INVALID;
            o->topic = topic;
        },
        nullptr, &obs);
    const Guid local_writer = make_guid(0x01u, 0x02u, 0x03u, make_entity(1u, 1u, 1u, 0x03u));
    t.set_local_prefix(local_writer.prefix);

    const Locator loc = make_udpv4_locator(0xC0A80190u, 7411u);
    DiscoveredReaderData remote = make_reader(make_guid(0x0Au, 0x0Bu, 0x0Cu, make_entity(2u, 2u, 2u, 0x04u)),
                                              "rt/chatter", "std_msgs::msg::dds_::String_",
                                              ReliabilityKind::RELIABLE, DurabilityKind::VOLATILE);
    remote.unicastLocatorList[0] = loc;
    remote.unicastLocatorCount = 1u;
    t.add_reader(remote);
    t.add_reader(make_reader(make_guid(0x0Au, 0x0Bu, 0x0Cu, make_entity(3u, 3u, 3u, 0x04u)),
                             "rt/other", "std_msgs::msg::dds_::String_",
                             ReliabilityKind::RELIABLE, DurabilityKind::VOLATILE));
    t.add_writer(make_writer(make_guid(0x0Du, 0x0Eu, 0x0Fu, make_entity(4u, 4u, 4u, 0x03u)),
                             "rt/chatter", "std_msgs::msg::dds_::String_",
                             ReliabilityKind::RELIABLE, DurabilityKind::VOLATILE));
    CHECK(obs.calls == 0, "remote endpoints are never matched with each other");

    t.add_writer(make_writer(local_writer, "rt/chatter", "std_msgs::msg::dds_::String_",
                             ReliabilityKind::RELIABLE, DurabilityKind::VOLATILE));
    CHECK(obs.calls == 1, "the late local writer matches the stored remote reader, and only it");
    CHECK(obs.last_w == local_writer && obs.last_r == remote.readerGuid, "match GUIDs");
    CHECK(obs.reader_locators == 1u && obs.first_locator == loc,
          "the remote reader's announced locator is reported");
    CHECK(strcmp(obs.topic.str, "rt/chatter") == 0, "the local endpoint's topic is reported");

    t.add_writer(make_writer(local_writer, "rt/chatter", "std_msgs::msg::dds_::String_",
                             ReliabilityKind::RELIABLE, DurabilityKind::VOLATILE));
    CHECK(obs.calls == 1, "re-adding the writer reports no pair twice");
    return true;
}

// A name longer than RTPS_MAX_TOPIC_NAME_LEN - 1 cannot be stored whole. A
// remote endpoint announced with one must not match a local endpoint whose
// name equals its stored prefix: it is ignored.
static bool tc_sedp_8_5_5_1_006(std::string& err) {
    Obs obs{};
    EndpointMatchTable t(on_match, on_unmatch, &obs);
    const Guid local_reader = make_guid(0x61u, 0x62u, 0x63u, make_entity(1u, 1u, 1u, 0x04u));
    t.set_local_prefix(local_reader.prefix);

    std::string fits(RTPS_MAX_TOPIC_NAME_LEN - 1u, 'a');
    fits[0] = 'r'; fits[1] = 't'; fits[2] = '/';
    const std::string too_long = fits + "_and_more";
    t.add_reader(make_reader(local_reader, fits.c_str(), "pkg::msg::dds_::T_",
                             ReliabilityKind::RELIABLE, DurabilityKind::VOLATILE));
    t.add_writer(make_writer(make_guid(0x71u, 0x72u, 0x73u, make_entity(2u, 2u, 2u, 0x03u)),
                             too_long.c_str(), "pkg::msg::dds_::T_",
                             ReliabilityKind::RELIABLE, DurabilityKind::VOLATILE));
    CHECK(obs.match_calls == 0, "a remote name longer than the limit never matches a local prefix of it");

    t.add_writer(make_writer(make_guid(0x71u, 0x72u, 0x73u, make_entity(3u, 3u, 3u, 0x03u)),
                             fits.c_str(), "pkg::msg::dds_::T_",
                             ReliabilityKind::RELIABLE, DurabilityKind::VOLATILE));
    CHECK(obs.match_calls == 1, "a name of exactly the limit matches");
    return true;
}

// name_key() is 32-bit FNV-1a, and a remote endpoint stored by key is
// matched only when both its topic key and its type key agree.
static bool tc_sedp_8_5_5_1_007(std::string& err) {
    CHECK(name_key("") == 0x811C9DC5u, "the empty name hashes to the FNV-1a offset basis");
    CHECK(name_key(nullptr) == 0x811C9DC5u, "null hashes as the empty name");
    CHECK(name_key("a") == 0xE40C292Cu, "FNV-1a test vector for \"a\"");
    CHECK(name_key("foobar") == 0xBF9CF968u, "FNV-1a test vector for \"foobar\"");

    Obs obs{};
    EndpointMatchTable t(on_match, on_unmatch, &obs);
    const Guid local_writer = make_guid(0x81u, 0x82u, 0x83u, make_entity(1u, 1u, 1u, 0x03u));
    t.set_local_prefix(local_writer.prefix);
    t.add_reader(make_reader(make_guid(0x91u, 0x92u, 0x93u, make_entity(2u, 2u, 2u, 0x04u)),
                             "rt/chatter", "std_msgs::msg::dds_::Int32_",
                             ReliabilityKind::RELIABLE, DurabilityKind::VOLATILE));
    t.add_writer(make_writer(local_writer, "rt/chatter", "std_msgs::msg::dds_::String_",
                             ReliabilityKind::RELIABLE, DurabilityKind::VOLATILE));
    CHECK(obs.match_calls == 0, "a stored remote reader of another type does not match");
    return true;
}

// A remote table full of endpoints no local endpoint pairs with must still
// admit the one that completes a match: on a network much larger than the
// table, the first announcements to arrive are rarely the ones this
// participant needs. The displaced record is one in no match. The filler
// endpoints all belong to one participant, as a busy stock ROS 2 process's
// do, so only the record tables run out, not the origins.
struct DisplaceObs {
    int  match_calls   = 0;
    int  unmatch_calls = 0;
    int  change_calls  = 0;
    Guid last_remote   = GUID_UNKNOWN;
};

static bool tc_sedp_8_5_5_1_008(std::string& err) {
    DisplaceObs obs{};
    EndpointMatchTable t(
        [](const MatchedEndpoint& w, const MatchedEndpoint& r, const TopicName&,
           void* u) noexcept {
            auto* o = static_cast<DisplaceObs*>(u);
            o->match_calls++;
            o->last_remote = w.guid.prefix.value[0] == 0x10u ? r.guid : w.guid;
        },
        [](const Guid&, const Guid&, void* u) noexcept {
            static_cast<DisplaceObs*>(u)->unmatch_calls++;
        },
        &obs,
        [](void* u) noexcept { static_cast<DisplaceObs*>(u)->change_calls++; });

    const Guid local_reader = make_guid(0x10u, 0x00u, 0x00u, make_entity(1u, 1u, 1u, 0x04u));
    const Guid local_writer = make_guid(0x10u, 0x00u, 0x00u, make_entity(1u, 1u, 2u, 0x03u));
    t.set_local_prefix(local_reader.prefix);
    t.add_reader(make_reader(local_reader, "rt/machine/estop", "std_msgs::msg::dds_::Bool_",
                             ReliabilityKind::RELIABLE, DurabilityKind::VOLATILE));
    t.add_writer(make_writer(local_writer, "rt/machine/faults",
                             "diagnostic_msgs::msg::dds_::DiagnosticArray_",
                             ReliabilityKind::RELIABLE, DurabilityKind::VOLATILE));

    for (uint32_t i = 0; i < RTPS_MAX_DISCOVERED_WRITERS; ++i) {
        t.add_writer(make_writer(make_guid(0x20u, 0x00u, 0x00u,
                                           make_entity(0u, 1u, static_cast<Octet>(i), 0x03u)),
                                 "rt/rosout", "rcl_interfaces::msg::dds_::Log_",
                                 ReliabilityKind::RELIABLE, DurabilityKind::VOLATILE));
    }
    for (uint32_t i = 0; i < RTPS_MAX_DISCOVERED_READERS; ++i) {
        t.add_reader(make_reader(make_guid(0x20u, 0x00u, 0x00u,
                                           make_entity(0u, 2u, static_cast<Octet>(i), 0x04u)),
                                 "rt/parameter_events",
                                 "rcl_interfaces::msg::dds_::ParameterEvent_",
                                 ReliabilityKind::RELIABLE, DurabilityKind::VOLATILE));
    }
    CHECK(t.count("rt/rosout", true) == RTPS_MAX_DISCOVERED_WRITERS, "remote writer table full");
    CHECK(t.count("rt/parameter_events", false) == RTPS_MAX_DISCOVERED_READERS,
          "remote reader table full");

    const int changes_before = obs.change_calls;
    const Guid estop_writer = make_guid(0x30u, 0x00u, 0x00u, make_entity(0u, 0u, 3u, 0x03u));
    t.add_writer(make_writer(estop_writer, "rt/machine/estop", "std_msgs::msg::dds_::Bool_",
                             ReliabilityKind::RELIABLE, DurabilityKind::VOLATILE));
    CHECK(obs.match_calls == 1 && obs.last_remote == estop_writer,
          "a remote writer that completes a match displaces one that completes none");
    CHECK(t.count("rt/rosout", true) == RTPS_MAX_DISCOVERED_WRITERS - 1u,
          "exactly one unmatched remote writer is displaced");
    CHECK(obs.change_calls > changes_before, "displacement is a graph change");

    const Guid faults_reader = make_guid(0x31u, 0x00u, 0x00u, make_entity(0u, 0u, 4u, 0x04u));
    t.add_reader(make_reader(faults_reader, "rt/machine/faults",
                             "diagnostic_msgs::msg::dds_::DiagnosticArray_",
                             ReliabilityKind::RELIABLE, DurabilityKind::VOLATILE));
    CHECK(obs.match_calls == 2 && obs.last_remote == faults_reader,
          "a remote reader that completes a match displaces one that completes none");
    CHECK(t.count("rt/parameter_events", false) == RTPS_MAX_DISCOVERED_READERS - 1u,
          "exactly one unmatched remote reader is displaced");

    t.add_writer(make_writer(make_guid(0x32u, 0x00u, 0x00u, make_entity(0u, 0u, 5u, 0x03u)),
                             "rt/rosout", "rcl_interfaces::msg::dds_::Log_",
                             ReliabilityKind::RELIABLE, DurabilityKind::VOLATILE));
    CHECK(t.count("rt/rosout", true) == RTPS_MAX_DISCOVERED_WRITERS - 1u,
          "an announcement that completes no match never displaces a record");
    CHECK(obs.unmatch_calls == 0, "no match is dissolved by displacement");
    return true;
}

// A remote record that is part of a match is never displaced: when every
// record is matched, a further announcement is refused, even one that would
// complete a match of its own.
static bool tc_sedp_8_5_5_1_009(std::string& err) {
    Obs obs{};
    EndpointMatchTable t(on_match, on_unmatch, &obs);
    const Guid local_reader = make_guid(0x40u, 0x00u, 0x00u, make_entity(1u, 1u, 1u, 0x04u));
    t.set_local_prefix(local_reader.prefix);
    t.add_reader(make_reader(local_reader, "rt/machine/estop", "std_msgs::msg::dds_::Bool_",
                             ReliabilityKind::RELIABLE, DurabilityKind::VOLATILE));

    for (uint32_t i = 0; i < RTPS_MAX_DISCOVERED_WRITERS; ++i) {
        t.add_writer(make_writer(
            make_guid(0x50u, static_cast<Octet>(i), 0x00u, make_entity(0u, 0u, 1u, 0x03u)),
            "rt/machine/estop", "std_msgs::msg::dds_::Bool_", ReliabilityKind::RELIABLE,
            DurabilityKind::VOLATILE));
    }
    CHECK(obs.match_calls == static_cast<int>(RTPS_MAX_DISCOVERED_WRITERS),
          "every remote estop writer matches while there is room");

    t.add_writer(make_writer(make_guid(0x60u, 0x00u, 0x00u, make_entity(0u, 0u, 1u, 0x03u)),
                             "rt/machine/estop", "std_msgs::msg::dds_::Bool_",
                             ReliabilityKind::RELIABLE, DurabilityKind::VOLATILE));
    CHECK(obs.match_calls == static_cast<int>(RTPS_MAX_DISCOVERED_WRITERS),
          "a full table of matched records admits no further writer");
    CHECK(obs.unmatch_calls == 0, "no matched record is displaced");
    CHECK(t.count("rt/machine/estop", true) == RTPS_MAX_DISCOVERED_WRITERS,
          "the stored remote writers are unchanged");
    return true;
}

// §8.5.4.2: an endpoint lists the locators it is reached at. The table keeps
// the UDP ones this stack can send to, in announced order, so a kind it
// cannot send to (Fast DDS's shared-memory kind, 16) never occupies a slot
// ahead of a reachable one.
static bool tc_sedp_8_5_5_1_010(std::string& err) {
    PairObs obs{};
    EndpointMatchTable t(
        [](const MatchedEndpoint&, const MatchedEndpoint& r, const TopicName&,
           void* u) noexcept {
            auto* o = static_cast<PairObs*>(u);
            o->calls++;
            o->last_r = r.guid;
            o->reader_locators = r.locatorCount;
            o->first_locator = r.locatorCount ? r.locators[0] : LOCATOR_INVALID;
        },
        nullptr, &obs);
    const Guid local_writer = make_guid(0xA1u, 0xA2u, 0xA3u, make_entity(1u, 1u, 1u, 0x03u));
    t.set_local_prefix(local_writer.prefix);
    t.add_writer(make_writer(local_writer, "rt/machine/faults",
                             "diagnostic_msgs::msg::dds_::DiagnosticArray_",
                             ReliabilityKind::RELIABLE, DurabilityKind::VOLATILE));

    Locator shm{};
    shm.kind = 16;
    shm.port = 7411u;
    const Locator udp = make_udpv4_locator(0xC0A80190u, 7411u);
    DiscoveredReaderData remote =
        make_reader(make_guid(0xB1u, 0xB2u, 0xB3u, make_entity(2u, 2u, 2u, 0x04u)),
                    "rt/machine/faults", "diagnostic_msgs::msg::dds_::DiagnosticArray_",
                    ReliabilityKind::RELIABLE, DurabilityKind::VOLATILE);
    remote.unicastLocatorList[0] = shm;
    remote.unicastLocatorList[1] = udp;
    remote.unicastLocatorCount = 2u;
    t.add_reader(remote);
    CHECK(obs.calls == 1 && obs.last_r == remote.readerGuid, "the remote reader matches");
    CHECK(obs.reader_locators == 1u && obs.first_locator == udp,
          "only the UDP locator is kept and reported");
    return true;
}

// Remote endpoints share an origin (participant prefix and announced
// locators). When every origin is taken, an endpoint that completes a match
// frees an origin none of whose endpoints is in a match, forgetting those
// endpoints, and never one with an endpoint in a match.
static bool tc_sedp_8_5_5_1_011(std::string& err) {
    Obs obs{};
    EndpointMatchTable t(on_match, on_unmatch, &obs);
    const Guid local_reader = make_guid(0xC0u, 0x00u, 0x00u, make_entity(1u, 1u, 1u, 0x04u));
    t.set_local_prefix(local_reader.prefix);
    t.add_reader(make_reader(local_reader, "rt/machine/estop", "std_msgs::msg::dds_::Bool_",
                             ReliabilityKind::RELIABLE, DurabilityKind::VOLATILE));

    // Participant 0 has a matched estop writer and an unmatched rosout reader;
    // every other origin holds one unmatched rosout reader.
    t.add_writer(make_writer(make_guid(0xD0u, 0x00u, 0x00u, make_entity(0u, 0u, 1u, 0x03u)),
                             "rt/machine/estop", "std_msgs::msg::dds_::Bool_",
                             ReliabilityKind::RELIABLE, DurabilityKind::VOLATILE));
    for (uint32_t i = 0; i < RTPS_MAX_REMOTE_ORIGINS; ++i) {
        t.add_reader(make_reader(make_guid(0xD0u, static_cast<Octet>(i), 0x00u,
                                           make_entity(0u, 0u, 2u, 0x04u)),
                                 "rt/rosout", "rcl_interfaces::msg::dds_::Log_",
                                 ReliabilityKind::RELIABLE, DurabilityKind::VOLATILE));
    }
    CHECK(obs.match_calls == 1, "participant 0's estop writer matches");
    CHECK(t.count("rt/rosout", false) == RTPS_MAX_REMOTE_ORIGINS, "every origin is taken");

    t.add_reader(make_reader(make_guid(0xE0u, 0x00u, 0x00u, make_entity(0u, 0u, 2u, 0x04u)),
                             "rt/rosout", "rcl_interfaces::msg::dds_::Log_",
                             ReliabilityKind::RELIABLE, DurabilityKind::VOLATILE));
    CHECK(t.count("rt/rosout", false) == RTPS_MAX_REMOTE_ORIGINS,
          "an endpoint that completes no match frees no origin");

    t.add_writer(make_writer(make_guid(0xE1u, 0x00u, 0x00u, make_entity(0u, 0u, 1u, 0x03u)),
                             "rt/machine/estop", "std_msgs::msg::dds_::Bool_",
                             ReliabilityKind::RELIABLE, DurabilityKind::VOLATILE));
    CHECK(obs.match_calls == 2, "an estop writer from a new participant matches");
    CHECK(t.count("rt/rosout", false) == RTPS_MAX_REMOTE_ORIGINS - 1u,
          "exactly one unmatched origin's endpoints are forgotten");
    CHECK(t.count("rt/machine/estop", true) == 2u && obs.unmatch_calls == 0,
          "participant 0's matched writer is kept");
    return true;
}

// §8.5.4.2: a proxy refers to the locators of the origin its remote endpoint
// was matched with. When a matched endpoint re-announces other locators, its
// pairs are dissolved and reported again with the new origin, so no proxy
// refers to the old origin once another participant's endpoint takes it.
// An unchanged re-announcement dissolves nothing.
static bool tc_sedp_8_5_5_1_012(std::string& err) {
    struct RelocObs {
        int matches = 0;
        int unmatches = 0;
        const Locator* reader_locators = nullptr;
    } obs{};

    EndpointMatchTable t(
        [](const MatchedEndpoint&, const MatchedEndpoint& r, const TopicName&, void* u) noexcept {
            auto* o = static_cast<RelocObs*>(u);
            o->matches++;
            o->reader_locators = r.locators;
        },
        [](const Guid&, const Guid&, void* u) noexcept { static_cast<RelocObs*>(u)->unmatches++; },
        &obs);
    const Guid local_writer = make_guid(0x01u, 0x02u, 0x03u, make_entity(1u, 1u, 1u, 0x03u));
    t.set_local_prefix(local_writer.prefix);
    t.add_writer(make_writer(local_writer, "rt/moving", "pkg::msg::dds_::T_",
                             ReliabilityKind::RELIABLE, DurabilityKind::VOLATILE));

    const Locator before = make_udpv4_locator(0xC0A801A0u, 7411u);
    const Locator after = make_udpv4_locator(0xC0A801A1u, 7411u);
    DiscoveredReaderData mover =
        make_reader(make_guid(0x0Au, 0x0Bu, 0x0Cu, make_entity(2u, 2u, 2u, 0x04u)), "rt/moving",
                    "pkg::msg::dds_::T_", ReliabilityKind::RELIABLE, DurabilityKind::VOLATILE);
    mover.unicastLocatorList[0] = before;
    mover.unicastLocatorCount = 1u;
    t.add_reader(mover);
    CHECK(obs.matches == 1 && obs.reader_locators != nullptr && *obs.reader_locators == before,
          "the reader matches at its announced locator");

    t.add_reader(mover);
    CHECK(obs.matches == 1 && obs.unmatches == 0,
          "re-announcing the same locators dissolves nothing");

    mover.unicastLocatorList[0] = after;
    t.add_reader(mover);
    CHECK(obs.unmatches == 1 && obs.matches == 2,
          "re-announcing other locators dissolves the pair and reports it again");
    const Locator* reported = obs.reader_locators;
    CHECK(reported != nullptr && *reported == after, "the pair is reported at the new locator");

    // Another participant's endpoint with locators of its own takes the
    // origin the mover left; the reported reference must not see them.
    DiscoveredReaderData other =
        make_reader(make_guid(0x0Du, 0x0Eu, 0x0Fu, make_entity(3u, 3u, 3u, 0x04u)), "rt/unrelated",
                    "pkg::msg::dds_::T_", ReliabilityKind::RELIABLE, DurabilityKind::VOLATILE);
    other.unicastLocatorList[0] = make_udpv4_locator(0xC0A801B0u, 7411u);
    other.unicastLocatorCount = 1u;
    t.add_reader(other);
    CHECK(*reported == after, "the matched reference still holds the mover's locator");
    return true;
}

// §8.5.4.2: an endpoint that announces its own locators is reported at the
// most reachable of them first, as a participant's lists are (TC-SPDP-8.5.3-010):
// the one inside this participant's prefix, then the rest in announced order.
static bool tc_sedp_8_5_5_1_013(std::string& err) {
    struct Reported {
        int calls = 0;
        uint8_t count = 0u;
        Locator first = LOCATOR_INVALID;
        Locator second = LOCATOR_INVALID;
    } obs{};

    EndpointMatchTable t(
        [](const MatchedEndpoint& w, const MatchedEndpoint&, const TopicName&, void* u) noexcept {
            auto* o = static_cast<Reported*>(u);
            o->calls++;
            o->count = w.locatorCount;
            o->first = w.locatorCount > 0u ? w.locators[0] : LOCATOR_INVALID;
            o->second = w.locatorCount > 1u ? w.locators[1] : LOCATOR_INVALID;
        },
        nullptr, &obs);
    LocalSubnets subnets{};
    CHECK(subnets.add(make_udpv4_locator(0xC0A80105u, 0u), 24u), "add 192.168.1.5/24");
    t.set_local_subnets(subnets);
    const Guid local_reader = make_guid(0xA1u, 0xA2u, 0xA3u, make_entity(1u, 1u, 1u, 0x04u));
    t.set_local_prefix(local_reader.prefix);
    t.add_reader(make_reader(local_reader, "rt/machine/estop", "std_msgs::msg::dds_::Bool_",
                             ReliabilityKind::RELIABLE, DurabilityKind::VOLATILE));

    const Locator docker = make_udpv4_locator(0xAC110001u, 7411u);
    const Locator wifi = make_udpv4_locator(0x0A000005u, 7411u);
    const Locator lan = make_udpv4_locator(0xC0A80164u, 7411u);
    DiscoveredWriterData remote = make_writer(
        make_guid(0xB1u, 0xB2u, 0xB3u, make_entity(2u, 2u, 2u, 0x03u)), "rt/machine/estop",
        "std_msgs::msg::dds_::Bool_", ReliabilityKind::RELIABLE, DurabilityKind::VOLATILE);
    remote.unicastLocatorList[0] = docker;
    remote.unicastLocatorList[1] = wifi;
    remote.unicastLocatorList[2] = lan;
    remote.unicastLocatorCount = 3u;
    t.add_writer(remote);
    CHECK(obs.calls == 1, "the remote writer matches");
    CHECK(obs.count == 2u && obs.first == lan && obs.second == docker,
          "the on-link locator is reported first, then the first routed one");
    return true;
}

void append_sedp_tests_8_5_5_1(std::vector<TestCase>& tests) {
    tests.push_back({"TC-SEDP-8.5.5.1-013",
                     "§8.5.4.2: a remote endpoint's announced locators are kept most reachable "
                     "first against this participant's addresses, as a participant's are.",
                     tc_sedp_8_5_5_1_013});
    tests.push_back({"TC-SEDP-8.5.5.1-011",
                     "§8.5.5.1 capacity: when every remote origin is taken, an endpoint that completes a match frees an origin with no endpoint in a match; an origin with a matched endpoint is kept.",
                     tc_sedp_8_5_5_1_011});
    tests.push_back({"TC-SEDP-8.5.5.1-010",
                     "§8.5.4.2 implementation limit: a remote endpoint keeps at most RTPS_MAX_REMOTE_ENDPOINT_LOCATORS UDP locators, in announced order; locator kinds the stack cannot send to are not kept.",
                     tc_sedp_8_5_5_1_010});
    tests.push_back({"TC-SEDP-8.5.5.1-009",
                     "§8.5.5.1 capacity: a remote endpoint that is part of a match is never displaced; when every stored record is matched, further announcements are refused.",
                     tc_sedp_8_5_5_1_009});
    tests.push_back({"TC-SEDP-8.5.5.1-008",
                     "§8.5.5.1 capacity: when the remote writer or reader table is full, an announcement that completes a match with a local endpoint displaces a record in no match; one that completes none is refused.",
                     tc_sedp_8_5_5_1_008});
    tests.push_back({"TC-SEDP-8.5.5.1-007",
                     "§8.5.5.1: name_key() is 32-bit FNV-1a; a remote endpoint stored by key matches a later local endpoint only when topic and type keys both agree.",
                     tc_sedp_8_5_5_1_007});
    tests.push_back({"TC-SEDP-8.5.5.1-006",
                     "§8.5.5.1: an endpoint whose topic or type name exceeds RTPS_MAX_TOPIC_NAME_LEN - 1 is ignored, never matched by its truncated prefix.",
                     tc_sedp_8_5_5_1_006});
    tests.push_back({"TC-SEDP-8.5.5.1-005",
                     "§8.5.5.1: a remote endpoint stored by topic/type key matches a local endpoint created later, with its announced locators; remote-remote pairs never match.",
                     tc_sedp_8_5_5_1_005});
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
    tests.push_back({"TC-SEDP-8.5.5.1-012",
                     "§8.5.4.2: a matched remote endpoint that re-announces other locators is "
                     "unmatched and matched again at them; its old origin is never referenced "
                     "after another endpoint takes it.",
                     tc_sedp_8_5_5_1_012});
}

} // namespace spec_test
