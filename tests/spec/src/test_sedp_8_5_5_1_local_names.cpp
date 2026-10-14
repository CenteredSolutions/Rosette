/**
 * @file test_sedp_8_5_5_1_local_names.cpp
 * @brief TC-SEDP-8.5.5.1-014: local endpoint names interned in the match table's arena
 * (rosette/rtps/string_arena.hpp) are stored once however many endpoints share them, read back
 * exactly through removal and renaming, and still drive matching, count() and the SEDP readback
 * (local_writer_data()).
 */

#include "spec_test_common.hpp"
#include "spec_test_registry.hpp"

#include <string.h>

#include "rosette/rtps/endpoint_match_table.hpp"

namespace spec_test {

#define CHECK(COND, MSG)  \
    do {                  \
        if (!(COND)) {    \
            err = (MSG);  \
            return false; \
        }                 \
    } while (0)

namespace {

struct LocalNamesObs {
    int       calls = 0;
    TopicName topic{};
};

bool record_match(const MatchedEndpoint&, const MatchedEndpoint&, const char* topic,
                  void* user) noexcept {
    auto* obs = static_cast<LocalNamesObs*>(user);
    obs->calls++;
    (void)obs->topic.assign(topic);
    return true;
}

Guid guid_in(const GuidPrefix& prefix, Octet key, Octet kind) {
    return Guid{prefix, make_entity(0u, 0u, key, kind)};
}

DiscoveredWriterData writer(const Guid& guid, const char* topic, const char* type) {
    DiscoveredWriterData wd{};
    wd.writerGuid = guid;
    (void)wd.topicName.assign(topic);
    (void)wd.typeName.assign(type);
    wd.reliabilityKind = ReliabilityKind::RELIABLE;
    return wd;
}

DiscoveredReaderData reader(const Guid& guid, const char* topic, const char* type) {
    DiscoveredReaderData rd{};
    rd.readerGuid = guid;
    (void)rd.topicName.assign(topic);
    (void)rd.typeName.assign(type);
    rd.reliabilityKind = ReliabilityKind::RELIABLE;
    return rd;
}

/// The arena itself: strings read back, an equal string is stored once, and a release closes its
/// gap.
bool check_arena(std::string& err) {
    StringArena<40> names;
    const uint16_t  a = names.intern("rt/a");
    const uint16_t  b = names.intern("rt/bbbb");
    const uint16_t  t = names.intern("T_");
    CHECK(a == 0u && b == 5u && t == 13u && names.used() == 16u,
          "strings are packed end to end, each with its terminator");
    CHECK(names.intern("rt/bbbb") == b && names.cost("rt/bbbb") == 0u && names.used() == 16u,
          "an equal string is not stored again: intern() returns the stored one");
    CHECK(names.find("rt/bbb") == StringArena<40>::kNone
              && names.find("bbbb") == StringArena<40>::kNone,
          "a prefix or a suffix of a stored string is not found");
    // 25 octets, more than the 24 left.
    CHECK(names.intern("rt/twenty-four-chars-xxx") == StringArena<40>::kNone && names.used() == 16u,
          "a string that does not fit is refused and writes nothing");

    const uint16_t gone = names.release(b);
    const uint16_t t_now = static_cast<uint16_t>(t - gone);
    CHECK(gone == 8u && names.used() == 8u, "release returns the string's size");
    CHECK(names.str(t)[0] == '\0', "the octets the release vacated read as an empty string");
    CHECK(strcmp(names.str(a), "rt/a") == 0 && strcmp(names.str(t_now), "T_") == 0,
          "the strings below and above the released one read back unchanged");
    CHECK(names.intern("rt/twenty-four-chars-xxx") == 8u && names.used() == 33u,
          "the released octets are free again at the end of the arena");
    return true;
}

}  // namespace

static bool tc_sedp_8_5_5_1_014(std::string& err) {
    if (!check_arena(err)) {
        return false;
    }

    LocalNamesObs      obs{};
    EndpointMatchTable t(record_match, nullptr, &obs);
    GuidPrefix         local{};
    local.value[0] = 0x10u;
    t.set_local_prefix(local);
    GuidPrefix remote{};
    remote.value[0] = 0x20u;

    const Guid w1 = guid_in(local, 1u, 0x02u);
    const Guid w2 = guid_in(local, 2u, 0x02u);
    const Guid w3 = guid_in(local, 3u, 0x02u);
    t.add_writer(writer(w1, "rt/a", "pkg::A_"));
    t.add_writer(writer(w2, "rt/bbbbbbbb", "pkg::B_"));
    t.add_writer(writer(w3, "rt/c", "pkg::C_"));
    CHECK(t.local_name_bytes_used() == 13u + 20u + 13u,
          "each local writer takes strlen(topic) + strlen(type) + 2 octets");

    t.remove_writer(w2);
    DiscoveredWriterData back{};
    CHECK(t.local_name_bytes_used() == 26u, "removing a writer frees its names");
    CHECK(t.local_writer_data(w3, back) && strcmp(back.topicName.str, "rt/c") == 0
              && strcmp(back.typeName.str, "pkg::C_") == 0,
          "a writer above the removed one reads its names back for SEDP");

    t.add_reader(reader(guid_in(remote, 9u, 0x07u), "rt/c", "pkg::C_"));
    CHECK(obs.calls == 1 && strcmp(obs.topic.str, "rt/c") == 0,
          "a remote reader matches the moved writer and on_match names its topic");

    t.add_writer(writer(w1, "rt/a2", "pkg::A_"));
    CHECK(t.local_writer_data(w1, back) && strcmp(back.topicName.str, "rt/a2") == 0
              && t.local_name_bytes_used() == 27u,
          "renaming a writer keeps its type, stores the new topic and frees the old one");
    CHECK(t.local_writer_data(w3, back) && strcmp(back.topicName.str, "rt/c") == 0,
          "renaming leaves the other writers' names intact");
    CHECK(
        t.count("rt/a", true) == 0u && t.count("rt/a2", true) == 1u && t.count("rt/c", true) == 1u,
        "count() finds local writers by their stored names");

    obs.calls = 0;
    t.add_reader(reader(guid_in(local, 4u, 0x07u), "rt/c", "pkg::C_"));
    CHECK(obs.calls == 1 && t.local_name_bytes_used() == 27u,
          "a local reader of a writer's topic and type shares its names and matches it");
    t.add_reader(reader(guid_in(local, 5u, 0x07u), "rt/c", "pkg::D_"));
    CHECK(obs.calls == 1 && t.local_name_bytes_used() == 35u,
          "a local reader of another type stores only that type and does not match");

    const Guid w4 = guid_in(local, 6u, 0x02u);
    t.add_writer(writer(w4, "rt/d", "pkg::C_"));
    CHECK(t.local_name_bytes_used() == 40u, "a writer of a stored type stores only its topic");
    t.remove_writer(w3);
    CHECK(t.local_name_bytes_used() == 40u,
          "removing a writer frees no name another endpoint still uses");
    t.remove_reader(guid_in(local, 4u, 0x07u));
    t.remove_reader(guid_in(local, 5u, 0x07u));
    CHECK(t.local_name_bytes_used() == 27u && t.local_writer_data(w4, back)
              && strcmp(back.topicName.str, "rt/d") == 0
              && strcmp(back.typeName.str, "pkg::C_") == 0,
          "once the last user of a name is removed it is freed, and shared names still read back");
    return true;
}

void append_sedp_tests_8_5_5_1_local_names(std::vector<TestCase>& tests) {
    tests.push_back({"TC-SEDP-8.5.5.1-014",
                     "§8.5.4.2/§8.5.5.1: local endpoint names interned in one arena are stored "
                     "once however many endpoints share them, read back exactly after another "
                     "endpoint is removed or renamed, and still decide matching, count() and "
                     "the SEDP readback.",
                     tc_sedp_8_5_5_1_014});
}

}  // namespace spec_test
