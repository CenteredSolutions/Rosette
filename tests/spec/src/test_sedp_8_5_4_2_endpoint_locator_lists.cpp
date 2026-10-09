/**
 * @file test_sedp_8_5_4_2_endpoint_locator_lists.cpp
 * @brief Where EndpointMatchTable keeps a remote endpoint's announced locators (§8.5.4.2).
 *
 * An endpoint that announces no unicast locators, or a list whose preferred
 * entry is that of its participant's defaults as the SPDP peer table keeps
 * them, keeps no list of its own and is reported with none, so the
 * participant reaches it at the defaults. Only an endpoint addressed
 * elsewhere takes one of the
 * RTPS_MAX_REMOTE_LOCATOR_LISTS lists, under the same capacity rule as an
 * origin (see Storage and Capacity in rtps/endpoint_match_table.hpp).
 */

#include "spec_test_common.hpp"

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

/// One remote participant's defaults, and what on_match last reported.
struct Fixture {
    GuidPrefix prefix = {};
    Locator defaults[RTPS_MAX_REMOTE_ENDPOINT_LOCATORS] = {};
    uint8_t default_count = 1u;
    int matches = 0;
    int unmatches = 0;
    uint8_t reported_count = 0xFFu;
    Locator reported_first = LOCATOR_INVALID;
};

bool lookup_defaults(const GuidPrefix& prefix, const Locator*& locators, uint8_t& count,
                     void* user) noexcept {
    auto* f = static_cast<Fixture*>(user);
    if (prefix != f->prefix) {
        return false;
    }

    locators = f->defaults;
    count = f->default_count;
    return true;
}

/// The fixture's defaults for every remote participant, as for a network of
/// peers that each announce the same default list.
bool lookup_defaults_of_any(const GuidPrefix&, const Locator*& locators, uint8_t& count,
                            void* user) noexcept {
    auto* f = static_cast<Fixture*>(user);
    locators = f->defaults;
    count = f->default_count;
    return true;
}

bool on_match(const MatchedEndpoint& w, const MatchedEndpoint&, const TopicName&,
              void* user) noexcept {
    auto* f = static_cast<Fixture*>(user);
    f->matches++;
    f->reported_count = w.locatorCount;
    f->reported_first = w.locatorCount > 0u ? w.locators[0] : LOCATOR_INVALID;
    return true;
}

void on_unmatch(const Guid&, const Guid&, void* user) noexcept {
    static_cast<Fixture*>(user)->unmatches++;
}

Guid make_guid(Octet p0, Octet key) {
    Guid g{};
    g.prefix.value[0] = p0;
    g.prefix.value[1] = 0x5Au;
    g.entityId = make_entity(0u, 0u, key, 0x03u);
    return g;
}

DiscoveredWriterData make_writer(const Guid& g, const char* topic, const Locator* locator) {
    DiscoveredWriterData wd{};
    wd.writerGuid = g;
    wd.topicName.assign(topic);
    wd.typeName.assign("pkg::msg::dds_::T_");
    wd.reliabilityKind = ReliabilityKind::RELIABLE;
    wd.durabilityKind = DurabilityKind::VOLATILE;
    if (locator != nullptr) {
        wd.unicastLocatorList[0] = *locator;
        wd.unicastLocatorCount = 1u;
    }

    return wd;
}

DiscoveredReaderData make_reader(const Guid& g, const char* topic, const Locator* locator) {
    DiscoveredReaderData rd{};
    rd.readerGuid = g;
    rd.readerGuid.entityId.entityKind = 0x04u;
    rd.topicName.assign(topic);
    rd.typeName.assign("pkg::msg::dds_::T_");
    rd.reliabilityKind = ReliabilityKind::RELIABLE;
    rd.durabilityKind = DurabilityKind::VOLATILE;
    if (locator != nullptr) {
        rd.unicastLocatorList[0] = *locator;
        rd.unicastLocatorCount = 1u;
    }

    return rd;
}

}  // namespace

// §8.5.4.2: an endpoint announcing its participant's defaults is reported
// with no locators, as one announcing none is, so it is reached at the
// defaults the peer table keeps; any other list is reported as announced,
// and moving between the two rematches the endpoint.
static bool tc_sedp_8_5_4_2_001(std::string& err) {
    Fixture f{};
    const Locator lan = make_udpv4_locator(0xC0A80164u, 7411u);
    const Locator other = make_udpv4_locator(0xC0A80165u, 7411u);
    const Guid local_reader = make_guid(0x01u, 1u);
    f.prefix = make_guid(0x20u, 0u).prefix;
    f.defaults[0] = lan;

    EndpointMatchTable t(on_match, on_unmatch, &f);
    t.set_participant_defaults(lookup_defaults);
    t.set_local_prefix(local_reader.prefix);
    t.add_reader(make_reader(local_reader, "rt/a", nullptr));

    t.add_writer(make_writer(make_guid(0x20u, 1u), "rt/a", &lan));
    CHECK(f.matches == 1 && f.reported_count == 0u,
          "a writer announcing its participant's defaults is reported with none");

    t.add_writer(make_writer(make_guid(0x20u, 2u), "rt/a", nullptr));
    CHECK(f.matches == 2 && f.reported_count == 0u,
          "a writer announcing nothing is reported with none");

    t.add_writer(make_writer(make_guid(0x20u, 2u), "rt/a", &lan));
    CHECK(f.unmatches == 0, "announcing the defaults instead of nothing rematches nothing");

    t.add_writer(make_writer(make_guid(0x20u, 3u), "rt/a", &other));
    CHECK(f.matches == 3 && f.reported_count == 1u && f.reported_first == other,
          "a writer announcing other locators is reported with them");

    t.add_writer(make_writer(make_guid(0x30u, 1u), "rt/a", &lan));
    CHECK(f.matches == 4 && f.reported_count == 1u && f.reported_first == lan,
          "a writer of a participant with no known defaults keeps its own list");

    t.add_writer(make_writer(make_guid(0x20u, 1u), "rt/a", &other));
    CHECK(f.unmatches == 1 && f.matches == 5 && f.reported_count == 1u && f.reported_first == other,
          "leaving the defaults rematches the writer at its own locators");

    t.add_writer(make_writer(make_guid(0x20u, 1u), "rt/a", &lan));
    CHECK(f.unmatches == 2 && f.matches == 6 && f.reported_count == 0u,
          "returning to the defaults rematches the writer with none");
    return true;
}

static_assert(RTPS_MAX_REMOTE_ORIGINS >= RTPS_MAX_REMOTE_LOCATOR_LISTS + 3,
              "TC-SEDP-8.5.4.2-002 needs free origins beyond the locator lists");
static_assert(RTPS_MAX_DISCOVERED_READERS >= RTPS_MAX_REMOTE_LOCATOR_LISTS + 2,
              "TC-SEDP-8.5.4.2-002 needs room for a reader per list and two without");

// §8.5.4.2 capacity: with every locator list taken, an endpoint with
// locators of its own that completes no match is not kept, while one with
// none still takes a free origin; one that completes a match frees an
// unmatched origin that has a list, never one without.
static bool tc_sedp_8_5_4_2_002(std::string& err) {
    Fixture f{};
    const Guid local_reader = make_guid(0x01u, 1u);
    EndpointMatchTable t(on_match, on_unmatch, &f);
    t.set_local_prefix(local_reader.prefix);
    t.add_reader(make_reader(local_reader, "rt/estop", nullptr));

    // Taken first, so an eviction that ignores lists would free its origin.
    t.add_reader(make_reader(make_guid(0x40u, 2u), "rt/parameter_events", nullptr));
    for (uint8_t i = 0; i < RTPS_MAX_REMOTE_LOCATOR_LISTS; ++i) {
        const Locator own = make_udpv4_locator(0xC0A80170u + i, 7411u);
        t.add_reader(make_reader(make_guid(static_cast<Octet>(0x50u + i), 2u), "rt/rosout", &own));
    }

    CHECK(t.count("rt/rosout", false) == RTPS_MAX_REMOTE_LOCATOR_LISTS,
          "a reader with its own locators is kept per list");

    const Locator spare = make_udpv4_locator(0xC0A801A0u, 7411u);
    t.add_reader(make_reader(make_guid(0x60u, 2u), "rt/rosout", &spare));
    CHECK(t.count("rt/rosout", false) == RTPS_MAX_REMOTE_LOCATOR_LISTS,
          "with every list taken, an unmatched reader with its own locators is not kept");

    t.add_reader(make_reader(make_guid(0x61u, 2u), "rt/parameter_events", nullptr));
    CHECK(t.count("rt/parameter_events", false) == 2u,
          "a reader with no locators of its own still takes a free origin");

    t.add_writer(make_writer(make_guid(0x62u, 1u), "rt/estop", &spare));
    CHECK(f.matches == 1 && f.reported_count == 1u && f.reported_first == spare,
          "a writer with its own locators that completes a match is admitted with them");
    CHECK(t.count("rt/rosout", false) == RTPS_MAX_REMOTE_LOCATOR_LISTS - 1u,
          "exactly one unmatched origin with a list is freed");
    CHECK(t.count("rt/parameter_events", false) == 2u,
          "no origin without a list is freed for a list");
    return true;
}

namespace {

/// The locator each remote reader was last matched at, by the first prefix octet.
struct ReaderLocators {
    Locator by_prefix[256] = {};
};

bool record_reader(const MatchedEndpoint&, const MatchedEndpoint& r, const TopicName&,
                   void* user) noexcept {
    auto* seen = static_cast<ReaderLocators*>(user);
    seen->by_prefix[r.guid.prefix.value[0]] = r.locatorCount > 0u ? r.locators[0] : LOCATOR_INVALID;
    return true;
}

}  // namespace

// §8.5.4.2 capacity: with every locator list taken, an endpoint that holds
// one and re-announces other locators of its own moves to them, its old list
// serving the new ones, even when it completes no match.
static bool tc_sedp_8_5_4_2_003(std::string& err) {
    ReaderLocators seen{};
    EndpointMatchTable t(record_reader, nullptr, &seen);
    const Guid local_writer = make_guid(0x01u, 1u);
    t.set_local_prefix(local_writer.prefix);

    for (uint8_t i = 0; i < RTPS_MAX_REMOTE_LOCATOR_LISTS; ++i) {
        const Locator own = make_udpv4_locator(0xC0A80170u + i, 7411u);
        t.add_reader(make_reader(make_guid(static_cast<Octet>(0x50u + i), 2u), "rt/rosout", &own));
    }

    const Locator moved = make_udpv4_locator(0xC0A801B0u, 7411u);
    t.add_reader(make_reader(make_guid(0x50u, 2u), "rt/rosout", &moved));
    CHECK(t.count("rt/rosout", false) == RTPS_MAX_REMOTE_LOCATOR_LISTS,
          "the moved reader is still the only record of itself");

    t.add_writer(make_writer(local_writer, "rt/rosout", nullptr));
    CHECK(seen.by_prefix[0x50u] == moved, "the moved reader is matched at its new locator");
    CHECK(seen.by_prefix[0x51u] == make_udpv4_locator(0xC0A80171u, 7411u),
          "the other readers keep their own locators");
    return true;
}

static_assert(RTPS_MAX_REMOTE_ENDPOINT_LOCATORS >= 2,
              "TC-SEDP-8.5.4.2-004 needs endpoints that keep two locators");
static_assert(RTPS_MAX_DISCOVERED_WRITERS >= RTPS_MAX_REMOTE_LOCATOR_LISTS + 2,
              "TC-SEDP-8.5.4.2-004 needs a writer per list and two more");

namespace {

/// A writer of `prefix_octet` on `topic` announcing `first` then `second`.
DiscoveredWriterData make_writer_with_two(Octet prefix_octet, const char* topic,
                                          const Locator& first, const Locator& second) {
    DiscoveredWriterData wd = make_writer(make_guid(prefix_octet, 1u), topic, &first);
    wd.unicastLocatorList[1] = second;
    wd.unicastLocatorCount = 2u;
    return wd;
}

}  // namespace

// §8.5.4.2 with two-tier peers: a peer reactivated from its known-peer
// record has only its preferred default until its next SPDP announcement,
// while its replayed endpoints announce its full default list. Those
// endpoints share the preferred entry, so they take no locator list however
// many such peers there are, and the refresh restoring the full defaults
// rematches nothing; an endpoint whose preferred entry differs keeps its list.
static bool tc_sedp_8_5_4_2_004(std::string& err) {
    Fixture f{};
    const Locator lan = make_udpv4_locator(0xC0A80164u, 7411u);
    const Locator lan_alt = make_udpv4_locator(0xC0A80166u, 7411u);
    const Locator other = make_udpv4_locator(0xC0A80165u, 7411u);
    const Guid local_reader = make_guid(0x01u, 1u);
    f.defaults[0] = lan;
    f.defaults[1] = lan_alt;
    f.default_count = 1u;

    EndpointMatchTable t(on_match, on_unmatch, &f);
    t.set_participant_defaults(lookup_defaults_of_any);
    t.set_local_prefix(local_reader.prefix);
    t.add_reader(make_reader(local_reader, "rt/a", nullptr));

    for (uint8_t i = 0; i <= RTPS_MAX_REMOTE_LOCATOR_LISTS; ++i) {
        t.add_writer(make_writer_with_two(static_cast<Octet>(0x20u + i), "rt/a", lan, lan_alt));
        CHECK(f.matches == i + 1 && f.reported_count == 0u,
              "a replayed writer announcing the full defaults against a rebuilt slot is "
              "reported with none");
    }

    f.default_count = 2u;
    t.add_writer(make_writer_with_two(0x20u, "rt/a", lan, lan_alt));
    CHECK(f.unmatches == 0, "the SPDP refresh restoring the full defaults rematches nothing");

    t.add_writer(make_writer_with_two(0x40u, "rt/a", other, lan));
    CHECK(f.matches == RTPS_MAX_REMOTE_LOCATOR_LISTS + 2 && f.reported_count == 2u
              && f.reported_first == other,
          "with every list still free, a writer addressed elsewhere is kept with its own list");
    return true;
}

void append_sedp_tests_8_5_4_2_endpoint_locator_lists(std::vector<TestCase>& tests) {
    tests.push_back({"TC-SEDP-8.5.4.2-001",
                     "§8.5.4.2: a remote endpoint addressed at its participant's preferred "
                     "default keeps no list and is reported with none, like one announcing none; "
                     "other lists are kept and reported as announced.",
                     tc_sedp_8_5_4_2_001});
    tests.push_back({"TC-SEDP-8.5.4.2-002",
                     "§8.5.4.2 capacity: with every locator list taken, an endpoint with its own "
                     "locators is kept only when it completes a match, and then frees an "
                     "unmatched origin that has a list.",
                     tc_sedp_8_5_4_2_002});
    tests.push_back({"TC-SEDP-8.5.4.2-003",
                     "§8.5.4.2 capacity: with every locator list taken, an endpoint that "
                     "re-announces other locators of its own moves to them, completing a match "
                     "or not.",
                     tc_sedp_8_5_4_2_003});
    tests.push_back({"TC-SEDP-8.5.4.2-004",
                     "§8.5.4.2 with two-tier peers: endpoints replayed by peers reactivated from "
                     "their records, whose defaults are rebuilt to the preferred entry, take no "
                     "locator list, and the SPDP refresh restoring the defaults rematches none.",
                     tc_sedp_8_5_4_2_004});
}

}  // namespace spec_test
