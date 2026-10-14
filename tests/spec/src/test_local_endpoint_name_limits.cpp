/**
 * @file test_local_endpoint_name_limits.cpp
 * @brief What happens when the match table's local name arena is full: TC-SEDP-8.5.5.1-015 for
 * the table, TC-PART-8.5.1-012 for the participant.
 *
 * Built as its own program, local_name_limit_spec_tests, with RTPS_LOCAL_NAME_BYTES=160, small
 * enough for a few short names to fill, while every other spec program keeps the default arena
 * that the short names of their own tests never fill.
 *
 *   ./local_name_limit_spec_tests
 */

#include "participant_test_support.hpp"

#include <stdio.h>

#include <string>

static_assert(RTPS_LOCAL_NAME_BYTES == 160, "this program checks a 160-octet name arena");

namespace spec_test {

using participant_support::CaptureTransport;

#define CHECK(COND, MSG)  \
    do {                  \
        if (!(COND)) {    \
            err = (MSG);  \
            return false; \
        }                 \
    } while (0)

namespace {

/// A topic name that takes @p octets in the arena (strlen + 1), filled with @p fill.
std::string topic_of_size(uint32_t octets, char fill = 'x') {
    return "rt/" + std::string(octets - 4u, fill);
}

TopicName name(const std::string& text) {
    TopicName n{};
    (void)n.assign(text.c_str());
    return n;
}

DiscoveredWriterData local_writer(const GuidPrefix& prefix, Octet key, const std::string& topic) {
    DiscoveredWriterData wd{};
    wd.writerGuid = Guid{prefix, make_entity(0u, 0u, key, 0x03u)};
    (void)wd.topicName.assign(topic.c_str());
    (void)wd.typeName.assign("T_");
    return wd;
}

DiscoveredReaderData local_reader(const GuidPrefix& prefix, Octet key, const std::string& topic) {
    DiscoveredReaderData rd{};
    rd.readerGuid = Guid{prefix, make_entity(0u, 0u, key, 0x04u)};
    (void)rd.topicName.assign(topic.c_str());
    (void)rd.typeName.assign("T_");
    return rd;
}

}  // namespace

// Every endpoint below has type "T_" (3 octets), stored once and shared, so
// after the first endpoint each new one costs only its topic.
static bool tc_sedp_8_5_5_1_015(std::string& err) {
    EndpointMatchTable t;
    GuidPrefix         local{};
    local.value[0] = 0x10u;
    t.set_local_prefix(local);
    const TopicName type = name("T_");

    const DiscoveredWriterData w1 = local_writer(local, 1u, topic_of_size(97u));
    t.add_writer(w1);
    CHECK(t.local_name_bytes_used() == 100u,
          "a 97-octet topic and its type take 100 of 160 octets");
    CHECK(
        t.has_room_for_local(name(topic_of_size(60u, 'y')), type, true)
            && !t.has_room_for_local(name(topic_of_size(61u, 'y')), type, true)
            && !t.has_room_for_local(name(topic_of_size(61u, 'y')), type, false),
        "with the type stored, a topic of up to the 60 free octets fits, for a writer or a reader");
    CHECK(t.has_room_for_local(name(topic_of_size(57u, 'y')), name("U_"), true)
              && !t.has_room_for_local(name(topic_of_size(58u, 'y')), name("U_"), true),
          "a type not yet stored counts against the free octets too");

    const DiscoveredWriterData w2 = local_writer(local, 2u, topic_of_size(67u, 'y'));
    DiscoveredWriterData       back{};
    t.add_writer(w2);
    CHECK(!t.local_writer_data(w2.writerGuid, back) && t.local_name_bytes_used() == 100u,
          "a writer whose names do not fit is not recorded and the arena is unchanged");

    t.add_reader(local_reader(local, 3u, topic_of_size(60u, 'z')));
    CHECK(t.local_name_bytes_used() == 160u, "names that fit exactly are recorded");

    t.remove_writer(w1.writerGuid);
    CHECK(t.local_name_bytes_used() == 63u,
          "removing the writer frees its topic but not the type the reader still uses");
    t.add_writer(w2);
    CHECK(t.local_writer_data(w2.writerGuid, back)
              && strcmp(back.topicName.str, w2.topicName.str) == 0,
          "removing a writer makes room for one that did not fit");

    EndpointMatchTable records;
    records.set_local_prefix(local);
    for (Octet key = 1u; key <= RTPS_MAX_LOCAL_WRITERS; ++key) {
        records.add_writer(local_writer(local, key, "rt/n"));
    }

    CHECK(!records.has_room_for_local(name("rt/n"), type, true)
              && records.has_room_for_local(name("rt/n"), type, false),
          "with every writer record taken there is no room for a writer, but still for a reader");
    return true;
}

static bool tc_part_8_5_1_012(std::string& err) {
    CaptureTransport tx{};
    RTPSParticipant  p(0u, 1u, tx);
    CHECK(p.add_ipv4_address(0xC0A8010Au), "local address setup should succeed");

    WriterConfig big{};
    (void)big.topicName.assign(topic_of_size(97u).c_str());
    (void)big.typeName.assign("T_");
    StatefulWriter* first = p.create_writer<1>(big);
    CHECK(first != nullptr, "a writer whose names fit the arena is created");
    WriterConfig other = big;
    (void)other.topicName.assign(topic_of_size(61u, 'y').c_str());
    CHECK(p.create_writer<1>(other) == nullptr,
          "a 61-octet topic does not fit in the 60 octets left and is refused");

    ReaderConfig rcfg{};
    (void)rcfg.typeName.assign("T_");
    (void)rcfg.topicName.assign(topic_of_size(61u, 'z').c_str());
    CHECK(p.create_reader(rcfg) == nullptr, "readers share the arena: 61 octets do not fit in 60");
    (void)rcfg.topicName.assign(topic_of_size(60u, 'z').c_str());
    CHECK(p.create_reader(rcfg) != nullptr, "a reader whose names fit exactly is created");

    CHECK(p.delete_writer(first), "the first writer is deleted");
    (void)other.topicName.assign(topic_of_size(97u, 'w').c_str());
    CHECK(p.create_writer<1>(other) != nullptr,
          "its topic is freed, and the refusals took no writer slot or proxy floor");
    return true;
}

}  // namespace spec_test

int main() {
    const spec_test::TestCase tests[] = {
        {"TC-SEDP-8.5.5.1-015",
         "§8.5.4.2/§8.5.5.1: a local endpoint the match table has no record or name room for is "
         "not recorded, has_room_for_local() says so beforehand, and removal makes room again.",
         spec_test::tc_sedp_8_5_5_1_015},
        {"TC-PART-8.5.1-012",
         "§8.5.1/§8.5.4.2: the participant refuses a writer or reader the match table could not "
         "record (and SEDP so could not announce) before building it.",
         spec_test::tc_part_8_5_1_012},
    };

    int failures = 0;
    for (const spec_test::TestCase& tc : tests) {
        std::string err{};
        if (tc.fn(err)) {
            printf("[PASS] %s\n  Spec: %s\n", tc.id, tc.doc);
        } else {
            fprintf(stderr, "[FAIL] %s\n  Spec: %s\n  Error: %s\n", tc.id, tc.doc, err.c_str());
            ++failures;
        }
    }

    return failures == 0 ? 0 : 1;
}
