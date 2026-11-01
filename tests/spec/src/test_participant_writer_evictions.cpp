// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file test_participant_writer_evictions.cpp
 * @brief TC-PART-8.5.1-014: the participant's total of evicted unacknowledged changes.
 *
 * Each StatefulWriter counts the changes KEEP_LAST evicted before their
 * readers had them (StatefulWriter::evicted_unacknowledged()). The
 * participant sums them over its user writers, deleted ones included, so a
 * board reads one number for the session: for an rmw_rosette board, a count
 * of service replies lost to too shallow a reply history.
 */

#include "participant_test_support.hpp"

namespace spec_test {

#define CHECK(COND, MSG)  \
    do {                  \
        if (!(COND)) {    \
            err = (MSG);  \
            return false; \
        }                 \
    } while (0)

namespace {

using participant_support::CaptureTransport;

constexpr Octet kPayload[4] = {0x00u, 0x01u, 0x00u, 0x00u};

/// A service reply writer's configuration on `topic`.
WriterConfig reply_writer_on(const char* topic) {
    WriterConfig cfg{};
    (void)cfg.topicName.assign(topic);
    (void)cfg.typeName.assign("T_");
    cfg.durability = DurabilityKind::TRANSIENT_LOCAL;
    cfg.releaseRepliesOnDelivery = true;
    return cfg;
}

/// Writes `count` replies to a client that has not matched, so none is delivered.
bool write_replies(StatefulWriter* w, int count) {
    InlineQos qos{};
    qos.hasRelatedSampleIdentity = true;
    qos.relatedSampleIdentity.writerGuid.prefix.value[0] = 0x77u;
    qos.relatedSampleIdentity.writerGuid.entityId = make_entity(7u, 8u, 9u, 0x03u);
    for (int i = 0; i < count; ++i) {
        qos.relatedSampleIdentity.sequenceNumber = SequenceNumber::from_int64(i + 1);
        if (!w->write_async(ChangeKind::ALIVE, kPayload, sizeof(kPayload), qos)) {
            return false;
        }
    }

    return true;
}

}  // namespace

static bool tc_part_8_5_1_014(std::string& err) {
    CaptureTransport tx{};
    RTPSParticipant  p(0u, 1u, tx);
    CHECK(p.add_ipv4_address(0xC0A8010Au), "local address setup should succeed");
    CHECK(p.evicted_unacknowledged() == 0u, "a new participant has evicted nothing");

    StatefulWriter* first = p.create_writer<1>(reply_writer_on("rr/first"));
    StatefulWriter* second = p.create_writer<2>(reply_writer_on("rr/second"));
    CHECK(first != nullptr && second != nullptr, "two reply writers are created");
    CHECK(write_replies(first, 2) && write_replies(second, 4),
          "replies wait for a client that has not matched");
    CHECK(first->evicted_unacknowledged() == 1u && second->evicted_unacknowledged() == 2u,
          "each writer counts the undelivered replies its history evicted");
    CHECK(p.evicted_unacknowledged() == 3u, "the participant sums its writers' counts");

    CHECK(p.delete_writer(first) && p.evicted_unacknowledged() == 3u,
          "deleting a writer keeps its evictions in the total");
    CHECK(write_replies(second, 1) && p.evicted_unacknowledged() == 4u,
          "the live writers' evictions still add to it");
    return true;
}

void append_participant_writer_eviction_tests(std::vector<TestCase>& tests) {
    tests.push_back({"TC-PART-8.5.1-014",
                     "§8.5.1/§8.3.3.1: the participant's count of changes evicted before their "
                     "readers had them sums every user writer's count, deleted writers "
                     "included.",
                     tc_part_8_5_1_014});
}

}  // namespace spec_test
