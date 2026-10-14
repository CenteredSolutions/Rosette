/**
 * @file test_participant_writer_slot_classes.cpp
 * @brief TC-PART-8.5.1-013: the participant's deep and shallow writer slots.
 *
 * Built as its own program, writer_slot_class_spec_tests, with four writer
 * slots of which two are shallow slots for writers of depth 2 at most
 * (RTPS_MAX_SHALLOW_WRITERS=2, RTPS_SHALLOW_WRITER_DEPTH=2), while every
 * other spec program keeps the default of deep slots only.
 *
 *   ./writer_slot_class_spec_tests
 */

#include "participant_test_support.hpp"

#include <stdio.h>

#include <string>

static_assert(RTPS_MAX_USER_WRITERS == 4 && RTPS_MAX_SHALLOW_WRITERS == 2
                  && RTPS_SHALLOW_WRITER_DEPTH == 2,
              "this program checks two deep and two depth-2 shallow writer slots");

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

WriterConfig writer_on(const char* topic) {
    WriterConfig cfg{};
    (void)cfg.topicName.assign(topic);
    (void)cfg.typeName.assign("T_");
    // TRANSIENT_LOCAL keeps every change with no reader matched, so the
    // history fills to its depth.
    cfg.durability = DurabilityKind::TRANSIENT_LOCAL;
    return cfg;
}

/// Writes @p count changes and reports whether the history holds @p expected of them.
bool fills_to(StatefulWriter* w, uint32_t count, uint8_t expected) {
    const Octet sample[4] = {1u, 2u, 3u, 4u};
    for (uint32_t i = 0u; i < count; ++i) {
        if (!w->write_async(ChangeKind::ALIVE, sample, sizeof(sample))) {
            return false;
        }
    }

    return w->cache().capacity() == expected && w->cache().size() == expected;
}

}  // namespace

static bool tc_part_8_5_1_013(std::string& err) {
    CaptureTransport tx{};
    RTPSParticipant  p(0u, 1u, tx);
    CHECK(p.add_ipv4_address(0xC0A8010Au), "local address setup should succeed");
    CHECK(p.free_writer_slots(false) == 2u && p.free_writer_slots(true) == 2u,
          "the table starts with two free slots of each class");

    StatefulWriter* deep1 = p.create_writer<8>(writer_on("rt/deep1"));
    StatefulWriter* deep2 = p.create_writer<4>(writer_on("rt/deep2"));
    CHECK(deep1 != nullptr && deep2 != nullptr && p.free_writer_slots(false) == 0u
              && p.free_writer_slots(true) == 2u,
          "writers deeper than a shallow slot take deep slots");
    CHECK(p.create_writer<8>(writer_on("rt/deep3")) == nullptr && p.free_writer_slots(true) == 2u,
          "a deep writer is refused rather than put in a shallow slot too small for it");

    StatefulWriter* shallow1 = p.create_writer<1>(writer_on("rt/shallow1"));
    StatefulWriter* shallow2 = p.create_writer<2>(writer_on("rt/shallow2"));
    CHECK(shallow1 != nullptr && shallow2 != nullptr && p.free_writer_slots(true) == 0u,
          "writers that fit a shallow slot take one");
    CHECK(fills_to(shallow2, 3u, 2u) && fills_to(shallow1, 2u, 1u) && fills_to(deep1, 9u, 8u)
              && fills_to(deep2, 5u, 4u) && shallow2->cache().size() == 2u,
          "each writer keeps its own depth, and the shallow slots do not overlap their "
          "neighbours");
    CHECK(p.create_writer<1>(writer_on("rt/full")) == nullptr, "a full table refuses any writer");

    CHECK(p.delete_writer(deep2), "a deep writer is deleted");
    StatefulWriter* overflow = p.create_writer<1>(writer_on("rt/overflow"));
    CHECK(overflow != nullptr && p.free_writer_slots(false) == 0u,
          "a shallow writer takes a deep slot once every shallow slot is taken");
    CHECK(p.delete_writer(shallow1) && p.free_writer_slots(true) == 1u
              && p.create_writer<4>(writer_on("rt/deep4")) == nullptr,
          "a free shallow slot does not hold a writer deeper than it");

    CHECK(p.delete_writer(overflow), "the overflowing shallow writer is deleted");
    StatefulWriter* deep5 = p.create_writer<8>(writer_on("rt/deep5"));
    CHECK(deep5 != nullptr && fills_to(deep5, 8u, 8u),
          "a freed deep slot holds a deep writer again");
    StatefulWriter* shallow3 = p.create_writer<2>(writer_on("rt/shallow3"));
    CHECK(shallow3 != nullptr && p.free_writer_slots(true) == 0u && fills_to(shallow3, 2u, 2u)
              && shallow2->cache().size() == 2u,
          "a freed shallow slot is reused by the next shallow writer");
    return true;
}

}  // namespace spec_test

int main() {
    const spec_test::TestCase tests[] = {
        {"TC-PART-8.5.1-013",
         "§8.5.1: a writer that fits a shallow slot takes a free one before a deep slot, a "
         "deeper writer takes only a deep slot, and every writer keeps its own depth.",
         spec_test::tc_part_8_5_1_013},
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
