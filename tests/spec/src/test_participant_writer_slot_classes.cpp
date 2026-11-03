// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

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

WriterConfig writer_on(const char* topic, WriterSlotClass slot_class = WriterSlotClass::DEEP) {
    WriterConfig cfg{};
    (void)cfg.topicName.assign(topic);
    (void)cfg.typeName.assign("T_");
    cfg.slotClass = slot_class;
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
    constexpr WriterSlotClass kShallow = WriterSlotClass::SHALLOW;

    CaptureTransport tx{};
    RTPSParticipant  p(0u, 1u, tx);
    CHECK(p.add_ipv4_address(0xC0A8010Au), "local address setup should succeed");
    CHECK(p.free_writer_slots(false) == 2u && p.free_writer_slots(true) == 2u,
          "the table starts with two free slots of each class");

    // A DEEP writer takes a deep slot whatever its depth, so a depth-1 writer
    // never uses up a slot a SHALLOW writer is sized for.
    StatefulWriter* deep1 = p.create_writer<8>(writer_on("rt/deep1"));
    StatefulWriter* deep2 = p.create_writer<1>(writer_on("rt/deep2"));
    CHECK(deep1 != nullptr && deep2 != nullptr && p.free_writer_slots(false) == 0u
              && p.free_writer_slots(true) == 2u,
          "DEEP writers, of any depth, take deep slots");
    CHECK(p.create_writer<1>(writer_on("rt/deep3")) == nullptr && p.free_writer_slots(true) == 2u,
          "a DEEP writer is refused rather than put in a shallow slot, even one it would fit");

    StatefulWriter* shallow1 = p.create_writer<1>(writer_on("rt/shallow1", kShallow));
    StatefulWriter* shallow2 = p.create_writer<2>(writer_on("rt/shallow2", kShallow));
    CHECK(shallow1 != nullptr && shallow2 != nullptr && p.free_writer_slots(true) == 0u,
          "SHALLOW writers take shallow slots");
    CHECK(fills_to(shallow2, 3u, 2u) && fills_to(shallow1, 2u, 1u) && fills_to(deep1, 9u, 8u)
              && fills_to(deep2, 2u, 1u) && shallow2->cache().size() == 2u,
          "each writer keeps its own depth, and the shallow slots do not overlap their "
          "neighbours");
    CHECK(p.create_writer<1>(writer_on("rt/full")) == nullptr
              && p.create_writer<1>(writer_on("rt/full", kShallow)) == nullptr,
          "a full table refuses any writer");

    // Endpoint churn: a freed slot returns to its own class only.
    CHECK(p.delete_writer(deep2), "a deep writer is deleted");
    CHECK(p.create_writer<1>(writer_on("rt/spill", kShallow)) == nullptr
              && p.free_writer_slots(false) == 1u,
          "a SHALLOW writer is refused rather than spilled into a free deep slot");
    CHECK(p.delete_writer(shallow1) && p.free_writer_slots(true) == 1u
              && p.create_writer<4>(writer_on("rt/toodeep", kShallow)) == nullptr,
          "a free shallow slot does not hold a writer deeper than it");

    StatefulWriter* deep3 = p.create_writer<8>(writer_on("rt/deep3"));
    CHECK(deep3 != nullptr && fills_to(deep3, 8u, 8u) && p.free_writer_slots(true) == 1u,
          "a freed deep slot holds a deep writer again, leaving the shallow slot free");
    StatefulWriter* shallow3 = p.create_writer<2>(writer_on("rt/shallow3", kShallow));
    CHECK(shallow3 != nullptr && p.free_writer_slots(true) == 0u && fills_to(shallow3, 2u, 2u)
              && shallow2->cache().size() == 2u,
          "a freed shallow slot is reused by the next SHALLOW writer");
    return true;
}

}  // namespace spec_test

int main(int argc, char** argv) {
    const spec_test::TestCase tests[] = {
        {"TC-PART-8.5.1-013",
         "§8.5.1: a SHALLOW writer takes only a shallow slot and a DEEP writer only a deep "
         "one, whatever their depths, so endpoint churn never leaves one class short, and "
         "every writer keeps its own depth.",
         spec_test::tc_part_8_5_1_013},
    };

    return spec_test::run_standalone_tests(tests, sizeof(tests) / sizeof(tests[0]), argc, argv);
}
