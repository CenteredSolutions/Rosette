// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

#include "spec_test_registry.hpp"

namespace spec_test {

void register_all_stateful_writer_tests(std::vector<TestCase>& tests) {
    append_stateful_writer_tests_8_2_2(tests);
    append_stateful_writer_tests_8_4_1_1_locator_reference(tests);
    append_stateful_writer_tests_8_4_1_4(tests);
    append_stateful_writer_tests_8_4_1_4_rematch(tests);
    append_stateful_writer_tests_8_4_1_4_replay(tests);
    append_stateful_writer_tests_8_4_2_2(tests);
    append_stateful_writer_tests_8_4_3_8_4_4(tests);
    append_stateful_writer_tests_8_4_6_requested_window(tests);
    append_stateful_writer_tests_8_4_9_irrelevant_gap(tests);
    append_stateful_writer_tests_8_4_9_announcing(tests);
    append_stateful_writer_tests_8_4_15_piggyback_heartbeat(tests);
    append_stateful_writer_tests_8_4_7_release_finished(tests);
    append_stateful_writer_tests_8_4_7_undelivered_eviction(tests);
    append_stateful_writer_tests_8_4_9_1_best_effort_reader(tests);
    append_stateful_writer_tests_8_4_4_fragment_repair(tests);
    append_proxy_pool_tests(tests);
}

} // namespace spec_test
