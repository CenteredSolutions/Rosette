// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

#include "spec_test_registry.hpp"

namespace spec_test {

void register_all_stateful_reader_tests(std::vector<TestCase>& tests) {
    append_stateful_reader_tests_8_4_7_4(tests);
    append_stateful_reader_tests_8_4_8_8_4_9(tests);
    append_stateful_reader_tests_8_4_9_gap_ranges(tests);
    append_stateful_reader_tests_8_4_10_4(tests);
    append_stateful_reader_tests_8_4_10_4_locator_reference(tests);
    append_stateful_reader_tests_8_4_10_4_in_order(tests);
    append_stateful_reader_tests_8_4_10_4_shared_hold_pool(tests);
    append_stateful_reader_tests_8_4_10_4_simulated_writers(tests);
    append_stateful_reader_tests_8_4_12_gap_nack_on_quiet(tests);
    append_stateful_reader_tests_8_4_12_nack_frag_on_heartbeat(tests);
    append_stateful_reader_tests_8_4_12_nack_frag_on_quiet(tests);
}

} // namespace spec_test
