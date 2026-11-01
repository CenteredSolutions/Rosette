// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

#include "spec_test_registry.hpp"

namespace spec_test {

void register_all_participant_tests(std::vector<TestCase>& tests) {
    append_participant_tests_8_5_1(tests);
    append_participant_waiting_match_tests(tests);
    append_participant_writer_eviction_tests(tests);
    append_participant_reader_reliability_tests(tests);
    append_participant_driver_pass_budget_tests(tests);
    append_transport_multicast_interface_tests(tests);
}

} // namespace spec_test
