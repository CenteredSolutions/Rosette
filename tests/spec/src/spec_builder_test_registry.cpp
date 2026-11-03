// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

#include "spec_test_registry.hpp"

namespace spec_test {

void register_all_builder_tests(std::vector<TestCase>& tests) {
    append_builder_tests_9_4_1(tests);
    append_builder_tests_9_4_5_data(tests);
    append_builder_tests_9_4_5_ctrl(tests);
    append_builder_tests_10_2(tests);
}

} // namespace spec_test

