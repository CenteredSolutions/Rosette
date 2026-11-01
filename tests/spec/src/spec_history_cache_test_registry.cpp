// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

#include "spec_test_registry.hpp"

namespace spec_test {

void register_all_history_cache_tests(std::vector<TestCase>& tests) {
    append_history_cache_tests_8_2_2_8_3_1(tests);
}

} // namespace spec_test
