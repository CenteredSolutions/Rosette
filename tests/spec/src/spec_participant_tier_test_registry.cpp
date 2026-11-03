// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

#include "spec_test_registry.hpp"

namespace spec_test {

// participant_tier_spec_tests runs only the peer-tier tests, which need a
// smaller active tier than the other participant tests are built with.
void register_all_participant_tests(std::vector<TestCase>& tests) {
    append_participant_tests_8_5_3_peer_tiers(tests);
}

}  // namespace spec_test
