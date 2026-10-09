#include "spec_test_registry.hpp"

namespace spec_test {

void register_all_participant_tests(std::vector<TestCase>& tests) {
    append_participant_tests_8_5_1(tests);
    append_participant_waiting_match_tests(tests);
}

} // namespace spec_test
