#include "spec_test_registry.hpp"

namespace spec_test {

void register_all_spdp_tests(std::vector<TestCase>& tests) {
    append_spdp_tests_8_5_3(tests);
    append_spdp_tests_8_5_3_locator_ranking(tests);
    append_spdp_tests_8_5_3_peer_records(tests);
    append_spdp_tests_9_6_1(tests);
    append_spdp_tests_9_6_2_9_6_3_1(tests);
}

} // namespace spec_test
