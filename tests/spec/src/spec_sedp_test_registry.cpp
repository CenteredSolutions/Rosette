#include "spec_test_registry.hpp"

namespace spec_test {

void register_all_sedp_tests(std::vector<TestCase>& tests) {
    append_sedp_tests_8_5_4(tests);
    append_sedp_tests_8_5_4_repairs(tests);
    append_sedp_tests_8_5_4_rebuild(tests);
    append_sedp_tests_8_5_5_1(tests);
    append_sedp_tests_9_6_3_2(tests);
}

} // namespace spec_test
