#include "spec_test_registry.hpp"

namespace spec_test {

void register_all_parser_tests(std::vector<TestCase>& tests) {
    append_tests_9_4_2(tests);
    append_tests_9_4_5_1(tests);
    append_tests_9_4_5_3(tests);
    append_tests_9_4_5_4(tests);
    append_tests_9_4_5_7(tests);
    append_tests_9_4_5_9(tests);
    append_tests_9_4_5_11(tests);
    append_tests_9_4_5_13(tests);
    append_tests_9_6_2(tests);
}

} // namespace spec_test
