#include "spec_test_registry.hpp"

namespace spec_test {

void register_all_fragment_assembler_tests(std::vector<TestCase>& tests) {
    append_fragment_tests_8_3_3_1(tests);
    append_fragment_tests_8_4_12(tests);
    append_fragment_tests_8_4_14(tests);
}

} // namespace spec_test
