#include "spec_test_registry.hpp"

namespace spec_test {

void register_all_stateful_reader_tests(std::vector<TestCase>& tests) {
    append_stateful_reader_tests_8_4_7_4(tests);
    append_stateful_reader_tests_8_4_8_8_4_9(tests);
    append_stateful_reader_tests_8_4_9_gap_ranges(tests);
    append_stateful_reader_tests_8_4_10_4(tests);
    append_stateful_reader_tests_8_4_12_nack_frag_on_heartbeat(tests);
}

} // namespace spec_test
