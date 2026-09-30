#include "spec_test_registry.hpp"

namespace spec_test {

void register_all_stateful_writer_tests(std::vector<TestCase>& tests) {
    append_stateful_writer_tests_8_2_2(tests);
    append_stateful_writer_tests_8_4_1_4(tests);
    append_stateful_writer_tests_8_4_2_2(tests);
    append_stateful_writer_tests_8_4_3_8_4_4(tests);
    append_stateful_writer_tests_8_4_6_requested_window(tests);
    append_stateful_writer_tests_8_4_9_irrelevant_gap(tests);
    append_stateful_writer_tests_8_4_9_announcing(tests);
    append_stateful_writer_tests_8_4_15_piggyback_heartbeat(tests);
}

} // namespace spec_test
