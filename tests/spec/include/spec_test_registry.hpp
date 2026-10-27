#pragma once

#include "spec_test_common.hpp"

#include <vector>

namespace spec_test {

void append_tests_9_4_2(std::vector<TestCase>& tests);
void append_tests_9_4_5_1(std::vector<TestCase>& tests);
void append_tests_9_4_5_3(std::vector<TestCase>& tests);
void append_tests_9_4_5_4(std::vector<TestCase>& tests);
void append_tests_9_4_5_7(std::vector<TestCase>& tests);
void append_tests_9_4_5_9(std::vector<TestCase>& tests);
void append_tests_9_4_5_11(std::vector<TestCase>& tests);
void append_tests_9_4_5_13(std::vector<TestCase>& tests);
void append_tests_9_6_2(std::vector<TestCase>& tests);

void register_all_parser_tests(std::vector<TestCase>& tests);

void append_builder_tests_9_4_1(std::vector<TestCase>& tests);
void append_builder_tests_9_4_5_data(std::vector<TestCase>& tests);
void append_builder_tests_9_4_5_ctrl(std::vector<TestCase>& tests);
void append_builder_tests_10_2(std::vector<TestCase>& tests);

void register_all_builder_tests(std::vector<TestCase>& tests);

void append_fragment_tests_8_3_3_1(std::vector<TestCase>& tests);
void append_fragment_tests_8_4_12(std::vector<TestCase>& tests);
void append_fragment_tests_8_4_14(std::vector<TestCase>& tests);
void append_fragment_tests_8_4_14_shared_pool(std::vector<TestCase>& tests);

void register_all_fragment_assembler_tests(std::vector<TestCase>& tests);

void append_spdp_tests_8_5_3(std::vector<TestCase>& tests);
void append_spdp_tests_8_5_3_locator_ranking(std::vector<TestCase>& tests);
void append_spdp_tests_8_5_3_peer_records(std::vector<TestCase>& tests);
void append_spdp_tests_9_6_1(std::vector<TestCase>& tests);
void append_spdp_tests_9_6_2_9_6_3_1(std::vector<TestCase>& tests);

void register_all_spdp_tests(std::vector<TestCase>& tests);

void append_sedp_tests_8_5_4(std::vector<TestCase>& tests);
void append_sedp_tests_8_5_4_repairs(std::vector<TestCase>& tests);
void append_sedp_tests_8_5_4_rebuild(std::vector<TestCase>& tests);
void append_sedp_tests_8_5_4_peer_locators(std::vector<TestCase>& tests);
void append_sedp_tests_8_5_4_2_endpoint_locator_lists(std::vector<TestCase>& tests);
void append_sedp_tests_8_5_5_1(std::vector<TestCase>& tests);
void append_sedp_tests_8_5_5_1_local_names(std::vector<TestCase>& tests);
void append_sedp_tests_9_6_3_2(std::vector<TestCase>& tests);

void register_all_sedp_tests(std::vector<TestCase>& tests);

void append_history_cache_tests_8_2_2_8_3_1(std::vector<TestCase>& tests);
void register_all_history_cache_tests(std::vector<TestCase>& tests);

void append_stateful_writer_tests_8_4_1_1_locator_reference(std::vector<TestCase>& tests);
void append_stateful_writer_tests_8_4_1_4(std::vector<TestCase>& tests);
void append_stateful_writer_tests_8_4_1_4_rematch(std::vector<TestCase>& tests);
void append_stateful_writer_tests_8_4_1_4_replay(std::vector<TestCase>& tests);
void append_stateful_writer_tests_8_4_2_2(std::vector<TestCase>& tests);
void append_stateful_writer_tests_8_4_3_8_4_4(std::vector<TestCase>& tests);
void append_stateful_writer_tests_8_2_2(std::vector<TestCase>& tests);
void append_stateful_writer_tests_8_4_9_irrelevant_gap(std::vector<TestCase>& tests);
void append_stateful_writer_tests_8_4_6_requested_window(std::vector<TestCase>& tests);
void append_stateful_writer_tests_8_4_9_announcing(std::vector<TestCase>& tests);
void append_stateful_writer_tests_8_4_7_release_finished(std::vector<TestCase>& tests);
void append_stateful_writer_tests_8_4_7_undelivered_eviction(std::vector<TestCase>& tests);
void append_stateful_writer_tests_8_4_9_1_best_effort_reader(std::vector<TestCase>& tests);
void append_stateful_writer_tests_8_4_4_fragment_repair(std::vector<TestCase>& tests);
void append_stateful_writer_tests_8_4_15_piggyback_heartbeat(std::vector<TestCase>& tests);
void append_proxy_pool_tests(std::vector<TestCase>& tests);

void register_all_stateful_writer_tests(std::vector<TestCase>& tests);

void append_stateful_reader_tests_8_4_7_4(std::vector<TestCase>& tests);
void append_stateful_reader_tests_8_4_8_8_4_9(std::vector<TestCase>& tests);
void append_stateful_reader_tests_8_4_9_gap_ranges(std::vector<TestCase>& tests);
void append_stateful_reader_tests_8_4_10_4(std::vector<TestCase>& tests);
void append_stateful_reader_tests_8_4_10_4_locator_reference(std::vector<TestCase>& tests);
void append_stateful_reader_tests_8_4_12_nack_frag_on_heartbeat(std::vector<TestCase>& tests);

void register_all_stateful_reader_tests(std::vector<TestCase>& tests);

void append_participant_tests_8_5_1(std::vector<TestCase>& tests);
void append_participant_waiting_match_tests(std::vector<TestCase>& tests);
void append_participant_writer_eviction_tests(std::vector<TestCase>& tests);
void append_participant_reader_reliability_tests(std::vector<TestCase>& tests);
void append_participant_tests_8_5_3_peer_tiers(std::vector<TestCase>& tests);

void register_all_participant_tests(std::vector<TestCase>& tests);

} // namespace spec_test
