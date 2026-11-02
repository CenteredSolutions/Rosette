# SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20)

project(rosette_spec_tests CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

enable_testing()

include(${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/rosette_sanitizers.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/../../cmake/rosette_warnings.cmake)
rosette_warnings_for_directory()

# Every target below builds src/rtps/*.cpp directly (bare #include
# "foo.hpp" style, relative to include/rosette/rtps) alongside test code
# that includes the same headers via "rosette/rtps/foo.hpp" (relative to
# include/). Both forms need to resolve for a standalone
# `cmake -S tests/spec -B build` per this directory's own README — neither
# path was on any target's include_directories() before this fix.
# Spec tests run on the POSIX platform port (include/rosette/platform/platform.hpp).
# Tests that control time link a test chip layer (src/*_clock_stub.cpp,
# src/test_participant_platform_stub.cpp) instead of the host chip layer;
# see rosette_spec_use_platform() below.
add_compile_definitions(ROSETTE_OS_POSIX ROSETTE_CHIP_HOST)
# The cases check what a reader's ACKNACK holds and where it goes at the
# moment a HEARTBEAT owes one, so readers answer at once here; the cases that
# check heartbeatResponseDelay itself (TC-SRR-8.4.7.4-007) set their own.
add_compile_definitions(RTPS_HEARTBEAT_RESPONSE_DELAY_MS=0u)
set(ROSETTE_SPEC_PLATFORM_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../src/platform)
set(ROSETTE_SPEC_HOST_CHIP ${CMAKE_CURRENT_SOURCE_DIR}/../../ports/chip/host/chip_host.cpp)

# Links a spec target against the platform: WallClock and FNV-1a always, plus
# the real host chip layer unless the target supplies its own test chip layer.
function(rosette_spec_use_platform target)
    target_sources(${target} PRIVATE
        ${ROSETTE_SPEC_PLATFORM_DIR}/wall_clock.cpp
        ${ROSETTE_SPEC_PLATFORM_DIR}/../support/fnv1a.cpp)
    get_target_property(_srcs ${target} SOURCES)
    list(FILTER _srcs INCLUDE REGEX "(_clock_stub|_platform_stub)\\.cpp$")
    if(NOT _srcs)
        target_sources(${target} PRIVATE ${ROSETTE_SPEC_HOST_CHIP})
    endif()
endfunction()

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/../../include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../include/rosette/rtps
    ${CMAKE_CURRENT_SOURCE_DIR}/../../ports/os/posix
)

add_executable(message_parser_spec_tests
    src/message_parser_spec_tests.cpp
    src/spec_test_common.cpp
    src/spec_test_registry.cpp
    src/test_9_4_2_header.cpp
    src/test_9_4_5_1_submessage.cpp
    src/test_9_4_5_3_data.cpp
    src/test_9_4_5_4_data_frag.cpp
    src/test_9_4_5_7_acknack.cpp
    src/test_9_4_5_9_info_ts.cpp
    src/test_9_4_5_11_gap.cpp
    src/test_9_4_5_13_info_dst.cpp
    src/test_9_6_2_inline_qos.cpp
    ../../src/rtps/message_parser.cpp
)

target_include_directories(message_parser_spec_tests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../..
)

target_compile_definitions(message_parser_spec_tests PRIVATE
    RTPS_LOG_LEVEL=0
)

set(MESSAGE_PARSER_PASSED_JSON
    ${CMAKE_CURRENT_BINARY_DIR}/message_parser_passed.json)

add_test(
    NAME message_parser_spec_conformance
    COMMAND message_parser_spec_tests --emit-pass-json ${MESSAGE_PARSER_PASSED_JSON}
)

add_executable(message_builder_spec_tests
    src/message_builder_spec_tests.cpp
    src/spec_test_common.cpp
    src/spec_builder_test_registry.cpp
    src/test_builder_9_4_1_header.cpp
    src/test_builder_9_4_5_data.cpp
    src/test_builder_9_4_5_ctrl.cpp
    src/test_builder_10_2_encapsulation.cpp
    ../../src/rtps/buf_pool.cpp
    ../../src/rtps/message_parser.cpp
)

target_include_directories(message_builder_spec_tests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../..
)

target_compile_definitions(message_builder_spec_tests PRIVATE
    RTPS_LOG_LEVEL=0
)

set(MESSAGE_BUILDER_PASSED_JSON
    ${CMAKE_CURRENT_BINARY_DIR}/message_builder_passed.json)

add_test(
    NAME message_builder_spec_conformance
    COMMAND message_builder_spec_tests --emit-pass-json ${MESSAGE_BUILDER_PASSED_JSON}
)

add_executable(fragment_assembler_spec_tests
    src/fragment_assembler_spec_tests.cpp
    src/spec_test_common.cpp
    src/spec_fragment_test_registry.cpp
    src/test_fragment_8_3_3_1.cpp
    src/test_fragment_8_4_12.cpp
    src/test_fragment_8_4_14.cpp
    src/test_fragment_8_4_14_shared_pool.cpp
    ../../src/rtps/buf_pool.cpp
    ../../src/rtps/fragment_assembler.cpp
)

target_include_directories(fragment_assembler_spec_tests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../..
)

target_compile_definitions(fragment_assembler_spec_tests PRIVATE
    RTPS_LOG_LEVEL=0
)

set(FRAGMENT_ASSEMBLER_PASSED_JSON
    ${CMAKE_CURRENT_BINARY_DIR}/fragment_assembler_passed.json)

add_test(
    NAME fragment_assembler_spec_conformance
    COMMAND fragment_assembler_spec_tests --emit-pass-json ${FRAGMENT_ASSEMBLER_PASSED_JSON}
)

add_executable(spdp_spec_tests
    src/spdp_spec_tests.cpp
    src/spec_test_common.cpp
    src/spec_spdp_test_registry.cpp
    src/test_spdp_8_5_3_behavior.cpp
    src/test_spdp_8_5_3_locator_ranking.cpp
    src/test_spdp_8_5_3_peer_records.cpp
    src/test_spdp_9_6_1_locators.cpp
    src/test_spdp_9_6_2_9_6_3_1_plcdr.cpp
    ../../src/rtps/spdp.cpp
    ../../src/rtps/spdp_peer_records.cpp
    ../../src/rtps/locator_ranking.cpp
    ../../src/rtps/message_parser.cpp
    ../../src/rtps/buf_pool.cpp
)

target_include_directories(spdp_spec_tests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../..
)

target_compile_definitions(spdp_spec_tests PRIVATE
    RTPS_LOG_LEVEL=0
)

set(SPDP_PASSED_JSON
    ${CMAKE_CURRENT_BINARY_DIR}/spdp_passed.json)

add_test(
    NAME spdp_spec_conformance
    COMMAND spdp_spec_tests --emit-pass-json ${SPDP_PASSED_JSON}
)

# Built twice: sedp_spec_tests_rebuild runs every SEDP writer test with
# announcements rebuilt for each send (RTPS_SEDP_REBUILD_ANNOUNCEMENTS, the
# firmware profile's mode) instead of stored.
set(SEDP_SPEC_TEST_SOURCES
    src/sedp_spec_tests.cpp
    src/spec_test_common.cpp
    src/spec_sedp_test_registry.cpp
    src/test_stateful_writer_clock_stub.cpp
    src/test_sedp_8_5_4_behavior.cpp
    src/test_sedp_8_5_4_peer_locators.cpp
    src/test_sedp_8_5_4_2_endpoint_locator_lists.cpp
    src/test_sedp_8_5_4_rebuild.cpp
    src/test_sedp_8_5_4_repairs.cpp
    src/test_sedp_8_5_5_1_matching.cpp
    src/test_sedp_8_5_5_1_local_names.cpp
    src/test_sedp_9_6_3_2_plcdr.cpp
    ../../src/rtps/sedp.cpp
    ../../src/rtps/endpoint_match_table.cpp
    ../../src/rtps/locator_ranking.cpp
    ../../src/rtps/stateful_reader.cpp
    ../../src/rtps/fragment_assembler.cpp
    ../../src/rtps/writer_proxy.cpp
    ../../src/rtps/sn_window.cpp
    ../../src/rtps/history_cache.cpp
    ../../src/rtps/reader_proxy.cpp
    ../../src/rtps/stateful_writer.cpp
    ../../src/rtps/stateful_writer_replay.cpp
    ../../src/rtps/proxy_pool.cpp
    ../../src/rtps/message_parser.cpp
    ../../src/rtps/buf_pool.cpp
)

add_executable(sedp_spec_tests ${SEDP_SPEC_TEST_SOURCES})
add_executable(sedp_spec_tests_rebuild ${SEDP_SPEC_TEST_SOURCES})
foreach(_sedp_target sedp_spec_tests sedp_spec_tests_rebuild)
    target_include_directories(${_sedp_target} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/../..
    )
    target_compile_definitions(${_sedp_target} PRIVATE
        RTPS_LOG_LEVEL=0
    )
endforeach()
target_compile_definitions(sedp_spec_tests_rebuild PRIVATE SPEC_SEDP_REBUILD_ANNOUNCEMENTS=1)

set(LOCAL_NAME_LIMIT_PASSED_JSON
    ${CMAKE_CURRENT_BINARY_DIR}/local_name_limit_passed.json)
set(SEDP_PASSED_JSON
    ${CMAKE_CURRENT_BINARY_DIR}/sedp_passed.json)

add_test(
    NAME sedp_spec_conformance
    COMMAND sedp_spec_tests --emit-pass-json ${SEDP_PASSED_JSON}
)

add_test(
    NAME sedp_spec_conformance_rebuild
    COMMAND sedp_spec_tests_rebuild
)

add_executable(history_cache_spec_tests
    src/history_cache_spec_tests.cpp
    src/spec_test_common.cpp
    src/spec_history_cache_test_registry.cpp
    src/test_history_cache_8_2_2_8_3_1.cpp
    ../../src/rtps/history_cache.cpp
    ../../src/rtps/reader_proxy.cpp
    ../../src/rtps/stateful_writer.cpp
    ../../src/rtps/stateful_writer_replay.cpp
    ../../src/rtps/proxy_pool.cpp
    ../../src/rtps/buf_pool.cpp
)

target_include_directories(history_cache_spec_tests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../..
)

target_compile_definitions(history_cache_spec_tests PRIVATE
    RTPS_LOG_LEVEL=0
)

set(HISTORY_CACHE_PASSED_JSON
    ${CMAKE_CURRENT_BINARY_DIR}/history_cache_passed.json)

add_test(
    NAME history_cache_spec_conformance
    COMMAND history_cache_spec_tests --emit-pass-json ${HISTORY_CACHE_PASSED_JSON}
)

add_executable(stateful_writer_spec_tests
    src/stateful_writer_spec_tests.cpp
    src/spec_test_common.cpp
    src/spec_stateful_writer_test_registry.cpp
    src/test_stateful_writer_clock_stub.cpp
    src/test_stateful_writer_8_2_2_history_cache.cpp
    ../../src/rtps/history_cache.cpp
    ../../src/rtps/reader_proxy.cpp
    ../../src/rtps/stateful_writer.cpp
    ../../src/rtps/stateful_writer_replay.cpp
    ../../src/rtps/proxy_pool.cpp
    src/test_stateful_writer_8_4_1_1_locator_reference.cpp
    ../../src/rtps/fragment_assembler.cpp
    src/test_proxy_pool_admission.cpp
    src/test_stateful_writer_8_4_1_4.cpp
    src/test_stateful_writer_8_4_1_4_rematch.cpp
    src/test_stateful_writer_8_4_1_4_replay.cpp
    src/test_stateful_writer_8_4_2_2.cpp
    src/test_stateful_writer_8_4_3_8_4_4.cpp
    src/test_stateful_writer_8_4_6_requested_window.cpp
    src/test_stateful_writer_8_4_9_irrelevant_gap.cpp
    src/test_stateful_writer_8_4_9_announcing.cpp
    src/test_stateful_writer_8_4_15_piggyback_heartbeat.cpp
    src/test_stateful_writer_8_4_7_release_finished.cpp
    src/test_stateful_writer_8_4_7_undelivered_eviction.cpp
    src/test_stateful_writer_8_4_9_1_best_effort_reader.cpp
    src/test_stateful_writer_8_4_4_fragment_repair.cpp
    ../../src/rtps/message_parser.cpp
    ../../src/rtps/buf_pool.cpp
)

target_include_directories(stateful_writer_spec_tests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../..
)

target_compile_definitions(stateful_writer_spec_tests PRIVATE
    RTPS_LOG_LEVEL=0
)

set(STATEFUL_WRITER_PASSED_JSON
    ${CMAKE_CURRENT_BINARY_DIR}/stateful_writer_passed.json)

add_test(
    NAME stateful_writer_spec_conformance
    COMMAND stateful_writer_spec_tests --emit-pass-json ${STATEFUL_WRITER_PASSED_JSON}
)

add_executable(stateful_reader_spec_tests
    src/stateful_reader_spec_tests.cpp
    src/spec_test_common.cpp
    src/spec_stateful_reader_test_registry.cpp
    src/test_stateful_reader_clock_stub.cpp
    src/test_stateful_reader_8_4_7_4.cpp
    src/test_stateful_reader_8_4_8_8_4_9.cpp
    src/test_stateful_reader_8_4_9_gap_ranges.cpp
    src/test_stateful_reader_8_4_10_4_changes_from_writer.cpp
    src/test_stateful_reader_8_4_10_4_locator_reference.cpp
    src/test_stateful_reader_8_4_12_nack_frag_on_heartbeat.cpp
    src/test_stateful_reader_8_4_12_nack_frag_on_quiet.cpp
    ../../src/rtps/stateful_reader.cpp
    ../../src/rtps/fragment_assembler.cpp
    ../../src/rtps/sn_window.cpp
    ../../src/rtps/history_cache.cpp
    ../../src/rtps/reader_proxy.cpp
    ../../src/rtps/stateful_writer.cpp
    ../../src/rtps/stateful_writer_replay.cpp
    ../../src/rtps/proxy_pool.cpp
    ../../src/rtps/writer_proxy.cpp
    ../../src/rtps/message_parser.cpp
    ../../src/rtps/buf_pool.cpp
)

target_include_directories(stateful_reader_spec_tests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../..
)

target_compile_definitions(stateful_reader_spec_tests PRIVATE
    RTPS_LOG_LEVEL=0
)

set(STATEFUL_READER_PASSED_JSON
    ${CMAKE_CURRENT_BINARY_DIR}/stateful_reader_passed.json)

add_test(
    NAME stateful_reader_spec_conformance
    COMMAND stateful_reader_spec_tests --emit-pass-json ${STATEFUL_READER_PASSED_JSON}
)

# Built twice: participant_spec_tests_rebuild runs the participant with SEDP
# announcements rebuilt from its endpoint table (RTPS_SEDP_REBUILD_ANNOUNCEMENTS=1).
set(PARTICIPANT_SPEC_TEST_SOURCES
    src/participant_spec_tests.cpp
    src/spec_test_common.cpp
    src/spec_participant_test_registry.cpp
    src/test_participant_platform_stub.cpp
    src/test_participant_8_5_1.cpp
    src/test_participant_waiting_matches.cpp
    src/test_participant_writer_evictions.cpp
    src/test_participant_8_4_4_reader_reliability.cpp
    src/test_participant_driver_pass_budget.cpp
    src/test_transport_multicast_interface.cpp
    ../../src/rtps/participant.cpp
    ../../src/rtps/participant_driver.cpp
    ../../src/rtps/transport.cpp
    ../../src/rtps/participant_peer_tiers.cpp
    ../../src/rtps/stateful_reader.cpp
    ../../src/rtps/fragment_assembler.cpp
    ../../src/rtps/sn_window.cpp
    ../../src/rtps/history_cache.cpp
    ../../src/rtps/reader_proxy.cpp
    ../../src/rtps/stateful_writer.cpp
    ../../src/rtps/stateful_writer_replay.cpp
    ../../src/rtps/proxy_pool.cpp
    ../../src/rtps/writer_proxy.cpp
    ../../src/rtps/spdp.cpp
    ../../src/rtps/spdp_peer_records.cpp
    ../../src/rtps/sedp.cpp
    ../../src/rtps/endpoint_match_table.cpp
    ../../src/rtps/locator_ranking.cpp
    ../../src/rtps/message_parser.cpp
    ../../src/rtps/buf_pool.cpp
)

add_executable(participant_spec_tests ${PARTICIPANT_SPEC_TEST_SOURCES})

target_include_directories(participant_spec_tests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../..
)

# Three waiting matches: TC-PART-8.4.1.4-001 queues three readers, and
# TC-PART-8.4.1.4-004 reaches the limit within the default remote endpoint
# table.
target_compile_definitions(participant_spec_tests PRIVATE
    RTPS_LOG_LEVEL=0
    RTPS_MAX_WAITING_MATCHES=3
)

set(PARTICIPANT_PASSED_JSON
    ${CMAKE_CURRENT_BINARY_DIR}/participant_passed.json)
set(WRITER_SLOT_CLASS_PASSED_JSON
    ${CMAKE_CURRENT_BINARY_DIR}/writer_slot_class_passed.json)
set(PARTICIPANT_TIER_PASSED_JSON
    ${CMAKE_CURRENT_BINARY_DIR}/participant_tier_passed.json)

add_test(
    NAME participant_spec_conformance
    COMMAND participant_spec_tests --emit-pass-json ${PARTICIPANT_PASSED_JSON}
)

add_executable(participant_spec_tests_rebuild ${PARTICIPANT_SPEC_TEST_SOURCES})
target_include_directories(participant_spec_tests_rebuild PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../..
)
target_compile_definitions(participant_spec_tests_rebuild PRIVATE
    RTPS_LOG_LEVEL=0
    RTPS_MAX_WAITING_MATCHES=3
    RTPS_SEDP_REBUILD_ANNOUNCEMENTS=1
)
add_test(
    NAME participant_spec_conformance_rebuild
    COMMAND participant_spec_tests_rebuild
)

# The match table's local name arena at 160 octets, so it can be filled: the
# default holds every local endpoint at the longest names and is never the
# limit reached.
add_executable(local_name_limit_spec_tests
    src/test_local_endpoint_name_limits.cpp
    src/spec_test_common.cpp
    src/test_participant_platform_stub.cpp
    ../../src/rtps/participant.cpp
    ../../src/rtps/participant_peer_tiers.cpp
    ../../src/rtps/stateful_reader.cpp
    ../../src/rtps/fragment_assembler.cpp
    ../../src/rtps/sn_window.cpp
    ../../src/rtps/history_cache.cpp
    ../../src/rtps/reader_proxy.cpp
    ../../src/rtps/stateful_writer.cpp
    ../../src/rtps/stateful_writer_replay.cpp
    ../../src/rtps/proxy_pool.cpp
    ../../src/rtps/writer_proxy.cpp
    ../../src/rtps/spdp.cpp
    ../../src/rtps/spdp_peer_records.cpp
    ../../src/rtps/sedp.cpp
    ../../src/rtps/endpoint_match_table.cpp
    ../../src/rtps/locator_ranking.cpp
    ../../src/rtps/message_parser.cpp
    ../../src/rtps/buf_pool.cpp
)
target_include_directories(local_name_limit_spec_tests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../..
)
target_compile_definitions(local_name_limit_spec_tests PRIVATE
    RTPS_LOG_LEVEL=0
    RTPS_LOCAL_NAME_BYTES=160
)
add_test(
    NAME local_name_limit_spec_conformance
    COMMAND local_name_limit_spec_tests
)

# Two deep and two shallow writer slots, so a test can fill each class: every
# other spec program keeps the default of deep slots only.
add_executable(writer_slot_class_spec_tests
    src/test_participant_writer_slot_classes.cpp
    src/spec_test_common.cpp
    src/test_participant_platform_stub.cpp
    ../../src/rtps/participant.cpp
    ../../src/rtps/participant_peer_tiers.cpp
    ../../src/rtps/stateful_reader.cpp
    ../../src/rtps/fragment_assembler.cpp
    ../../src/rtps/sn_window.cpp
    ../../src/rtps/history_cache.cpp
    ../../src/rtps/reader_proxy.cpp
    ../../src/rtps/stateful_writer.cpp
    ../../src/rtps/stateful_writer_replay.cpp
    ../../src/rtps/proxy_pool.cpp
    ../../src/rtps/writer_proxy.cpp
    ../../src/rtps/spdp.cpp
    ../../src/rtps/spdp_peer_records.cpp
    ../../src/rtps/sedp.cpp
    ../../src/rtps/endpoint_match_table.cpp
    ../../src/rtps/locator_ranking.cpp
    ../../src/rtps/message_parser.cpp
    ../../src/rtps/buf_pool.cpp
)
target_include_directories(writer_slot_class_spec_tests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../..
)
target_compile_definitions(writer_slot_class_spec_tests PRIVATE
    RTPS_LOG_LEVEL=0
    RTPS_MAX_USER_WRITERS=4
    RTPS_MAX_SHALLOW_WRITERS=2
    RTPS_SHALLOW_WRITER_DEPTH=2
)
add_test(
    NAME writer_slot_class_spec_conformance
    COMMAND writer_slot_class_spec_tests
)

# The buffer pool's debug build, which no other program compiles: the
# RTPS_BUF_ALLOC*_DBG macros and the per-class statistics firmware sizes its
# pool classes from.
add_executable(buf_pool_debug_spec_tests
    src/test_buf_pool_debug_alloc.cpp
    src/spec_test_common.cpp
    ../../src/rtps/buf_pool.cpp
)
target_include_directories(buf_pool_debug_spec_tests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../..
)
target_compile_definitions(buf_pool_debug_spec_tests PRIVATE
    RTPS_LOG_LEVEL=0
    RTPS_BUFPOOL_DEBUG=1
)
add_test(
    NAME buf_pool_debug_spec_conformance
    COMMAND buf_pool_debug_spec_tests
)

# The buffer pool's usage statistics, with two blocks per class so a class
# runs out in a few allocations, and a log sink of the test's own.
add_executable(buf_pool_stats_spec_tests
    src/test_buf_pool_stats.cpp
    src/spec_test_common.cpp
    ../../src/rtps/buf_pool.cpp
    ../../src/rtps/buf_pool_stats_log.cpp
)
target_include_directories(buf_pool_stats_spec_tests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../..
    ${CMAKE_CURRENT_SOURCE_DIR}/../../include
)
target_compile_definitions(buf_pool_stats_spec_tests PRIVATE
    RTPS_LOG_LEVEL=2
    RTPS_BUFPOOL_STATS=1
    RTPS_POOL_CLASS0_COUNT=2u RTPS_POOL_CLASS1_COUNT=2u RTPS_POOL_CLASS2_COUNT=2u
    RTPS_POOL_CLASS3_COUNT=2u RTPS_POOL_CLASS4_COUNT=2u
)
add_test(
    NAME buf_pool_stats_spec_conformance
    COMMAND buf_pool_stats_spec_tests
)

# The peer-tier tests need an active tier smaller than the network: two
# active slots among six known peers, and a short settle time.
set(PARTICIPANT_TIER_SPEC_TEST_SOURCES ${PARTICIPANT_SPEC_TEST_SOURCES})
list(REMOVE_ITEM PARTICIPANT_TIER_SPEC_TEST_SOURCES
    src/spec_participant_test_registry.cpp
    src/test_participant_8_5_1.cpp
    src/test_participant_waiting_matches.cpp
    src/test_participant_driver_pass_budget.cpp
    src/test_transport_multicast_interface.cpp)
list(APPEND PARTICIPANT_TIER_SPEC_TEST_SOURCES
    src/spec_participant_tier_test_registry.cpp
    src/test_participant_8_5_3_peer_tiers.cpp)
add_executable(participant_tier_spec_tests ${PARTICIPANT_TIER_SPEC_TEST_SOURCES})
target_include_directories(participant_tier_spec_tests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../..
)
target_compile_definitions(participant_tier_spec_tests PRIVATE
    RTPS_LOG_LEVEL=0
    RTPS_MAX_PEERS=2
    RTPS_MAX_KNOWN_PEERS=6
    RTPS_PEER_SETTLE_MS=100u
)
add_test(
    NAME participant_tier_spec_conformance
    COMMAND participant_tier_spec_tests
)

find_package(Python3 COMPONENTS Interpreter)
if (Python3_Interpreter_FOUND)
    add_custom_target(message_parser_spec_report
        COMMAND message_parser_spec_tests --emit-pass-json ${MESSAGE_PARSER_PASSED_JSON}
        COMMAND ${Python3_EXECUTABLE}
                ${CMAKE_CURRENT_SOURCE_DIR}/tools/spec_trace_report.py
                --matrix ${CMAKE_CURRENT_SOURCE_DIR}/spec/ddsi_rtps_2_5/message_parser.matrix.json
                --passed ${MESSAGE_PARSER_PASSED_JSON}
        DEPENDS message_parser_spec_tests
        COMMENT "Generating RTPS statement coverage report for message_parser"
    )

    add_custom_target(message_builder_spec_report
        COMMAND message_builder_spec_tests --emit-pass-json ${MESSAGE_BUILDER_PASSED_JSON}
        COMMAND ${Python3_EXECUTABLE}
                ${CMAKE_CURRENT_SOURCE_DIR}/tools/spec_trace_report.py
                --matrix ${CMAKE_CURRENT_SOURCE_DIR}/spec/ddsi_rtps_2_5/message_builder.matrix.json
                --passed ${MESSAGE_BUILDER_PASSED_JSON}
        DEPENDS message_builder_spec_tests
        COMMENT "Generating RTPS statement coverage report for message_builder"
    )

    add_custom_target(fragment_assembler_spec_report
        COMMAND fragment_assembler_spec_tests --emit-pass-json ${FRAGMENT_ASSEMBLER_PASSED_JSON}
        COMMAND ${Python3_EXECUTABLE}
                ${CMAKE_CURRENT_SOURCE_DIR}/tools/spec_trace_report.py
                --matrix ${CMAKE_CURRENT_SOURCE_DIR}/spec/ddsi_rtps_2_5/fragment_assembler.matrix.json
                --passed ${FRAGMENT_ASSEMBLER_PASSED_JSON}
        DEPENDS fragment_assembler_spec_tests
        COMMENT "Generating RTPS statement coverage report for fragment_assembler"
    )

    add_custom_target(spdp_spec_report
        COMMAND spdp_spec_tests --emit-pass-json ${SPDP_PASSED_JSON}
        COMMAND ${Python3_EXECUTABLE}
                ${CMAKE_CURRENT_SOURCE_DIR}/tools/spec_trace_report.py
                --matrix ${CMAKE_CURRENT_SOURCE_DIR}/spec/ddsi_rtps_2_5/spdp.matrix.json
                --passed ${SPDP_PASSED_JSON}
        DEPENDS spdp_spec_tests
        COMMENT "Generating RTPS statement coverage report for SPDP"
    )

    add_custom_target(sedp_spec_report
        COMMAND sedp_spec_tests --emit-pass-json ${SEDP_PASSED_JSON}
        COMMAND local_name_limit_spec_tests --emit-pass-json ${LOCAL_NAME_LIMIT_PASSED_JSON}
        COMMAND ${Python3_EXECUTABLE}
                ${CMAKE_CURRENT_SOURCE_DIR}/tools/spec_trace_report.py
                --matrix ${CMAKE_CURRENT_SOURCE_DIR}/spec/ddsi_rtps_2_5/sedp.matrix.json
                --passed ${SEDP_PASSED_JSON} ${LOCAL_NAME_LIMIT_PASSED_JSON}
        DEPENDS sedp_spec_tests local_name_limit_spec_tests
        COMMENT "Generating RTPS statement coverage report for SEDP"
    )

    add_custom_target(history_cache_spec_report
        COMMAND history_cache_spec_tests --emit-pass-json ${HISTORY_CACHE_PASSED_JSON}
        COMMAND ${Python3_EXECUTABLE}
                ${CMAKE_CURRENT_SOURCE_DIR}/tools/spec_trace_report.py
                --matrix ${CMAKE_CURRENT_SOURCE_DIR}/spec/ddsi_rtps_2_5/history_cache.matrix.json
                --passed ${HISTORY_CACHE_PASSED_JSON}
        DEPENDS history_cache_spec_tests
        COMMENT "Generating RTPS statement coverage report for history_cache"
    )

    add_custom_target(stateful_writer_spec_report
        COMMAND stateful_writer_spec_tests --emit-pass-json ${STATEFUL_WRITER_PASSED_JSON}
        COMMAND ${Python3_EXECUTABLE}
                ${CMAKE_CURRENT_SOURCE_DIR}/tools/spec_trace_report.py
                --matrix ${CMAKE_CURRENT_SOURCE_DIR}/spec/ddsi_rtps_2_5/stateful_writer.matrix.json
                --passed ${STATEFUL_WRITER_PASSED_JSON}
        DEPENDS stateful_writer_spec_tests
        COMMENT "Generating RTPS statement coverage report for stateful_writer"
    )

    add_custom_target(stateful_reader_spec_report
        COMMAND stateful_reader_spec_tests --emit-pass-json ${STATEFUL_READER_PASSED_JSON}
        COMMAND ${Python3_EXECUTABLE}
                ${CMAKE_CURRENT_SOURCE_DIR}/tools/spec_trace_report.py
                --matrix ${CMAKE_CURRENT_SOURCE_DIR}/spec/ddsi_rtps_2_5/stateful_reader.matrix.json
                --passed ${STATEFUL_READER_PASSED_JSON}
        DEPENDS stateful_reader_spec_tests
        COMMENT "Generating RTPS statement coverage report for stateful_reader"
    )

    # The participant matrix is run by the default-capacity program and by the
    # standalone programs built with their own capacity definitions.
    add_custom_target(participant_spec_report
        COMMAND participant_spec_tests --emit-pass-json ${PARTICIPANT_PASSED_JSON}
        COMMAND local_name_limit_spec_tests --emit-pass-json ${LOCAL_NAME_LIMIT_PASSED_JSON}
        COMMAND writer_slot_class_spec_tests --emit-pass-json ${WRITER_SLOT_CLASS_PASSED_JSON}
        COMMAND participant_tier_spec_tests --emit-pass-json ${PARTICIPANT_TIER_PASSED_JSON}
        COMMAND ${Python3_EXECUTABLE}
                ${CMAKE_CURRENT_SOURCE_DIR}/tools/spec_trace_report.py
                --matrix ${CMAKE_CURRENT_SOURCE_DIR}/spec/ddsi_rtps_2_5/participant.matrix.json
                --passed ${PARTICIPANT_PASSED_JSON} ${LOCAL_NAME_LIMIT_PASSED_JSON}
                         ${WRITER_SLOT_CLASS_PASSED_JSON} ${PARTICIPANT_TIER_PASSED_JSON}
        DEPENDS participant_spec_tests local_name_limit_spec_tests writer_slot_class_spec_tests
                participant_tier_spec_tests
        COMMENT "Generating RTPS statement coverage report for participant"
    )
endif()

option(ROSETTE_SPEC_ENABLE_FUZZ "Build fuzz targets (corpus replay always; libFuzzer with Clang) for the parser, fragment assembler, SPDP, SEDP, and endpoint matching" OFF)

if (ROSETTE_SPEC_ENABLE_FUZZ)
    # Each fuzz target has one harness, fuzz/fuzz_<name>.cpp, defining
    # LLVMFuzzerTestOneInput. <name>_fuzz_replay links that harness with
    # fuzz/replay_driver.cpp, which feeds it every file of a corpus directory,
    # so the replay and the libFuzzer build run the same code. <name>_fuzz is
    # the libFuzzer build, available with Clang.
    function(rosette_spec_add_fuzz name harness)
        set(_sources ${ARGN})
        list(TRANSFORM _sources PREPEND ../../src/rtps/)

        add_executable(${name}_fuzz_replay ${harness} fuzz/replay_driver.cpp ${_sources})
        target_include_directories(${name}_fuzz_replay PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../.. ${CMAKE_CURRENT_SOURCE_DIR}/include)
        target_compile_definitions(${name}_fuzz_replay PRIVATE RTPS_LOG_LEVEL=0)
        rosette_spec_use_platform(${name}_fuzz_replay)

        if (CMAKE_CXX_COMPILER_ID MATCHES "Clang")
            add_executable(${name}_fuzz ${harness} ${_sources})
            target_include_directories(${name}_fuzz PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../.. ${CMAKE_CURRENT_SOURCE_DIR}/include)
            target_compile_definitions(${name}_fuzz PRIVATE RTPS_LOG_LEVEL=0)
            target_compile_options(${name}_fuzz PRIVATE
                -fsanitize=fuzzer,address,undefined -fno-omit-frame-pointer -g)
            target_link_options(${name}_fuzz PRIVATE -fsanitize=fuzzer,address,undefined)
            rosette_spec_use_platform(${name}_fuzz)
        endif()

        # The seed corpus is generated at configure time so ctest can replay it.
        if (Python3_Interpreter_FOUND)
            set(_vectors ${CMAKE_CURRENT_SOURCE_DIR}/fuzz/${name}_corpus_vectors.json)
            set(_corpus ${CMAKE_CURRENT_BINARY_DIR}/fuzz/corpus/${name})
            set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${_vectors})
            execute_process(
                COMMAND ${Python3_EXECUTABLE}
                        ${CMAKE_CURRENT_SOURCE_DIR}/tools/gen_fuzz_corpus.py
                        --vectors ${_vectors} --out ${_corpus}
                RESULT_VARIABLE _gen_result)
            if (NOT _gen_result EQUAL 0)
                message(FATAL_ERROR "Generating the ${name} fuzz corpus failed")
            endif()
            add_test(NAME ${name}_fuzz_replay COMMAND ${name}_fuzz_replay ${_corpus})
        endif()
    endfunction()

    rosette_spec_add_fuzz(message_parser fuzz/fuzz_parse_message.cpp
        message_parser.cpp)
    rosette_spec_add_fuzz(fragment_assembler fuzz/fuzz_fragment_assembler.cpp
        buf_pool.cpp fragment_assembler.cpp)
    rosette_spec_add_fuzz(spdp fuzz/fuzz_spdp.cpp
        spdp.cpp spdp_peer_records.cpp locator_ranking.cpp buf_pool.cpp)
    rosette_spec_add_fuzz(sedp fuzz/fuzz_sedp.cpp
        sedp.cpp endpoint_match_table.cpp locator_ranking.cpp stateful_reader.cpp
        fragment_assembler.cpp writer_proxy.cpp sn_window.cpp history_cache.cpp
        reader_proxy.cpp stateful_writer.cpp stateful_writer_replay.cpp proxy_pool.cpp
        message_parser.cpp buf_pool.cpp)
    rosette_spec_add_fuzz(endpoint_match fuzz/fuzz_endpoint_match_table.cpp
        endpoint_match_table.cpp locator_ranking.cpp)

endif()

# Every spec and fuzz target runs on the platform port.
foreach(_t message_parser_spec_tests message_builder_spec_tests fragment_assembler_spec_tests spdp_spec_tests sedp_spec_tests sedp_spec_tests_rebuild history_cache_spec_tests stateful_writer_spec_tests stateful_reader_spec_tests participant_spec_tests participant_spec_tests_rebuild participant_tier_spec_tests local_name_limit_spec_tests writer_slot_class_spec_tests buf_pool_debug_spec_tests)
    if(TARGET ${_t})
        rosette_spec_use_platform(${_t})
    endif()
endforeach()
