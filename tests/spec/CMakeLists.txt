cmake_minimum_required(VERSION 3.20)

project(rosette_spec_tests CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

enable_testing()

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
set(ROSETTE_SPEC_PLATFORM_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../src/platform)

# Links a spec target against the platform: WallClock always, plus the real
# host chip layer unless the target supplies its own test chip layer.
function(rosette_spec_use_platform target)
    target_sources(${target} PRIVATE ${ROSETTE_SPEC_PLATFORM_DIR}/wall_clock.cpp)
    get_target_property(_srcs ${target} SOURCES)
    list(FILTER _srcs INCLUDE REGEX "(_clock_stub|_platform_stub)\\.cpp$")
    if(NOT _srcs)
        target_sources(${target} PRIVATE ${ROSETTE_SPEC_PLATFORM_DIR}/chip/host/chip_host.cpp)
    endif()
endfunction()

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/../../include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../include/rosette/rtps
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
    ../../src/rtps/buf_pool.cpp
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
    src/test_spdp_9_6_1_locators.cpp
    src/test_spdp_9_6_2_9_6_3_1_plcdr.cpp
    ../../src/rtps/spdp.cpp
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

add_executable(sedp_spec_tests
    src/sedp_spec_tests.cpp
    src/spec_test_common.cpp
    src/spec_sedp_test_registry.cpp
    src/test_stateful_writer_clock_stub.cpp
    src/test_sedp_8_5_4_behavior.cpp
    src/test_sedp_8_5_4_repairs.cpp
    src/test_sedp_8_5_5_1_matching.cpp
    src/test_sedp_9_6_3_2_plcdr.cpp
    ../../src/rtps/sedp.cpp
    ../../src/rtps/endpoint_match_table.cpp
    ../../src/rtps/stateful_reader.cpp
    ../../src/rtps/writer_proxy.cpp
    ../../src/rtps/sn_window.cpp
    ../../src/rtps/history_cache.cpp
    ../../src/rtps/reader_proxy.cpp
    ../../src/rtps/stateful_writer.cpp
    ../../src/rtps/message_parser.cpp
    ../../src/rtps/buf_pool.cpp
)

target_include_directories(sedp_spec_tests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../..
)

target_compile_definitions(sedp_spec_tests PRIVATE
    RTPS_LOG_LEVEL=0
)

set(SEDP_PASSED_JSON
    ${CMAKE_CURRENT_BINARY_DIR}/sedp_passed.json)

add_test(
    NAME sedp_spec_conformance
    COMMAND sedp_spec_tests --emit-pass-json ${SEDP_PASSED_JSON}
)

add_executable(history_cache_spec_tests
    src/history_cache_spec_tests.cpp
    src/spec_test_common.cpp
    src/spec_history_cache_test_registry.cpp
    src/test_history_cache_8_2_2_8_3_1.cpp
    ../../src/rtps/history_cache.cpp
    ../../src/rtps/reader_proxy.cpp
    ../../src/rtps/stateful_writer.cpp
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
    src/test_stateful_writer_8_4_1_4.cpp
    src/test_stateful_writer_8_4_2_2.cpp
    src/test_stateful_writer_8_4_3_8_4_4.cpp
    src/test_stateful_writer_8_4_6_requested_window.cpp
    src/test_stateful_writer_8_4_9_irrelevant_gap.cpp
    src/test_stateful_writer_8_4_9_announcing.cpp
    src/test_stateful_writer_8_4_15_piggyback_heartbeat.cpp
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
    ../../src/rtps/stateful_reader.cpp
    ../../src/rtps/sn_window.cpp
    ../../src/rtps/history_cache.cpp
    ../../src/rtps/reader_proxy.cpp
    ../../src/rtps/stateful_writer.cpp
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

add_executable(participant_spec_tests
    src/participant_spec_tests.cpp
    src/spec_test_common.cpp
    src/spec_participant_test_registry.cpp
    src/test_participant_platform_stub.cpp
    src/test_participant_8_5_1.cpp
    ../../src/rtps/participant.cpp
    ../../src/rtps/stateful_reader.cpp
    ../../src/rtps/sn_window.cpp
    ../../src/rtps/history_cache.cpp
    ../../src/rtps/reader_proxy.cpp
    ../../src/rtps/stateful_writer.cpp
    ../../src/rtps/writer_proxy.cpp
    ../../src/rtps/spdp.cpp
    ../../src/rtps/sedp.cpp
    ../../src/rtps/endpoint_match_table.cpp
    ../../src/rtps/message_parser.cpp
    ../../src/rtps/buf_pool.cpp
)

target_include_directories(participant_spec_tests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../..
)

target_compile_definitions(participant_spec_tests PRIVATE
    RTPS_LOG_LEVEL=0
)

set(PARTICIPANT_PASSED_JSON
    ${CMAKE_CURRENT_BINARY_DIR}/participant_passed.json)

add_test(
    NAME participant_spec_conformance
    COMMAND participant_spec_tests --emit-pass-json ${PARTICIPANT_PASSED_JSON}
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
        COMMAND ${Python3_EXECUTABLE}
                ${CMAKE_CURRENT_SOURCE_DIR}/tools/spec_trace_report.py
                --matrix ${CMAKE_CURRENT_SOURCE_DIR}/spec/ddsi_rtps_2_5/sedp.matrix.json
                --passed ${SEDP_PASSED_JSON}
        DEPENDS sedp_spec_tests
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

    add_custom_target(participant_spec_report
        COMMAND participant_spec_tests --emit-pass-json ${PARTICIPANT_PASSED_JSON}
        COMMAND ${Python3_EXECUTABLE}
                ${CMAKE_CURRENT_SOURCE_DIR}/tools/spec_trace_report.py
                --matrix ${CMAKE_CURRENT_SOURCE_DIR}/spec/ddsi_rtps_2_5/participant.matrix.json
                --passed ${PARTICIPANT_PASSED_JSON}
        DEPENDS participant_spec_tests
        COMMENT "Generating RTPS statement coverage report for participant"
    )
endif()

option(ROSETTE_SPEC_ENABLE_FUZZ "Build libFuzzer targets for parser, fragment assembler, SPDP, SEDP, and endpoint matching" OFF)

if (ROSETTE_SPEC_ENABLE_FUZZ)
    add_executable(message_parser_fuzz_replay
        fuzz/replay_corpus.cpp
        ../../src/rtps/message_parser.cpp
    )
    target_include_directories(message_parser_fuzz_replay PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../..
    )
    target_compile_definitions(message_parser_fuzz_replay PRIVATE
        RTPS_LOG_LEVEL=0
    )

    add_executable(fragment_assembler_fuzz_replay
        fuzz/replay_fragment_corpus.cpp
        ../../src/rtps/buf_pool.cpp
    )
    target_include_directories(fragment_assembler_fuzz_replay PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../..
    )
    target_compile_definitions(fragment_assembler_fuzz_replay PRIVATE
        RTPS_LOG_LEVEL=0
    )

    add_executable(spdp_fuzz_replay
        fuzz/replay_spdp_corpus.cpp
        ../../src/rtps/spdp.cpp
        ../../src/rtps/buf_pool.cpp
    )
    target_include_directories(spdp_fuzz_replay PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../..
    )
    target_compile_definitions(spdp_fuzz_replay PRIVATE
        RTPS_LOG_LEVEL=0
    )

    add_executable(sedp_fuzz_replay
        fuzz/replay_sedp_corpus.cpp
        ../../src/rtps/sedp.cpp
        ../../src/rtps/endpoint_match_table.cpp
        ../../src/rtps/sn_window.cpp
        ../../src/rtps/history_cache.cpp
        ../../src/rtps/reader_proxy.cpp
        ../../src/rtps/stateful_writer.cpp
    ../../src/rtps/history_cache.cpp
    ../../src/rtps/reader_proxy.cpp
    ../../src/rtps/stateful_writer.cpp
        ../../src/rtps/buf_pool.cpp
    )
    target_include_directories(sedp_fuzz_replay PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../..
    )
    target_compile_definitions(sedp_fuzz_replay PRIVATE
        RTPS_LOG_LEVEL=0
    )

    add_executable(endpoint_match_fuzz_replay
        fuzz/replay_endpoint_match_corpus.cpp
    )
    target_include_directories(endpoint_match_fuzz_replay PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../..
    )
    target_compile_definitions(endpoint_match_fuzz_replay PRIVATE
        RTPS_LOG_LEVEL=0
    )

    if (Python3_Interpreter_FOUND)
        set(MESSAGE_PARSER_FUZZ_CORPUS_DIR
            ${CMAKE_CURRENT_BINARY_DIR}/fuzz/corpus/message_parser)
        add_custom_target(message_parser_fuzz_corpus
            COMMAND ${Python3_EXECUTABLE}
                    ${CMAKE_CURRENT_SOURCE_DIR}/tools/gen_fuzz_corpus.py
                    --vectors ${CMAKE_CURRENT_SOURCE_DIR}/fuzz/corpus_vectors.json
                    --out ${MESSAGE_PARSER_FUZZ_CORPUS_DIR}
            COMMENT "Generating fuzz corpus from canonical RTPS vectors"
        )
        add_custom_target(message_parser_fuzz_smoke
            COMMAND message_parser_fuzz_replay ${MESSAGE_PARSER_FUZZ_CORPUS_DIR}
            DEPENDS message_parser_fuzz_replay message_parser_fuzz_corpus
            COMMENT "Running parser corpus replay smoke test"
        )

        set(FRAGMENT_ASSEMBLER_FUZZ_CORPUS_DIR
            ${CMAKE_CURRENT_BINARY_DIR}/fuzz/corpus/fragment_assembler)
        add_custom_target(fragment_assembler_fuzz_corpus
            COMMAND ${Python3_EXECUTABLE}
                    ${CMAKE_CURRENT_SOURCE_DIR}/tools/gen_fuzz_corpus.py
                    --vectors ${CMAKE_CURRENT_SOURCE_DIR}/fuzz/fragment_assembler_corpus_vectors.json
                    --out ${FRAGMENT_ASSEMBLER_FUZZ_CORPUS_DIR}
            COMMENT "Generating fragment assembler fuzz corpus from seed vectors"
        )
        add_custom_target(fragment_assembler_fuzz_smoke
            COMMAND fragment_assembler_fuzz_replay ${FRAGMENT_ASSEMBLER_FUZZ_CORPUS_DIR}
            DEPENDS fragment_assembler_fuzz_replay fragment_assembler_fuzz_corpus
            COMMENT "Running fragment assembler corpus replay smoke test"
        )

        set(SPDP_FUZZ_CORPUS_DIR
            ${CMAKE_CURRENT_BINARY_DIR}/fuzz/corpus/spdp)
        add_custom_target(spdp_fuzz_corpus
            COMMAND ${Python3_EXECUTABLE}
                    ${CMAKE_CURRENT_SOURCE_DIR}/tools/gen_fuzz_corpus.py
                    --vectors ${CMAKE_CURRENT_SOURCE_DIR}/fuzz/spdp_corpus_vectors.json
                    --out ${SPDP_FUZZ_CORPUS_DIR}
            COMMENT "Generating SPDP fuzz corpus from seed vectors"
        )
        add_custom_target(spdp_fuzz_smoke
            COMMAND spdp_fuzz_replay ${SPDP_FUZZ_CORPUS_DIR}
            DEPENDS spdp_fuzz_replay spdp_fuzz_corpus
            COMMENT "Running SPDP corpus replay smoke test"
        )

        set(SEDP_FUZZ_CORPUS_DIR
            ${CMAKE_CURRENT_BINARY_DIR}/fuzz/corpus/sedp)
        add_custom_target(sedp_fuzz_corpus
            COMMAND ${Python3_EXECUTABLE}
                    ${CMAKE_CURRENT_SOURCE_DIR}/tools/gen_fuzz_corpus.py
                    --vectors ${CMAKE_CURRENT_SOURCE_DIR}/fuzz/sedp_corpus_vectors.json
                    --out ${SEDP_FUZZ_CORPUS_DIR}
            COMMENT "Generating SEDP fuzz corpus from seed vectors"
        )
        add_custom_target(sedp_fuzz_smoke
            COMMAND sedp_fuzz_replay ${SEDP_FUZZ_CORPUS_DIR}
            DEPENDS sedp_fuzz_replay sedp_fuzz_corpus
            COMMENT "Running SEDP corpus replay smoke test"
        )

        set(ENDPOINT_MATCH_FUZZ_CORPUS_DIR
            ${CMAKE_CURRENT_BINARY_DIR}/fuzz/corpus/endpoint_match)
        add_custom_target(endpoint_match_fuzz_corpus
            COMMAND ${Python3_EXECUTABLE}
                    ${CMAKE_CURRENT_SOURCE_DIR}/tools/gen_fuzz_corpus.py
                    --vectors ${CMAKE_CURRENT_SOURCE_DIR}/fuzz/endpoint_match_corpus_vectors.json
                    --out ${ENDPOINT_MATCH_FUZZ_CORPUS_DIR}
            COMMENT "Generating endpoint-match fuzz corpus from seed vectors"
        )
        add_custom_target(endpoint_match_fuzz_smoke
            COMMAND endpoint_match_fuzz_replay ${ENDPOINT_MATCH_FUZZ_CORPUS_DIR}
            DEPENDS endpoint_match_fuzz_replay endpoint_match_fuzz_corpus
            COMMENT "Running endpoint-match corpus replay smoke test"
        )

        add_custom_target(fuzz_smoke_all
            DEPENDS
                message_parser_fuzz_smoke
                fragment_assembler_fuzz_smoke
                spdp_fuzz_smoke
                sedp_fuzz_smoke
                endpoint_match_fuzz_smoke
            COMMENT "Running all fuzz corpus replay smoke tests"
        )
    endif()

    if (CMAKE_CXX_COMPILER_ID MATCHES "Clang")
        add_executable(message_parser_fuzz
            fuzz/fuzz_parse_message.cpp
            ../../src/rtps/message_parser.cpp
        )
        target_include_directories(message_parser_fuzz PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/../..
        )
        target_compile_definitions(message_parser_fuzz PRIVATE
            RTPS_LOG_LEVEL=0
        )
        target_compile_options(message_parser_fuzz PRIVATE
            -fsanitize=fuzzer,address
            -fno-omit-frame-pointer
            -g
        )
        target_link_options(message_parser_fuzz PRIVATE
            -fsanitize=fuzzer,address
        )

        add_executable(fragment_assembler_fuzz
            fuzz/fuzz_fragment_assembler.cpp
            ../../src/rtps/buf_pool.cpp
        )
        target_include_directories(fragment_assembler_fuzz PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/../..
        )
        target_compile_definitions(fragment_assembler_fuzz PRIVATE
            RTPS_LOG_LEVEL=0
        )
        target_compile_options(fragment_assembler_fuzz PRIVATE
            -fsanitize=fuzzer,address
            -fno-omit-frame-pointer
            -g
        )
        target_link_options(fragment_assembler_fuzz PRIVATE
            -fsanitize=fuzzer,address
        )

        add_executable(spdp_fuzz
            fuzz/fuzz_spdp.cpp
            ../../src/rtps/spdp.cpp
            ../../src/rtps/buf_pool.cpp
        )
        target_include_directories(spdp_fuzz PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/../..
        )
        target_compile_definitions(spdp_fuzz PRIVATE
            RTPS_LOG_LEVEL=0
        )
        target_compile_options(spdp_fuzz PRIVATE
            -fsanitize=fuzzer,address
            -fno-omit-frame-pointer
            -g
        )
        target_link_options(spdp_fuzz PRIVATE
            -fsanitize=fuzzer,address
        )

        add_executable(sedp_fuzz
            fuzz/fuzz_sedp.cpp
            ../../src/rtps/sedp.cpp
            ../../src/rtps/endpoint_match_table.cpp
            ../../src/rtps/sn_window.cpp
        ../../src/rtps/history_cache.cpp
        ../../src/rtps/reader_proxy.cpp
        ../../src/rtps/stateful_writer.cpp
    ../../src/rtps/history_cache.cpp
    ../../src/rtps/reader_proxy.cpp
    ../../src/rtps/stateful_writer.cpp
            ../../src/rtps/buf_pool.cpp
        )
        target_include_directories(sedp_fuzz PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/../..
        )
        target_compile_definitions(sedp_fuzz PRIVATE
            RTPS_LOG_LEVEL=0
        )
        target_compile_options(sedp_fuzz PRIVATE
            -fsanitize=fuzzer,address
            -fno-omit-frame-pointer
            -g
        )
        target_link_options(sedp_fuzz PRIVATE
            -fsanitize=fuzzer,address
        )

        add_executable(endpoint_match_fuzz
            fuzz/fuzz_endpoint_match_table.cpp
            ../../src/rtps/endpoint_match_table.cpp
        )
        target_include_directories(endpoint_match_fuzz PRIVATE
            ${CMAKE_CURRENT_SOURCE_DIR}/../..
        )
        target_compile_definitions(endpoint_match_fuzz PRIVATE
            RTPS_LOG_LEVEL=0
        )
        target_compile_options(endpoint_match_fuzz PRIVATE
            -fsanitize=fuzzer,address
            -fno-omit-frame-pointer
            -g
        )
        target_link_options(endpoint_match_fuzz PRIVATE
            -fsanitize=fuzzer,address
        )
    else()
        message(WARNING "ROSETTE_SPEC_ENABLE_FUZZ is ON, but compiler is not Clang; libFuzzer targets skipped. Use *_fuzz_replay/*_fuzz_smoke targets.")
    endif()
endif()

# Every spec and fuzz target runs on the platform port.
foreach(_t message_parser_spec_tests message_builder_spec_tests fragment_assembler_spec_tests spdp_spec_tests sedp_spec_tests history_cache_spec_tests stateful_writer_spec_tests stateful_reader_spec_tests participant_spec_tests message_parser_fuzz_replay fragment_assembler_fuzz_replay spdp_fuzz_replay sedp_fuzz_replay endpoint_match_fuzz_replay message_parser_fuzz fragment_assembler_fuzz spdp_fuzz sedp_fuzz endpoint_match_fuzz)
    if(TARGET ${_t})
        rosette_spec_use_platform(${_t})
    endif()
endforeach()
