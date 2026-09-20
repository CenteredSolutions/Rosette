cmake_minimum_required(VERSION 3.20)

project(rosette_loopback_tests CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

enable_testing()

add_executable(rtps_loopback_smoke
    rtps_loopback_smoke.cpp
    ../../src/rtps/buf_pool.cpp
    ../../src/rtps/message_parser.cpp
    ../../src/rtps/participant.cpp
    ../../src/rtps/sedp.cpp
    ../../src/rtps/spdp.cpp
    ../../src/rtps/stateful_reader.cpp
)

target_include_directories(rtps_loopback_smoke PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../..
    ${CMAKE_CURRENT_SOURCE_DIR}/../../include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../include/rosette/rtps
)

target_compile_definitions(rtps_loopback_smoke PRIVATE
    RTPS_NO_FREERTOS
    RTPS_LOG_LEVEL=0
)

add_test(NAME rtps_loopback_smoke COMMAND rtps_loopback_smoke)
