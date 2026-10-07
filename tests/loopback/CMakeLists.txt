cmake_minimum_required(VERSION 3.20)

project(rosette_loopback_tests CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

enable_testing()

get_filename_component(ROSETTE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../.." REALPATH)
include(${ROSETTE_ROOT}/cmake/rosette_rtps_sources.cmake)

add_executable(rtps_loopback_smoke
    rtps_loopback_smoke.cpp
    ${ROSETTE_RTPS_PROTOCOL_SOURCES}
    ../../src/platform/wall_clock.cpp
    ../../src/support/fnv1a.cpp
    ../../ports/chip/host/chip_host.cpp
)

target_include_directories(rtps_loopback_smoke PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../..
    ${CMAKE_CURRENT_SOURCE_DIR}/../../include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../include/rosette/rtps
    ${CMAKE_CURRENT_SOURCE_DIR}/../../ports/os/posix
)

target_compile_definitions(rtps_loopback_smoke PRIVATE
    ROSETTE_OS_POSIX
    ROSETTE_CHIP_HOST
    RTPS_LOG_LEVEL=0
)

add_test(NAME rtps_loopback_smoke COMMAND rtps_loopback_smoke)
