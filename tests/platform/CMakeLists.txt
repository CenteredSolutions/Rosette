# SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
# SPDX-License-Identifier: Apache-2.0

# Platform contract conformance test (host port).
#
# Builds tests/platform/platform_contract_test.cpp against the POSIX OS layer
# and host chip layer. Board builds compile the same test against their own
# ports (see cmake/rosette_platform.cmake).
cmake_minimum_required(VERSION 3.20)
project(rosette_platform_tests CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
enable_testing()

set(ROSETTE_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/../..)
set(ROSETTE_OS posix)
set(ROSETTE_CHIP host)
include(${ROSETTE_ROOT}/cmake/rosette_platform.cmake)

add_executable(platform_contract_test platform_contract_test.cpp)
target_link_libraries(platform_contract_test PRIVATE rosette_platform)
add_test(NAME platform_contract COMMAND platform_contract_test)
