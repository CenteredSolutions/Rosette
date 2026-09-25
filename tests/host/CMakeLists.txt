# Host-side unit tests that need only a C++17 toolchain and the Rosette headers.
#
#   cmake -S tests/host -B build/tests-host
#   cmake --build build/tests-host -j
#   ctest --test-dir build/tests-host --output-on-failure

cmake_minimum_required(VERSION 3.16)
project(rosette_host_tests CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

enable_testing()

# Host tests run on the POSIX platform port (include/rosette/platform/platform.hpp).
add_compile_definitions(ROSETTE_OS_POSIX ROSETTE_CHIP_HOST)

# add_host_test(<name>) — one .cpp, header-only against include/, no link deps.
function(add_host_test name)
    add_executable(${name} ${name}.cpp)
    target_include_directories(${name} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/../../include
        ${CMAKE_CURRENT_SOURCE_DIR}/../../include/rosette/net)
    target_compile_options(${name} PRIVATE -Wall -Wextra)
    add_test(NAME ${name} COMMAND ${name})
endfunction()

add_executable(mdns_wire_test mdns_wire_test.cpp)
target_include_directories(mdns_wire_test
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../../include)
target_compile_options(mdns_wire_test PRIVATE -Wall -Wextra)
add_test(NAME mdns_wire_test COMMAND mdns_wire_test)

# WIZnet chip-trait SPI frame encoding + the conflict / socket-less ARP traits.
add_host_test(wiznet_header_test)

# Address bring-up layer.
add_host_test(address_snapshot_test)
add_host_test(ipv4ll_provider_test)
add_host_test(address_manager_test)
add_host_test(network_address_test)
add_host_test(wiznet_address_provider_test)

# RTPS buffer pool size-class selection, on a pool of two blocks per class.
add_executable(buf_pool_class_test
    buf_pool_class_test.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../src/rtps/buf_pool.cpp)
target_include_directories(buf_pool_class_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../../include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../include/rosette/rtps)
target_compile_definitions(buf_pool_class_test PRIVATE
    RTPS_LOG_LEVEL=0
    RTPS_POOL_CLASS0_COUNT=2u RTPS_POOL_CLASS1_COUNT=2u RTPS_POOL_CLASS2_COUNT=2u
    RTPS_POOL_CLASS3_COUNT=2u RTPS_POOL_CLASS4_COUNT=2u)
target_compile_options(buf_pool_class_test PRIVATE -Wall -Wextra)
add_test(NAME buf_pool_class_test COMMAND buf_pool_class_test)
