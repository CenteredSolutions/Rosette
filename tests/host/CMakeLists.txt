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

add_executable(mdns_wire_test mdns_wire_test.cpp)
target_include_directories(mdns_wire_test
    PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../../include)
target_compile_options(mdns_wire_test PRIVATE -Wall -Wextra)
add_test(NAME mdns_wire_test COMMAND mdns_wire_test)

# WIZnet chip-trait SPI frame encoding (W5100S regression + W6100 / W6300).
add_executable(wiznet_header_test wiznet_header_test.cpp)
target_include_directories(wiznet_header_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../../include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../include/rosette/net)
target_compile_definitions(wiznet_header_test PRIVATE ROSETTE_NO_FREERTOS)
target_compile_options(wiznet_header_test PRIVATE -Wall -Wextra)
add_test(NAME wiznet_header_test COMMAND wiznet_header_test)
