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

# The out-of-line pieces of the address layer (candidates, lists, manager, config, WIZnet DHCP).
add_library(rosette_host_address STATIC
    ${CMAKE_CURRENT_SOURCE_DIR}/../../src/net/address_candidate.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../src/net/address_list.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../src/net/address_manager.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../src/net/network_config.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../src/net/wiznet/dhcp_client.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../src/net/wiznet/dhcp_provider.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../src/net/wiznet/dhcp_wire.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../src/support/fnv1a.cpp)
target_include_directories(rosette_host_address PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../../include)
target_compile_options(rosette_host_address PRIVATE -Wall -Wextra)

# add_host_test(<name>) — one .cpp against include/, linking only rosette_host_address.
function(add_host_test name)
    add_executable(${name} ${name}.cpp)
    target_link_libraries(${name} PRIVATE rosette_host_address)
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

# WIZnet UDP receive/send, IPv4 and IPv6, against a register-file chip model.
add_host_test(wiznet_udp_socket_test)
add_host_test(wiznet_slaac_provider_test)

# RFC 4861 Router Advertisement decoding, chip-neutral.
add_host_test(ipv6_router_advert_test)

# Address bring-up layer.
add_host_test(address_snapshot_test)
add_host_test(address_candidate_test)
add_host_test(ipv4ll_provider_test)
add_host_test(address_manager_test)
add_host_test(network_address_test)
add_host_test(wiznet_address_provider_test)
add_host_test(dhcp_client_test)
add_host_test(address_upgrade_test)
add_host_test(network_config_test)

# The ROS session supervisor, against a fake network, application, and agent gate.
add_host_test(session_supervisor_test)
target_sources(session_supervisor_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../../src/ros/session_supervisor.cpp)

# The micro-ROS agent's session gate, against a fake agent link, locator, and transport.
add_host_test(agent_session_gate_test)
target_sources(agent_session_gate_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../../src/ros/agent_session_gate.cpp)

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

# SNTP: the NTPv4 wire format, and the client against a simulated server, path, and tick.
add_host_test(sntp_wire_test)
target_sources(sntp_wire_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../../src/net/sntp_wire.cpp)
add_host_test(sntp_client_test)
target_sources(sntp_client_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../../src/net/sntp_wire.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../../src/net/sntp_client.cpp)

# SntpConfig defaults on each ROS backend (the agent backend's build defines ROSETTE_ROS_BACKEND_AGENT).
add_host_test(sntp_config_default_test)
add_executable(sntp_config_default_agent_test sntp_config_default_test.cpp)
target_include_directories(sntp_config_default_agent_test PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/../../include)
target_compile_definitions(sntp_config_default_agent_test PRIVATE ROSETTE_ROS_BACKEND_AGENT)
target_compile_options(sntp_config_default_agent_test PRIVATE -Wall -Wextra)
add_test(NAME sntp_config_default_agent_test COMMAND sntp_config_default_agent_test)
