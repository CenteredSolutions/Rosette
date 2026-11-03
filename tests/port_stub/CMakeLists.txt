# SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
# SPDX-License-Identifier: Apache-2.0

# tests/port_stub — the portable core, built and linked on the stub port.
#
# Builds every portable library (RTPS, the network core with DHCP and SNTP,
# storage, the ROS session supervisor) and the header-only WIZnet driver for a chip that is neither an RP2040 /
# RP2350 nor a host, on ports/chip/stub and an OS port with no Pico SDK, no
# RTOS, and no POSIX: ports/os/stub, or ports/os/baremetal
# (-DROSETTE_OS=baremetal), which shows the bare-metal OS layer needs only
# the chip's sync hooks. The image links every object of each library, so a portable
# file that calls a Pico SDK, RTOS, or host OS function fails to link here
# even when nothing in the image calls that file.
#
#   cmake -S tests/port_stub -B build-port-stub \
#         -DCMAKE_TOOLCHAIN_FILE=tests/port_stub/arm-none-eabi.cmake
#   cmake --build build-port-stub
#
# Verification: the build succeeds; adding a call to, for example,
# `time_us_64()` or `socket()` to any portable source makes it fail to link.

cmake_minimum_required(VERSION 3.20)
project(rosette_port_stub C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

get_filename_component(ROSETTE_ROOT "${CMAKE_CURRENT_LIST_DIR}/../.." REALPATH)
set(ROSETTE_OS stub CACHE STRING "OS port to build the stub chip with: stub or baremetal")
set(ROSETTE_CHIP stub)

include(${ROSETTE_ROOT}/cmake/rosette_platform.cmake)

# The portable network core (sockets, address management, SNTP) and the
# WIZnet network port, which adds DHCP; the WIZnet driver itself is
# header-only.
set(ROSETTE_NET_STACK wiznet)
include(${ROSETTE_ROOT}/cmake/rosette_net.cmake)

include(${ROSETTE_ROOT}/cmake/rosette_rtps.cmake)

include(${ROSETTE_ROOT}/cmake/littlefs.cmake)
add_library(rosette_fs STATIC
  ${ROSETTE_LITTLEFS_DIR}/lfs.c
  ${ROSETTE_LITTLEFS_DIR}/lfs_util.c)
target_include_directories(rosette_fs SYSTEM PUBLIC ${ROSETTE_LITTLEFS_DIR})

add_library(rosette_storage STATIC
  ${ROSETTE_ROOT}/src/storage/volume.cpp
  ${ROSETTE_ROOT}/src/storage/file_store.cpp
  ${ROSETTE_ROOT}/src/storage/device_config.cpp
  ${ROSETTE_CHIP_STORAGE_SOURCES})
target_link_libraries(rosette_storage PUBLIC rosette_fs rosette_platform)

add_library(rosette_session STATIC
  ${ROSETTE_ROOT}/src/ros/session_supervisor.cpp
  ${ROSETTE_ROOT}/src/ros/agent_session_gate.cpp
  ${ROSETTE_ROOT}/src/ros/time_sync.cpp)
target_link_libraries(rosette_session PUBLIC rosette_net)

# Every object of every library goes into the image, not just the ones main()
# reaches, so each needs every symbol it calls. Linking the libraries as well
# brings their include paths and definitions, and pulls nothing more.
set(_stub_libraries rosette_platform rosette_net rosette_rtps rosette_storage
    rosette_session rosette_fs)
add_executable(rosette_port_stub_image stub_image.cpp)
target_include_directories(rosette_port_stub_image PRIVATE ${ROSETTE_ROOT}/ports/chip/stub)
foreach(_library IN LISTS _stub_libraries)
  target_sources(rosette_port_stub_image PRIVATE $<TARGET_OBJECTS:${_library}>)
endforeach()
target_link_libraries(rosette_port_stub_image PRIVATE ${_stub_libraries})

# Warnings apply to Rosette's code; rosette_fs is the fetched littlefs.
include(${ROSETTE_ROOT}/cmake/rosette_warnings.cmake)
rosette_enable_warnings(rosette_platform rosette_net rosette_rtps rosette_storage
                        rosette_session rosette_port_stub_image)
