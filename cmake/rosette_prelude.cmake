# cmake/rosette_prelude.cmake
#
# Board selection and toolchain setup that MUST run before the first project()
# call (so the Pico SDK can install its cross toolchain). Included by the repo
# root CMakeLists and by any standalone example that wants to build for a real
# board.
#
# Usage in a standalone top-level CMakeLists:
#
#   cmake_minimum_required(VERSION 3.20)
#   include(${ROSETTE_DIR}/cmake/rosette_prelude.cmake)
#   project(my_firmware C CXX ASM)
#   add_subdirectory(${ROSETTE_DIR} rosette)
#   add_executable(my_firmware main.cpp)
#   target_link_libraries(my_firmware PRIVATE rosette_agent)   # or rmw_rosette_firmware

include_guard(GLOBAL)

set(ROSETTE_SOURCE_DIR "${CMAKE_CURRENT_LIST_DIR}/.." CACHE INTERNAL
    "Rosette repository root")

# ------------------------------------------------------------
# Board selection — pass -DROSETTE_BOARD=<name>:
#   posix              Host / POSIX (default on macOS)
#   w5100s-evb-pico   WIZnet W5100S-EVB-Pico   (RP2040)
#   w5100s-evb-pico2  WIZnet W5100S-EVB-Pico2  (RP2350, default elsewhere)
#   w6100-evb-pico    WIZnet W6100-EVB-Pico    (RP2040)
#   w6100-evb-pico2   WIZnet W6100-EVB-Pico2   (RP2350)
#   w6300-evb-pico2   WIZnet W6300-EVB-Pico2   (RP2350, QSPI single-lane — provisional)
# ------------------------------------------------------------
set(_rosette_default_board "w5100s-evb-pico2")
if(CMAKE_HOST_SYSTEM_NAME STREQUAL "Darwin")
    set(_rosette_default_board "posix")
endif()

set(ROSETTE_BOARD "${_rosette_default_board}" CACHE STRING
    "Target board: posix | w5100s-evb-pico | w5100s-evb-pico2 | w6100-evb-pico | w6100-evb-pico2 | w6300-evb-pico2")
set_property(CACHE ROSETTE_BOARD PROPERTY STRINGS
    posix w5100s-evb-pico w5100s-evb-pico2 w6100-evb-pico w6100-evb-pico2 w6300-evb-pico2)

include("${CMAKE_CURRENT_LIST_DIR}/boards.cmake")
rosette_select_board(${ROSETTE_BOARD})

if(ROSETTE_BOARD_ENABLE_PICO_SDK AND NOT DEFINED ENV{PICO_SDK_PATH} AND NOT DEFINED PICO_SDK_PATH)
    message(STATUS "")
    message(STATUS "Rosette: first configure for '${ROSETTE_BOARD}' fetches the Pico SDK,")
    message(STATUS "  and FreeRTOS (a few minutes; progress shown below). Set")
    message(STATUS "  PICO_SDK_PATH / FREERTOS_KERNEL_PATH / FREERTOS_TCP_PATH to use")
    message(STATUS "  existing checkouts. The micro-ROS library")
    message(STATUS "  is cross-built later, during 'cmake --build' (~20-30 min, one time).")
    message(STATUS "")
endif()

if(ROSETTE_BOARD_ENABLE_PICO_SDK)
    include("${CMAKE_CURRENT_LIST_DIR}/pico_sdk.cmake")
endif()
