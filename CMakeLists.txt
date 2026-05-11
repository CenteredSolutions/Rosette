cmake_minimum_required(VERSION 3.20)

# ============================================================
# Board selection
#
# Pass -DRTPS_BOARD=<name> to select a target board.
# Valid values:
#   posix               — Host/POSIX build (default on macOS)
#   w5100s-evb-pico    — WIZnet W5100S-EVB-Pico  (RP2040)
#   w5100s-evb-pico2   — WIZnet W5100S-EVB-Pico2 (RP2350, default elsewhere)
#   bc-gen2            — Custom board based on RP2040 + W5100S
#
# This MUST be set before the project() call so that the Pico SDK
# can pick up PICO_BOARD correctly when building for Pico boards.
# ============================================================

set(_rtps_default_board "w5100s-evb-pico2")
if(CMAKE_HOST_SYSTEM_NAME STREQUAL "Darwin")
    set(_rtps_default_board "posix")
endif()

set(RTPS_BOARD "${_rtps_default_board}" CACHE STRING
    "Target board: posix | w5100s-evb-pico | w5100s-evb-pico2 | bc-gen2")

set_property(CACHE RTPS_BOARD PROPERTY STRINGS
    posix
    w5100s-evb-pico
    w5100s-evb-pico2
    bc-gen2)

# Map RTPS_BOARD → board-specific configuration.
include(cmake/boards.cmake)
rtps_select_board(${RTPS_BOARD})

if(RTPS_BOARD_ENABLE_PICO_SDK)
    include(cmake/pico_sdk.cmake)
endif()

project(rtps_stack C CXX ASM)
set(CMAKE_C_STANDARD   11)
set(CMAKE_CXX_STANDARD 17)

include(cmake/micro_ros.cmake)

if(RTPS_BOARD_ENABLE_PICO_SDK)
    pico_sdk_init()
endif()

set(RTPS_HAVE_LITTLEFS OFF)

# Embedded boards require LittleFS persistence support.
if(RTPS_BOARD_ENABLE_FREERTOS)
    include(cmake/littlefs.cmake)
else()
    # Host/posix builds may compile without LittleFS present.
    set(RTPS_LITTLEFS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../littlefs")
    if(EXISTS "${RTPS_LITTLEFS_DIR}/lfs.c" AND
       EXISTS "${RTPS_LITTLEFS_DIR}/lfs_util.c" AND
       EXISTS "${RTPS_LITTLEFS_DIR}/lfs.h")
        set(RTPS_HAVE_LITTLEFS ON)
    endif()
endif()

if(RTPS_BOARD_ENABLE_FREERTOS)
    include(cmake/freertos_tcp.cmake)
endif()

if(RTPS_BOARD_ENABLE_WIZNET)
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/cmake/wiznet.cmake")
        include(cmake/wiznet.cmake)
    else()
        message(FATAL_ERROR
            "Selected board '${RTPS_BOARD}' requires cmake/wiznet.cmake, but it is missing.")
    endif()
endif()

set(RTPS_STACK_SOURCES
    src/rtps/message_parser.cpp
    src/rtps/stateful_reader.cpp
    src/rtps/spdp.cpp
    src/rtps/sedp.cpp
    src/rtps/participant.cpp
    src/rtps/buf_pool.cpp
    src/rtps/transport.cpp
    src/net/socket.cpp
)

if(RTPS_HAVE_LITTLEFS)
    list(APPEND RTPS_STACK_SOURCES
        src/rtps/persistence.cpp
        ${RTPS_LITTLEFS_DIR}/lfs.c
        ${RTPS_LITTLEFS_DIR}/lfs_util.c
    )
endif()

add_library(rtps_stack STATIC ${RTPS_STACK_SOURCES})

set(ROSETTE_INTERNAL_INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/include/rosette/net
    ${CMAKE_CURRENT_SOURCE_DIR}/include/rosette/platform
    ${CMAKE_CURRENT_SOURCE_DIR}/include/rosette/ros
    ${CMAKE_CURRENT_SOURCE_DIR}/include/rosette/rtps)

target_include_directories(rtps_stack PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${RTPS_BOARD_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/config
    ${ROSETTE_INTERNAL_INCLUDE_DIRS}
)

if(RTPS_HAVE_LITTLEFS)
    target_include_directories(rtps_stack PUBLIC
        ${RTPS_LITTLEFS_DIR}
    )
endif()

if(NOT RTPS_BOARD_ENABLE_FREERTOS)
    target_compile_definitions(rtps_stack PUBLIC
        RTPS_NO_FREERTOS
    )
endif()

if(RTPS_BOARD_ENABLE_FREERTOS)
    target_link_libraries(rtps_stack PUBLIC
        freertos_kernel
        freertos_tcp
    )
endif()

if(RTPS_BOARD_ENABLE_WIZNET)
    target_link_libraries(rtps_stack PUBLIC
        wiznet_iolib
    )
endif()

if(RTPS_BOARD_ENABLE_PICO_SDK)
    target_link_libraries(rtps_stack PUBLIC
        pico_stdlib
        hardware_spi
        hardware_flash
        hardware_sync
    )
endif()

if(RTPS_BOARD_BUILD_BSP)
    add_library(rtps_bsp STATIC
        ${RTPS_BOARD_DIR}/board_init.c
        src/net/freertos/igmp.cpp
        src/net/freertos/socket.cpp
        src/net/wiznet/macraw.cpp
        src/platform/pico_freertos.c
        src/platform/pico_log.c
    )

    target_include_directories(rtps_bsp PUBLIC
        ${RTPS_BOARD_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${ROSETTE_INTERNAL_INCLUDE_DIRS}
    )

    target_link_libraries(rtps_bsp PUBLIC
        rtps_stack
        freertos_kernel
        freertos_tcp
        wiznet_iolib
        pico_stdlib
        hardware_spi
    )
endif()

if(RTPS_BOARD_BUILD_EXAMPLE)
    add_executable(rtps_example
        ${RTPS_BOARD_EXAMPLE_SOURCE}
    )

    target_link_libraries(rtps_example PRIVATE
        rtps_stack
        rtps_bsp
    )

    pico_add_extra_outputs(rtps_example)
    pico_enable_stdio_usb(rtps_example 1)
    pico_enable_stdio_uart(rtps_example 0)
endif()

if(ROSETTE_ENABLE_MICRO_ROS AND ROSETTE_BUILD_MICRO_ROS_BOARD_SMOKE)
    set(ROSETTE_MICRO_ROS_BOARD_SMOKE_SOURCES
        examples/main_micro_ros_board_smoke.cpp
    )

    if(RTPS_BOARD_ENABLE_FREERTOS)
        list(APPEND ROSETTE_MICRO_ROS_BOARD_SMOKE_SOURCES
            src/net/ntp_sync.cpp
        )
    else()
        list(APPEND ROSETTE_MICRO_ROS_BOARD_SMOKE_SOURCES
            src/platform/posix/rtps_posix_platform.cpp
            src/platform/posix/rtps_posix_log.cpp
            src/net/posix/socket.cpp
        )
    endif()

    add_executable(rtps_micro_ros_board_smoke
        ${ROSETTE_MICRO_ROS_BOARD_SMOKE_SOURCES}
    )

    target_link_libraries(rtps_micro_ros_board_smoke PRIVATE
        rtps_stack
        micro_ros_vendor
    )

    if(RTPS_BOARD_BUILD_BSP)
        target_link_libraries(rtps_micro_ros_board_smoke PRIVATE
            rtps_bsp
        )
    endif()

    if(RTPS_BOARD_ENABLE_PICO_SDK)
        target_link_libraries(rtps_micro_ros_board_smoke PRIVATE
            hardware_adc
        )
        pico_add_extra_outputs(rtps_micro_ros_board_smoke)
        pico_enable_stdio_usb(rtps_micro_ros_board_smoke 1)
        pico_enable_stdio_uart(rtps_micro_ros_board_smoke 0)
    endif()
endif()

message(STATUS "")
message(STATUS "RTPS Stack build configuration")
message(STATUS "  Board       : ${RTPS_BOARD}")
message(STATUS "  Family      : ${RTPS_BOARD_FAMILY}")
message(STATUS "  Board dir   : ${RTPS_BOARD_DIR}")
message(STATUS "  LittleFS    : ${RTPS_HAVE_LITTLEFS}")
if(RTPS_BOARD_ENABLE_PICO_SDK)
    message(STATUS "  PICO_BOARD  : ${PICO_BOARD}")
endif()
message(STATUS "  Processor   : ${RTPS_PROCESSOR}")
if(RTPS_BOARD_ENABLE_WIZNET)
    message(STATUS "  SPI pins    : SCK=${RTPS_SPI_SCK} TX=${RTPS_SPI_TX} RX=${RTPS_SPI_RX} CS=${RTPS_SPI_CS}")
    message(STATUS "  W5100S RST  : ${RTPS_W5100S_RST}")
    message(STATUS "  W5100S INT  : ${RTPS_W5100S_INT}")
endif()
message(STATUS "")
