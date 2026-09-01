cmake_minimum_required(VERSION 3.20)

# ============================================================
# Board selection
#
# Pass -DROSETTE_BOARD=<name> to select a target board.
# Valid values:
#   posix               — Host/POSIX build (default on macOS)
#   w5100s-evb-pico    — WIZnet W5100S-EVB-Pico  (RP2040)
#   w5100s-evb-pico2   — WIZnet W5100S-EVB-Pico2 (RP2350, default elsewhere)
#   bc-gen2            — Custom board based on RP2040 + W5100S
#
# This MUST be set before the project() call so that the Pico SDK
# can pick up PICO_BOARD correctly when building for Pico boards.
# ============================================================

set(_rosette_default_board "w5100s-evb-pico2")
if(CMAKE_HOST_SYSTEM_NAME STREQUAL "Darwin")
    set(_rosette_default_board "posix")
endif()

set(ROSETTE_BOARD "${_rosette_default_board}" CACHE STRING
    "Target board: posix | w5100s-evb-pico | w5100s-evb-pico2 | bc-gen2")

set_property(CACHE ROSETTE_BOARD PROPERTY STRINGS
    posix
    w5100s-evb-pico
    w5100s-evb-pico2
    bc-gen2)

# Map ROSETTE_BOARD → board-specific configuration.
include(cmake/boards.cmake)
rosette_select_board(${ROSETTE_BOARD})

if(ROSETTE_BOARD_ENABLE_PICO_SDK)
    include(cmake/pico_sdk.cmake)
endif()

project(rosette_net C CXX ASM)
set(CMAKE_C_STANDARD   11)
set(CMAKE_CXX_STANDARD 17)

include(cmake/micro_ros.cmake)

if(ROSETTE_BOARD_ENABLE_PICO_SDK)
    pico_sdk_init()
endif()

if(ROSETTE_BOARD_ENABLE_FREERTOS)
    include(cmake/freertos_tcp.cmake)
endif()

if(ROSETTE_BOARD_ENABLE_WIZNET)
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/cmake/wiznet.cmake")
        include(cmake/wiznet.cmake)
    else()
        message(FATAL_ERROR
            "Selected board '${ROSETTE_BOARD}' requires cmake/wiznet.cmake, but it is missing.")
    endif()
endif()

# ------------------------------------------------------------
# rosette_fs — persistent device-configuration store (LittleFS)
#
# Boards keep a small LittleFS volume in a reserved flash region so device
# configuration (network mode, addressing, Agent endpoint, XRCE client key)
# survives power cycles and reflashing of the application image. Host builds
# link it only when a local LittleFS checkout is available.
# ------------------------------------------------------------
set(ROSETTE_HAVE_LITTLEFS OFF)
if(ROSETTE_BOARD_ENABLE_FREERTOS)
    include(cmake/littlefs.cmake)
else()
    set(_rosette_host_littlefs "${CMAKE_CURRENT_SOURCE_DIR}/../littlefs")
    if(DEFINED ENV{LITTLEFS_PATH} AND EXISTS "$ENV{LITTLEFS_PATH}/lfs.c")
        set(ROSETTE_LITTLEFS_DIR "$ENV{LITTLEFS_PATH}")
        set(ROSETTE_HAVE_LITTLEFS ON)
    elseif(EXISTS "${_rosette_host_littlefs}/lfs.c")
        set(ROSETTE_LITTLEFS_DIR "${_rosette_host_littlefs}")
        set(ROSETTE_HAVE_LITTLEFS ON)
    endif()
endif()

if(ROSETTE_HAVE_LITTLEFS)
    add_library(rosette_fs STATIC
        ${ROSETTE_LITTLEFS_DIR}/lfs.c
        ${ROSETTE_LITTLEFS_DIR}/lfs_util.c
    )
    target_include_directories(rosette_fs PUBLIC
        ${ROSETTE_LITTLEFS_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/include
    )
endif()

# ------------------------------------------------------------
# rosette_net — socket abstraction layer (rosette::net)
#
# The custom DDSI-RTPS stack that this library used to carry has been
# archived (tag archive/rtps-stack) as part of the micro-ROS-only
# refactor. What remains is the transport-neutral socket abstraction
# that both the board BSP and the micro-ROS client build on.
# See docs/design/rosette_micro_ros_refactor.md.
# ------------------------------------------------------------
add_library(rosette_net STATIC
    src/net/socket.cpp
)

set(ROSETTE_INTERNAL_INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/include/rosette/net
    ${CMAKE_CURRENT_SOURCE_DIR}/include/rosette/platform
    ${CMAKE_CURRENT_SOURCE_DIR}/include/rosette/ros)

target_include_directories(rosette_net PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${ROSETTE_BOARD_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/config
    ${ROSETTE_INTERNAL_INCLUDE_DIRS}
)

if(NOT ROSETTE_BOARD_ENABLE_FREERTOS)
    target_compile_definitions(rosette_net PUBLIC
        ROSETTE_NO_FREERTOS
    )
endif()

if(ROSETTE_BOARD_ENABLE_FREERTOS)
    target_link_libraries(rosette_net PUBLIC
        freertos_kernel
        freertos_tcp
    )
endif()

if(ROSETTE_BOARD_ENABLE_WIZNET)
    target_link_libraries(rosette_net PUBLIC
        wiznet_iolib
    )
endif()

if(ROSETTE_BOARD_ENABLE_PICO_SDK)
    target_link_libraries(rosette_net PUBLIC
        pico_stdlib
        hardware_spi
        hardware_flash
        hardware_sync
    )
endif()

# ------------------------------------------------------------
# rosette_bsp — board bring-up, WIZnet MACRAW driver, FreeRTOS glue
# ------------------------------------------------------------
if(ROSETTE_BOARD_BUILD_BSP)
    add_library(rosette_bsp STATIC
        ${ROSETTE_BOARD_DIR}/board_init.c
        src/net/freertos/igmp.cpp
        src/net/freertos/socket.cpp
        src/net/wiznet/macraw.cpp
        src/platform/pico_freertos.c
        src/platform/pico_log.c
    )

    target_include_directories(rosette_bsp PUBLIC
        ${ROSETTE_BOARD_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${ROSETTE_INTERNAL_INCLUDE_DIRS}
    )

    target_link_libraries(rosette_bsp PUBLIC
        rosette_net
        freertos_kernel
        freertos_tcp
        wiznet_iolib
        pico_stdlib
        hardware_spi
    )

    if(TARGET rosette_fs)
        target_link_libraries(rosette_bsp PUBLIC rosette_fs)
    endif()
endif()

# ------------------------------------------------------------
# Firmware examples and the micro-ROS smoke executable are rebuilt
# under rosette_* target names in later steps of the micro-ROS
# refactor (see docs/design/rosette_micro_ros_refactor.md, phases
# 4 and 7). examples/main_micro_ros_board_smoke.cpp is kept as the
# reference for that rewrite.
# ------------------------------------------------------------

message(STATUS "")
message(STATUS "Rosette build configuration")
message(STATUS "  Board       : ${ROSETTE_BOARD}")
message(STATUS "  Family      : ${ROSETTE_BOARD_FAMILY}")
message(STATUS "  Board dir   : ${ROSETTE_BOARD_DIR}")
if(ROSETTE_BOARD_ENABLE_PICO_SDK)
    message(STATUS "  PICO_BOARD  : ${PICO_BOARD}")
endif()
message(STATUS "  Processor   : ${ROSETTE_PROCESSOR}")
if(ROSETTE_BOARD_ENABLE_WIZNET)
    message(STATUS "  SPI pins    : SCK=${ROSETTE_SPI_SCK} TX=${ROSETTE_SPI_TX} RX=${ROSETTE_SPI_RX} CS=${ROSETTE_SPI_CS}")
    message(STATUS "  W5100S RST  : ${ROSETTE_W5100S_RST}")
    message(STATUS "  W5100S INT  : ${ROSETTE_W5100S_INT}")
endif()
message(STATUS "")
