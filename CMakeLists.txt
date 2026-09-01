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
# rosette_fs — the LittleFS sources
#
# cmake/littlefs.cmake locates a local checkout (LITTLEFS_PATH) or fetches one.
# rosette_fs compiles it; rosette_storage layers the file API on top.
# ------------------------------------------------------------
include(cmake/littlefs.cmake)

add_library(rosette_fs STATIC
    ${ROSETTE_LITTLEFS_DIR}/lfs.c
    ${ROSETTE_LITTLEFS_DIR}/lfs_util.c
)
target_include_directories(rosette_fs PUBLIC
    ${ROSETTE_LITTLEFS_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

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

if(NOT ROSETTE_BOARD_ENABLE_FREERTOS)
    # Host platform runtime (monotonic clock, local address). The FreeRTOS
    # boards get the same symbols from src/platform/pico_freertos.c via the BSP.
    target_sources(rosette_net PRIVATE src/platform/posix/host_runtime.cpp)
endif()

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
        hardware_flash
        hardware_sync
    )
endif()

# ------------------------------------------------------------
# rosette_storage — LittleFS volume + generic file API
#
#   rosette::storage::Volume        mounts the board storage region
#   rosette::storage::FileStore     content-agnostic keyed blob store
#   rosette::storage::DeviceConfig  the micro-ROS client's own boot config
#
# The board block-device implementation is compiled in per target: the Pico
# flash binding for FreeRTOS boards, a file-backed volume for host builds.
# ------------------------------------------------------------
add_library(rosette_storage STATIC
    src/storage/volume.cpp
    src/storage/file_store.cpp
    src/storage/device_config.cpp
)
target_include_directories(rosette_storage PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(rosette_storage PUBLIC rosette_fs)

if(ROSETTE_BOARD_ENABLE_FREERTOS)
    target_sources(rosette_storage PRIVATE src/platform/pico_littlefs.cpp)
    target_include_directories(rosette_storage PRIVATE ${ROSETTE_BOARD_DIR})
    target_link_libraries(rosette_storage PUBLIC rosette_bsp)
else()
    target_sources(rosette_storage PRIVATE src/platform/posix/host_littlefs.cpp)
endif()

# ------------------------------------------------------------
# rosette_ros — the header-only micro-ROS client wrapper
#
# All of include/rosette/ros/*.hpp (Context, Node, Publisher, Subscription,
# Timer, Service, Client, ActionServer/Client, transports, Agent locators).
# Consumers link this one target to pull the wrappers plus the generated
# micro-ROS library and its compile definitions.
# ------------------------------------------------------------
if(ROSETTE_ENABLE_MICRO_ROS)
    add_library(rosette_ros INTERFACE)
    target_include_directories(rosette_ros INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${ROSETTE_INTERNAL_INCLUDE_DIRS}
    )
    target_link_libraries(rosette_ros INTERFACE
        rosette_net
        micro_ros_vendor
    )
endif()

# ------------------------------------------------------------
# rosette_smoke — smallest end-to-end micro-ROS client
#
# Stock XRCE-DDS UDP transport, no BSP, builds on the host. Board transports
# and the reconnect state machine arrive in later refactor phases;
# examples/main_micro_ros_board_smoke.cpp is kept as the board reference.
# ------------------------------------------------------------
if(ROSETTE_ENABLE_MICRO_ROS)
    add_executable(rosette_smoke examples/rosette_smoke.cpp)
    target_link_libraries(rosette_smoke PRIVATE rosette_ros)

    if(ROSETTE_BOARD_ENABLE_PICO_SDK)
        pico_add_extra_outputs(rosette_smoke)
        pico_enable_stdio_usb(rosette_smoke 1)
        pico_enable_stdio_uart(rosette_smoke 0)
    endif()
endif()

# ------------------------------------------------------------
# storage_demo — exercises Volume / FileStore / DeviceConfig on the host
# ------------------------------------------------------------
if(NOT ROSETTE_BOARD_ENABLE_FREERTOS)
    add_executable(storage_demo examples/storage_demo.cpp)
    target_link_libraries(storage_demo PRIVATE rosette_storage)
endif()

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
