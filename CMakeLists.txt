cmake_minimum_required(VERSION 3.20)

# ============================================================
# Board selection
#
# Pass -DRTPS_BOARD=<name> to select a target board.
# Valid values:
#   w5100s-evb-pico    — WIZnet W5100S-EVB-Pico  (RP2040)
#   w5100s-evb-pico2   — WIZnet W5100S-EVB-Pico2 (RP2350, default)
#   custom-pico2       — Custom board based on RP2350 + W5100S
#
# This MUST be set before the project() call so that the Pico SDK
# can pick up PICO_BOARD correctly.
# ============================================================

set(RTPS_BOARD "w5100s-evb-pico2" CACHE STRING
    "Target board: w5100s-evb-pico | w5100s-evb-pico2 | custom-pico2")

set_property(CACHE RTPS_BOARD PROPERTY STRINGS
    w5100s-evb-pico
    w5100s-evb-pico2
    custom-pico2)

# Map RTPS_BOARD → Pico SDK PICO_BOARD string.
include(cmake/boards.cmake)
rtps_select_board(${RTPS_BOARD})   # Sets PICO_BOARD in cache before SDK init.

set(PICO_SDK_PATH $ENV{PICO_SDK_PATH} CACHE STRING "Path to the Pico SDK" FORCE)

# ── Pico SDK bootstrap ────────────────────────────────────────────────────────
# Set PICO_SDK_PATH via environment or -DPICO_SDK_PATH=...
set(PICO_SDK_FETCH_FROM_GIT OFF CACHE BOOL "" FORCE)
include(${PICO_SDK_PATH}/pico_sdk_init.cmake)

project(rtps_stack C CXX ASM)
set(CMAKE_C_STANDARD   11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

# ── FreeRTOS + FreeRTOS+TCP ───────────────────────────────────────────────────
include(cmake/freertos_tcp.cmake)

# ── WIZnet ioLibrary ─────────────────────────────────────────────────────────
# Set WIZNET_IOLIB_PATH via environment or -DWIZNET_IOLIB_PATH=...
include(cmake/wiznet.cmake)

# ── RTPS stack library ────────────────────────────────────────────────────────
add_library(rtps_stack STATIC
    rtps/message_parser.cpp
    rtps/stateful_reader.cpp
    rtps/spdp.cpp
    rtps/sedp.cpp
    rtps/participant.cpp
    rtps/buf_pool.cpp
    rtps/persistence.cpp
    rtps/transport.cpp
    net/socket.cpp
    ../littlefs/lfs.c
    ../littlefs/lfs_util.c
)

target_include_directories(rtps_stack PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/rtps
    ${CMAKE_CURRENT_SOURCE_DIR}/platform
    ${CMAKE_CURRENT_SOURCE_DIR}/boards/${RTPS_BOARD}
    ${CMAKE_CURRENT_SOURCE_DIR}/config
    ${CMAKE_CURRENT_SOURCE_DIR}/../littlefs
)

target_link_libraries(rtps_stack PUBLIC
    freertos_kernel
    freertos_tcp
    wiznet_iolib
    pico_stdlib
    hardware_spi
    hardware_flash
    hardware_sync
)

# ── Board-specific BSP ────────────────────────────────────────────────────────
add_library(rtps_bsp STATIC
    boards/${RTPS_BOARD}/board_init.c
    net/freertos/igmp.cpp
    net/freertos/socket.cpp
    net/wiznet/macraw.cpp
    platform/pico_freertos.c
    platform/pico_log.c
)

target_include_directories(rtps_bsp PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/boards/${RTPS_BOARD}
    ${CMAKE_CURRENT_SOURCE_DIR}/net
    ${CMAKE_CURRENT_SOURCE_DIR}/platform
    ${CMAKE_CURRENT_SOURCE_DIR}/rtps
)

target_link_libraries(rtps_bsp PUBLIC
    rtps_stack
    freertos_kernel
    freertos_tcp
    wiznet_iolib
    pico_stdlib
    hardware_spi
)

# ── Example application ───────────────────────────────────────────────────────
add_executable(rtps_example
    example/main_rp2350.cpp
)

target_link_libraries(rtps_example PRIVATE
    rtps_stack
    rtps_bsp
)

# Generate .uf2 for drag-and-drop flashing.
pico_add_extra_outputs(rtps_example)

# USB stdio for debug output (optional — disable for production).
pico_enable_stdio_usb(rtps_example 1)
pico_enable_stdio_uart(rtps_example 0)

# ── Print summary ─────────────────────────────────────────────────────────────
message(STATUS "")
message(STATUS "RTPS Stack build configuration")
message(STATUS "  Board       : ${RTPS_BOARD}")
message(STATUS "  PICO_BOARD  : ${PICO_BOARD}")
message(STATUS "  Processor   : ${RTPS_PROCESSOR}")
message(STATUS "  SPI pins    : SCK=${RTPS_SPI_SCK} TX=${RTPS_SPI_TX} RX=${RTPS_SPI_RX} CS=${RTPS_SPI_CS}")
message(STATUS "  W5100S RST  : ${RTPS_W5100S_RST}")
message(STATUS "  W5100S INT  : ${RTPS_W5100S_INT}")
message(STATUS "")
