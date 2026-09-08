# cmake/wiznet.cmake
#
# Exposes the WIZnet ioLibrary as a CMake target: wiznet_iolib.
#
# Variables (set before including):
#   WIZNET_IOLIB_PATH     — path to ioLibrary_Driver repo root, or unset to
#                           let FetchContent pull it.
#   ROSETTE_WIZNET_CHIP   — chip variant: W5100S (default), W6100 or W6300. This
#                           drives the vendored ioLibrary only, and is only the
#                           legacy path — the register-level template driver
#                           (WiznetDevice / WiznetLink) needs no vendored code
#                           and takes its part from ROSETTE_WIZNET_CHIP in the
#                           board's board_config.h (see chip_select.hpp).
#
#                           Note: the pinned ioLibrary_Driver tag below carries
#                           W5100S only. Point WIZNET_IOLIB_PATH at a checkout
#                           that has W6100 / W6300 to use those here.
#
# Only the Ethernet/socket layer is compiled; the HTTP/MQTT/DHCP layers
# are excluded to keep code size minimal.

include(FetchContent)
set(FETCHCONTENT_QUIET OFF)

if(DEFINED ENV{WIZNET_IOLIB_PATH})
    set(WIZNET_IOLIB_PATH "$ENV{WIZNET_IOLIB_PATH}")
endif()

if(DEFINED WIZNET_IOLIB_PATH AND EXISTS "${WIZNET_IOLIB_PATH}")
    message(STATUS "Using local WIZnet ioLibrary: ${WIZNET_IOLIB_PATH}")
    set(WIZNET_SRC_DIR "${WIZNET_IOLIB_PATH}")
else()
    message(STATUS "Fetching WIZnet ioLibrary_Driver v3.2.0 — first run only")
    FetchContent_Declare(wiznet_iolib_src
        GIT_REPOSITORY https://github.com/Wiznet/ioLibrary_Driver.git
        GIT_TAG        v3.2.0
        GIT_SHALLOW    TRUE
        GIT_PROGRESS   TRUE
    )
    FetchContent_MakeAvailable(wiznet_iolib_src)
    set(WIZNET_SRC_DIR "${wiznet_iolib_src_SOURCE_DIR}")
endif()

if(NOT DEFINED ROSETTE_WIZNET_CHIP OR ROSETTE_WIZNET_CHIP STREQUAL "W5100S")
    set(_wiznet_chip "W5100S")
    set(_wiznet_chip_src "${WIZNET_SRC_DIR}/Ethernet/W5100S/w5100s.c")
    set(_wiznet_chip_inc "${WIZNET_SRC_DIR}/Ethernet/W5100S")
elseif(ROSETTE_WIZNET_CHIP STREQUAL "W6100")
    set(_wiznet_chip "W6100")
    set(_wiznet_chip_src "${WIZNET_SRC_DIR}/Ethernet/W6100/w6100.c")
    set(_wiznet_chip_inc "${WIZNET_SRC_DIR}/Ethernet/W6100")
elseif(ROSETTE_WIZNET_CHIP STREQUAL "W6300")
    set(_wiznet_chip "W6300")
    set(_wiznet_chip_src "${WIZNET_SRC_DIR}/Ethernet/W6300/w6300.c")
    set(_wiznet_chip_inc "${WIZNET_SRC_DIR}/Ethernet/W6300")
else()
    message(FATAL_ERROR
        "wiznet.cmake: Unknown ROSETTE_WIZNET_CHIP='${ROSETTE_WIZNET_CHIP}'. "
        "Valid values: W5100S, W6100, W6300.")
endif()

# Core Ethernet driver + the DHCP client (no HTTP / MQTT / DNS layers).
add_library(wiznet_iolib STATIC
    ${WIZNET_SRC_DIR}/Ethernet/socket.c
    ${WIZNET_SRC_DIR}/Ethernet/wizchip_conf.c
    ${_wiznet_chip_src}
    ${WIZNET_SRC_DIR}/Internet/DHCP/dhcp.c
)

target_include_directories(wiznet_iolib PUBLIC
    ${WIZNET_SRC_DIR}/Ethernet
    ${_wiznet_chip_inc}
    ${WIZNET_SRC_DIR}/Internet/DHCP
)

# Tell the WIZnet library which chip is fitted.
target_compile_definitions(wiznet_iolib PUBLIC
    _WIZCHIP_=${_wiznet_chip}
)

if(TARGET pico_stdlib)
    target_link_libraries(wiznet_iolib PUBLIC pico_stdlib)
endif()
