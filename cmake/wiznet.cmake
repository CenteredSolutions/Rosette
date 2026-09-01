# cmake/wiznet.cmake
#
# Exposes the WIZnet ioLibrary as a CMake target: wiznet_iolib.
#
# Variables (set before including):
#   WIZNET_IOLIB_PATH     — path to ioLibrary_Driver repo root, or unset to
#                           let FetchContent pull it.
#   ROSETTE_WIZNET_CHIP   — chip variant: W5100S (default) or W6300.
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
    message(STATUS "Fetching WIZnet ioLibrary_Driver — first run only")
    FetchContent_Declare(wiznet_iolib_src
        GIT_REPOSITORY https://github.com/Wiznet/ioLibrary_Driver.git
        GIT_TAG        master
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
elseif(ROSETTE_WIZNET_CHIP STREQUAL "W6300")
    set(_wiznet_chip "W6300")
    set(_wiznet_chip_src "${WIZNET_SRC_DIR}/Ethernet/W6300/w6300.c")
    set(_wiznet_chip_inc "${WIZNET_SRC_DIR}/Ethernet/W6300")
else()
    message(FATAL_ERROR
        "wiznet.cmake: Unknown ROSETTE_WIZNET_CHIP='${ROSETTE_WIZNET_CHIP}'. "
        "Valid values: W5100S, W6300.")
endif()

# Core Ethernet driver files only.
add_library(wiznet_iolib STATIC
    ${WIZNET_SRC_DIR}/Ethernet/socket.c
    ${WIZNET_SRC_DIR}/Ethernet/wizchip_conf.c
    ${_wiznet_chip_src}
)

target_include_directories(wiznet_iolib PUBLIC
    ${WIZNET_SRC_DIR}/Ethernet
    ${_wiznet_chip_inc}
)

# Tell the WIZnet library which chip is fitted.
target_compile_definitions(wiznet_iolib PUBLIC
    _WIZCHIP_=${_wiznet_chip}
)
