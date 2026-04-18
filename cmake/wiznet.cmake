# cmake/wiznet.cmake
#
# Exposes the WIZnet ioLibrary as a CMake target: wiznet_iolib.
#
# Set WIZNET_IOLIB_PATH to the root of the ioLibrary_Driver repository,
# or let FetchContent pull it.
#
# Only the Ethernet/socket layer is compiled; the HTTP/MQTT/DHCP layers
# are excluded to keep code size minimal.

include(FetchContent)

if(DEFINED ENV{WIZNET_IOLIB_PATH})
    set(WIZNET_IOLIB_PATH "$ENV{WIZNET_IOLIB_PATH}")
endif()

if(DEFINED WIZNET_IOLIB_PATH AND EXISTS "${WIZNET_IOLIB_PATH}")
    message(STATUS "Using local WIZnet ioLibrary: ${WIZNET_IOLIB_PATH}")
    set(WIZNET_SRC_DIR "${WIZNET_IOLIB_PATH}")
else()
    FetchContent_Declare(wiznet_iolib_src
        GIT_REPOSITORY https://github.com/Wiznet/ioLibrary_Driver.git
        GIT_TAG        master
        GIT_SHALLOW    TRUE
    )
    FetchContent_MakeAvailable(wiznet_iolib_src)
    set(WIZNET_SRC_DIR "${wiznet_iolib_src_SOURCE_DIR}")
endif()

# Core Ethernet driver files only.
add_library(wiznet_iolib STATIC
    ${WIZNET_SRC_DIR}/Ethernet/socket.c
    ${WIZNET_SRC_DIR}/Ethernet/wizchip_conf.c
    # W5100S-specific register file — select chip via WIZCHIP define.
    ${WIZNET_SRC_DIR}/Ethernet/W5100S/w5100s.c
)

target_include_directories(wiznet_iolib PUBLIC
    ${WIZNET_SRC_DIR}/Ethernet
    ${WIZNET_SRC_DIR}/Ethernet/W5100S
    # Board config provides the _WIZCHIP_ define and SPI callbacks.
    ${CMAKE_CURRENT_SOURCE_DIR}/boards/${RTPS_BOARD}
)

# Tell the WIZnet library which chip is fitted.
target_compile_definitions(wiznet_iolib PUBLIC
    _WIZCHIP_=W5100S
)
