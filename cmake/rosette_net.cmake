# cmake/rosette_net.cmake
#
# Defines the `rosette_net` CMake target that boards link against for Rosette
# network, DHCP, and NTP services.
#
# Set ROSETTE_NET_STACK before including this file:
#   wiznet_iolib   — WIZnet ioLibrary (requires wiznet.cmake already included)
#   freertos_tcp   — FreeRTOS+TCP (requires freertos_tcp.cmake already included)
#
# The selected stack target is aliased as rosette_net so callers only need to
# link one name regardless of which stack was chosen.

set(ROSETTE_NET_STACK "wiznet_iolib" CACHE STRING
    "Rosette network stack: wiznet_iolib or freertos_tcp")
set_property(CACHE ROSETTE_NET_STACK PROPERTY STRINGS wiznet_iolib freertos_tcp)

# Optional: name of an existing CMake target that provides FreeRTOS headers.
# rosette_net links against it so ntp_sync.cpp and socket sources can find
# FreeRTOS.h at compile time.
set(ROSETTE_FREERTOS_TARGET "" CACHE STRING
    "CMake target that provides FreeRTOS kernel headers (e.g. app_freertos_kernel)")

set(_rosette_root "${CMAKE_CURRENT_LIST_DIR}/..")

if(ROSETTE_NET_STACK STREQUAL "wiznet_iolib")
    if(NOT TARGET wiznet_iolib)
        message(FATAL_ERROR
            "rosette_net.cmake: ROSETTE_NET_STACK=wiznet_iolib but wiznet_iolib "
            "target does not exist. Include wiznet.cmake before rosette_net.cmake.")
    endif()

    # The register-level WIZnet driver (WiznetSocketProvider / WiznetLink /
    # Dhcp*) is header-only and templated on <Chip, Bus>; the consuming firmware
    # instantiates its BoardLink for the fitted part. Nothing to compile here.
    add_library(rosette_net_wiznet STATIC
        "${_rosette_root}/src/net/socket.cpp"
        "${_rosette_root}/src/net/ntp_sync.cpp")

    target_include_directories(rosette_net_wiznet PUBLIC
        "${_rosette_root}/include"
        "${_rosette_root}/include/rosette/net")

    target_link_libraries(rosette_net_wiznet PUBLIC wiznet_iolib)

    if(NOT ROSETTE_FREERTOS_TARGET STREQUAL "")
        target_link_libraries(rosette_net_wiznet PUBLIC "${ROSETTE_FREERTOS_TARGET}")
    endif()

    # Additional pico-sdk targets for WIZnet SPI and GPIO drivers.
    if(TARGET hardware_spi)
        target_link_libraries(rosette_net_wiznet PUBLIC hardware_spi hardware_gpio)
    endif()

    add_library(rosette_net ALIAS rosette_net_wiznet)

elseif(ROSETTE_NET_STACK STREQUAL "freertos_tcp")
    if(NOT TARGET freertos_tcp)
        message(FATAL_ERROR
            "rosette_net.cmake: ROSETTE_NET_STACK=freertos_tcp but freertos_tcp "
            "target does not exist. Include freertos_tcp.cmake before rosette_net.cmake.")
    endif()

    add_library(rosette_net_freertos_tcp STATIC
        "${_rosette_root}/src/net/socket.cpp"
        "${_rosette_root}/src/net/ntp_sync.cpp"
        "${_rosette_root}/src/net/freertos/socket.cpp"
        "${_rosette_root}/src/net/freertos/interface.cpp"
        "${_rosette_root}/src/net/freertos/igmp.cpp")

    target_include_directories(rosette_net_freertos_tcp PUBLIC
        "${_rosette_root}/include"
        "${_rosette_root}/include/rosette/net")

    target_link_libraries(rosette_net_freertos_tcp PUBLIC freertos_tcp)

    if(NOT ROSETTE_FREERTOS_TARGET STREQUAL "")
        target_link_libraries(rosette_net_freertos_tcp PUBLIC "${ROSETTE_FREERTOS_TARGET}")
    endif()

    add_library(rosette_net ALIAS rosette_net_freertos_tcp)

else()
    message(FATAL_ERROR
        "rosette_net.cmake: Unknown ROSETTE_NET_STACK='${ROSETTE_NET_STACK}'. "
        "Valid values: wiznet_iolib, freertos_tcp.")
endif()
