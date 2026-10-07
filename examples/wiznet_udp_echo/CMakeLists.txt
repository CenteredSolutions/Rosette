# SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
# SPDX-License-Identifier: Apache-2.0

# wiznet_udp_echo — RFC 862 UDP echo on a WIZnet board, over IPv4 and, on the
# W6100 / W6300, IPv6 (link-local and SLAAC global). Proves the chip driver's
# socket path and address bring-up without any ROS layer on top:
#
#   cmake -S . -B build-hw/w6100-pico -DROSETTE_BOARD=w6100-evb-pico \
#         -DROSETTE_MICRO_ROS_BOOTSTRAP_ON_BUILD=OFF
#   cmake --build build-hw/w6100-pico --target wiznet_udp_echo
#   tools/pico_uf2_flash.py build-hw/w6100-pico/examples/wiznet_udp_echo/wiznet_udp_echo.uf2 --monitor 30
#   tools/wiznet_udp_echo_probe.py            # from a host on the same link
#
# On the SparkFun MicroMod Teensy with the Ethernet Function Board, the console
# is MicroMod UART1 (TX1/RX1) at 115200, and the image loads through the
# Teensy bootloader (press the main board's BOOT button when the loader waits):
#
#   cmake -S . -B build-hw/teensy-mm -DROSETTE_BOARD=micromod-teensy-w5500 \
#         -DROSETTE_ENABLE_MICRO_ROS=OFF
#   cmake --build build-hw/teensy-mm --target wiznet_udp_echo
#   teensy_loader_cli --mcu=TEENSY_MICROMOD -w -v \
#       build-hw/teensy-mm/examples/wiznet_udp_echo/wiznet_udp_echo.hex
#
# -DWIZNET_UDP_ECHO_NTP_SERVER=<IPv4> names an NTP server the board asks
# before any DHCP offers; the status reply's `clock` line shows the result.

if(NOT ROSETTE_BOARD_ENABLE_WIZNET)
    return()
endif()

set(WIZNET_UDP_ECHO_NTP_SERVER "" CACHE STRING
    "IPv4 address of an NTP server wiznet_udp_echo asks first (empty: only DHCP's)")

add_executable(wiznet_udp_echo main.cpp)
target_compile_definitions(wiznet_udp_echo PRIVATE
    WIZNET_UDP_ECHO_NTP_SERVER="${WIZNET_UDP_ECHO_NTP_SERVER}")
target_link_libraries(wiznet_udp_echo PRIVATE rosette_bsp)
rosette_add_image_outputs(wiznet_udp_echo)
