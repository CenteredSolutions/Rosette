# udp_echo — RFC 862 UDP echo over IPv4 and, where the board's network has
# it, IPv6 (link-local and SLAAC global). Proves the socket path and address
# bring-up without any ROS layer on top. It builds for any board with a
# network (a rosette_bsp): the WIZnet boards' hardware sockets, or
# FreeRTOS+TCP over the MIMXRT1020-EVK's ENET.
#
# W6100-EVB-Pico:
#
#   cmake -S . -B build-hw/w6100-pico -DROSETTE_BOARD=w6100-evb-pico \
#         -DROSETTE_MICRO_ROS_BOOTSTRAP_ON_BUILD=OFF
#   cmake --build build-hw/w6100-pico --target udp_echo
#   tools/pico_uf2_flash.py build-hw/w6100-pico/examples/udp_echo/udp_echo.uf2 --monitor 30
#   tools/udp_echo_probe.py            # from a host on the same link
#
# SparkFun MicroMod Teensy with the Ethernet Function Board: the console is
# MicroMod UART1 (TX1/RX1) at 115200, and the image loads through the Teensy
# bootloader (press the main board's BOOT button when the loader waits):
#
#   cmake -S . -B build-hw/teensy-mm -DROSETTE_BOARD=micromod-teensy-w5500 \
#         -DROSETTE_ENABLE_MICRO_ROS=OFF
#   cmake --build build-hw/teensy-mm --target udp_echo
#   teensy_loader_cli --mcu=TEENSY_MICROMOD -w -v \
#       build-hw/teensy-mm/examples/udp_echo/udp_echo.hex
#
# MIMXRT1020-EVK, flashed through its on-board DAPLink:
#
#   cmake -S . -B build-hw/rt1020-evk -DROSETTE_BOARD=mimxrt1020-evk \
#         -DROSETTE_ENABLE_MICRO_ROS=OFF
#   cmake --build build-hw/rt1020-evk --target udp_echo
#   pyocd flash -t mimxrt1020 build-hw/rt1020-evk/examples/udp_echo/udp_echo.hex
#
# -DUDP_ECHO_NTP_SERVER=<IPv4> names an NTP server the board asks before any
# DHCP offers; the status reply's `clock` line shows the result.
if(NOT TARGET rosette_bsp)
    return()
endif()

set(UDP_ECHO_NTP_SERVER "" CACHE STRING
    "IPv4 address of an NTP server udp_echo asks first (empty: only DHCP's)")

add_executable(udp_echo main.cpp)
target_compile_definitions(udp_echo PRIVATE
    UDP_ECHO_NTP_SERVER="${UDP_ECHO_NTP_SERVER}")
target_link_libraries(udp_echo PRIVATE rosette_bsp)
rosette_add_image_outputs(udp_echo)
