# wiznet_udp_echo — RFC 862 UDP echo on a WIZnet board, over IPv4 and, on the
# W6100 / W6300, IPv6 (link-local and SLAAC global). Proves the chip driver's
# socket path and address bring-up without any ROS layer on top:
#
#   cmake -S . -B build-hw/w6100-pico -DROSETTE_BOARD=w6100-evb-pico \
#         -DROSETTE_MICRO_ROS_BOOTSTRAP_ON_BUILD=OFF
#   cmake --build build-hw/w6100-pico --target wiznet_udp_echo
#   tools/pico_uf2_flash.py build-hw/w6100-pico/examples/wiznet_udp_echo/wiznet_udp_echo.uf2 --monitor 30
#   tools/wiznet_udp_echo_probe.py            # from a host on the same link

if(NOT ROSETTE_BOARD_ENABLE_WIZNET OR NOT ROSETTE_BOARD_ENABLE_PICO_SDK)
    return()
endif()

add_executable(wiznet_udp_echo main.cpp)
target_link_libraries(wiznet_udp_echo PRIVATE rosette_bsp pico_stdlib)
pico_add_extra_outputs(wiznet_udp_echo)
pico_enable_stdio_usb(wiznet_udp_echo 1)
pico_enable_stdio_uart(wiznet_udp_echo 0)
