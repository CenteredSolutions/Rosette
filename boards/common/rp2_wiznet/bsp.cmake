# boards/common/rp2_wiznet — the composition the WIZnet EVB-Pico boards share.
#
# A board selects it with ROSETTE_BOARD_COMMON=rp2_wiznet in its board.cmake;
# the root CMakeLists includes this file after creating `rosette_bsp` from the
# board's board_init.c. It combines the rp2 chip port's buses with the
# portable WIZnet driver:
#   wiznet_board_types.hpp        BoardChip / BoardBus / BoardDevice from board_config.h
#   wiznet_board_network.cpp      rosette/net/board_network.hpp over the chip's hardware
#                                 sockets (ROSETTE_NET_STACK=wiznet)
#   wiznet_macraw_interface.cpp   ports/net/freertos_tcp/freertos_tcp_board_interface.hpp:
#                                 the chip in MACRAW mode (ports/net/wiznet/macraw_device.hpp)
#                                 as FreeRTOS+TCP's network interface
#                                 (ROSETTE_NET_STACK=freertos_tcp)
#
# Both compositions use the WIZnet driver, named from the repository root
# ("ports/net/wiznet/device.hpp"), whichever network port the build selected.
target_include_directories(rosette_bsp PUBLIC ${CMAKE_CURRENT_LIST_DIR} ${ROSETTE_ROOT})

if(ROSETTE_NET_STACK STREQUAL "wiznet")
  target_sources(rosette_bsp PRIVATE ${CMAKE_CURRENT_LIST_DIR}/wiznet_board_network.cpp)
elseif(ROSETTE_NET_STACK STREQUAL "freertos_tcp")
  target_sources(rosette_bsp PRIVATE ${CMAKE_CURRENT_LIST_DIR}/wiznet_macraw_interface.cpp)
else()
  message(FATAL_ERROR "boards/common/rp2_wiznet: no composition for ROSETTE_NET_STACK=${ROSETTE_NET_STACK}")
endif()

# Next hops the ARP cache holds (ports/net/wiznet/arp_cache.hpp): IPv4 sends
# then carry their MAC and the chip never ARPs inline. 0 leaves ARP to the chip.
set(ROSETTE_WIZNET_ARP_CACHE_ENTRIES "8" CACHE STRING
  "WIZnet ARP cache entries (0: the chip ARPs on each change of destination)")
target_compile_definitions(rosette_bsp PUBLIC
  "ROSETTE_WIZNET_ARP_CACHE_ENTRIES=${ROSETTE_WIZNET_ARP_CACHE_ENTRIES}")

# Receive-buffer KiB per hardware socket, one entry per socket; empty keeps the
# chip's default (see kBoardRxLayoutKib in wiznet_board_types.hpp, which checks
# it at compile time).
set(ROSETTE_WIZNET_RX_LAYOUT_KIB "" CACHE STRING
  "WIZnet receive-buffer KiB per socket, e.g. 8;2;1;1;1;1;1;1 (empty: chip default)")
if(ROSETTE_WIZNET_RX_LAYOUT_KIB)
  string(REPLACE ";" "," _rp2_wiznet_rx_layout "${ROSETTE_WIZNET_RX_LAYOUT_KIB}")
  target_compile_definitions(rosette_bsp PUBLIC
    "ROSETTE_WIZNET_RX_LAYOUT_KIB=${_rp2_wiznet_rx_layout}")
endif()
