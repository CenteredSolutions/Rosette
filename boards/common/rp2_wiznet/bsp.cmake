# boards/common/rp2_wiznet — the composition the WIZnet EVB-Pico boards share.
#
# A board selects it with ROSETTE_BOARD_COMMON=rp2_wiznet in its board.cmake;
# the root CMakeLists includes this file after creating `rosette_bsp` from the
# board's board_init.c. It combines the rp2 chip port's buses with the
# portable WIZnet driver:
#   wiznet_board_types.hpp        BoardChip / BoardBus / BoardDevice from board_config.h
#   wiznet_board_network.cpp      rosette/net/board_network.hpp over the chip's hardware
#                                 sockets (ROSETTE_NET_STACK=wiznet)
#   wiznet_macraw_interface.cpp   the chip in MACRAW mode as FreeRTOS+TCP's network
#                                 interface (ROSETTE_NET_STACK=freertos_tcp)
target_include_directories(rosette_bsp PUBLIC ${CMAKE_CURRENT_LIST_DIR})

if(ROSETTE_NET_STACK STREQUAL "wiznet")
  target_sources(rosette_bsp PRIVATE ${CMAKE_CURRENT_LIST_DIR}/wiznet_board_network.cpp)
elseif(ROSETTE_NET_STACK STREQUAL "freertos_tcp")
  target_sources(rosette_bsp PRIVATE ${CMAKE_CURRENT_LIST_DIR}/wiznet_macraw_interface.cpp)
else()
  message(FATAL_ERROR "boards/common/rp2_wiznet: no composition for ROSETTE_NET_STACK=${ROSETTE_NET_STACK}")
endif()

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
