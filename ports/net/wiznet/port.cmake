# ports/net/wiznet — the WIZnet chip's hardware sockets.
#
# Included by cmake/rosette_net.cmake with ROSETTE_PORT_DIR set to this
# directory, after the `rosette_net` target exists. There is no adapter here:
# the IP stack runs in the chip, and the register-level driver in this
# directory is portable, header-only, and templated on <Chip, Bus>. The
# board's composition instantiates it and implements
# rosette/net/board_network.hpp (boards/common/rp2_wiznet/ for the EVB-Pico
# boards). This port adds the DHCP client to rosette_net: it is portable, but
# only a socket-offload chip needs it, since a MAC-level stack brings its own.
#
# The same driver also serves a MAC-level stack: macraw_device.hpp presents the
# chip in MACRAW mode as a rosette::net::MacDevice, which ports/net/freertos_tcp
# consumes; that build does not include this file, and the board composition
# reaches the headers on its own.
#
# Code outside this directory names the driver's headers by their path from
# the repository root ("ports/net/wiznet/device.hpp"); names such as
# interface.hpp would otherwise collide with include/rosette/net/.
target_sources(rosette_net PRIVATE
  ${ROSETTE_ROOT}/ports/net/wiznet/arp_cache.cpp
  ${ROSETTE_ROOT}/ports/net/wiznet/dhcp_client.cpp
  ${ROSETTE_ROOT}/ports/net/wiznet/dhcp_provider.cpp
  ${ROSETTE_ROOT}/ports/net/wiznet/dhcp_wire.cpp)
target_include_directories(rosette_net PUBLIC ${ROSETTE_ROOT})

# Examples that use the WIZnet driver directly check this.
set(ROSETTE_BOARD_ENABLE_WIZNET ON)
