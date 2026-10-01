# ports/net/wiznet — the WIZnet chip's hardware sockets.
#
# Included by cmake/rosette_net.cmake with ROSETTE_PORT_DIR set to this
# directory, after the `rosette_net` target exists. There is no adapter here:
# the IP stack runs in the chip, and the register-level driver
# (include/rosette/net/wiznet/) is portable, header-only, and templated on
# <Chip, Bus>. The board's composition instantiates it and implements
# rosette/net/board_network.hpp (boards/common/rp2_wiznet/ for the EVB-Pico
# boards). This port adds the DHCP client to rosette_net: it is portable, but
# only a socket-offload chip needs it, since a MAC-level stack brings its own.
target_sources(rosette_net PRIVATE
  ${ROSETTE_ROOT}/ports/net/wiznet/dhcp_client.cpp
  ${ROSETTE_ROOT}/ports/net/wiznet/dhcp_provider.cpp
  ${ROSETTE_ROOT}/ports/net/wiznet/dhcp_wire.cpp)

# Examples that use the WIZnet driver directly check this.
set(ROSETTE_BOARD_ENABLE_WIZNET ON)
