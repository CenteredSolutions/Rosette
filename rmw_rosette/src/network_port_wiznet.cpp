/**
 * @file network_port_wiznet.cpp
 * @brief The WIZnet network port: the chip's hardware UDP sockets and assigned address.
 *
 * See rmw_rosette/network_port.hpp. Forwards to the board network module
 * (rosette/net/wiznet/board_network.hpp), which owns the chip; the firmware
 * calls bring_up_board_network() before rcl_init().
 */

#include "rmw_rosette/network_port.hpp"

#include "rosette/net/wiznet/board_network.hpp"

namespace rmw_rosette {

/**
 * @brief The board's WIZnet socket provider.
 *
 * @return The provider; valid once bring_up_board_network() has returned.
 */
rosette::net::SocketProvider& network_socket_provider() noexcept {
    return rosette::net::wiznet::board_socket_provider();
}

/**
 * @brief The address DHCP or IPv4 link-local assigned.
 *
 * @return Address in host byte order, or 0 before bring-up.
 */
uint32_t network_ipv4_address() noexcept {
    return rosette::net::wiznet::board_ipv4_address();
}

}  // namespace rmw_rosette
