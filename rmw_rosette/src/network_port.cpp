/**
 * @file network_port.cpp
 * @brief The network port: the board network's sockets and IPv4 address and prefix.
 *
 * See rmw_rosette/network_port.hpp. Forwards to the board network
 * (rosette/net/board_network.hpp), which the image links for its own stack:
 * BSD sockets on a host, the chip's hardware sockets on a WIZnet board. On a
 * board the firmware calls bring_up_board_network() before rcl_init(); on a
 * host both functions are valid without it.
 */

#include "rmw_rosette/network_port.hpp"

#include "rosette/net/board_network.hpp"

namespace rmw_rosette {

/**
 * @brief The board network's socket provider.
 *
 * @return The provider; valid on a board once bring_up_board_network() has
 *   returned, and at any time on a host.
 */
rosette::net::SocketProvider& network_socket_provider() noexcept {
    return rosette::net::board_socket_provider();
}

/**
 * @brief The IPv4 address the board network holds.
 *
 * @return Address in host byte order, or 0 while it holds none.
 */
uint32_t network_ipv4_address() noexcept {
    return rosette::net::board_ipv4_address();
}

/**
 * @brief The prefix length of the board network's IPv4 address.
 *
 * @return The primary IPv4 entry's prefix length, or 0 while it holds none.
 */
uint8_t network_ipv4_prefix_len() noexcept {
    return rosette::net::board_address_snapshot().v4.primary().prefix_len;
}

}  // namespace rmw_rosette
