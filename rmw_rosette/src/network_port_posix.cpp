/**
 * @file network_port_posix.cpp
 * @brief The host network port: BSD sockets and the host's primary IPv4 address.
 *
 * See rmw_rosette/network_port.hpp. A firmware image does not compile this
 * file; it defines the same two functions over its own network stack.
 */

#include "rmw_rosette/network_port.hpp"

#include "rosette/net/posix/socket.hpp"
#include "rosette/platform/host_runtime.hpp"

namespace rmw_rosette {

/**
 * @brief A process-wide POSIX socket backend, constructed on first use.
 *
 * @return The provider; it lives until the process exits.
 */
rosette::net::SocketProvider& network_socket_provider() noexcept {
    static rosette::net::posix::PosixSocketProvider provider;
    return provider;
}

/**
 * @brief The host's primary IPv4 address, overridable with `ROSETTE_LOCAL_IP`.
 *
 * @return Address in host byte order, or 0 when the host has none.
 */
uint32_t network_ipv4_address() noexcept {
    return rosette_platform_get_ipv4_addr();
}

}  // namespace rmw_rosette
