/**
 * @file rmw_rosette/network_port.hpp
 * @brief The build-time network port: which sockets and address rmw_init() uses.
 *
 * rmw_rosette never chooses a network stack. src/network_port.cpp defines
 * the two functions below over the board network
 * (rosette/net/board_network.hpp), and the image links exactly one board
 * network, the same way it links exactly one platform port
 * (rosette/platform/platform.hpp): BSD sockets and the host's primary IPv4
 * address on a host, the chip's hardware UDP sockets and its DHCP or
 * link-local address on a WIZnet board. Only the firmware knows when its
 * link and address are ready, so on a board rcl_init() runs after
 * bring_up_board_network().
 *
 * rmw_init() calls both once per context, before the participant is created,
 * and keeps the provider for the context's lifetime.
 */

#pragma once

#include "rosette/net/socket.hpp"

#include <cstdint>

namespace rmw_rosette {

/**
 * @brief The socket backend every rmw_rosette context opens its transport on.
 *
 * The RTPS transport opens three UDP sockets on it (SPDP multicast,
 * metatraffic unicast, user unicast; OMG DDSI-RTPS 2.5 §9.6.1.1), and the
 * participant driver registers its wake event with it so an interrupt-driven
 * backend ends rmw_wait()'s sleep when a datagram arrives.
 *
 * @return A provider that outlives every context; never a temporary.
 *
 * Verification:
 * - Correct means: rmw_init() opens its transport on the returned provider,
 *   and a stock RMW in the same domain discovers the participant.
 * - Covered by: tests/ros2/run_rmw_interop_loopback.sh (host);
 *   examples/rclc_chatter against stock Fast DDS (WIZnet board).
 *
 * References:
 * - OMG DDSI-RTPS 2.5 §9.6.1.1 (port mapping)
 */
rosette::net::SocketProvider& network_socket_provider() noexcept;

/**
 * @brief The IPv4 address the participant advertises in its locators.
 *
 * Peers send unicast metatraffic and user data to this address, so it must be
 * the interface address other participants can reach, never loopback: stock
 * DDS implementations discard loopback locators from participants they cannot
 * prove are on the same host.
 *
 * @return The address in host byte order, or 0 when no address is assigned
 *   (the participant then advertises no unicast locator and cannot be reached).
 *
 * Verification:
 * - Correct means: the SPDP announcement's default and metatraffic unicast
 *   locators carry this address.
 *
 * References:
 * - OMG DDSI-RTPS 2.5 §8.5.3.2 (SPDPdiscoveredParticipantData locators)
 */
uint32_t network_ipv4_address() noexcept;

}  // namespace rmw_rosette
