/**
 * @file examples/rclc_chatter/wiznet_network.hpp
 * @brief WIZnet link and address bring-up, and rmw_rosette's network port over it.
 *
 * rmw_rosette opens its RTPS sockets on whatever the image's network port
 * returns (rmw_rosette/network_port.hpp). This module is that port for a
 * WIZnet board: the chip's hardware UDP sockets and the address DHCP (or IPv4
 * link-local, when no DHCP server answers) assigned. rcl_init() must run
 * after bring_up_wiznet_network() returns.
 */

#pragma once

#include <cstdint>

namespace rclc_chatter {

/**
 * @brief Brings the WIZnet chip, PHY link, and an IPv4 address up.
 *
 * Blocks until the PHY reports link and an address is assigned, printing a
 * progress line every two seconds, then routes the chip's INT pin to the
 * socket provider's receive event so rmw_wait() sleeps until a datagram
 * arrives.
 *
 * @param[in] hostname DHCP host name option (RFC 2132 §3.14); must outlive
 *   the program.
 *
 * Verification:
 * - Correct means: on return, rmw_rosette::network_ipv4_address() is nonzero
 *   and matches the lease the DHCP server reports.
 * - Covered by: examples/rclc_chatter on the W5100S-EVB-Pico2.
 */
void bring_up_wiznet_network(const char* hostname) noexcept;

/**
 * @brief Services DHCP lease renewal; call from the main loop.
 *
 * @param[in] now_ms Monotonic time in milliseconds (platform::monotonic_ms()).
 */
void poll_wiznet_network(uint32_t now_ms) noexcept;

/**
 * @brief Sends the chip abandoned because ARP for the destination failed.
 *
 * @return Count since boot.
 */
uint32_t wiznet_send_timeouts() noexcept;

}  // namespace rclc_chatter
