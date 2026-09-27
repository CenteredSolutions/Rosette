/**
 * @file rosette/net/wiznet/board_network.hpp
 * @brief The board's WIZnet chip, PHY link, and IP addresses, brought up once.
 *
 * A WIZnet board has exactly one Ethernet chip, so its network is one
 * board-level object: this module owns the chip driver (bus, device, hardware
 * UDP socket provider, link), IPv4 addressing (DHCP, falling back to IPv4
 * link-local), and the INT-pin routing, all configured from the board's
 * `board.h` / `board_config.h` (SPI pins, ROSETTE_PIN_WIZNET_INT, MAC). On
 * parts with hardware IPv6 (W6100, W6300) it also runs IPv6 stateless address
 * autoconfiguration (`slaac_provider.hpp`) beside IPv4, so the board holds a
 * link-local and, when a router advertises a prefix, a global IPv6 address.
 *
 * Every consumer of the network reaches it through these functions: an RTPS
 * participant opens its transport on board_socket_provider(), and rmw_rosette's
 * WIZnet network port (rmw_rosette/src/network_port_wiznet.cpp) forwards to
 * board_socket_provider() and board_ipv4_address(). The driver templates are
 * instantiated only in board_network.cpp, so including this header costs
 * nothing.
 *
 * Usage (single-threaded superloop):
 *
 *   board_init();
 *   rosette::net::wiznet::bring_up_board_network("my-board");  // blocks
 *   rcl_init(...);                                              // or an RTPS participant
 *   for (;;) {
 *       spin_some(...);
 *       rosette::net::wiznet::poll_board_network(platform::monotonic_ms());
 *   }
 *
 * Concurrency: all functions run in thread context on the core that runs the
 * network; none may be called from an interrupt handler.
 */

#pragma once

#include <array>
#include <cstdint>

#include "rosette/net/socket.hpp"

namespace rosette::net::wiznet {

/**
 * @brief Brings the chip, PHY link, and the board's addresses up; blocks until they are ready.
 *
 * Constructs the chip driver on first call, waits for the PHY to report link,
 * then runs DHCP (IPv4 link-local after DHCP gives up) until an IPv4 address
 * is assigned, printing a `[net]` progress line every two seconds. On parts
 * with hardware IPv6, SLAAC starts at the same time and bring-up also waits
 * for the IPv6 link-local address to pass duplicate address detection; the
 * global address follows in poll_board_network() when a router advertises a
 * prefix. Finally routes the chip's INT pin to the socket provider's receive
 * event (attach_rx_interrupt()), so a participant driver sleeping in
 * wait_readable() wakes when a datagram arrives. Later calls return at once.
 *
 * @param[in] hostname DHCP host name option (RFC 2132 §3.14); must outlive
 *   the program. Only the first call's value is used.
 *
 * Verification:
 * - Correct means: on return board_ipv4_address() is nonzero and equals the
 *   lease the DHCP server granted (or a 169.254/16 address without one); on
 *   the W6100 / W6300 board_ipv6_link_local() is the fe80:: address formed
 *   from the MAC; and a datagram to any open socket ends a kWaitForever
 *   wait_readable() within interrupt latency.
 * - Covered by: examples/rtps_chatter and examples/rclc_chatter on the
 *   W5100S-EVB-Pico2 (bidirectional traffic with stock Fast DDS);
 *   examples/wiznet_udp_echo with tools/wiznet_udp_echo_probe.py on the
 *   W6100-EVB-Pico (ping and UDP echo on every address).
 *
 * References:
 * - RFC 2131 (DHCP), RFC 3927 (IPv4 link-local), RFC 4862 (IPv6 SLAAC)
 * - W5100S / W6100 / W6300 datasheets, INT pin and Sn_IR
 */
void bring_up_board_network(const char* hostname) noexcept;

/**
 * @brief Services DHCP renewal, IPv6 autoconfiguration, and address conflicts.
 *
 * Call from the main loop between protocol passes; it never blocks.
 *
 * @param[in] now_ms Monotonic time in milliseconds (platform::monotonic_ms()).
 *
 * Verification:
 * - Correct means: a board running past half its lease renews it (RFC 2131
 *   §4.4.5, T1) without losing its address.
 */
void poll_board_network(uint32_t now_ms) noexcept;

/**
 * @brief The chip's hardware UDP socket provider.
 *
 * @return The provider. Only valid after bring_up_board_network() returns;
 *   it lives for the rest of the program.
 */
rosette::net::SocketProvider& board_socket_provider() noexcept;

/**
 * @brief The assigned IPv4 address.
 *
 * @return Address in host byte order, or 0 before bring_up_board_network()
 *   has assigned one.
 */
uint32_t board_ipv4_address() noexcept;

/**
 * @brief The IPv6 link-local address (W6100 / W6300).
 *
 * @return fe80:: address once duplicate address detection has passed; all
 *   zero before then, after a duplicate, and on parts without hardware IPv6.
 */
std::array<uint8_t, 16> board_ipv6_link_local() noexcept;

/**
 * @brief The SLAAC global IPv6 address (W6100 / W6300).
 *
 * @return The address formed from an advertised autonomous /64 prefix; all
 *   zero until a router advertises one, after its valid lifetime ends, and on
 *   parts without hardware IPv6.
 */
std::array<uint8_t, 16> board_ipv6_global() noexcept;

/**
 * @brief Sends the chip abandoned because ARP for the destination failed.
 *
 * @return Count since boot; 0 before bring-up.
 */
uint32_t board_send_timeouts() noexcept;

}  // namespace rosette::net::wiznet
