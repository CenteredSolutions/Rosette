/**
 * @file rosette/net/wiznet/board_network.hpp
 * @brief The board's WIZnet chip, PHY link, and IP addresses, brought up once from a NetworkConfig.
 *
 * A WIZnet board has exactly one Ethernet chip, so its network is one
 * board-level object: this module owns the chip driver (bus, device, hardware
 * UDP socket provider, link), the address providers the config names, and
 * the INT-pin routing, all configured from the board's `board.h` /
 * `board_config.h` (SPI pins, ROSETTE_PIN_WIZNET_INT, MAC).
 *
 * What the board does for addresses is the firmware's `NetworkConfig`
 * (`net/network_config.hpp`): an IPv4 method list drawn from DHCP, IPv4
 * link-local, and a static address, and, on parts with hardware IPv6 (W6100,
 * W6300), IPv6 SLAAC (`slaac_provider.hpp`). Without a config the board runs
 * board_default_network(): DHCP, falling back to link-local, plus SLAAC where
 * the part has IPv6.
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
 *   rosette::net::wiznet::bring_up_board_network("my-board");  // or ("my-board", kNetwork)
 *   rcl_init(...);                                              // or an RTPS participant
 *   for (;;) {
 *       spin_some(...);
 *       rosette::net::wiznet::poll_board_network(platform::monotonic_ms());
 *   }
 *
 * An rclc firmware that must survive the address moving hands
 * board_network_watch() to a `rosette::ros::SessionSupervisor`
 * (`ros/session_supervisor.hpp`), which polls the network and rebuilds the
 * session for it.
 *
 * Concurrency: all functions run in thread context on the core that runs the
 * network; none may be called from an interrupt handler.
 */

#pragma once

#include <array>
#include <cstdint>

#include "rosette/net/address_state.hpp"
#include "rosette/net/network_config.hpp"
#include "rosette/net/network_watch.hpp"
#include "rosette/net/socket.hpp"

namespace rosette::net::wiznet {

/**
 * @brief The addressing methods this board can run, per family: DHCP, link-local, static; SLAAC.
 *
 * @return The board's capabilities, for check_config().
 */
NetworkCapabilities board_network_capabilities() noexcept;

/**
 * @brief The config bring_up_board_network() uses when the firmware names none.
 *
 * @return IPv4 DHCP then link-local, IPv6 SLAAC on the W6100 / W6300, the
 *   default probe interval, ready once any IPv4 address is held, no timeout.
 */
NetworkConfig board_default_network() noexcept;

/**
 * @brief Brings the chip, link, and addresses up; blocks until `ready_when` holds or time runs out.
 *
 * Checks the config against board_network_capabilities() first and starts
 * nothing if the board cannot run it. Otherwise constructs the chip driver
 * and the providers the config lists, ordered as listed, and polls them
 * until `config.ready_when` accepts the address snapshot, printing a `[net]`
 * progress line every two seconds (waiting for the chip, for the PHY link, or
 * for an address). The chip's INT pin is routed to the socket provider's
 * receive event (attach_rx_interrupt()) as soon as the chip answers, so a
 * participant driver sleeping in wait_readable() wakes when a datagram
 * arrives.
 *
 * A timeout does not stop anything: poll_board_network() carries on from
 * where bring-up left off, including retrying a chip that did not answer, so
 * a supervisor can wait for the network in its own loop. Later calls start
 * nothing and report whether the first call's `ready_when` holds now.
 *
 * @param[in] hostname DHCP host name option (RFC 2132 §3.14); must outlive
 *   the program. Only the first successful call's value is used.
 * @param[in] config   Methods per family in preference order, probe interval,
 *   ready predicate, and timeout; copied.
 * @return `ready` when `ready_when` held before the timeout; `timed_out`
 *   when it did not (the network keeps coming up in poll_board_network());
 *   `unsupported_config` when check_config() refused the config (printed),
 *   in which case nothing was started and the call may be repeated with
 *   another config.
 *
 * Verification:
 * - Correct means: `ready` is returned only when `ready_when` accepts
 *   board_address_snapshot(); every address in the snapshot was obtained by
 *   a listed method, and an earlier method's address replaces a later one's
 *   when it binds; a config naming a method the part cannot run (SLAAC on the
 *   W5100S) returns `unsupported_config` without touching the chip; and a
 *   datagram to any open socket ends a kWaitForever wait_readable() within
 *   interrupt latency.
 * - Covered by: tests/host/network_config_test.cpp (config checking and
 *   provider order, TC-NET-CFG-001 to TC-NET-CFG-005);
 *   examples/rtps_chatter and examples/rclc_chatter on the W5100S-EVB-Pico2
 *   (bidirectional traffic with stock Fast DDS); examples/wiznet_udp_echo
 *   with tools/wiznet_udp_echo_probe.py on the W6100-EVB-Pico (ping and UDP
 *   echo on every address).
 *
 * References:
 * - RFC 2131 (DHCP), RFC 3927 (IPv4 link-local), RFC 4862 (IPv6 SLAAC)
 * - W5100S / W6100 / W6300 datasheets, INT pin and Sn_IR
 */
BringUpResult bring_up_board_network(
    const char* hostname, const NetworkConfig& config = board_default_network()) noexcept;

/**
 * @brief Services DHCP renewal, IPv6 autoconfiguration, address conflicts, and carrier.
 *
 * Call from the main loop between protocol passes; it never blocks. While a
 * later IPv4 method holds the address (link-local after DHCP gave up), it
 * retries the earlier ones every `upgrade_probe_ms` and moves to the first
 * that binds. After a bring-up that timed out it also finishes bringing the
 * chip up.
 *
 * @param[in] now_ms Monotonic time in milliseconds (platform::monotonic_ms()).
 *
 * Verification:
 * - Correct means: a board running past half its lease renews it (RFC 2131
 *   §4.4.5, T1) without losing its address; a board on link-local takes a
 *   DHCP lease within a minute of a server appearing, and keeps answering on
 *   link-local until then (RFC 3927 §1.9).
 * - Covered by: tests/host/address_upgrade_test.cpp (TC-NET-UPG-001 to TC-NET-UPG-006).
 */
void poll_board_network(uint32_t now_ms) noexcept;

/**
 * @brief Whether the chip answers and the bring-up config's `ready_when` holds now.
 *
 * @return False before bring_up_board_network() has accepted a config.
 */
bool board_network_ready() noexcept;

/**
 * @brief The board network as a ROS session supervisor watches it.
 *
 * Bundles poll_board_network(), board_network_ready(), and
 * board_address_snapshot() for `rosette::ros::SessionSupervisor`, which
 * then services the network in every session state. Call
 * bring_up_board_network() first; a firmware that lets the supervisor do all
 * the waiting passes a config with `ready_timeout_ms = 0`.
 *
 * @return The watch; its functions stay valid for the rest of the program.
 *
 * Verification:
 * - Correct means: the supervisor sees the same readiness and snapshot that
 *   bring_up_board_network() waited on.
 * - Covered by: examples/rclc_chatter on the W6100-EVB-Pico (session up once
 *   ready, rebuilt when the address moved).
 */
NetworkWatch board_network_watch() noexcept;

/**
 * @brief The chip's hardware UDP socket provider.
 *
 * @return The provider. Only valid after bring_up_board_network() has
 *   returned `ready` or `timed_out`; it lives for the rest of the program,
 *   and its sockets open once the chip answers.
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
 * @brief Every address the board holds, per family, with each family's state.
 *
 * `v4.addresses` holds the IPv4 lease. On the W6100 / W6300, `v6.addresses`
 * lists the link-local address once it passes duplicate address detection
 * and, while a router advertises an autonomous prefix, the SLAAC global
 * address; `v6.primary()` is the one to announce (the global address when
 * present). On the W5100S `v6` is always empty.
 *
 * @return The live snapshot, updated by poll_board_network(); an empty
 *   snapshot before bring_up_board_network() has run. The reference stays
 *   valid for the rest of the program; copy it to keep a stable view.
 *
 * Verification:
 * - Correct means: every listed address is one the chip holds in SIPR,
 *   LLAR, or GUAR.
 * - Covered by: examples/wiznet_udp_echo with tools/wiznet_udp_echo_probe.py
 *   on the W6100-EVB-Pico (ping and UDP echo on every listed address).
 */
const AddressSnapshot& board_address_snapshot() noexcept;

/**
 * @brief Sends the chip abandoned because ARP for the destination failed.
 *
 * @return Count since boot; 0 before bring-up.
 */
uint32_t board_send_timeouts() noexcept;

}  // namespace rosette::net::wiznet
