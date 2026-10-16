// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file rosette/net/board_network.hpp
 * @brief The board's network (link, IP addresses, sockets), brought up once from a NetworkConfig.
 *
 * Every Rosette image has exactly one network, so it is one board-level
 * object reached through these functions. Applications, the RTPS participant,
 * rmw_rosette's network port (rmw_rosette/src/network_port.cpp), and the ROS
 * session supervisor all use it without knowing which stack is underneath.
 * The build links exactly one implementation:
 *
 *   - WIZnet boards (boards/common/wiznet/wiznet_board_network.cpp): the
 *     chip's hardware UDP sockets. The board network owns the chip driver
 *     (bus, device, socket provider, link), the address providers the config
 *     names, and the INT-pin routing, configured from the board's `board.h` /
 *     `board_config.h`.
 *   - Hosts (ports/net/posix/posix_board_network.cpp): BSD sockets. The
 *     operating system owns the interface and its addresses; the board
 *     network reports them.
 *
 * What the board does for addresses is the firmware's `NetworkConfig`
 * (`net/network_config.hpp`): an IPv4 method list drawn from DHCP, IPv4
 * link-local, and a static address, and IPv6 SLAAC where the backend has it.
 * board_network_capabilities() says which methods this backend can run, and
 * board_default_network() is the config used when the firmware names none.
 *
 * On WIZnet boards the board network also runs the SNTP client
 * (`net/sntp_client.hpp`): once the board holds an IPv4 address and knows an
 * NTP server (`NetworkConfig::time.server` or DHCP option 42), it sets
 * `platform::WallClock` and keeps it on UTC, so ROS stamps and rcl's system
 * clock read UTC. Agent-backend builds leave it off by default; the agent
 * sets the clock there. A host's clock is the operating system's.
 *
 * Usage (single-threaded superloop):
 *
 *   board_init();
 *   rosette::net::bring_up_board_network("my-board");  // or ("my-board", kNetwork)
 *   rcl_init(...);                                      // or an RTPS participant
 *   for (;;) {
 *       spin_some(...);
 *       rosette::net::poll_board_network(platform::monotonic_ms());
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

namespace rosette::net {

/**
 * @brief The addressing methods this board can run, per family.
 *
 * WIZnet boards run DHCP, IPv4 link-local, and a static address, plus SLAAC
 * on parts with hardware IPv6 (W6100, W6300). A host lists DHCP and IPv4
 * link-local, the methods its operating system runs; it cannot take a static
 * address from the firmware, and reports no IPv6.
 *
 * @return The board's capabilities, for check_config().
 */
NetworkCapabilities board_network_capabilities() noexcept;

/**
 * @brief The config bring_up_board_network() uses when the firmware names none.
 *
 * @return IPv4 DHCP then link-local, IPv6 SLAAC where the backend has it
 *   (W6100 / W6300), the default probe interval, ready once any IPv4
 *   address is held, no timeout.
 */
NetworkConfig board_default_network() noexcept;

/**
 * @brief Brings the link and addresses up; blocks until `ready_when` holds or time runs out.
 *
 * Checks the config against board_network_capabilities() first and starts
 * nothing if the board cannot run it. Otherwise starts the backend and polls
 * it until `config.ready_when` accepts the address snapshot, then prints each
 * address as a `[net]` line.
 *
 * On a WIZnet board, starting the backend constructs the chip driver and the
 * providers the config lists, ordered as listed, and bring-up prints a
 * progress line every two seconds (waiting for the chip, for the PHY link, or
 * for an address). The chip's INT pin is attached to the socket provider
 * (WiznetSocketProvider::attach_interrupt()) as soon as the chip answers, so
 * a participant driver sleeping in wait_readable() wakes when a datagram
 * arrives. On a host, bring-up waits for the operating system to hold an
 * IPv4 address.
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
 *   tests/host/posix_board_network_test.cpp (the host backend,
 *   TC-NET-BRD-001 to TC-NET-BRD-003);
 *   examples/rtps_chatter and examples/rclc_chatter on the W5100S-EVB-Pico2
 *   (bidirectional traffic with stock Fast DDS); examples/udp_echo
 *   with tools/udp_echo_probe.py on the W6100-EVB-Pico (ping and UDP
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
 * chip up. On a host it rereads the operating system's address.
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
 * @brief Whether the backend is up and the bring-up config's `ready_when` holds now.
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
 * @brief The socket provider every protocol opens its sockets on.
 *
 * On WIZnet boards it is the chip's hardware UDP socket provider; on a host,
 * BSD sockets.
 *
 * @return The provider; it lives for the rest of the program. On a WIZnet
 *   board it is valid only after bring_up_board_network() has returned
 *   `ready` or `timed_out`, and its sockets open once the chip answers. On a
 *   host it is valid at any time, bring-up or not.
 */
rosette::net::SocketProvider& board_socket_provider() noexcept;

/**
 * @brief The IPv4 address the board holds.
 *
 * @return Address in host byte order, or 0 while the board holds none. On a
 *   WIZnet board that includes the time before bring_up_board_network() has
 *   assigned one; a host reports the operating system's address at any time.
 */
uint32_t board_ipv4_address() noexcept;

/**
 * @brief Every address the board holds, per family, with each family's state.
 *
 * `v4.addresses` holds the IPv4 lease. On the W6100 / W6300, `v6.addresses`
 * lists the link-local address once it passes duplicate address detection
 * and, while a router advertises an autonomous prefix, the SLAAC global
 * address; `v6.primary()` is the one to announce (the global address when
 * present). On the W5100S and on a host `v6` is always empty; a host's `v4`
 * lists the operating system's address with source `manual`.
 *
 * @return The live snapshot, updated by poll_board_network(); an empty
 *   snapshot before bring_up_board_network() has run. The reference stays
 *   valid for the rest of the program; copy it to keep a stable view.
 *
 * Verification:
 * - Correct means: every listed address is one the chip holds in SIPR,
 *   LLAR, or GUAR.
 * - Covered by: examples/udp_echo with tools/udp_echo_probe.py
 *   on the W6100-EVB-Pico (ping and UDP echo on every listed address).
 */
const AddressSnapshot& board_address_snapshot() noexcept;

/**
 * @brief Sends the network abandoned because ARP for the destination failed.
 *
 * @return Count since boot; 0 before bring-up, and always 0 on a backend that
 *   does not report it (a host).
 */
uint32_t board_send_timeouts() noexcept;

}  // namespace rosette::net
