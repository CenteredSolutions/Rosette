/**
 * @file ports/net/posix/posix_host_interface.hpp
 * @brief The one network interface a host process is on: `ROSETTE_INTERFACE`, else the default
 * route's.
 *
 * A participant joins and sends SPDP multicast on one interface and
 * advertises that interface's address as its unicast locators, so a peer that
 * hears the announcement can answer it. A host often has several interfaces
 * (a bench Raspberry Pi: wlan0 with the default route, eth0 to the boards),
 * and the right one is the user's choice:
 *
 * - `ROSETTE_INTERFACE=eth0` names it,
 * - `ROSETTE_INTERFACE=10.42.0.1` names it by an address it holds,
 * - unset, it is the interface the routing table uses toward the internet,
 *   or loopback on a host with no default route.
 *
 * A setting that names no usable interface is an error, never a silent
 * fallback: the process would otherwise announce itself on a network the
 * user did not ask for. The setting names one interface; a list is refused.
 *
 * Embedded stacks make the same choice in their board network bring-up
 * (rosette/net/board_network.hpp); only the source of the choice differs.
 */

#pragma once

#include <cstddef>
#include <cstdint>

namespace rosette::net::posix {

/// Longest interface name, terminator included (IF_NAMESIZE on Linux and macOS).
inline constexpr std::size_t kInterfaceNameSize = 16u;

/// One IPv4 interface address as the operating system reports it.
struct HostInterface {
    uint32_t address = 0u;                   ///< Host byte order; 0 when none was chosen.
    uint8_t  prefix_len = 0u;                ///< Leading one bits of the interface's netmask.
    char     name[kInterfaceNameSize] = {};  ///< Interface name, such as "eth0".
};

/**
 * @brief Resolves a `ROSETTE_INTERFACE` setting to one IPv4 interface.
 *
 * The host board network calls this once per process, through
 * host_interface(); tests call it directly with chosen settings.
 *
 * An empty or null setting selects the interface holding the address the
 * kernel would use toward the internet (a connected but unsent UDP socket
 * reports it), else loopback. Otherwise a dotted-quad setting selects the
 * interface holding that address and any other text the interface of that
 * name, using its first IPv4 address. The interface must be up.
 *
 * @param[in]  setting  The setting's text; null or empty for the default.
 * @param[out] out      The interface, written only on success.
 * @param[out] why      On failure, a one-line reason naming the interfaces
 *   the host has; may be null when `why_size` is 0.
 * @param[in]  why_size Capacity of `why`, terminator included.
 * @return True with `out` filled; false when the setting names an interface
 *   that does not exist, has no IPv4 address, or is down, an address no
 *   interface holds, or a list of several interfaces.
 *
 * Verification:
 * - Correct means: "lo" and "127.0.0.1" both give lo at 127.0.0.1/8; an
 *   unknown name, an unheld address, and "a,b" fail with a reason; the
 *   default gives an address some interface holds, or loopback.
 * - Covered by: tests/host/posix_host_interface_test.cpp (TC-NET-IFC-001..004).
 *
 * References:
 * - POSIX getifaddrs(3), inet_pton(3); Linux netdevice(7) (IFF_UP)
 */
bool resolve_host_interface(const char* setting, HostInterface& out, char* why,
                            std::size_t why_size) noexcept;

/**
 * @brief The process's interface, resolved from `ROSETTE_INTERFACE` on first use.
 *
 * Resolution runs once; the result holds for the process's lifetime, as the
 * participant's locators do. A setting that cannot be resolved is reported
 * once on stderr, and the result is then the all-zero interface, which the
 * board network reports as no address, so rmw_init() fails. When the setting
 * is unset on a host with more than one up, non-loopback IPv4 interface, the
 * choice is reported once on stderr with the alternatives, since the default
 * route's interface is often not the one the boards are on.
 *
 * @return The interface; `address` is 0 when the setting could not be resolved.
 *
 * Verification:
 * - Correct means: equals resolve_host_interface(getenv("ROSETTE_INTERFACE"))
 *   on success and the zero interface otherwise, on every call.
 * - Covered by: tests/host/posix_board_network_test.cpp (TC-NET-BRD-002),
 *   tests/host/posix_board_network_unresolved_test.cpp (TC-NET-BRD-005).
 *
 * References:
 * - POSIX getenv(3)
 */
const HostInterface& host_interface() noexcept;

}  // namespace rosette::net::posix
