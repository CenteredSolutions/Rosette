// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

#pragma once

/**
 * @file net/network_config.hpp
 * @brief What the firmware asks of the network: addressing methods per family and a ready test.
 *
 * A `NetworkConfig` is a plain value the firmware passes to its backend's
 * bring-up function (`bring_up_board_network()`). It names, per
 * address family, the addressing methods to run in preference order; the
 * backend maps each method to an `AddressProvider` and runs the list under an
 * `AddressManager` (`address_manager.hpp`), so the first method in a list that
 * binds holds the address, and while a later one holds it the earlier ones
 * are retried every `upgrade_probe_ms`.
 *
 *     namespace net = rosette::net;
 *     constexpr net::NetworkConfig kNetwork{
 *         .v4 = {net::Method::dhcp, net::Method::ipv4_link_local},  // preference order
 *         .v6 = {net::Method::slaac},
 *         .upgrade_probe_ms = 300'000,
 *         .ready_when = net::ready::any_v4,
 *     };
 *     net::bring_up_board_network("my-board", kNetwork);
 *
 * (Designated initializers are C++20; GCC and Clang accept them in C++17 as
 * an extension, which is how the snippet above builds in this tree.)
 *
 * The value is data, not code, so a firmware can also build it at run time,
 * for instance from a method list stored in flash, and append whatever
 * fallbacks its product needs. Fallback policy is the firmware designer's:
 * Rosette runs exactly the list it is given.
 *
 * Bring-up blocks until `ready_when` accepts the address snapshot, or until
 * `ready_timeout_ms` passes. `ready_when` is a predicate rather than a fixed
 * set of levels, so a firmware that needs, say, a routable IPv4 address and
 * an IPv6 link-local one writes exactly that; `ready::` holds the common
 * cases.
 *
 * A backend states which methods it can run as a `NetworkCapabilities`, and
 * refuses a config that `check_config()` rejects against it, rather than
 * quietly skipping a method the firmware asked for.
 *
 * `time` is the network's clock service: once the board holds an IPv4
 * address, the backend keeps `platform::WallClock` on UTC with SNTP against
 * `time.server` (static configuration, or a stored `DeviceConfig`) and the
 * servers DHCP offered (net/sntp_client.hpp). With no server known it sends
 * nothing. Builds for the micro-ROS agent backend take their time from the
 * agent and default `time.enabled` to false.
 */

#include <array>
#include <cstddef>
#include <cstdint>

#include "address_provider.hpp"
#include "address_state.hpp"
#include "sntp_client.hpp"

namespace rosette::net {

/// One way of obtaining an address. `none` pads the unused tail of a `MethodList`.
enum class Method : uint8_t {
    none = 0u,
    dhcp,             ///< DHCPv4 (RFC 2131).
    ipv4_link_local,  ///< IPv4 link-local, 169.254/16 (RFC 3927).
    static_ipv4,      ///< The fixed address in `NetworkConfig::static_v4`.
    slaac,            ///< IPv6 stateless address autoconfiguration (RFC 4862).
};

/// Most methods one family's list can name.
inline constexpr std::size_t kMaxMethodsPerFamily = 3u;

/// Methods in preference order, padded with `Method::none`.
using MethodList = std::array<Method, kMaxMethodsPerFamily>;

/// A fixed IPv4 address, used when a v4 list names `Method::static_ipv4`.
struct StaticIpv4 {
    std::array<uint8_t, 4> address{};
    uint8_t prefix_len = 24u;
    std::array<uint8_t, 4> gateway{};  ///< All zero for no default gateway.
};

/// Decides, from the current addresses, whether the network is ready for the application.
using ReadyPredicate = bool (*)(const AddressSnapshot& snapshot);

namespace ready {

/// @brief True once the board holds any IPv4 address, link-local included.
bool any_v4(const AddressSnapshot& snapshot) noexcept;

/// @brief True once the board holds a routable IPv4 address (not 169.254/16).
bool routable_v4(const AddressSnapshot& snapshot) noexcept;

/// @brief True once the board holds an address of either family.
bool any_address(const AddressSnapshot& snapshot) noexcept;

}  // namespace ready

/// The network a firmware asks its backend for.
struct NetworkConfig {
    /// IPv4 methods in preference order. At least one is required.
    MethodList v4{Method::dhcp, Method::ipv4_link_local};
    /// IPv6 methods in preference order; empty for no IPv6.
    MethodList v6{};
    /// The address for `Method::static_ipv4`; ignored when v4 does not list it.
    StaticIpv4 static_v4{};
    /// While a later method holds the address, how often the earlier ones are
    /// retried in the background (0 = never; the holder keeps the address).
    uint32_t upgrade_probe_ms = 60000u;
    /// What bring-up waits for.
    ReadyPredicate ready_when = ready::any_v4;
    /// Longest bring-up wait, in milliseconds; UINT32_MAX (platform::kWaitForever) waits for ever.
    uint32_t ready_timeout_ms = UINT32_MAX;
    /// The SNTP client that keeps the wall clock on UTC.
    SntpConfig time{};
};

/// The methods one backend can run, per family.
struct NetworkCapabilities {
    MethodList v4{};
    MethodList v6{};
};

/// Why `check_config()` rejected a config.
enum class ConfigError : uint8_t {
    none = 0u,
    no_ipv4_method,          ///< v4 lists no method.
    gap_in_list,             ///< A method follows `Method::none` in a list.
    duplicate_method,        ///< A list names the same method twice.
    wrong_family,            ///< A list names a method of the other family.
    unsupported_method,      ///< The backend cannot run a listed method.
    missing_static_address,  ///< v4 lists `static_ipv4` but `static_v4.address` is all zero.
    no_ready_predicate,      ///< `ready_when` is null.
};

/// How `bring_up_*` ended.
enum class BringUpResult : uint8_t {
    ready = 0u,  ///< `ready_when` accepted the snapshot.
    timed_out,   ///< `ready_timeout_ms` passed first; the network keeps coming up in `poll`.
    unsupported_config,  ///< `check_config()` rejected the config; nothing was started.
};

/**
 * @brief Checks that a config is well formed and that a backend can run every method it lists.
 *
 * Bring-up calls this before touching the hardware, so a config the backend
 * cannot honor is refused as a whole instead of running with a method
 * silently missing.
 *
 * @param[in] config        The firmware's config.
 * @param[in] capabilities  The methods the backend can run, per family.
 * @return `ConfigError::none` when the backend can run the config as written;
 *   otherwise the first problem found, checking v4 before v6.
 *
 * Verification:
 * - Correct means: every rejected config has a non-`none` result naming a
 *   real defect, and every config whose lists are gap-free, duplicate-free,
 *   family-correct, supported, and complete (v4 non-empty, a static address
 *   when `static_ipv4` is listed, a predicate) is accepted.
 * - Covered by: tests/host/network_config_test.cpp (TC-NET-CFG-001, TC-NET-CFG-002).
 */
ConfigError check_config(const NetworkConfig& config,
                         const NetworkCapabilities& capabilities) noexcept;

/// @brief A short English description of a `ConfigError`, for the console.
const char* describe(ConfigError error) noexcept;

/// @brief The family a method addresses.
AddressFamily method_family(Method method) noexcept;

/// @brief Whether a list names a method.
bool lists_method(const MethodList& methods, Method method) noexcept;

/// A backend's provider for one method.
struct MethodBinding {
    Method method;
    AddressProvider* provider;
};

/**
 * @brief Orders a backend's providers the way a config's list names their methods.
 *
 * The backend owns one provider object per method it supports; this picks
 * the ones a list names, in the list's order, for an `AddressManager`.
 *
 * @param[in]  methods   One family's list from a config that passed `check_config()`.
 * @param[in]  bindings  The backend's providers, one per method; must outlive `out`'s use.
 * @param[in]  count     Entries in `bindings`.
 * @param[out] out       Receives up to kMaxMethodsPerFamily provider pointers.
 * @return The number of providers written; a method with no binding is skipped.
 *
 * Verification:
 * - Correct means: `out[i]` is the provider bound to the i-th listed method.
 * - Covered by: tests/host/network_config_test.cpp (TC-NET-CFG-003).
 */
std::size_t bind_methods(const MethodList& methods, const MethodBinding* bindings,
                         std::size_t count, AddressProvider** out) noexcept;

}  // namespace rosette::net
