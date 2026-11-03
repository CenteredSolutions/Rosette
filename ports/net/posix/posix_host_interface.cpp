// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file ports/net/posix/posix_host_interface.cpp
 * @brief Resolves `ROSETTE_INTERFACE`, or the default route, to one IPv4 interface.
 *
 * See posix_host_interface.hpp.
 */

#include "posix_host_interface.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <arpa/inet.h>
#include <ifaddrs.h>
#include <net/if.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

namespace rosette::net::posix {
namespace {

constexpr uint32_t kLoopback = 0x7F000001u;  // 127.0.0.1

/// The host-order IPv4 address of one getifaddrs() entry; the entry must be AF_INET.
uint32_t entry_address(const ifaddrs& ifa) noexcept {
    return ntohl(reinterpret_cast<const sockaddr_in*>(ifa.ifa_addr)->sin_addr.s_addr);
}

/// Whether one getifaddrs() entry carries an IPv4 address.
bool is_ipv4(const ifaddrs& ifa) noexcept {
    return ifa.ifa_addr != nullptr && ifa.ifa_addr->sa_family == AF_INET;
}

/**
 * @brief Asks the kernel which local IPv4 address would route to the internet.
 *
 * Connects an unbound UDP socket toward a public address, which sends
 * nothing, and reads back the source address the routing table selected.
 *
 * @return The address in host byte order, or 0 when there is no such route.
 */
uint32_t default_route_ipv4() noexcept {
    const int fd = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (fd < 0) {
        return 0u;
    }

    sockaddr_in remote{};
    remote.sin_family = AF_INET;
    remote.sin_port = htons(53u);
    (void)::inet_pton(AF_INET, "8.8.8.8", &remote.sin_addr);
    (void)::connect(fd, reinterpret_cast<const sockaddr*>(&remote), sizeof(remote));

    sockaddr_in local{};
    socklen_t   local_len = static_cast<socklen_t>(sizeof(local));
    const int   rc = ::getsockname(fd, reinterpret_cast<sockaddr*>(&local), &local_len);
    (void)::close(fd);
    return rc == 0 ? ntohl(local.sin_addr.s_addr) : 0u;
}

/**
 * @brief The first IPv4 entry with interface name `name`, or holding `address` when `name` is null.
 *
 * @param[in] list    getifaddrs() list.
 * @param[in] name    Interface name to match, or null to match by address.
 * @param[in] address Host-order address to match when `name` is null.
 * @return The entry, or null when none matches.
 */
const ifaddrs* find_ipv4_entry(const ifaddrs* list, const char* name, uint32_t address) noexcept {
    for (const ifaddrs* ifa = list; ifa != nullptr; ifa = ifa->ifa_next) {
        if (!is_ipv4(*ifa)) {
            continue;
        }

        if (name != nullptr ? std::strcmp(ifa->ifa_name, name) == 0
                            : entry_address(*ifa) == address) {
            return ifa;
        }
    }

    return nullptr;
}

/// Whether any entry, of any address family, has interface name `name`.
bool interface_exists(const ifaddrs* list, const char* name) noexcept {
    for (const ifaddrs* ifa = list; ifa != nullptr; ifa = ifa->ifa_next) {
        if (std::strcmp(ifa->ifa_name, name) == 0) {
            return true;
        }
    }

    return false;
}

/// Whether an entry is an interface peers can be on: IPv4, up, running, and not loopback.
bool is_network_interface(const ifaddrs& ifa) noexcept {
    return is_ipv4(ifa) && (ifa.ifa_flags & (IFF_UP | IFF_RUNNING)) == (IFF_UP | IFF_RUNNING)
           && (ifa.ifa_flags & IFF_LOOPBACK) == 0u;
}

/**
 * @brief Appends "name address, ..." for the host's IPv4 interfaces to `text`.
 *
 * @param[in]     list          getifaddrs() list.
 * @param[in]     networks_only List only is_network_interface() entries.
 * @param[in,out] text          Terminated string to append to.
 * @param[in]     size          Capacity of `text`; output is truncated to fit.
 * @return How many interfaces were listed.
 */
std::size_t append_interfaces(const ifaddrs* list, bool networks_only, char* text,
                              std::size_t size) noexcept {
    std::size_t listed = 0u;
    for (const ifaddrs* ifa = list; ifa != nullptr; ifa = ifa->ifa_next) {
        if (!is_ipv4(*ifa) || (networks_only && !is_network_interface(*ifa))) {
            continue;
        }

        char address[INET_ADDRSTRLEN] = {};
        (void)::inet_ntop(AF_INET, &reinterpret_cast<const sockaddr_in*>(ifa->ifa_addr)->sin_addr,
                          address, sizeof(address));
        const std::size_t used = std::strlen(text);
        (void)std::snprintf(text + used, size - used, "%s%s %s", listed > 0u ? ", " : "",
                            ifa->ifa_name, address);
        ++listed;
    }

    return listed;
}

/// Fills `out` from one getifaddrs() entry.
void fill_from_entry(const ifaddrs& ifa, HostInterface& out) noexcept {
    out.address = entry_address(ifa);
    out.prefix_len = 0u;
    if (ifa.ifa_netmask != nullptr) {
        const auto* mask = reinterpret_cast<const sockaddr_in*>(ifa.ifa_netmask);
        out.prefix_len = static_cast<uint8_t>(__builtin_popcount(mask->sin_addr.s_addr));
    }

    (void)std::snprintf(out.name, sizeof(out.name), "%s", ifa.ifa_name);
}

/// Every IPv4 interface the host has, as "name address, ...", for a console message.
void list_interfaces(bool networks_only, char* text, std::size_t size,
                     std::size_t& listed) noexcept {
    listed = 0u;
    ifaddrs* list = nullptr;
    if (::getifaddrs(&list) == 0) {
        listed = append_interfaces(list, networks_only, text, size);
        ::freeifaddrs(list);
    }
}

/**
 * @brief Reports on stderr why a setting names no usable interface, and what the host has.
 *
 * @param[in] setting The setting that failed to resolve; null or empty when
 *   ROSETTE_INTERFACE is unset, which is reported as `(unset)`.
 * @param[in] error   Why it failed; not `none`.
 */
void report_failure(const char* setting, InterfaceError error) noexcept {
    char        have[256] = {};
    std::size_t listed = 0u;
    list_interfaces(false, have, sizeof(have), listed);
    const bool unset = setting == nullptr || setting[0] == '\0';
    std::fprintf(stderr, "rosette: ROSETTE_INTERFACE=%s: %s (this host has: %s)\n",
                 unset ? "(unset)" : setting, describe(error), have);
}

/**
 * @brief Reports the default choice on stderr when the host has other networks to choose from.
 *
 * @param[in] chosen The interface the default resolved to.
 */
void report_default_choice(const HostInterface& chosen) noexcept {
    char        choices[256] = {};
    std::size_t listed = 0u;
    list_interfaces(true, choices, sizeof(choices), listed);
    if (listed > 1u) {
        std::fprintf(stderr,
                     "rosette: using %s, the default route's interface; set ROSETTE_INTERFACE "
                     "to choose one of: %s\n",
                     chosen.name, choices);
    }
}

/// The interface host_interface() resolved, and why it failed when it did.
struct ResolvedInterface {
    HostInterface  chosen{};
    InterfaceError error = InterfaceError::none;
};

/// Resolves `ROSETTE_INTERFACE` once per process and reports the outcome on stderr.
const ResolvedInterface& resolved_interface() noexcept {
    static const ResolvedInterface resolved = [] {
        const char*       setting = std::getenv("ROSETTE_INTERFACE");
        ResolvedInterface out{};
        out.error = resolve_host_interface(setting, out.chosen);
        if (out.error != InterfaceError::none) {
            report_failure(setting, out.error);
            out.chosen = HostInterface{};
        } else if (setting == nullptr || setting[0] == '\0') {
            report_default_choice(out.chosen);
        }

        return out;
    }();
    return resolved;
}

}  // namespace

const char* describe(InterfaceError error) noexcept {
    switch (error) {
        case InterfaceError::none:
            return "ok";
        case InterfaceError::several_named:
            return "names several interfaces; set exactly one";
        case InterfaceError::cannot_list:
            return "cannot list the host's interfaces";
        case InterfaceError::no_such_interface:
            return "no interface has that name";
        case InterfaceError::no_ipv4_address:
            return "that interface has no IPv4 address";
        case InterfaceError::address_not_held:
            return "no interface holds that address";
        case InterfaceError::interface_down:
            return "that interface is down";
    }

    return "unknown";
}

InterfaceError resolve_host_interface(const char* setting, HostInterface& out) noexcept {
    const bool by_default = setting == nullptr || setting[0] == '\0';
    if (!by_default && std::strchr(setting, ',') != nullptr) {
        return InterfaceError::several_named;
    }

    ifaddrs* list = nullptr;
    if (::getifaddrs(&list) != 0) {
        return InterfaceError::cannot_list;
    }

    uint32_t    address = 0u;
    const char* name = nullptr;
    in_addr     parsed{};
    if (by_default) {
        address = default_route_ipv4();
        address = address != 0u ? address : kLoopback;
    } else if (::inet_pton(AF_INET, setting, &parsed) == 1) {
        address = ntohl(parsed.s_addr);
    } else {
        name = setting;
    }

    InterfaceError error = InterfaceError::none;
    const ifaddrs* entry = find_ipv4_entry(list, name, address);
    if (entry == nullptr && by_default) {
        // A detected address no interface reports still identifies the route.
        out = HostInterface{};
        out.address = address;
    } else if (entry == nullptr) {
        error = name == nullptr                ? InterfaceError::address_not_held
                : interface_exists(list, name) ? InterfaceError::no_ipv4_address
                                               : InterfaceError::no_such_interface;
    } else if ((entry->ifa_flags & IFF_UP) == 0u) {
        error = InterfaceError::interface_down;
    } else {
        fill_from_entry(*entry, out);
    }

    ::freeifaddrs(list);
    return error;
}

const HostInterface& host_interface() noexcept {
    return resolved_interface().chosen;
}

InterfaceError host_interface_error() noexcept {
    return resolved_interface().error;
}

}  // namespace rosette::net::posix
