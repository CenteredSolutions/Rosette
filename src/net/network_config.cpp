// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file src/net/network_config.cpp
 * @brief Config checking, provider ordering, and ready predicates; see
 * rosette/net/network_config.hpp.
 */

#include "rosette/net/network_config.hpp"

namespace rosette::net {
namespace {

/// @brief The structural and capability checks for one family's list.
ConfigError check_list(const MethodList& methods, const MethodList& supported,
                       AddressFamily family) noexcept {
    bool ended = false;
    for (std::size_t i = 0u; i < methods.size(); ++i) {
        const Method method = methods[i];
        if (method == Method::none) {
            ended = true;
            continue;
        }

        if (ended) {
            return ConfigError::gap_in_list;
        }

        for (std::size_t j = 0u; j < i; ++j) {
            if (methods[j] == method) {
                return ConfigError::duplicate_method;
            }
        }

        if (method_family(method) != family) {
            return ConfigError::wrong_family;
        }

        if (!lists_method(supported, method)) {
            return ConfigError::unsupported_method;
        }
    }

    return ConfigError::none;
}

}  // namespace

namespace ready {

bool any_v4(const AddressSnapshot& snapshot) noexcept {
    return snapshot.v4.has_address();
}

bool routable_v4(const AddressSnapshot& snapshot) noexcept {
    for (const AddressEntry& entry : snapshot.v4.addresses) {
        if (entry.routable()) {
            return true;
        }
    }

    return false;
}

bool any_address(const AddressSnapshot& snapshot) noexcept {
    return snapshot.v4.has_address() || snapshot.v6.has_address();
}

}  // namespace ready

AddressFamily method_family(Method method) noexcept {
    return method == Method::slaac ? AddressFamily::ipv6 : AddressFamily::ipv4;
}

bool lists_method(const MethodList& methods, Method method) noexcept {
    for (const Method listed : methods) {
        if (listed == method) {
            return true;
        }
    }

    return false;
}

ConfigError check_config(const NetworkConfig& config,
                         const NetworkCapabilities& capabilities) noexcept {
    if (config.v4[0] == Method::none) {
        return ConfigError::no_ipv4_method;
    }

    ConfigError error = check_list(config.v4, capabilities.v4, AddressFamily::ipv4);
    if (error == ConfigError::none) {
        error = check_list(config.v6, capabilities.v6, AddressFamily::ipv6);
    }

    if (error != ConfigError::none) {
        return error;
    }

    if (lists_method(config.v4, Method::static_ipv4)
        && config.static_v4.address == std::array<uint8_t, 4>{}) {
        return ConfigError::missing_static_address;
    }

    if (config.ready_when == nullptr) {
        return ConfigError::no_ready_predicate;
    }

    return ConfigError::none;
}

const char* describe(ConfigError error) noexcept {
    switch (error) {
        case ConfigError::none:
            return "ok";
        case ConfigError::no_ipv4_method:
            return "no IPv4 method listed";
        case ConfigError::gap_in_list:
            return "a method follows Method::none";
        case ConfigError::duplicate_method:
            return "a method is listed twice";
        case ConfigError::wrong_family:
            return "a method is listed under the wrong address family";
        case ConfigError::unsupported_method:
            return "this backend cannot run a listed method";
        case ConfigError::missing_static_address:
            return "static_ipv4 listed without static_v4.address";
        case ConfigError::no_ready_predicate:
            return "ready_when is null";
    }

    return "unknown";
}

std::size_t bind_methods(const MethodList& methods, const MethodBinding* bindings,
                         std::size_t count, AddressProvider** out) noexcept {
    std::size_t written = 0u;
    for (const Method method : methods) {
        for (std::size_t i = 0u; i < count && method != Method::none; ++i) {
            if (bindings[i].method == method) {
                out[written++] = bindings[i].provider;
                break;
            }
        }
    }

    return written;
}

}  // namespace rosette::net
