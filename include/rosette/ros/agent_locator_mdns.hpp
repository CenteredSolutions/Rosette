// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

#pragma once

/**
 * @file ros/agent_locator_mdns.hpp
 * @brief Agent discovery through multicast DNS and DNS-SD.
 *
 * Design goal
 * -----------
 * Resolve the micro-ROS Agent endpoint with the same mechanism the rest of a
 * local network already uses for zero-configuration discovery, so an Agent
 * running behind Docker or on a DHCP address can be found without hard-coding
 * an IP.
 *
 * How it fits in
 * --------------
 * This is a drop-in `AgentLocator`, a sibling of `StaticAgentLocator` and
 * `MulticastAgentLocator`. `AgentSessionGate` calls `resolve()` while no
 * session exists and hands the result to the XRCE transport.
 *
 * Two modes, chosen by configuration:
 * - "by advertisement": browse a DNS-SD service type and take the first
 *   instance (port from its SRV record).
 * - "by name": query the A record of a fixed host name.
 *
 * In both modes the address is the source address of the reply packet, which
 * is always routable from this device, rather than whatever locator the Agent
 * believes it has. See `net/mdns.hpp` for that policy.
 *
 * Advertising the Agent
 * ---------------------
 * On the Agent host, publish the service with Avahi:
 * @code
 *   avahi-publish -s "micro-ROS Agent" _microrosagent._udp 8888
 * @endcode
 * or resolve it by name using the host's own ".local" name.
 */

#include "agent_locator.hpp"

#include "rosette/net/mdns.hpp"
#include "rosette/net/socket.hpp"

#include <stddef.h>
#include <stdint.h>

#include <array>

namespace rosette {

struct MdnsAgentLocatorConfig {
    /// DNS-SD service type to browse in "by advertisement" mode.
    const char* service_type = "_microrosagent._udp.local";
    /// When set, resolve this host's A record instead of browsing a service.
    const char* host_name = nullptr;
    /// Port used in host-name mode, or when a service reply carries no SRV.
    uint16_t default_agent_port = 8888u;
    /// Per-attempt wait for mDNS replies.
    uint32_t receive_timeout_ms = 1000u;
    /// Number of query re-sends before the resolve fails.
    uint8_t attempts = 3u;
};

/**
 * @brief Resolve a micro-ROS Agent endpoint over mDNS / DNS-SD.
 */
class MdnsAgentLocator final : public AgentLocator {
public:
    MdnsAgentLocator(rosette::net::SocketProvider& socket_provider,
                     const MdnsAgentLocatorConfig& config = {}) noexcept
        : resolver_(socket_provider,
                    rosette::net::MdnsConfig{config.receive_timeout_ms,
                                             config.attempts})
        , config_(config)
    {}

    /**
     * @brief Discover the Agent endpoint.
     *
     * mDNS A-record lookups only ever resolve one IPv4 endpoint today, so this
     * writes at most one entry regardless of @p capacity. (AAAA/IPv6 lookups
     * are not yet implemented in `MdnsResolver`.)
     *
     * @param[out] out      Destination for the resolved endpoint.
     * @param[in]  capacity Number of entries @p out can hold.
     * @return 1 when an Agent endpoint was resolved, 0 otherwise.
     */
    size_t resolve(rosette::net::SocketAddress* out,
                   size_t capacity) noexcept override
    {
        if ((out == nullptr) || (capacity == 0u)) {
            return 0u;
        }

        std::array<uint8_t, 4> resolved_address{};
        uint16_t resolved_port = config_.default_agent_port;

        if ((config_.host_name != nullptr) && (config_.host_name[0] != '\0')) {
            if (!resolver_.resolve_host(config_.host_name, resolved_address)) {
                return 0u;
            }
        } else {
            uint16_t srv_port = 0u;
            if (!resolver_.resolve_service(config_.service_type,
                                           resolved_address, srv_port)) {
                return 0u;
            }
            if (srv_port != 0u) {
                resolved_port = srv_port;
            }
        }

        out[0] = rosette::net::make_socket_address_v4(resolved_address,
                                                       resolved_port);
        return 1u;
    }

private:
    rosette::net::MdnsResolver resolver_;
    MdnsAgentLocatorConfig config_{};
};

}  // namespace rosette
