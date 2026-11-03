// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

#pragma once

/**
 * @file ros/socket_transport.hpp
 * @brief The micro-ROS agent backend's XRCE transport over a Rosette UDP socket, and a fixed agent.
 *
 * The XRCE-DDS client reaches its agent through `UdpSocketTransport`, a
 * `CustomTransport` over Rosette's `SocketProvider`, on every platform: the
 * WIZnet chip's hardware sockets on a board, BSD sockets on a host. So the
 * agent path uses the same sockets, addresses, and network supervision as
 * the rest of Rosette, and no platform socket layer is needed inside
 * micro-ROS. `AgentSessionGate` hands the transport the agent's endpoint and
 * installs it in rmw_microxrcedds (see ros/agent_session_gate.hpp).
 *
 * `StaticAgentLocator` is the `AgentLocator` for an agent at a fixed,
 * configured address.
 */

#include "agent_locator.hpp"
#include "custom_transport.hpp"

#include "rosette/net/inet.h"
#include "rosette/net/socket.hpp"

#include <stddef.h>
#include <stdint.h>

#include <algorithm>
#include <cstdlib>
#include <memory>

// Receive buffering the XRCE socket asks for: a whole reliable-stream window
// from the agent, RMW_UXRCE_STREAM_HISTORY datagrams of the transport MTU sent
// back to back. The build passes it from ROSETTE_MICRO_ROS_STREAM_HISTORY and
// ROSETTE_MICRO_ROS_MAX_TRANSPORT_MTU (cmake/micro_ros.cmake); the fallback is
// their defaults, 8 x 1400.
#ifndef ROSETTE_XRCE_SOCKET_RECEIVE_BUFFER
#  define ROSETTE_XRCE_SOCKET_RECEIVE_BUFFER (8u * 1400u)
#endif

namespace rosette {

inline bool make_ipv4_socket_address(const char* text,
                                     uint16_t port,
                                     rosette::net::SocketAddress& out) noexcept
{
    return rosette::net::inet::make_socket_address_v4(text, port, out);
}

class StaticAgentLocator final : public AgentLocator {
public:
    StaticAgentLocator(const char* address,
                       const char* port) noexcept
    {
        valid_ = (address != nullptr) && (port != nullptr) &&
                 parse_endpoint(address, port, endpoint_);
    }

    size_t resolve(rosette::net::SocketAddress* out,
                   size_t capacity) noexcept override
    {
        if (!valid_ || (out == nullptr) || (capacity == 0u)) {
            return 0u;
        }
        out[0] = endpoint_;
        return 1u;
    }

private:
    /// Parses decimal port text; @p address is IPv4 dotted-decimal text.
    static bool parse_endpoint(const char* address,
                               const char* port,
                               rosette::net::SocketAddress& out) noexcept
    {
        char* parse_end = nullptr;
        const unsigned long parsed_port = std::strtoul(port, &parse_end, 10);
        if ((parse_end == port) || (parse_end == nullptr) ||
            (*parse_end != '\0') || (parsed_port > 65535u)) {
            return false;
        }
        return make_ipv4_socket_address(
            address, static_cast<uint16_t>(parsed_port), out);
    }

    rosette::net::SocketAddress endpoint_{};
    bool valid_ = false;
};

/**
 * @brief Packet-oriented XRCE transport over Rosette's UDP socket abstraction.
 *
 * This adapter speaks XRCE packets over an already supported UDP network path
 * rather than relying on platform-specific BSD socket availability inside the
 * upstream micro-ROS transport backends.
 */
class UdpSocketTransport final : public CustomTransport {
public:
    UdpSocketTransport(rosette::net::SocketProvider& socket_provider,
                       const rosette::net::SocketAddress& remote,
                       const rosette::net::SocketAddress& local,
                       uint32_t receive_timeout_ms = 100u) noexcept
        : socket_provider_(socket_provider)
        , remote_(remote)
        , local_(local)
        , receive_timeout_ms_(receive_timeout_ms)
    {}

    void set_remote(const rosette::net::SocketAddress& remote) noexcept
    {
        remote_ = remote;
    }

    const rosette::net::SocketAddress& remote() const noexcept
    {
        return remote_;
    }

    bool is_open() const noexcept
    {
        return socket_ && socket_->is_open();
    }

    bool configure_agent_endpoint(
        const rosette::net::SocketAddress& endpoint) noexcept override
    {
        remote_ = endpoint;
        return true;
    }

    bool open() noexcept override
    {
        if (socket_ && socket_->is_open()) {
            return true;
        }

        socket_ = socket_provider_.open(remote_.domain,
                                        rosette::net::SocketType::datagram,
                                        rosette::net::SocketProtocol::udp);
        if (!socket_) {
            return false;
        }

        // Before bind(), so a WIZnet backend places the socket on its largest
        // buffer; a backend that does not size per socket declines, harmlessly.
        (void)socket_->set_receive_buffer_size(ROSETTE_XRCE_SOCKET_RECEIVE_BUFFER);
        if (socket_->bind(local_) != 0) {
            socket_.reset();
            return false;
        }

        if (socket_->set_receive_timeout(receive_timeout_ms_) != 0) {
            socket_.reset();
            return false;
        }

        return true;
    }

    bool close() noexcept override
    {
        if (socket_) {
            socket_->close();
            socket_.reset();
        }
        return true;
    }

    size_t write(const uint8_t* data,
                 size_t size,
                 uint8_t* error) noexcept override
    {
        if (!socket_ || !socket_->is_open() || (data == nullptr)) {
            if (error != nullptr) {
                *error = 1u;
            }
            return 0u;
        }

        const std::ptrdiff_t rc = socket_->send_to(data, size, remote_);
        if (rc < 0) {
            if (error != nullptr) {
                *error = 1u;
            }
            return 0u;
        }

        if (error != nullptr) {
            *error = 0u;
        }
        return static_cast<size_t>(rc);
    }

    size_t read(uint8_t* data,
                size_t size,
                int timeout_ms,
                uint8_t* error) noexcept override
    {
        if (!socket_ || !socket_->is_open() || (data == nullptr)) {
            if (error != nullptr) {
                *error = 1u;
            }
            return 0u;
        }

        if (timeout_ms >= 0) {
            (void)socket_->set_receive_timeout(static_cast<uint32_t>(timeout_ms));
        }

        rosette::net::SocketAddress from{};
        const std::ptrdiff_t rc = socket_->recv_from(data, size, &from);
        if (rc < 0) {
            if (error != nullptr) {
                *error = 1u;
            }
            return 0u;
        }

        if ((from.domain != remote_.domain) ||
            (from.port != remote_.port) ||
            (from.address != remote_.address)) {
            if (error != nullptr) {
                *error = 1u;
            }
            return 0u;
        }

        if (error != nullptr) {
            *error = 0u;
        }
        return static_cast<size_t>(rc);
    }

private:
    rosette::net::SocketProvider& socket_provider_;
    rosette::net::SocketAddress remote_{};
    rosette::net::SocketAddress local_{};
    uint32_t receive_timeout_ms_ = 100u;
    std::unique_ptr<rosette::net::Socket> socket_{};
};

}  // namespace rosette
