#pragma once

/**
 * @file ros/micro_ros/socket_transport.hpp
 * @brief Socket-backed custom transport helpers for the micro-ROS backend.
 *
 * Design goal
 * -----------
 * Reuse Rosette's `SocketProvider` abstraction as the XRCE transport layer so
 * MCU and POSIX smoke tests can exercise the same network path that the rest
 * of the project already uses.
 *
 * Why this is separate from `Context`
 * ----------------------------------
 * `Context` owns the ROS/micro-ROS session lifecycle. Transport wiring is a
 * platform concern: some builds use built-in UDP discovery, while others speak
 * XRCE over a Rosette-managed UDP socket pair and therefore need an explicit
 * Agent address. Keeping the adapter here makes that split visible and avoids
 * hiding board policy inside the backend core.
 */

#include "common.hpp"

#include "rosette/net/inet.h"
#include "rosette/net/socket.hpp"

#include <stddef.h>
#include <stdint.h>

#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <memory>

namespace milkros::micro_ros {

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
        : address_(address)
        , port_(port)
    {}

    bool resolve(char* address,
                 size_t address_capacity,
                 char* port,
                 size_t port_capacity) noexcept override
    {
        if ((address_ == nullptr) || (port_ == nullptr) ||
            (address_capacity == 0u) || (port_capacity == 0u)) {
            return false;
        }

        const size_t address_length = std::strlen(address_);
        const size_t port_length = std::strlen(port_);
        if ((address_length + 1u) > address_capacity ||
            (port_length + 1u) > port_capacity) {
            return false;
        }

        std::memcpy(address, address_, address_length + 1u);
        std::memcpy(port, port_, port_length + 1u);
        return true;
    }

private:
    const char* address_ = nullptr;
    const char* port_ = nullptr;
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

    bool configure_agent_endpoint(const char* address,
                                  const char* port) noexcept override
    {
        if ((address == nullptr) || (port == nullptr)) {
            return false;
        }

        const int parsed_port = std::atoi(port);
        if ((parsed_port < 0) || (parsed_port > 65535)) {
            return false;
        }

        rosette::net::SocketAddress remote{};
        if (!make_ipv4_socket_address(address,
                                      static_cast<uint16_t>(parsed_port),
                                      remote)) {
            return false;
        }

        remote_ = remote;
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

}  // namespace milkros::micro_ros
