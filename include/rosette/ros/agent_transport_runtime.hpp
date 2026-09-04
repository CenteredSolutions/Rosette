#pragma once

/**
 * @file ros/agent_transport_runtime.hpp
 * @brief Lightweight Agent transport runtime for higher-level firmware stacks.
 *
 * Design goal
 * -----------
 * Let board firmware own Rosette's socket transport and Agent endpoint
 * selection without forcing it to also compile the full micro-ROS context and
 * executor stack.
 *
 * Why this exists
 * ---------------
 * Higher-level projects often want a supervised "communications is ready"
 * service before they are prepared to construct all ROS entities. This runtime
 * provides that narrower transport-level milestone by pairing a Rosette UDP
 * transport with either a static endpoint or an `AgentLocator`.
 */

#include "agent_locator.hpp"
#include "socket_transport.hpp"

#include <stddef.h>
#include <stdint.h>

namespace rosette {

enum class AgentTransportError : uint8_t {
    kOk = 0u,
    kLocatorFailed,
    kEndpointRejected,
    kOpenFailed,
};

inline const char* agent_transport_error_string(
    const AgentTransportError error) noexcept
{
    switch (error) {
    case AgentTransportError::kOk:
        return "ok";
    case AgentTransportError::kLocatorFailed:
        return "agent locator did not resolve an endpoint";
    case AgentTransportError::kEndpointRejected:
        return "transport rejected the selected agent endpoint";
    case AgentTransportError::kOpenFailed:
        return "transport could not open the selected agent endpoint";
    }
    return "unknown agent transport error";
}

/**
 * @brief Transport-level runtime for connecting one UDP socket transport.
 *
 * This object is intentionally smaller than `Context`. It tracks only endpoint
 * selection and transport-open state so firmware supervisors can reason about
 * whether the network path to a candidate Agent is configured.
 */
class UdpAgentTransportRuntime {
 public:
  UdpAgentTransportRuntime(
      rosette::net::SocketProvider& socket_provider,
      const rosette::net::SocketAddress& local =
          rosette::net::make_socket_address_v4({0u, 0u, 0u, 0u}, 0u),
      uint32_t receive_timeout_ms = 100u) noexcept
      : transport_(socket_provider,
                   rosette::net::make_socket_address_v4({0u, 0u, 0u, 0u}, 0u),
                   local,
                   receive_timeout_ms) {}

  /**
   * @brief Resolve an Agent endpoint through a locator and open the transport.
   *
   * @param[in] locator Rosette-side Agent locator implementation.
   * @return `true` when the locator resolved a usable endpoint and the UDP
   *     transport opened successfully.
   */
  bool begin_connect(AgentLocator& locator) noexcept {
    rosette::net::SocketAddress remote{};
    if (locator.resolve(&remote, 1u) == 0u) {
      last_error_ = AgentTransportError::kLocatorFailed;
      connected_ = false;
      return false;
    }

    return begin_connect(remote);
  }

  /**
   * @brief Configure one concrete Agent endpoint and open the UDP transport.
   *
   * @param[in] remote Resolved Agent UDP endpoint.
   * @return `true` when the transport accepted the endpoint and opened.
   */
  bool begin_connect(const rosette::net::SocketAddress& remote) noexcept {
    disconnect();
    transport_.set_remote(remote);
    if (!transport_.open()) {
      last_error_ = AgentTransportError::kOpenFailed;
      connected_ = false;
      return false;
    }

    remote_ = remote;
    connected_ = true;
    last_error_ = AgentTransportError::kOk;
    return true;
  }

  void disconnect() noexcept {
    (void)transport_.close();
    connected_ = false;
  }

  bool connected() const noexcept {
    return connected_ && transport_.is_open();
  }

  AgentTransportError last_error() const noexcept { return last_error_; }

  const rosette::net::SocketAddress& remote() const noexcept { return remote_; }

  UdpSocketTransport& transport() noexcept { return transport_; }
  const UdpSocketTransport& transport() const noexcept { return transport_; }

 private:
  UdpSocketTransport transport_;
  rosette::net::SocketAddress remote_ =
      rosette::net::make_socket_address_v4({0u, 0u, 0u, 0u}, 0u);
  bool connected_ = false;
  AgentTransportError last_error_ = AgentTransportError::kOk;
};

}  // namespace rosette
