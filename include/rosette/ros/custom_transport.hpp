#pragma once

/**
 * @file ros/custom_transport.hpp
 * @brief The transport contract the micro-ROS XRCE client speaks through.
 *
 * micro-ROS custom transports are four C callbacks (open, close, write,
 * read) plus an opaque argument. `rmw_agent_link()` (src/ros/agent_rmw_link.cpp)
 * turns any `CustomTransport` into that table and registers it with
 * `rmw_uros_set_custom_transport()`. The contract itself needs no micro-ROS
 * headers, so transports and the agent gate compile and test without the
 * micro-ROS toolchain.
 */

#include "rosette/net/socket.hpp"

#include <stddef.h>
#include <stdint.h>

namespace rosette {

/**
 * @brief Transport hook that lets Rosette reuse board/network abstractions.
 *
 * A transport keeps its socket and driver state in one object, and accepts
 * the agent endpoint that `AgentSessionGate` located before the XRCE client
 * opens it.
 */
class CustomTransport {
 public:
  virtual ~CustomTransport() = default;

  /**
   * @brief Update the remote Agent endpoint before the transport is opened.
   *
   * This is how a Rosette-side discovery shim passes a resolved Agent address
   * into a transport such as UDP-over-`SocketProvider`.
   *
   * @return `true` when the endpoint was accepted.
   * @return `false` when the transport does not support endpoint updates.
   */
  virtual bool configure_agent_endpoint(
      const rosette::net::SocketAddress& endpoint) noexcept {
    (void)endpoint;
    return false;
  }

  virtual bool open() noexcept = 0;
  virtual bool close() noexcept = 0;
  virtual size_t write(const uint8_t* data,
                       size_t size,
                       uint8_t* error) noexcept = 0;
  virtual size_t read(uint8_t* data,
                      size_t size,
                      int timeout_ms,
                      uint8_t* error) noexcept = 0;
};

}  // namespace rosette
