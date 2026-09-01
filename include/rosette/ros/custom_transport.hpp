#pragma once

/**
 * @file ros/custom_transport.hpp
 * @brief Lightweight transport contract for Rosette micro-ROS integration.
 *
 * Design goal
 * -----------
 * Keep transport bring-up, endpoint selection, and socket ownership reusable
 * even when higher-level projects do not compile the full micro-ROS toolchain.
 *
 * Why this exists
 * ---------------
 * `Context` uses this interface to hand C callback hooks to
 * `rmw_uros_options_set_custom_transport()`, but the transport contract itself
 * is independent of `rcl`, `rclc`, and executor setup. Breaking it out lets
 * board- and firmware-level code reuse Rosette's transport runtime objects as
 * ordinary C++ infrastructure.
 */

#include <stddef.h>
#include <stdint.h>

namespace rosette {

/**
 * @brief Transport hook that lets Rosette reuse board/network abstractions.
 *
 * micro-ROS custom transports are C callback based. This interface provides a
 * small C++ shim so board code can keep transport state and driver ownership
 * in one place while `Context` exposes the function pointers required by
 * `rmw_uros_options_set_custom_transport()`. A transport may also accept a
 * discovered Agent endpoint before the first XRCE session is attempted.
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
  virtual bool configure_agent_endpoint(const char* address,
                                        const char* port) noexcept {
    (void)address;
    (void)port;
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
