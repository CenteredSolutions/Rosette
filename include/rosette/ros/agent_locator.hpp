#pragma once

/**
 * @file ros/agent_locator.hpp
 * @brief Lightweight Agent endpoint discovery contract for Rosette transports.
 *
 * Design goal
 * -----------
 * Rosette's Agent discovery helpers should be reusable by network bring-up
 * code even when the full micro-ROS toolchain is not present. This header
 * therefore carries only the tiny endpoint-resolution interface needed by
 * discovery adapters and transport shims.
 *
 * Why this exists
 * ---------------
 * The multicast Micro XRCE-DDS discovery exchange is transport-level logic.
 * It does not require `rcl`, `rclc`, or `rmw_microros`, and forcing those
 * dependencies into every caller would make Rosette's discovery layer much
 * harder to reuse from firmware bootstrap code.
 */

#include <stddef.h>

namespace rosette {

/**
 * @brief Optional Rosette-side Agent discovery shim.
 *
 * Use this when the board/network stack can discover the Agent, but the built
 * in micro-ROS UDP multicast discovery path is unavailable or inappropriate
 * for the selected transport topology.
 */
class AgentLocator {
 public:
  virtual ~AgentLocator() = default;

  virtual bool resolve(char* address,
                       size_t address_capacity,
                       char* port,
                       size_t port_capacity) noexcept = 0;
};

}  // namespace rosette
