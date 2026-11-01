// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

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
 *
 * Endpoint type
 * -------------
 * `rosette::net::SocketAddress` already models exactly what an Agent endpoint
 * is: an address family, a 16-byte address (IPv4 or IPv6), and a 16-bit UDP
 * port. Using it here means a locator that hears from more than one transport
 * (multicast discovery, or a future mDNS AAAA lookup) has a real place to put
 * an IPv6 candidate instead of forcing everything through IPv4 text.
 */

#include "rosette/net/socket.hpp"

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

  /**
   * @brief Resolve up to @p capacity candidate Agent endpoints.
   *
   * Implementations write candidates to @p out in preference order (most
   * likely correct first) and return however many they wrote. A locator that
   * only ever finds one endpoint (a static configuration, a single mDNS
   * answer) writes exactly one entry; a locator that can see several
   * candidates in one reply (multicast XRCE discovery advertising multiple
   * transport locators) can write more, leaving the choice to the caller.
   *
   * @param[out] out      Destination array for resolved endpoints.
   * @param[in]  capacity Number of entries @p out can hold.
   * @return Number of endpoints written, from 0 (nothing resolved) up to
   *         @p capacity.
   */
  virtual size_t resolve(rosette::net::SocketAddress* out,
                        size_t capacity) noexcept = 0;
};

}  // namespace rosette
