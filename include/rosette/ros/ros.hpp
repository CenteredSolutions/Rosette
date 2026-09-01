#pragma once

/**
 * @file ros/ros.hpp
 * @brief micro-ROS backend for the Rosette ROS facade.
 *
 * Include this header when the MCU should speak to a micro-ROS Agent through
 * `rclc` and `rmw_microxrcedds`.
 *
 * Typical usage:
 *
 *   #include "rosette/ros/ros.hpp"
 *
 *   rosette::Context context(...);
 *   context.start();
 *   auto node = context.create_node("motor_controller", "/drive");
 *   auto pub = node.create_publisher<MyMsg>("status");
 *
 * Included surfaces
 * -----------------
 *   rosette::Context                — XRCE client session + executor
 *   rosette::Node                   — factory for communication entities
 *   rosette::Publisher<T>           — typed topic publisher
 *   rosette::Subscription<T, CB>    — typed topic subscription
 *   rosette::Service<T, H>          — typed service server
 *   rosette::Client<T, N>           — typed async service client
 *   rosette::ActionServer<T, G, E>  — typed action server
 *   rosette::ActionClient<T, N>     — typed action client
 *
 * Caveats
 * -------
 * - Requires vendored micro-ROS headers and libraries from the
 *   `micro_ros_setup` generate-lib workflow.
 * - Action wrappers require upstream `rclc` action headers, available in ROS 2
 *   Humble and newer.
 */

#include "common.hpp"
#include "context.hpp"
#include "multicast_agent_locator.hpp"
#include "node.hpp"
