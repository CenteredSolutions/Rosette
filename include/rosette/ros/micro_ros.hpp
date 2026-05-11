#pragma once

/**
 * @file ros/micro_ros.hpp
 * @brief micro-ROS backend for the Rosette ROS facade.
 *
 * Include this header when the MCU should speak to a micro-ROS Agent through
 * `rclc` and `rmw_microxrcedds` instead of acting as a direct DDS-RTPS peer.
 *
 * Typical usage:
 *
 *   #include "rosette/ros/micro_ros.hpp"
 *
 *   milkros::micro_ros::Context context(...);
 *   context.start();
 *   auto node = context.create_node("motor_controller", "/drive");
 *   auto pub = node.create_publisher<MyMsg>("status");
 *
 * Included surfaces
 * -----------------
 *   milkros::micro_ros::Context                — XRCE client session + executor
 *   milkros::micro_ros::Node                   — factory for communication entities
 *   milkros::micro_ros::Publisher<T>           — typed topic publisher
 *   milkros::micro_ros::Subscription<T, CB>    — typed topic subscription
 *   milkros::micro_ros::Service<T, H>          — typed service server
 *   milkros::micro_ros::Client<T, N>           — typed async service client
 *   milkros::micro_ros::ActionServer<T, G, E>  — typed action server
 *   milkros::micro_ros::ActionClient<T, N>     — typed action client
 *
 * Caveats
 * -------
 * - Requires vendored micro-ROS headers and libraries from the
 *   `micro_ros_setup` generate-lib workflow.
 * - Action wrappers require upstream `rclc` action headers, available in ROS 2
 *   Humble and newer.
 */

#include "micro_ros/common.hpp"
#include "micro_ros/context.hpp"
#include "micro_ros/multicast_agent_locator.hpp"
#include "micro_ros/node.hpp"
