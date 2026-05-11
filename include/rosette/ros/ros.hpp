#pragma once

/**
 * @file ros/ros.hpp
 * @brief Umbrella header for the milkros ROS 2-compatible embedded layer.
 *
 * Include this single header to access the full milkros API:
 *
 *   #include "rosette/ros/ros.hpp"
 *
 * Included surfaces
 * -----------------
 *   milkros::Context             — owns participant, endpoint pools, timer registry
 *   milkros::Node                — factory for all endpoint types
 *   milkros::Publisher<T>        — typed topic publisher (analogous to rclcpp::Publisher)
 *   milkros::Subscription<T,CB>  — typed topic subscriber
 *   milkros::Timer<CB>           — periodic callback timer (analogous to rclcpp::Timer)
 *   milkros::Service<T,H>        — service server (analogous to rclcpp::Service)
 *   milkros::Client<T,N>         — service client (analogous to rclcpp::Client)
 *   milkros::ActionServer<T,G,E> — action server  (analogous to rclcpp_action::Server)
 *   milkros::ActionClient<T>     — action client  (analogous to rclcpp_action::Client)
 *   milkros::GoalHandle<T>       — passed to the execute callback by the ActionServer
 *
 * Trait templates (user specializes these for their message/service/action types)
 * ---------------------------------------------------------------------------------
 *   milkros::message_traits<T>   — topic serialization
 *   milkros::service_traits<T>   — service request/response serialization
 *   milkros::action_traits<T>    — action goal/result/feedback serialization
 *
 * Utilities (milkros/topic_names.hpp — also included transitively)
 * ----------------------------------------------------------------
 *   milkros::compose_topic_name()            — rt/<ns>/<topic>
 *   milkros::compose_service_request_topic() — rq/<ns>/<svc>Request
 *   milkros::compose_service_response_topic()— rr/<ns>/<svc>Reply
 *   milkros::compose_action_subtopic()       — rq|rr|rt/<ns>/<action>/<suffix>
 *
 * Configuration constants (override with -D before including this header)
 * -------------------------------------------------------------------------
 *   MILKROS_MAX_TIMER_SIZE    (default 64  B) — max sizeof(Timer<CallbackT>)
 *   MILKROS_MAX_TIMERS        (default 8)     — timer pool depth
 *   MILKROS_MAX_SERVICE_SIZE  (default 128 B) — max sizeof(Service<T,H>)
 *   MILKROS_MAX_SERVICES      (default 4)     — service server pool depth
 *   MILKROS_MAX_CLIENT_SIZE   (default 256 B) — max sizeof(Client<T,N>)
 *   MILKROS_MAX_CLIENTS       (default 4)     — service client pool depth
 *   MILKROS_MAX_ACTION_SIZE   (default 512 B) — max sizeof(ActionServer/Client<T,...>)
 *   MILKROS_MAX_ACTIONS       (default 2)     — action pool depth (servers + clients)
 *
 * Also relevant (from the RTPS layer):
 *   RTPS_MAX_USER_WRITERS  — increase to ≥ 8 when using action servers/clients
 *   RTPS_MAX_USER_READERS  — increase to ≥ 8 when using action servers/clients
 */

#include "context.hpp"
