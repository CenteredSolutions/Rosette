#pragma once

/**
 * @file ros/micro_ros/common.hpp
 * @brief Shared micro-ROS adapter types for the Rosette ROS facade.
 *
 * Design goal
 * -----------
 * This layer preserves the shape of the existing `milkros` node/publisher/
 * subscription/service/timer API while swapping the transport and discovery
 * machinery to the community-supported micro-ROS stack (`rcl` + `rclc` +
 * `rmw_microxrcedds`).
 *
 * Why this exists
 * ---------------
 * Rosette already has a clean MCU-oriented application surface.  The existing
 * `ros/` backend speaks DDS-RTPS directly, which is powerful but asks the MCU
 * to implement discovery, endpoint matching, reliability, and graph exposure
 * locally.  micro-ROS deliberately moves that complexity to the Agent and uses
 * XRCE-DDS as the MCU-to-Agent protocol.  This adapter keeps the application
 * code close to the current `milkros` style while leaning on the supported
 * micro-ROS execution and discovery model.
 *
 * Architectural fit
 * -----------------
 * - `Context` owns `rcl_init_options_t`, `rclc_support_t`, and the executor.
 * - `Node` owns the `rcl_node_t` handle and creates communication entities.
 * - Publishers/subscriptions/services/clients/timers own their respective
 *   `rcl` handles directly and register themselves with the shared executor.
 * - Agent discovery is performed at context startup through
 *   `rmw_uros_discover_agent()` when stock UDP transport is selected, which
 *   matches the documented micro-ROS discovery flow. Custom transports can run
 *   on MCUs too, but they need either a static Agent address or a Rosette-side
 *   resolver that discovers the Agent out-of-band and feeds that address back
 *   into `rmw_uros_options_set_udp_address()`.
 *
 * References
 * ----------
 * - micro-ROS RMW API: rmw_uros_discover_agent(), rmw_uros_options_set_*()
 * - micro-ROS overview: https://micro.ros.org/docs/overview/features/
 * - micro-ROS discovery article: https://micro.ros.org/blog/2020/09/30/discovery/
 * - rclc executor API: https://docs.ros.org/en/rolling/p/rclc/generated/
 *
 * Example
 * -------
 *   #include "ros/micro_ros.hpp"
 *
 *   milkros::micro_ros::Context context;
 *   context.start();
 *   auto node = context.create_node("imu_bridge", "/robot");
 *   auto pub = node.create_publisher<MyMsg>("imu");
 *
 * Caveats
 * -------
 * - This layer is transport-agnostic at the API surface, but the selected
 *   `rmw_microxrcedds` transport still controls discovery behavior and
 *   connectivity constraints.
 * - Action support depends on upstream `rclc` action headers being present in
 *   the vendored micro-ROS toolchain. Humble and newer distros provide them.
 */

#if !__has_include(<rcl/rcl.h>) || \
    !__has_include(<rclc/rclc.h>) || \
    !__has_include(<rclc/executor.h>) || \
    !__has_include(<rmw_microros/rmw_microros.h>)
#error "Rosette micro-ROS support requires rcl, rclc, and rmw_microros headers in the include path."
#endif

#include "../message_traits.hpp"

#include <rcl/rcl.h>
#include <rcl/init_options.h>
#include <rcl/node.h>
#include <rcl/publisher.h>
#include <rcl/subscription.h>
#include <rcl/service.h>
#include <rcl/client.h>
#include <rcl/timer.h>
#include <rclc/client.h>
#include <rclc/executor.h>
#include <rclc/init.h>
#include <rclc/node.h>
#include <rclc/publisher.h>
#include <rclc/service.h>
#include <rclc/subscription.h>
#include <rclc/timer.h>
#if __has_include(<rclc/action_client.h>) && __has_include(<rclc/action_server.h>)
#define ROSETTE_MICRO_ROS_HAS_ACTION_API 1
#include <rclc/action_client.h>
#include <rclc/action_server.h>
#else
#define ROSETTE_MICRO_ROS_HAS_ACTION_API 0
#endif
#include <rmw/init.h>
#include <rmw/types.h>
#include <rmw/qos_profiles.h>
#include <rmw_microros/rmw_microros.h>

#include <stddef.h>
#include <stdint.h>

#include <type_traits>
#include <utility>

namespace milkros {

template<typename ServiceT>
struct service_traits;

template<typename ActionT>
struct action_traits;

}  // namespace milkros

namespace milkros::micro_ros {

enum class Reliability : uint8_t {
    BEST_EFFORT = 0u,
    RELIABLE = 1u,
};

enum class Durability : uint8_t {
    VOLATILE = 0u,
    TRANSIENT_LOCAL = 1u,
};

struct PublisherOptions {
    Reliability reliability = Reliability::BEST_EFFORT;
    Durability durability = Durability::VOLATILE;
    size_t depth = 1u;
    int session_timeout_ms = 0;
};

struct SubscriptionOptions {
    Reliability reliability = Reliability::BEST_EFFORT;
    Durability durability = Durability::VOLATILE;
    size_t depth = 1u;
};

struct ServiceOptions {
    Reliability reliability = Reliability::RELIABLE;
    Durability durability = Durability::VOLATILE;
    size_t depth = 1u;
    int session_timeout_ms = 0;
};

/**
 * @brief Transport hook that lets Rosette reuse board/network abstractions.
 *
 * micro-ROS custom transports are C callback based.  This interface provides a
 * small C++ shim so board code can keep transport state and driver ownership in
 * one place while `Context` exposes the function pointers required by
 * `rmw_uros_options_set_custom_transport()`.  A transport may also accept a
 * discovered Agent endpoint from a Rosette-side locator before the first XRCE
 * session ping is attempted.
 */
class CustomTransport {
public:
    virtual ~CustomTransport() = default;

    /**
     * @brief Update the remote Agent endpoint before the transport is opened.
     *
     * This is how a Rosette-side discovery shim passes a resolved Agent
     * address into a transport such as UDP-over-`SocketProvider`.
     *
     * @return `true` when the endpoint was accepted.
     * @return `false` when the transport does not support endpoint updates.
     */
    virtual bool configure_agent_endpoint(const char* address,
                                          const char* port) noexcept
    {
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

struct ContextOptions {
    size_t domain_id = 0u;
    uint32_t client_key = 0x01020304u;
    size_t executor_handles = 16u;
    uint64_t spin_timeout_ns = 1000000ull;  ///< 1 ms default poll window.

    bool auto_discover_agent = true;
    int discovery_timeout_ms = 1000;
    uint8_t discovery_attempts = 3u;
    bool synchronize_epoch = false;
    int epoch_sync_timeout_ms = 1000;

    const char* udp_agent_address = nullptr;
    const char* udp_agent_port = "8888";

    bool use_custom_transport = false;
    bool custom_transport_framing = true;
    CustomTransport* custom_transport = nullptr;

    AgentLocator* agent_locator = nullptr;
};

class IActionTickable {
public:
    virtual ~IActionTickable() = default;
    virtual void tick() noexcept = 0;
};

struct GoalId {
    uint8_t uuid[16] = {};

    bool operator==(const GoalId& other) const noexcept
    {
        for (size_t i = 0; i < sizeof(uuid); ++i) {
            if (uuid[i] != other.uuid[i]) {
                return false;
            }
        }
        return true;
    }

    bool operator!=(const GoalId& other) const noexcept
    {
        return !(*this == other);
    }

    bool is_zero() const noexcept
    {
        for (uint8_t byte : uuid) {
            if (byte != 0u) {
                return false;
            }
        }
        return true;
    }
};

enum class GoalStatus : int8_t {
    UNKNOWN = 0,
    ACCEPTED = 1,
    EXECUTING = 2,
    CANCELING = 3,
    SUCCEEDED = 4,
    CANCELED = 5,
    ABORTED = 6,
};

enum class GoalResponse : uint8_t {
    ACCEPT = 0,
    REJECT = 1,
};

#if ROSETTE_MICRO_ROS_HAS_ACTION_API
inline GoalId to_goal_id(const unique_identifier_msgs__msg__UUID& source) noexcept
{
    GoalId goal_id{};
    for (size_t i = 0; i < sizeof(goal_id.uuid); ++i) {
        goal_id.uuid[i] = source.uuid[i];
    }
    return goal_id;
}

inline void copy_goal_id(const GoalId& source,
                         unique_identifier_msgs__msg__UUID& destination) noexcept
{
    for (size_t i = 0; i < sizeof(source.uuid); ++i) {
        destination.uuid[i] = source.uuid[i];
    }
}

inline GoalStatus to_goal_status(rcl_action_goal_state_t status) noexcept
{
    switch (status) {
        case GOAL_STATE_ACCEPTED:
            return GoalStatus::ACCEPTED;
        case GOAL_STATE_EXECUTING:
            return GoalStatus::EXECUTING;
        case GOAL_STATE_CANCELING:
            return GoalStatus::CANCELING;
        case GOAL_STATE_SUCCEEDED:
            return GoalStatus::SUCCEEDED;
        case GOAL_STATE_CANCELED:
            return GoalStatus::CANCELED;
        case GOAL_STATE_ABORTED:
            return GoalStatus::ABORTED;
        case GOAL_STATE_UNKNOWN:
        default:
            return GoalStatus::UNKNOWN;
    }
}

inline rcl_action_goal_state_t to_rcl_goal_status(GoalStatus status) noexcept
{
    switch (status) {
        case GoalStatus::ACCEPTED:
            return GOAL_STATE_ACCEPTED;
        case GoalStatus::EXECUTING:
            return GOAL_STATE_EXECUTING;
        case GoalStatus::CANCELING:
            return GOAL_STATE_CANCELING;
        case GoalStatus::SUCCEEDED:
            return GOAL_STATE_SUCCEEDED;
        case GoalStatus::CANCELED:
            return GOAL_STATE_CANCELED;
        case GoalStatus::ABORTED:
            return GOAL_STATE_ABORTED;
        case GoalStatus::UNKNOWN:
        default:
            return GOAL_STATE_UNKNOWN;
    }
}
#endif  // ROSETTE_MICRO_ROS_HAS_ACTION_API

inline rmw_qos_reliability_policy_t to_rmw_reliability(
    Reliability reliability) noexcept
{
    return reliability == Reliability::RELIABLE
        ? RMW_QOS_POLICY_RELIABILITY_RELIABLE
        : RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
}

inline rmw_qos_durability_policy_t to_rmw_durability(
    Durability durability) noexcept
{
    return durability == Durability::TRANSIENT_LOCAL
        ? RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL
        : RMW_QOS_POLICY_DURABILITY_VOLATILE;
}

inline rmw_qos_profile_t make_qos_profile(
    Reliability reliability,
    Durability durability,
    size_t depth) noexcept
{
    rmw_qos_profile_t qos = rmw_qos_profile_default;
    qos.reliability = to_rmw_reliability(reliability);
    qos.durability = to_rmw_durability(durability);
    qos.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
    qos.depth = depth;
    return qos;
}

inline rmw_qos_profile_t make_qos_profile(
    const PublisherOptions& options) noexcept
{
    return make_qos_profile(options.reliability, options.durability, options.depth);
}

inline rmw_qos_profile_t make_qos_profile(
    const SubscriptionOptions& options) noexcept
{
    return make_qos_profile(options.reliability, options.durability, options.depth);
}

inline rmw_qos_profile_t make_qos_profile(
    const ServiceOptions& options) noexcept
{
    return make_qos_profile(options.reliability, options.durability, options.depth);
}

template<typename MessageT, typename = void>
struct has_micro_ros_message_traits : std::false_type {};

template<typename MessageT>
struct has_micro_ros_message_traits<
    MessageT,
    std::void_t<decltype(::milkros::message_traits<MessageT>::rosidl_type_support())>>
    : std::true_type {};

template<typename MessageT>
inline constexpr bool has_micro_ros_message_traits_v =
    has_micro_ros_message_traits<MessageT>::value;

template<typename ServiceT, typename = void>
struct has_micro_ros_service_traits : std::false_type {};

template<typename ServiceT>
struct has_micro_ros_service_traits<
    ServiceT,
    std::void_t<decltype(::milkros::service_traits<ServiceT>::rosidl_type_support())>>
    : std::true_type {};

template<typename ServiceT>
inline constexpr bool has_micro_ros_service_traits_v =
    has_micro_ros_service_traits<ServiceT>::value;

template<typename ActionT, typename = void>
struct has_micro_ros_action_traits : std::false_type {};

template<typename ActionT>
struct has_micro_ros_action_traits<
    ActionT,
    std::void_t<decltype(::milkros::action_traits<ActionT>::rosidl_action_type_support())>>
    : std::true_type {};

template<typename ActionT>
inline constexpr bool has_micro_ros_action_traits_v =
    has_micro_ros_action_traits<ActionT>::value;

template<typename MessageT, typename = void>
struct has_init_message : std::false_type {};

template<typename MessageT>
struct has_init_message<
    MessageT,
    std::void_t<decltype(::milkros::message_traits<MessageT>::init_message(
        std::declval<MessageT&>()))>> : std::true_type {};

template<typename MessageT, typename = void>
struct has_fini_message : std::false_type {};

template<typename MessageT>
struct has_fini_message<
    MessageT,
    std::void_t<decltype(::milkros::message_traits<MessageT>::fini_message(
        std::declval<MessageT&>()))>> : std::true_type {};

template<typename MessageT>
inline void init_message(MessageT& message) noexcept
{
    if constexpr (has_init_message<MessageT>::value) {
        ::milkros::message_traits<MessageT>::init_message(message);
    } else {
        message = MessageT{};
    }
}

template<typename MessageT>
inline void fini_message(MessageT& message) noexcept
{
    if constexpr (has_fini_message<MessageT>::value) {
        ::milkros::message_traits<MessageT>::fini_message(message);
    } else {
        (void)message;
    }
}

template<typename ServiceT, typename = void>
struct has_init_request : std::false_type {};

template<typename ServiceT>
struct has_init_request<
    ServiceT,
    std::void_t<decltype(::milkros::service_traits<ServiceT>::init_request(
        std::declval<typename ::milkros::service_traits<ServiceT>::Request&>()))>>
    : std::true_type {};

template<typename ServiceT, typename = void>
struct has_fini_request : std::false_type {};

template<typename ServiceT>
struct has_fini_request<
    ServiceT,
    std::void_t<decltype(::milkros::service_traits<ServiceT>::fini_request(
        std::declval<typename ::milkros::service_traits<ServiceT>::Request&>()))>>
    : std::true_type {};

template<typename ServiceT, typename = void>
struct has_init_response : std::false_type {};

template<typename ServiceT>
struct has_init_response<
    ServiceT,
    std::void_t<decltype(::milkros::service_traits<ServiceT>::init_response(
        std::declval<typename ::milkros::service_traits<ServiceT>::Response&>()))>>
    : std::true_type {};

template<typename ServiceT, typename = void>
struct has_fini_response : std::false_type {};

template<typename ServiceT>
struct has_fini_response<
    ServiceT,
    std::void_t<decltype(::milkros::service_traits<ServiceT>::fini_response(
        std::declval<typename ::milkros::service_traits<ServiceT>::Response&>()))>>
    : std::true_type {};

template<typename ServiceT>
inline void init_request(typename ::milkros::service_traits<ServiceT>::Request& request) noexcept
{
    if constexpr (has_init_request<ServiceT>::value) {
        ::milkros::service_traits<ServiceT>::init_request(request);
    } else {
        request = typename ::milkros::service_traits<ServiceT>::Request{};
    }
}

template<typename ServiceT>
inline void fini_request(typename ::milkros::service_traits<ServiceT>::Request& request) noexcept
{
    if constexpr (has_fini_request<ServiceT>::value) {
        ::milkros::service_traits<ServiceT>::fini_request(request);
    } else {
        (void)request;
    }
}

template<typename ServiceT>
inline void init_response(typename ::milkros::service_traits<ServiceT>::Response& response) noexcept
{
    if constexpr (has_init_response<ServiceT>::value) {
        ::milkros::service_traits<ServiceT>::init_response(response);
    } else {
        response = typename ::milkros::service_traits<ServiceT>::Response{};
    }
}

template<typename ServiceT>
inline void fini_response(typename ::milkros::service_traits<ServiceT>::Response& response) noexcept
{
    if constexpr (has_fini_response<ServiceT>::value) {
        ::milkros::service_traits<ServiceT>::fini_response(response);
    } else {
        (void)response;
    }
}

template<typename ActionT, typename = void>
struct has_init_send_goal_request : std::false_type {};

template<typename ActionT>
struct has_init_send_goal_request<
    ActionT,
    std::void_t<decltype(::milkros::action_traits<ActionT>::init_send_goal_request(
        std::declval<typename ::milkros::action_traits<ActionT>::SendGoalRequest&>()))>>
    : std::true_type {};

template<typename ActionT, typename = void>
struct has_fini_send_goal_request : std::false_type {};

template<typename ActionT>
struct has_fini_send_goal_request<
    ActionT,
    std::void_t<decltype(::milkros::action_traits<ActionT>::fini_send_goal_request(
        std::declval<typename ::milkros::action_traits<ActionT>::SendGoalRequest&>()))>>
    : std::true_type {};

template<typename ActionT, typename = void>
struct has_init_result_response : std::false_type {};

template<typename ActionT>
struct has_init_result_response<
    ActionT,
    std::void_t<decltype(::milkros::action_traits<ActionT>::init_result_response(
        std::declval<typename ::milkros::action_traits<ActionT>::GetResultResponse&>()))>>
    : std::true_type {};

template<typename ActionT, typename = void>
struct has_fini_result_response : std::false_type {};

template<typename ActionT>
struct has_fini_result_response<
    ActionT,
    std::void_t<decltype(::milkros::action_traits<ActionT>::fini_result_response(
        std::declval<typename ::milkros::action_traits<ActionT>::GetResultResponse&>()))>>
    : std::true_type {};

template<typename ActionT, typename = void>
struct has_init_feedback_message : std::false_type {};

template<typename ActionT>
struct has_init_feedback_message<
    ActionT,
    std::void_t<decltype(::milkros::action_traits<ActionT>::init_feedback_message(
        std::declval<typename ::milkros::action_traits<ActionT>::FeedbackMessage&>()))>>
    : std::true_type {};

template<typename ActionT, typename = void>
struct has_fini_feedback_message : std::false_type {};

template<typename ActionT>
struct has_fini_feedback_message<
    ActionT,
    std::void_t<decltype(::milkros::action_traits<ActionT>::fini_feedback_message(
        std::declval<typename ::milkros::action_traits<ActionT>::FeedbackMessage&>()))>>
    : std::true_type {};

template<typename ActionT>
inline void init_send_goal_request(
    typename ::milkros::action_traits<ActionT>::SendGoalRequest& request) noexcept
{
    if constexpr (has_init_send_goal_request<ActionT>::value) {
        ::milkros::action_traits<ActionT>::init_send_goal_request(request);
    } else {
        request = typename ::milkros::action_traits<ActionT>::SendGoalRequest{};
    }
}

template<typename ActionT>
inline void fini_send_goal_request(
    typename ::milkros::action_traits<ActionT>::SendGoalRequest& request) noexcept
{
    if constexpr (has_fini_send_goal_request<ActionT>::value) {
        ::milkros::action_traits<ActionT>::fini_send_goal_request(request);
    } else {
        (void)request;
    }
}

template<typename ActionT>
inline void init_result_response(
    typename ::milkros::action_traits<ActionT>::GetResultResponse& response) noexcept
{
    if constexpr (has_init_result_response<ActionT>::value) {
        ::milkros::action_traits<ActionT>::init_result_response(response);
    } else {
        response = typename ::milkros::action_traits<ActionT>::GetResultResponse{};
    }
}

template<typename ActionT>
inline void fini_result_response(
    typename ::milkros::action_traits<ActionT>::GetResultResponse& response) noexcept
{
    if constexpr (has_fini_result_response<ActionT>::value) {
        ::milkros::action_traits<ActionT>::fini_result_response(response);
    } else {
        (void)response;
    }
}

template<typename ActionT>
inline void init_feedback_message(
    typename ::milkros::action_traits<ActionT>::FeedbackMessage& feedback) noexcept
{
    if constexpr (has_init_feedback_message<ActionT>::value) {
        ::milkros::action_traits<ActionT>::init_feedback_message(feedback);
    } else {
        feedback = typename ::milkros::action_traits<ActionT>::FeedbackMessage{};
    }
}

template<typename ActionT>
inline void fini_feedback_message(
    typename ::milkros::action_traits<ActionT>::FeedbackMessage& feedback) noexcept
{
    if constexpr (has_fini_feedback_message<ActionT>::value) {
        ::milkros::action_traits<ActionT>::fini_feedback_message(feedback);
    } else {
        (void)feedback;
    }
}

}  // namespace milkros::micro_ros
