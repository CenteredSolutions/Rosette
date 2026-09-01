#pragma once

/**
 * @file ros/common.hpp
 * @brief Shared micro-ROS adapter types for the Rosette ROS facade.
 *
 * Design goal
 * -----------
 * This layer gives Rosette a compact, MCU-oriented node/publisher/subscription/
 * service/timer/action API on top of the community-supported micro-ROS stack
 * (`rcl` + `rclc` + `rmw_microxrcedds`).
 *
 * Why this exists
 * ---------------
 * A microcontroller cannot practically implement DDS discovery, endpoint
 * matching, reliability, and graph exposure locally. micro-ROS deliberately
 * moves that complexity to the Agent and uses XRCE-DDS as the MCU-to-Agent
 * protocol. This adapter keeps application code close to the shape of `rclcpp`
 * while leaning on the supported micro-ROS execution and discovery model.
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
 *   #include "rosette/ros/ros.hpp"
 *
 *   rosette::Context context;
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

#include "message_traits.hpp"
#include "agent_locator.hpp"
#include "custom_transport.hpp"

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

#ifndef ROSETTE_MICRO_ROS_TRANSPORT_CUSTOM
#define ROSETTE_MICRO_ROS_TRANSPORT_CUSTOM 0
#endif

#if ROSETTE_MICRO_ROS_TRANSPORT_CUSTOM
#if !__has_include(<rmw_microros/custom_transport.h>)
#error "Rosette micro-ROS custom transport requires rmw_microros/custom_transport.h."
#endif
#include <rmw_microros/custom_transport.h>
#endif

#include <stddef.h>
#include <stdint.h>

#include <type_traits>
#include <utility>

namespace rosette {

template<typename ServiceT>
struct service_traits;

template<typename ActionT>
struct action_traits;

}  // namespace rosette

namespace rosette {

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
 * @brief Represents either a successful value or a typed error code.
 *
 * Rosette's MCU-facing APIs avoid exceptions and dynamic allocation. This
 * result type gives callers a compact, explicit way to branch on success while
 * preserving the reason for failure in a subsystem-specific enum.
 *
 * @param[in] T Type returned when the operation succeeds.
 * @param[in] ErrorCode Enum type returned when the operation fails. The enum
 * must provide `kOk` for successful construction from an error code.
 */
template<typename T, typename ErrorCode>
class Result {
public:
    Result(const T& value) noexcept
        : ok_(true)
        , error_(ErrorCode::kOk)
        , value_(value)
    {}

    Result(T&& value) noexcept
        : ok_(true)
        , error_(ErrorCode::kOk)
        , value_(static_cast<T&&>(value))
    {}

    Result(ErrorCode error) noexcept
        : ok_(error == ErrorCode::kOk)
        , error_(error)
        , value_()
    {}

    bool ok() const noexcept { return ok_; }
    explicit operator bool() const noexcept { return ok_; }
    const T& value() const noexcept { return value_; }
    T& value() noexcept { return value_; }
    ErrorCode error() const noexcept { return error_; }

private:
    bool ok_ = false;
    ErrorCode error_ = ErrorCode::kOk;
    T value_{};
};

/**
 * @brief Represents success without a value or a typed error code.
 *
 * Use this specialization for operations whose successful result is simply
 * completion, such as starting a context. The error enum still carries the
 * subsystem-specific failure reason.
 *
 * @param[in] ErrorCode Enum type returned when the operation fails. The enum
 * must provide `kOk` for successful construction.
 */
template<typename ErrorCode>
class Result<void, ErrorCode> {
public:
    Result() noexcept = default;

    Result(ErrorCode error) noexcept
        : ok_(error == ErrorCode::kOk)
        , error_(error)
    {}

    bool ok() const noexcept { return ok_; }
    explicit operator bool() const noexcept { return ok_; }
    ErrorCode error() const noexcept { return error_; }

private:
    bool ok_ = true;
    ErrorCode error_ = ErrorCode::kOk;
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
    uint16_t udp_agent_port = 8888u;

    bool use_custom_transport = false;
    bool custom_transport_framing = true;
    CustomTransport* custom_transport = nullptr;

    AgentLocator* agent_locator = nullptr;
};

enum class ContextStartError : uint8_t {
    kOk = 0u,
    kInitOptionsInitFailed,
    kDomainIdSetFailed,
    kRmwOptionsUnavailable,
    kClientKeySetFailed,
    kCustomTransportUnavailable,
    kCustomTransportMissing,
    kCustomTransportEndpointRejected,
    kUdpAddressSetFailed,
    kStaticAgentPingFailed,
    kAgentLocatorFailed,
    kAgentLocatorNoAddress,
    kAgentLocatorEndpointRejected,
    kDiscoveredUdpAddressSetFailed,
    kCustomTransportRequiresEndpoint,
    kAutoDiscoveryDisabled,
    kAutoDiscoveryFailed,
    kSupportInitFailed,
    kExecutorInitFailed,
};

/**
 * @brief Converts a context startup error into stable diagnostic text.
 *
 * Startup is intentionally split into transport setup, Agent discovery, and
 * RCL/RCLC initialization phases. This helper gives applications and examples
 * a compact way to report which phase failed without exposing upstream return
 * codes directly in user-facing logs.
 *
 * @param[in] error Context startup error to describe.
 * @return Static text describing the startup result or failure.
 */
inline const char* context_start_error_string(ContextStartError error) noexcept
{
    switch (error) {
    case ContextStartError::kOk:
        return "ok";
    case ContextStartError::kInitOptionsInitFailed:
        return "failed to initialize rcl init options";
    case ContextStartError::kDomainIdSetFailed:
        return "failed to set ROS domain id";
    case ContextStartError::kRmwOptionsUnavailable:
        return "failed to access rmw init options";
    case ContextStartError::kClientKeySetFailed:
        return "failed to set micro-ROS client key";
    case ContextStartError::kCustomTransportUnavailable:
        return "custom transport selected but generated micro-ROS library does not support it";
    case ContextStartError::kCustomTransportMissing:
        return "custom transport selected without a transport adapter";
    case ContextStartError::kCustomTransportEndpointRejected:
        return "custom transport rejected the configured agent endpoint";
    case ContextStartError::kUdpAddressSetFailed:
        return "failed to configure static UDP agent endpoint";
    case ContextStartError::kStaticAgentPingFailed:
        return "static agent endpoint did not respond";
    case ContextStartError::kAgentLocatorFailed:
        return "agent locator did not resolve an endpoint";
    case ContextStartError::kAgentLocatorNoAddress:
        return "agent locator returned an empty endpoint address";
    case ContextStartError::kAgentLocatorEndpointRejected:
        return "transport rejected the discovered agent endpoint";
    case ContextStartError::kDiscoveredUdpAddressSetFailed:
        return "failed to configure discovered UDP agent endpoint";
    case ContextStartError::kCustomTransportRequiresEndpoint:
        return "custom transport requires a static or located agent endpoint";
    case ContextStartError::kAutoDiscoveryDisabled:
        return "agent auto-discovery is disabled and no endpoint was configured";
    case ContextStartError::kAutoDiscoveryFailed:
        return "agent auto-discovery failed";
    case ContextStartError::kSupportInitFailed:
        return "failed to initialize rclc support";
    case ContextStartError::kExecutorInitFailed:
        return "failed to initialize rclc executor";
    }
    return "unknown context startup error";
}

using ContextStartResult = Result<void, ContextStartError>;

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
    std::void_t<decltype(::rosette::message_traits<MessageT>::rosidl_type_support())>>
    : std::true_type {};

template<typename MessageT>
inline constexpr bool has_micro_ros_message_traits_v =
    has_micro_ros_message_traits<MessageT>::value;

template<typename ServiceT, typename = void>
struct has_micro_ros_service_traits : std::false_type {};

template<typename ServiceT>
struct has_micro_ros_service_traits<
    ServiceT,
    std::void_t<decltype(::rosette::service_traits<ServiceT>::rosidl_type_support())>>
    : std::true_type {};

template<typename ServiceT>
inline constexpr bool has_micro_ros_service_traits_v =
    has_micro_ros_service_traits<ServiceT>::value;

template<typename ActionT, typename = void>
struct has_micro_ros_action_traits : std::false_type {};

template<typename ActionT>
struct has_micro_ros_action_traits<
    ActionT,
    std::void_t<decltype(::rosette::action_traits<ActionT>::rosidl_action_type_support())>>
    : std::true_type {};

template<typename ActionT>
inline constexpr bool has_micro_ros_action_traits_v =
    has_micro_ros_action_traits<ActionT>::value;

template<typename MessageT, typename = void>
struct has_init_message : std::false_type {};

template<typename MessageT>
struct has_init_message<
    MessageT,
    std::void_t<decltype(::rosette::message_traits<MessageT>::init_message(
        std::declval<MessageT&>()))>> : std::true_type {};

template<typename MessageT, typename = void>
struct has_fini_message : std::false_type {};

template<typename MessageT>
struct has_fini_message<
    MessageT,
    std::void_t<decltype(::rosette::message_traits<MessageT>::fini_message(
        std::declval<MessageT&>()))>> : std::true_type {};

template<typename MessageT>
inline void init_message(MessageT& message) noexcept
{
    if constexpr (has_init_message<MessageT>::value) {
        ::rosette::message_traits<MessageT>::init_message(message);
    } else {
        message = MessageT{};
    }
}

template<typename MessageT>
inline void fini_message(MessageT& message) noexcept
{
    if constexpr (has_fini_message<MessageT>::value) {
        ::rosette::message_traits<MessageT>::fini_message(message);
    } else {
        (void)message;
    }
}

template<typename ServiceT, typename = void>
struct has_init_request : std::false_type {};

template<typename ServiceT>
struct has_init_request<
    ServiceT,
    std::void_t<decltype(::rosette::service_traits<ServiceT>::init_request(
        std::declval<typename ::rosette::service_traits<ServiceT>::Request&>()))>>
    : std::true_type {};

template<typename ServiceT, typename = void>
struct has_fini_request : std::false_type {};

template<typename ServiceT>
struct has_fini_request<
    ServiceT,
    std::void_t<decltype(::rosette::service_traits<ServiceT>::fini_request(
        std::declval<typename ::rosette::service_traits<ServiceT>::Request&>()))>>
    : std::true_type {};

template<typename ServiceT, typename = void>
struct has_init_response : std::false_type {};

template<typename ServiceT>
struct has_init_response<
    ServiceT,
    std::void_t<decltype(::rosette::service_traits<ServiceT>::init_response(
        std::declval<typename ::rosette::service_traits<ServiceT>::Response&>()))>>
    : std::true_type {};

template<typename ServiceT, typename = void>
struct has_fini_response : std::false_type {};

template<typename ServiceT>
struct has_fini_response<
    ServiceT,
    std::void_t<decltype(::rosette::service_traits<ServiceT>::fini_response(
        std::declval<typename ::rosette::service_traits<ServiceT>::Response&>()))>>
    : std::true_type {};

template<typename ServiceT>
inline void init_request(typename ::rosette::service_traits<ServiceT>::Request& request) noexcept
{
    if constexpr (has_init_request<ServiceT>::value) {
        ::rosette::service_traits<ServiceT>::init_request(request);
    } else {
        request = typename ::rosette::service_traits<ServiceT>::Request{};
    }
}

template<typename ServiceT>
inline void fini_request(typename ::rosette::service_traits<ServiceT>::Request& request) noexcept
{
    if constexpr (has_fini_request<ServiceT>::value) {
        ::rosette::service_traits<ServiceT>::fini_request(request);
    } else {
        (void)request;
    }
}

template<typename ServiceT>
inline void init_response(typename ::rosette::service_traits<ServiceT>::Response& response) noexcept
{
    if constexpr (has_init_response<ServiceT>::value) {
        ::rosette::service_traits<ServiceT>::init_response(response);
    } else {
        response = typename ::rosette::service_traits<ServiceT>::Response{};
    }
}

template<typename ServiceT>
inline void fini_response(typename ::rosette::service_traits<ServiceT>::Response& response) noexcept
{
    if constexpr (has_fini_response<ServiceT>::value) {
        ::rosette::service_traits<ServiceT>::fini_response(response);
    } else {
        (void)response;
    }
}

template<typename ActionT, typename = void>
struct has_init_send_goal_request : std::false_type {};

template<typename ActionT>
struct has_init_send_goal_request<
    ActionT,
    std::void_t<decltype(::rosette::action_traits<ActionT>::init_send_goal_request(
        std::declval<typename ::rosette::action_traits<ActionT>::SendGoalRequest&>()))>>
    : std::true_type {};

template<typename ActionT, typename = void>
struct has_fini_send_goal_request : std::false_type {};

template<typename ActionT>
struct has_fini_send_goal_request<
    ActionT,
    std::void_t<decltype(::rosette::action_traits<ActionT>::fini_send_goal_request(
        std::declval<typename ::rosette::action_traits<ActionT>::SendGoalRequest&>()))>>
    : std::true_type {};

template<typename ActionT, typename = void>
struct has_init_result_response : std::false_type {};

template<typename ActionT>
struct has_init_result_response<
    ActionT,
    std::void_t<decltype(::rosette::action_traits<ActionT>::init_result_response(
        std::declval<typename ::rosette::action_traits<ActionT>::GetResultResponse&>()))>>
    : std::true_type {};

template<typename ActionT, typename = void>
struct has_fini_result_response : std::false_type {};

template<typename ActionT>
struct has_fini_result_response<
    ActionT,
    std::void_t<decltype(::rosette::action_traits<ActionT>::fini_result_response(
        std::declval<typename ::rosette::action_traits<ActionT>::GetResultResponse&>()))>>
    : std::true_type {};

template<typename ActionT, typename = void>
struct has_init_feedback_message : std::false_type {};

template<typename ActionT>
struct has_init_feedback_message<
    ActionT,
    std::void_t<decltype(::rosette::action_traits<ActionT>::init_feedback_message(
        std::declval<typename ::rosette::action_traits<ActionT>::FeedbackMessage&>()))>>
    : std::true_type {};

template<typename ActionT, typename = void>
struct has_fini_feedback_message : std::false_type {};

template<typename ActionT>
struct has_fini_feedback_message<
    ActionT,
    std::void_t<decltype(::rosette::action_traits<ActionT>::fini_feedback_message(
        std::declval<typename ::rosette::action_traits<ActionT>::FeedbackMessage&>()))>>
    : std::true_type {};

template<typename ActionT>
inline void init_send_goal_request(
    typename ::rosette::action_traits<ActionT>::SendGoalRequest& request) noexcept
{
    if constexpr (has_init_send_goal_request<ActionT>::value) {
        ::rosette::action_traits<ActionT>::init_send_goal_request(request);
    } else {
        request = typename ::rosette::action_traits<ActionT>::SendGoalRequest{};
    }
}

template<typename ActionT>
inline void fini_send_goal_request(
    typename ::rosette::action_traits<ActionT>::SendGoalRequest& request) noexcept
{
    if constexpr (has_fini_send_goal_request<ActionT>::value) {
        ::rosette::action_traits<ActionT>::fini_send_goal_request(request);
    } else {
        (void)request;
    }
}

template<typename ActionT>
inline void init_result_response(
    typename ::rosette::action_traits<ActionT>::GetResultResponse& response) noexcept
{
    if constexpr (has_init_result_response<ActionT>::value) {
        ::rosette::action_traits<ActionT>::init_result_response(response);
    } else {
        response = typename ::rosette::action_traits<ActionT>::GetResultResponse{};
    }
}

template<typename ActionT>
inline void fini_result_response(
    typename ::rosette::action_traits<ActionT>::GetResultResponse& response) noexcept
{
    if constexpr (has_fini_result_response<ActionT>::value) {
        ::rosette::action_traits<ActionT>::fini_result_response(response);
    } else {
        (void)response;
    }
}

template<typename ActionT>
inline void init_feedback_message(
    typename ::rosette::action_traits<ActionT>::FeedbackMessage& feedback) noexcept
{
    if constexpr (has_init_feedback_message<ActionT>::value) {
        ::rosette::action_traits<ActionT>::init_feedback_message(feedback);
    } else {
        feedback = typename ::rosette::action_traits<ActionT>::FeedbackMessage{};
    }
}

template<typename ActionT>
inline void fini_feedback_message(
    typename ::rosette::action_traits<ActionT>::FeedbackMessage& feedback) noexcept
{
    if constexpr (has_fini_feedback_message<ActionT>::value) {
        ::rosette::action_traits<ActionT>::fini_feedback_message(feedback);
    } else {
        (void)feedback;
    }
}

}  // namespace rosette
