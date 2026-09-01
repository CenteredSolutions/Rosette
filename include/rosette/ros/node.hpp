#pragma once

/**
 * @file ros/node.hpp
 * @brief Node wrapper that creates micro-ROS communication entities for Rosette.
 *
 * Design goal
 * -----------
 * `Node` is the object application code touches after a `Context` has started.
 * It owns the `rcl_node_t` handle and exposes factory methods for publishers,
 * subscriptions, timers, services, clients, and actions so user code can stay
 * close to the shape of `rclcpp::Node` while staying MCU-friendly.
 *
 * Example
 * -------
 *   rosette::Context context;
 *   context.start();
 *
 *   auto node = context.create_node("drive_controller", "/robot");
 *   auto publisher = node.create_publisher<DriveState>("state");
 *   auto timer = node.create_timer(10, [&]() noexcept {
 *       DriveState state{};
 *       publisher.publish(state);
 *   });
 *
 * Features
 * --------
 * - Typed factories for all primary ROS 2 communication patterns.
 * - No dynamic ownership graph: entity wrappers own their own `rcl` handles.
 * - Mirrors the mental model of `rclcpp::Node` while remaining MCU-friendly.
 *
 * Caveats
 * -------
 * - The `Context` must already be started before constructing a node.
 * - Entity lifetimes should not outlive the node or context that created them.
 */

#include "action.hpp"
#include "context.hpp"
#include "service.hpp"
#include "timer.hpp"
#include "topic.hpp"

#include <optional>

namespace rosette {

class Node {
public:
    Node() noexcept = default;

    /**
     * @brief Constructs a micro-ROS node wrapper over a started context.
     *
     * The node is a thin owner of `rcl_node_t`. Communication entities created
     * from it inherit the context's transport, executor, and XRCE session.
     *
     * @param[in] context Started micro-ROS context that owns support and executor state.
     * @param[in] name ROS node name.
     * @param[in] node_namespace ROS namespace for relative entity names.
     */
    Node(Context* context,
         const char* name,
         const char* node_namespace) noexcept
        : context_(context)
    {
        if (context_ == nullptr || !context_->started()) {
            return;
        }

        node_ = rcl_get_zero_initialized_node();
        if (rclc_node_init_default(
                &node_, name, node_namespace, &context_->support()) != RCL_RET_OK) {
            node_ = rcl_get_zero_initialized_node();
            return;
        }

        valid_ = true;
    }

    Node(const Node&) = delete;
    Node& operator=(const Node&) = delete;

    Node(Node&& other) noexcept
        : node_(other.node_)
        , context_(other.context_)
        , valid_(other.valid_)
    {
        other.node_ = rcl_get_zero_initialized_node();
        other.context_ = nullptr;
        other.valid_ = false;
    }

    Node& operator=(Node&& other) noexcept
    {
        if (this != &other) {
            fini();
            node_ = other.node_;
            context_ = other.context_;
            valid_ = other.valid_;
            other.node_ = rcl_get_zero_initialized_node();
            other.context_ = nullptr;
            other.valid_ = false;
        }
        return *this;
    }

    ~Node()
    {
        fini();
    }

    bool valid() const noexcept { return valid_; }
    rcl_node_t* native_handle() noexcept { return valid_ ? &node_ : nullptr; }
    Context* context() noexcept { return context_; }

    /**
     * @brief Creates a typed publisher attached to this node.
     *
     * @param[in] topic ROS topic name relative to the node namespace.
     * @param[in] options QoS and transport-session settings for the publisher.
     * @return Publisher wrapper. `valid()` is false on initialization failure.
     */
    template<typename MessageT>
    Publisher<MessageT> create_publisher(
        const char* topic,
        const PublisherOptions& options = {}) noexcept
    {
        return Publisher<MessageT>(&node_, context_, topic, options);
    }

    /**
     * @brief Creates a typed subscription attached to this node.
     *
     * @param[in] topic ROS topic name relative to the node namespace.
     * @param[in] callback Message callback invoked from the shared executor.
     * @param[in] options QoS settings for the subscription.
     * @return Subscription wrapper. `valid()` is false on initialization failure.
     */
    template<typename MessageT, typename CallbackT>
    Subscription<MessageT, CallbackT> create_subscription(
        const char* topic,
        CallbackT callback,
        const SubscriptionOptions& options = {}) noexcept
    {
        return Subscription<MessageT, CallbackT>(
            &node_, context_, topic, static_cast<CallbackT&&>(callback), options);
    }

    /**
     * @brief Creates a periodic timer attached to the shared executor.
     *
     * @param[in] period_ms Timer period in milliseconds.
     * @param[in] callback Timer callback invoked by the executor.
     * @return Timer wrapper. `valid()` is false on initialization failure.
     */
    template<typename F>
    Timer create_timer(uint32_t period_ms, F&& callback) noexcept
    {
        return Timer(context_, period_ms, static_cast<F&&>(callback));
    }

    /**
     * @brief Constructs a timer directly into a caller-owned slot.
     *
     * Modules hold their entities as `std::optional<...>` so they can be torn
     * down and rebuilt across an Agent reconnect. This overload emplaces into
     * that slot without a move.
     */
    template<typename F>
    void create_timer(std::optional<Timer>& slot, uint32_t period_ms, F&& callback) noexcept
    {
        slot.emplace(context_, period_ms, static_cast<F&&>(callback));
    }

    /**
     * @brief Creates a typed service server attached to this node.
     *
     * @param[in] service_name ROS service name relative to the node namespace.
     * @param[in] handler Request handler invoked as `handler(const Request&, Response&)`.
     * @param[in] options QoS and session-timeout settings for the service.
     * @return Service wrapper. `valid()` is false on initialization failure.
     */
    template<typename ServiceT, typename F>
    Service<ServiceT> create_service(
        const char* service_name,
        F&& handler,
        const ServiceOptions& options = {}) noexcept
    {
        return Service<ServiceT>(
            &node_, context_, service_name, static_cast<F&&>(handler), options);
    }

    /**
     * @brief Constructs a service directly into a caller-owned slot (see
     * `create_timer(std::optional<Timer>&, ...)`).
     */
    template<typename ServiceT, typename F>
    void create_service(
        std::optional<Service<ServiceT>>& slot,
        const char* service_name,
        F&& handler,
        const ServiceOptions& options = {}) noexcept
    {
        slot.emplace(&node_, context_, service_name, static_cast<F&&>(handler), options);
    }

    /**
     * @brief Creates a typed async service client attached to this node.
     *
     * @param[in] service_name ROS service name relative to the node namespace.
     * @param[in] options QoS and session-timeout settings for the client.
     * @return Client wrapper. `valid()` is false on initialization failure.
     */
    template<typename ServiceT, uint8_t MaxPending = 4>
    Client<ServiceT, MaxPending> create_client(
        const char* service_name,
        const ServiceOptions& options = {}) noexcept
    {
        return Client<ServiceT, MaxPending>(&node_, context_, service_name, options);
    }

    /**
     * @brief Creates a typed action server attached to this node.
     *
     * The action server uses cooperative execution: the goal callback accepts
     * or rejects the request, and the execute callback advances the active goal
     * on each `Context::tick()`.
     *
     * @param[in] action_name ROS action name relative to the node namespace.
     * @param[in] goal_callback Goal accept/reject callback.
     * @param[in] execute_callback Cooperative execution callback.
     * @return Action server wrapper. `valid()` is false on initialization failure.
     */
    template<typename ActionT, typename GoalCbT, typename ExecuteCbT>
    ActionServer<ActionT, GoalCbT, ExecuteCbT> create_action_server(
        const char* action_name,
        GoalCbT goal_callback,
        ExecuteCbT execute_callback) noexcept
    {
        return ActionServer<ActionT, GoalCbT, ExecuteCbT>(
            &node_,
            context_,
            action_name,
            static_cast<GoalCbT&&>(goal_callback),
            static_cast<ExecuteCbT&&>(execute_callback));
    }

    /**
     * @brief Creates a typed action client attached to this node.
     *
     * @param[in] action_name ROS action name relative to the node namespace.
     * @return Action client wrapper. `valid()` is false on initialization failure.
     */
    template<typename ActionT, uint8_t MaxTrackedGoals = 4>
    ActionClient<ActionT, MaxTrackedGoals> create_action_client(
        const char* action_name) noexcept
    {
        return ActionClient<ActionT, MaxTrackedGoals>(&node_, context_, action_name);
    }

private:
    /**
     * @brief Finalizes the underlying `rcl_node_t`.
     *
     * Child entities are expected to be destroyed before the node. Finalizing
     * the node invalidates all relative topic, service, and action handles that
     * were created from it.
     */
    void fini() noexcept
    {
        if (valid_) {
            [[maybe_unused]] const rcl_ret_t fini_ret = rcl_node_fini(&node_);
        }
        node_ = rcl_get_zero_initialized_node();
        valid_ = false;
    }

    rcl_node_t node_ = rcl_get_zero_initialized_node();
    Context* context_ = nullptr;
    bool valid_ = false;
};

inline Node Context::create_node(const char* name,
                                 const char* node_namespace) noexcept
{
    return Node(this, name, node_namespace);
}

}  // namespace rosette
