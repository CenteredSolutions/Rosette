#pragma once

#include "publisher.hpp"
#include "service.hpp"
#include "subscription.hpp"

namespace milkros {

// Forward declarations — full definitions in their respective headers.
template<typename CallbackT>         class Timer;
template<typename ServiceT, typename HandlerT> class Service;
template<typename ServiceT, uint8_t MaxPending> class Client;
template<typename ActionT, typename GoalCbT, typename ExecuteCbT> class ActionServer;
template<typename ActionT>           class ActionClient;

struct PublisherOptions;
struct SubscriptionOptions;
struct ServiceOptions;
class Context;

/**
 * @brief Lightweight ROS 2-like node facade.
 *
 * Analogous to `rclcpp::Node`.  A Node is a naming and factory surface over
 * a shared Context: it carries the node name and namespace and delegates all
 * entity creation to the Context.
 *
 * Lifecycle
 * ---------
 * Nodes are created via Context::create_node() and are lightweight value types
 * (just a Context reference + two fixed-size name buffers).  They may be
 * copied freely; all copies share the same Context.
 *
 * Factory methods
 * ---------------
 * Factories for Publisher and Subscription return by value (matches rclcpp API).
 * Factories for Timer, Service, Client, ActionServer, ActionClient return raw
 * pointers into the Context's pre-allocated pool.  The returned pointers are
 * stable for the Context's lifetime.  Never delete them manually.
 *
 * Example
 * -------
 *   auto node = context.create_node("my_node", "/my_ns");
 *
 *   // Timer — auto-driven by context.tick_timers()
 *   auto* hb_timer = node.create_timer(500u, [&]() { pub.publish(msg); });
 *
 *   // Service server
 *   auto* srv = node.create_service<AddTwoInts>("add_two_ints",
 *       [](const Request& req, Response& resp) {
 *           resp.sum = req.a + req.b;
 *       });
 *
 *   // Service client
 *   auto* client = node.create_client<AddTwoInts>("add_two_ints");
 *   client->async_send_request(req,
 *       [](const Response& r, void* ud) { ... }, nullptr);
 *
 *   // Action server
 *   auto* server = node.create_action_server<Fibonacci>("fibonacci",
 *       [](const GoalId& id, const Goal& g) { return GoalResponse::ACCEPT; },
 *       [](GoalHandle<Fibonacci>& h) {
 *           if (done) h.succeed(result);
 *           else      h.publish_feedback(fb);
 *       });
 *
 *   // Action client
 *   auto* ac = node.create_action_client<Fibonacci>("fibonacci");
 *   GoalId gid{};
 *   ac->send_goal(goal, gid, on_goal_resp, on_feedback, on_result, nullptr);
 */
class Node {
public:
    Node(Context&    context,
         const char* name,
         const char* node_namespace,
         int         node_index = -1) noexcept;

    // ---- Topics ----

    template<typename MessageT>
    Publisher<MessageT> create_publisher(
        const char* topic,
        const PublisherOptions& options = {}) noexcept;

    template<typename MessageT, typename CallbackT>
    Subscription<MessageT, CallbackT> create_subscription(
        const char* topic,
        CallbackT   callback,
        const SubscriptionOptions& options = {}) noexcept;


    // ---- Timers ----

    /**
     * @brief Create a periodic timer registered with the Context.
     *
     * The timer fires every period_ms milliseconds, driven by the Context's
     * tick_timers() call.  The returned pointer is stable for the Context's
     * lifetime; never delete or move the object it points to.
     *
     * @param period_ms  Period in milliseconds (must be > 0).
     * @param callback   Zero-argument callable, e.g. []() { ... }.
     * @returns Stable pointer to the registered Timer, or nullptr if the
     *          Context timer pool is full or the callback is too large.
     */
    template<typename CallbackT>
    Timer<CallbackT>* create_timer(uint32_t period_ms, CallbackT callback) noexcept;

    // ---- Services ----

    /**
     * @brief Create a service server.
     *
     * Topic names follow the ROS 2 convention:
     *   rq/<namespace>/<service_name>Request
     *   rr/<namespace>/<service_name>Reply
     *
     * @param service_name  Service name (without namespace prefix).
     * @param handler       Callable: void(const Request&, Response&) noexcept
     * @param options       Reliability and durability QoS settings.
     * @returns Stable pointer to the Service, or nullptr on failure.
     */
    template<typename ServiceT, typename HandlerT>
    Service<ServiceT, HandlerT>* create_service(
        const char* service_name,
        HandlerT    handler,
        const ServiceOptions& options = {}) noexcept;

    /**
     * @brief Create a service client.
     *
     * @tparam MaxPending  Maximum concurrent in-flight requests (default 4).
     * @param service_name  Service name (must match the server's service_name).
     * @param options       Reliability and durability QoS settings.
     * @returns Stable pointer to the Client, or nullptr on failure.
     */
    template<typename ServiceT, uint8_t MaxPending = 4>
    Client<ServiceT, MaxPending>* create_client(
        const char* service_name,
        const ServiceOptions& options = {}) noexcept;

    // ---- Actions ----

    /**
     * @brief Create an action server (full ROS 2 action protocol).
     *
     * Allocates 5 writers + 3 readers from the participant.
     * Requires RTPS_MAX_USER_WRITERS >= 8 and RTPS_MAX_USER_READERS >= 8.
     *
     * @param action_name  Action name (e.g. "fibonacci").
     * @param goal_cb      GoalResponse(const GoalId&, const Goal&) noexcept
     * @param execute_cb   void(GoalHandle<ActionT>&) noexcept
     *                     Called every tick while goal is EXECUTING or CANCELING.
     * @returns Stable pointer to ActionServer, or nullptr on failure.
     */
    template<typename ActionT, typename GoalCbT, typename ExecuteCbT>
    ActionServer<ActionT, GoalCbT, ExecuteCbT>* create_action_server(
        const char* action_name,
        GoalCbT     goal_cb,
        ExecuteCbT  execute_cb) noexcept;

    /**
     * @brief Create an action client (full ROS 2 action protocol).
     *
     * Allocates 3 writers + 5 readers from the participant.
     * Requires RTPS_MAX_USER_WRITERS >= 8 and RTPS_MAX_USER_READERS >= 8.
     *
     * @param action_name  Action name (must match the server).
     * @returns Stable pointer to ActionClient, or nullptr on failure.
     */
    template<typename ActionT>
    ActionClient<ActionT>* create_action_client(const char* action_name) noexcept;

    // ---- Accessors ----

    const char* name()           const noexcept { return name_; }
    const char* node_namespace() const noexcept { return namespace_; }

private:
    Context& context_;
    char     name_[32]      = {};
    char     namespace_[64] = {};
    int      node_index_    = -1;  ///< Index into Context::graph_.nodes[]; -1 if untracked.
};

} // namespace milkros
