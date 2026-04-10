#pragma once

#include "publisher.hpp"
#include "subscription.hpp"

namespace milkros {

struct PublisherOptions;
struct SubscriptionOptions;
class Context;

/**
 * @brief Lightweight ROS-like node facade.
 *
 * This is the closest analogue to `rclcpp::Node` in the current `milkros`
 * layer. A Node is a naming and factory surface over a shared Context:
 * it carries the node name and namespace, and uses the Context to create
 * publishers/subscriptions with ROS-compatible topic/type metadata.
 */
class Node {
public:
    Node(Context& context, const char* name, const char* node_namespace) noexcept;

    template<typename MessageT>
    Publisher<MessageT> create_publisher(const char* topic,
                                         const PublisherOptions& options = {}) noexcept;

    template<typename MessageT, typename CallbackT>
    Subscription<MessageT, CallbackT> create_subscription(
        const char* topic,
        CallbackT callback,
        const SubscriptionOptions& options = {}) noexcept;

private:
    Context& context_;
    char name_[32] = {};
    char namespace_[64] = {};
};

} // namespace milkros
