#pragma once

/**
 * @file ros/topic.hpp
 * @brief Typed topic publishers and subscriptions for the micro-ROS backend.
 *
 * Design goal
 * -----------
 * This module preserves the compact `create_publisher()` /
 * `create_subscription()` experience used elsewhere in Rosette while routing
 * actual wire traffic through `rcl_publish()` and `rclc_executor_t`.
 *
 * Example
 * -------
 *   auto publisher = node.create_publisher<ImuMsg>("imu");
 *   auto subscription = node.create_subscription<CommandMsg>(
 *       "cmd_vel",
 *       [](const CommandMsg& command) noexcept {
 *           (void)command;
 *       });
 *
 * Features
 * --------
 * - Typed ROSIDL-backed publishers and subscriptions.
 * - Optional XRCE session-timeout tuning for publishers.
 * - Subscription callbacks dispatched through the shared executor with
 *   per-subscription callback context.
 *
 * Caveats
 * -------
 * - Subscription callbacks execute on the context's executor path, so they
 *   should remain short and non-blocking.
 * - Message initialization/finalization relies on optional helpers in
 *   `message_traits<MessageT>` when the generated message type owns dynamic
 *   memory.
 */

#include "common.hpp"
#include "context.hpp"

namespace rosette {

template<typename MessageT>
class Publisher {
public:
    Publisher() noexcept = default;

    /**
     * @brief Constructs a typed publisher wrapper.
     *
     * The wrapper owns the underlying `rcl_publisher_t` and configures it with
     * the QoS profile derived from `PublisherOptions`.
     *
     * @param[in] node Initialized node that will own the publisher.
     * @param[in] context Running micro-ROS context associated with the node.
     * @param[in] topic_name ROS topic name relative to the node namespace.
     * @param[in] options QoS and session-timeout settings for the publisher.
     */
    Publisher(rcl_node_t* node,
              Context* context,
              const char* topic_name,
              const PublisherOptions& options) noexcept
        : node_(node)
        , context_(context)
    {
        static_assert(has_micro_ros_message_traits_v<MessageT>,
                      "message_traits<MessageT>::rosidl_type_support() is required for micro-ROS publishers");

        if (node_ == nullptr || context_ == nullptr) {
            return;
        }

        publisher_ = rcl_get_zero_initialized_publisher();
        const rmw_qos_profile_t qos = make_qos_profile(options);
        if (!ros_ok(rclc_publisher_init(
                &publisher_,
                node_,
                ::rosette::message_traits<MessageT>::rosidl_type_support(),
                topic_name,
                &qos))) {
            publisher_ = rcl_get_zero_initialized_publisher();
            return;
        }

        if (options.session_timeout_ms > 0) {
            rmw_publisher_t* rmw_publisher = rcl_publisher_get_rmw_handle(&publisher_);
            if (rmw_publisher != nullptr) {
                (void)ros_ok(rmw_uros_set_publisher_session_timeout(
                    rmw_publisher, options.session_timeout_ms));
            }
        }

        valid_ = true;
    }

    Publisher(const Publisher&) = delete;
    Publisher& operator=(const Publisher&) = delete;

    Publisher(Publisher&& other) noexcept
        : publisher_(other.publisher_)
        , node_(other.node_)
        , context_(other.context_)
        , valid_(other.valid_)
    {
        other.publisher_ = rcl_get_zero_initialized_publisher();
        other.node_ = nullptr;
        other.context_ = nullptr;
        other.valid_ = false;
    }

    Publisher& operator=(Publisher&& other) noexcept
    {
        if (this != &other) {
            fini();
            publisher_ = other.publisher_;
            node_ = other.node_;
            context_ = other.context_;
            valid_ = other.valid_;
            other.publisher_ = rcl_get_zero_initialized_publisher();
            other.node_ = nullptr;
            other.context_ = nullptr;
            other.valid_ = false;
        }
        return *this;
    }

    ~Publisher()
    {
        fini();
    }

    bool valid() const noexcept { return valid_; }

    /**
     * @brief Publishes a typed ROS message.
     *
     * @param[in] message Generated ROS message payload.
     * @return `true` if the publish call succeeded; otherwise `false`.
     */
    bool publish(const MessageT& message) noexcept
    {
        if (!valid_) {
            return false;
        }
        return ros_ok(rcl_publish(&publisher_, &message, nullptr));
    }

    /**
     * @brief Blocks until every matched subscription has acknowledged every
     * sample published so far, or the timeout passes.
     *
     * publish() never waits for delivery; this is the opt-in wait for
     * applications that must know a RELIABLE sample arrived (for example a
     * KEEP_LAST(1) command that must not be overwritten unseen). It maps to
     * DDS `DataWriter::wait_for_acknowledgments()` through
     * `rcl_publisher_wait_for_all_acked()`. Under rmw_rosette the wait keeps
     * running the participant, so acknowledgments are received while waiting,
     * bare metal included.
     *
     * @param[in] timeout_ms Longest wait in milliseconds; 0 checks once.
     * @return `true` when all samples are acknowledged (immediately for a
     *   BEST_EFFORT publisher or with no matched subscriptions); `false` on
     *   timeout, or when the middleware does not support the wait.
     *
     * Verification:
     * - Correct means: returns true once a reliable peer ACKs the last sample.
     * - Covered by: rmw_rosette `rmw_publisher_wait_for_all_acked()`
     *   (TC-SWR-8.4.7-001 for the writer state it waits on).
     *
     * References:
     * - rcl/publisher.h (Humble), `rcl_publisher_wait_for_all_acked`
     * - OMG DDS 1.4 §2.2.2.4.1.12
     */
    bool wait_for_all_acked(uint32_t timeout_ms) noexcept
    {
        if (!valid_) {
            return false;
        }
        const rcl_duration_value_t timeout_ns =
            static_cast<rcl_duration_value_t>(timeout_ms) * 1000000;
        return rcl_publisher_wait_for_all_acked(&publisher_, timeout_ns) == RCL_RET_OK;
    }

private:
    /**
     * @brief Finalizes the underlying `rcl_publisher_t`.
     *
     * Publisher teardown is straightforward because `rcl` owns the serialized
     * transport state; the wrapper only needs to release the handle when it is
     * valid.
     */
    void fini() noexcept
    {
        if (valid_) {
            (void)ros_ok(rcl_publisher_fini(&publisher_, node_));
        }
        publisher_ = rcl_get_zero_initialized_publisher();
        valid_ = false;
    }

    rcl_publisher_t publisher_ = rcl_get_zero_initialized_publisher();
    rcl_node_t* node_ = nullptr;
    Context* context_ = nullptr;
    bool valid_ = false;
};

template<typename MessageT, typename CallbackT>
class Subscription {
public:
    Subscription() noexcept = default;

    /**
     * @brief Constructs a typed subscription wrapper.
     *
     * The wrapper owns the `rcl_subscription_t` handle and a preallocated
     * message instance used by the shared executor. `rclc` provides a callback
     * context hook for subscriptions, so the wrapper can dispatch directly to
     * the user callback without any global registry.
     *
     * @param[in] node Initialized node that will own the subscription.
     * @param[in] context Running micro-ROS context whose executor dispatches the subscription.
     * @param[in] topic_name ROS topic name relative to the node namespace.
     * @param[in] callback Typed callback invoked on each received message.
     * @param[in] options QoS settings for the subscription.
     */
    Subscription(rcl_node_t* node,
                 Context* context,
                 const char* topic_name,
                 CallbackT callback,
                 const SubscriptionOptions& options) noexcept
        : node_(node)
        , context_(context)
        , callback_(static_cast<CallbackT&&>(callback))
    {
        static_assert(has_micro_ros_message_traits_v<MessageT>,
                      "message_traits<MessageT>::rosidl_type_support() is required for micro-ROS subscriptions");

        if (node_ == nullptr || context_ == nullptr) {
            return;
        }

        init_message<MessageT>(message_);

        subscription_ = rcl_get_zero_initialized_subscription();
        const rmw_qos_profile_t qos = make_qos_profile(options);
        if (!ros_ok(rclc_subscription_init(
                &subscription_,
                node_,
                ::rosette::message_traits<MessageT>::rosidl_type_support(),
                topic_name,
                &qos))) {
            fini_message<MessageT>(message_);
            subscription_ = rcl_get_zero_initialized_subscription();
            return;
        }

        if (!ros_ok(rclc_executor_add_subscription_with_context(
                &context_->executor(),
                &subscription_,
                &message_,
                &Subscription::on_message,
                this,
                ON_NEW_DATA))) {
            (void)ros_ok(rcl_subscription_fini(&subscription_, node_));
            fini_message<MessageT>(message_);
            subscription_ = rcl_get_zero_initialized_subscription();
            return;
        }

        valid_ = true;
    }

    Subscription(const Subscription&) = delete;
    Subscription& operator=(const Subscription&) = delete;
    Subscription(Subscription&&) = delete;
    Subscription& operator=(Subscription&&) = delete;

    ~Subscription()
    {
        fini();
    }

    bool valid() const noexcept { return valid_; }

private:
    /**
     * @brief Forwards an executor-delivered message to the typed user callback.
     *
     * The executor passes the wrapper instance back through its callback
     * context parameter, which lets the C callback bridge directly into the
     * stored C++ callable.
     *
     * @param[in] message Pointer to the preallocated typed message storage.
     * @param[in] context Wrapper instance supplied during executor registration.
     */
    static void on_message(const void* message, void* context) noexcept
    {
        if (message == nullptr || context == nullptr) {
            return;
        }

        auto* self = static_cast<Subscription*>(context);
        self->callback_(*static_cast<const MessageT*>(message));
    }

    /**
     * @brief Finalizes subscription resources and typed message storage.
     *
     * The wrapper removes the subscription from the executor before releasing
     * the `rcl_subscription_t`, then runs any optional generated-message fini
     * hook supplied through `message_traits<MessageT>`.
     */
    void fini() noexcept
    {
        if (valid_) {
            (void)ros_ok(rclc_executor_remove_subscription(&context_->executor(), &subscription_));
            (void)ros_ok(rcl_subscription_fini(&subscription_, node_));
        }
        fini_message<MessageT>(message_);
        subscription_ = rcl_get_zero_initialized_subscription();
        valid_ = false;
    }

    rcl_subscription_t subscription_ = rcl_get_zero_initialized_subscription();
    rcl_node_t* node_ = nullptr;
    Context* context_ = nullptr;
    MessageT message_{};
    CallbackT callback_{};
    bool valid_ = false;
};

}  // namespace rosette
