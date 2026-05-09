#pragma once

#include "common.hpp"

#include <string.h>

namespace milkros::micro_ros {

class Node;

/**
 * @brief micro-ROS execution context for one XRCE-DDS client session.
 *
 * The context owns ROS initialization state, transport configuration, and the
 * shared `rclc_executor_t`. Endpoint wrappers register themselves with this
 * executor so application code can keep a single `tick()` loop similar to the
 * existing RTPS-backed `milkros::Context`.
 *
 * Discovery model
 * ---------------
 * - Stock UDP transport: use `rmw_uros_discover_agent()` or a static Agent IP.
 * - Custom transport: use a Rosette-side `AgentLocator` or a static Agent IP.
 *
 * This split is intentional. micro-ROS documents automatic discovery as a UDP
 * Agent-discovery feature. A custom transport can absolutely run on MCU
 * hardware, but Agent location becomes a platform concern rather than an RMW
 * concern.
 *
 * Example
 * -------
 *   milkros::micro_ros::ContextOptions options{};
 *   options.client_key = 0x11223344u;
 *   options.auto_discover_agent = true;
 *
 *   milkros::micro_ros::Context context(options);
 *   if (context.start()) {
 *       auto node = context.create_node("motor_controller", "/drive");
 *       (void)node;
 *   }
 *
 * Features
 * --------
 * - Owns the `rcl_init_options_t`, `rclc_support_t`, and shared executor.
 * - Supports stock UDP discovery, fixed Agent endpoints, and Rosette-side
 *   custom transport + locator flows.
 * - Provides a cooperative tick hook used by the micro-ROS action wrapper.
 *
 * Caveats
 * -------
 * - The context should be started exactly once and finalized after child
 *   entities are destroyed.
 * - Custom transports must be configured before `rclc_support_init_with_options()`
 *   runs because that is when the XRCE session is created.
 */
class Context {
public:
    explicit Context(const ContextOptions& options = {}) noexcept
        : options_(options)
    {}

    Context(const Context&) = delete;
    Context& operator=(const Context&) = delete;

    Context(Context&&) = delete;
    Context& operator=(Context&&) = delete;

    ~Context() {
        fini();
    }

    bool start() noexcept
    {
        if (started_) {
            return true;
        }

        allocator_ = rcl_get_default_allocator();
        init_options_ = rcl_get_zero_initialized_init_options();
        if (rcl_init_options_init(&init_options_, allocator_) != RCL_RET_OK) {
            return false;
        }
        init_options_initialized_ = true;

        if (rcl_init_options_set_domain_id(&init_options_, options_.domain_id) != RCL_RET_OK) {
            fini();
            return false;
        }

        rmw_init_options_t* rmw_options =
            rcl_init_options_get_rmw_init_options(&init_options_);
        if (rmw_options == nullptr) {
            fini();
            return false;
        }

        if (rmw_uros_options_set_client_key(options_.client_key, rmw_options) != RMW_RET_OK) {
            fini();
            return false;
        }

        if (!configure_transport(*rmw_options)) {
            fini();
            return false;
        }

        if (!discover_agent(*rmw_options)) {
            fini();
            return false;
        }

        support_ = rclc_support_get_zero_initialized_support();
        if (rclc_support_init_with_options(
                &support_, 0, nullptr, &init_options_, &allocator_) != RCL_RET_OK) {
            fini();
            return false;
        }
        support_initialized_ = true;

        executor_ = rclc_executor_get_zero_initialized_executor();
        if (rclc_executor_init(
                &executor_,
                &support_.context,
                options_.executor_handles,
                &allocator_) != RCL_RET_OK) {
            fini();
            return false;
        }
        executor_initialized_ = true;

        started_ = true;

        if (options_.synchronize_epoch) {
            (void)rmw_uros_sync_session(options_.epoch_sync_timeout_ms);
        }

        return true;
    }

    void fini() noexcept
    {
        for (size_t i = 0; i < action_tickable_count_; ++i) {
            action_tickables_[i] = nullptr;
        }
        action_tickable_count_ = 0u;

        if (executor_initialized_) {
            (void)rclc_executor_fini(&executor_);
            executor_initialized_ = false;
        }

        if (support_initialized_) {
            (void)rclc_support_fini(&support_);
            support_initialized_ = false;
        }

        if (init_options_initialized_) {
            (void)rcl_init_options_fini(&init_options_);
            init_options_initialized_ = false;
        }

        started_ = false;
    }

    bool tick(uint32_t /*now_ms*/ = 0u) noexcept
    {
        if (!started_) {
            return false;
        }

        const rcl_ret_t rc = rclc_executor_spin_some(&executor_, options_.spin_timeout_ns);
        if (rc != RCL_RET_OK && rc != RCL_RET_TIMEOUT) {
            return false;
        }

        for (size_t i = 0; i < action_tickable_count_; ++i) {
            if (action_tickables_[i] != nullptr) {
                action_tickables_[i]->tick();
            }
        }
        return true;
    }

    Node create_node(const char* name,
                     const char* node_namespace = "") noexcept;

    const ContextOptions& options() const noexcept { return options_; }
    rcl_allocator_t& allocator() noexcept { return allocator_; }
    rclc_support_t& support() noexcept { return support_; }
    rclc_executor_t& executor() noexcept { return executor_; }
    bool started() const noexcept { return started_; }

    bool register_action_tickable(IActionTickable* tickable) noexcept
    {
        if (tickable == nullptr || action_tickable_count_ >= kMaxActionTickables) {
            return false;
        }
        action_tickables_[action_tickable_count_++] = tickable;
        return true;
    }

    void unregister_action_tickable(IActionTickable* tickable) noexcept
    {
        if (tickable == nullptr) {
            return;
        }
        for (size_t i = 0; i < action_tickable_count_; ++i) {
            if (action_tickables_[i] == tickable) {
                for (size_t j = i + 1u; j < action_tickable_count_; ++j) {
                    action_tickables_[j - 1u] = action_tickables_[j];
                }
                --action_tickable_count_;
                action_tickables_[action_tickable_count_] = nullptr;
                return;
            }
        }
    }

private:
    bool configure_transport(rmw_init_options_t& rmw_options) noexcept
    {
        if (options_.use_custom_transport) {
            if (options_.custom_transport == nullptr) {
                return false;
            }

            if (options_.udp_agent_address != nullptr &&
                !options_.custom_transport->configure_agent_endpoint(
                    options_.udp_agent_address,
                    options_.udp_agent_port)) {
                return false;
            }

            return rmw_uros_options_set_custom_transport(
                       options_.custom_transport_framing,
                       options_.custom_transport,
                       &Context::open_transport,
                       &Context::close_transport,
                       &Context::write_transport,
                       &Context::read_transport,
                       &rmw_options) == RMW_RET_OK;
        }

        if (options_.udp_agent_address != nullptr) {
            return rmw_uros_options_set_udp_address(
                       options_.udp_agent_address,
                       options_.udp_agent_port,
                       &rmw_options) == RMW_RET_OK;
        }

        return true;
    }

    bool discover_agent(rmw_init_options_t& rmw_options) noexcept
    {
        if (options_.udp_agent_address != nullptr) {
            return rmw_uros_ping_agent_options(
                       options_.discovery_timeout_ms,
                       options_.discovery_attempts,
                       &rmw_options) == RMW_RET_OK;
        }

        if (options_.agent_locator != nullptr) {
            char address[64] = {};
            char port[16] = {};
            if (!options_.agent_locator->resolve(
                    address, sizeof(address), port, sizeof(port))) {
                return false;
            }

            if (address[0] == '\0') {
                return false;
            }
            if (port[0] == '\0') {
                strncpy(port, options_.udp_agent_port, sizeof(port) - 1u);
                port[sizeof(port) - 1u] = '\0';
            }

            if (options_.use_custom_transport) {
                if ((options_.custom_transport == nullptr) ||
                    !options_.custom_transport->configure_agent_endpoint(address, port)) {
                    return false;
                }
            } else if (rmw_uros_options_set_udp_address(address, port, &rmw_options) !=
                       RMW_RET_OK) {
                return false;
            }

            return rmw_uros_ping_agent_options(
                       options_.discovery_timeout_ms,
                       options_.discovery_attempts,
                       &rmw_options) == RMW_RET_OK;
        }

        if (options_.use_custom_transport) {
            return false;
        }

        if (!options_.auto_discover_agent) {
            return false;
        }

        return rmw_uros_discover_agent(&rmw_options) == RMW_RET_OK;
    }

    static bool open_transport(struct uxrCustomTransport* transport) noexcept
    {
        if (transport == nullptr || transport->args == nullptr) {
            return false;
        }
        auto* adapter = static_cast<CustomTransport*>(transport->args);
        return adapter->open();
    }

    static bool close_transport(struct uxrCustomTransport* transport) noexcept
    {
        if (transport == nullptr || transport->args == nullptr) {
            return false;
        }
        auto* adapter = static_cast<CustomTransport*>(transport->args);
        return adapter->close();
    }

    static size_t write_transport(struct uxrCustomTransport* transport,
                                  const uint8_t* data,
                                  size_t length,
                                  uint8_t* error) noexcept
    {
        if (transport == nullptr || transport->args == nullptr) {
            return 0u;
        }
        auto* adapter = static_cast<CustomTransport*>(transport->args);
        return adapter->write(data, length, error);
    }

    static size_t read_transport(struct uxrCustomTransport* transport,
                                 uint8_t* data,
                                 size_t length,
                                 int timeout_ms,
                                 uint8_t* error) noexcept
    {
        if (transport == nullptr || transport->args == nullptr) {
            return 0u;
        }
        auto* adapter = static_cast<CustomTransport*>(transport->args);
        return adapter->read(data, length, timeout_ms, error);
    }

    static constexpr size_t kMaxActionTickables = 8u;

    ContextOptions options_{};
    rcl_allocator_t allocator_{};
    rcl_init_options_t init_options_ = rcl_get_zero_initialized_init_options();
    rclc_support_t support_ = rclc_support_get_zero_initialized_support();
    rclc_executor_t executor_ = rclc_executor_get_zero_initialized_executor();

    IActionTickable* action_tickables_[kMaxActionTickables] = {};
    size_t action_tickable_count_ = 0u;

    bool started_ = false;
    bool init_options_initialized_ = false;
    bool support_initialized_ = false;
    bool executor_initialized_ = false;
};

}  // namespace milkros::micro_ros
