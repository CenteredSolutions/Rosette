#pragma once

#include "common.hpp"

#include <string.h>
#include <stdio.h>

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
 *   if (context.start_result().ok()) {
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

    /**
     * @brief Starts the micro-ROS context and returns a typed result.
     *
     * Startup configures RCL initialization options, applies the selected
     * XRCE-DDS transport, resolves or discovers the Agent, and then creates
     * `rclc` support and executor state. Failed attempts call `fini()` before
     * returning, so callers may retry later when network or Agent availability
     * changes.
     *
     * @return Empty successful result when the context is running, or a
     * `ContextStartError` describing which startup phase failed.
     */
    ContextStartResult start_result() noexcept
    {
        if (started_) {
            last_start_error_ = ContextStartError::kOk;
            return {};
        }

        allocator_ = rcl_get_default_allocator();
        init_options_ = rcl_get_zero_initialized_init_options();
        if (rcl_init_options_init(&init_options_, allocator_) != RCL_RET_OK) {
            return fail_start(ContextStartError::kInitOptionsInitFailed);
        }
        init_options_initialized_ = true;

        if (rcl_init_options_set_domain_id(&init_options_, options_.domain_id) != RCL_RET_OK) {
            fini();
            return fail_start(ContextStartError::kDomainIdSetFailed);
        }

        rmw_init_options_t* rmw_options =
            rcl_init_options_get_rmw_init_options(&init_options_);
        if (rmw_options == nullptr) {
            fini();
            return fail_start(ContextStartError::kRmwOptionsUnavailable);
        }

        if (rmw_uros_options_set_client_key(options_.client_key, rmw_options) != RMW_RET_OK) {
            fini();
            return fail_start(ContextStartError::kClientKeySetFailed);
        }

        ContextStartError status = configure_transport(*rmw_options);
        if (status != ContextStartError::kOk) {
            fini();
            return fail_start(status);
        }

        status = discover_agent(*rmw_options);
        if (status != ContextStartError::kOk) {
            fini();
            return fail_start(status);
        }

        support_ = {};
        if (rclc_support_init_with_options(
                &support_, 0, nullptr, &init_options_, &allocator_) != RCL_RET_OK) {
            fini();
            return fail_start(ContextStartError::kSupportInitFailed);
        }
        support_initialized_ = true;

        executor_ = rclc_executor_get_zero_initialized_executor();
        if (rclc_executor_init(
                &executor_,
                &support_.context,
                options_.executor_handles,
                &allocator_) != RCL_RET_OK) {
            fini();
            return fail_start(ContextStartError::kExecutorInitFailed);
        }
        executor_initialized_ = true;

        started_ = true;

        if (options_.synchronize_epoch) {
            (void)rmw_uros_sync_session(options_.epoch_sync_timeout_ms);
        }

        last_start_error_ = ContextStartError::kOk;
        return {};
    }

    /**
     * @brief Starts the micro-ROS context and reports boolean success.
     *
     * This compatibility wrapper preserves the original API while
     * `start_result()` provides detailed diagnostics for applications that need
     * retry and failure reporting.
     *
     * @return True when the context is running after the call.
     */
    bool start() noexcept
    {
        return start_result().ok();
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
            [[maybe_unused]] const rcl_ret_t fini_ret =
                rcl_init_options_fini(&init_options_);
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
    ContextStartError last_start_error() const noexcept { return last_start_error_; }

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
    /**
     * @brief Stores and returns a context startup failure.
     *
     * Startup helpers use this path so `last_start_error()` reflects the most
     * recent failed attempt while the returned result carries the same typed
     * reason to the caller.
     *
     * @param[in] error Startup error detected by the current attempt.
     * @return Failed context startup result carrying `error`.
     */
    ContextStartResult fail_start(ContextStartError error) noexcept
    {
        last_start_error_ = error;
        return ContextStartResult(error);
    }

    /**
     * @brief Applies the selected XRCE-DDS transport to RMW initialization.
     *
     * The context calls this before `rclc_support_init_with_options()` so the
     * micro-ROS session is created with the board-selected transport profile.
     * UDP builds may use a static Agent endpoint or defer endpoint selection
     * to discovery, while custom-transport builds bind Rosette's transport
     * adapter callbacks into the RMW options.
     *
     * The function first validates transport-specific prerequisites, then
     * installs either the custom callback table or the fixed UDP address into
     * the supplied RMW options. Custom transport code is compiled only when the
     * generated micro-ROS library was built for custom transport support.
     *
     * @param[in,out] rmw_options RMW initialization options updated with the
     * transport settings used by the XRCE-DDS client.
     * @return `ContextStartError::kOk` when the selected transport was
     * configured successfully, or a transport-specific failure status.
     */
    ContextStartError configure_transport(rmw_init_options_t& rmw_options) noexcept
    {
        char port_str[6];
        snprintf(port_str, sizeof(port_str), "%u", options_.udp_agent_port);

        if (options_.use_custom_transport) {
#if ROSETTE_MICRO_ROS_TRANSPORT_CUSTOM
            if (options_.custom_transport == nullptr) {
                return ContextStartError::kCustomTransportMissing;
            }

            if (options_.udp_agent_address != nullptr &&
                !options_.custom_transport->configure_agent_endpoint(
                    options_.udp_agent_address,
                    port_str)) {
                return ContextStartError::kCustomTransportEndpointRejected;
            }

            return rmw_uros_options_set_custom_transport(
                       options_.custom_transport_framing,
                       options_.custom_transport,
                       &Context::open_transport,
                       &Context::close_transport,
                       &Context::write_transport,
                       &Context::read_transport,
                       &rmw_options) == RMW_RET_OK
                       ? ContextStartError::kOk
                       : ContextStartError::kCustomTransportUnavailable;
#else
            return ContextStartError::kCustomTransportUnavailable;
#endif
        }

        if (options_.udp_agent_address != nullptr) {
            return rmw_uros_options_set_udp_address(
                       options_.udp_agent_address,
                       port_str,
                       &rmw_options) == RMW_RET_OK
                       ? ContextStartError::kOk
                       : ContextStartError::kUdpAddressSetFailed;
        }

        return ContextStartError::kOk;
    }

    /**
     * @brief Resolves and validates the micro-ROS Agent endpoint.
     *
     * This step runs after transport selection and before RCL support
     * initialization. It keeps the Agent-location policy in one place: static
     * endpoints are pinged directly, Rosette locators can feed a discovered
     * endpoint into UDP or custom transports, and stock UDP builds can ask
     * `rmw_microxrcedds` to perform Agent discovery.
     *
     * The function chooses the highest-priority endpoint source available,
     * writes discovered endpoints back into the RMW options or custom adapter,
     * and pings the Agent before allowing startup to continue.
     *
     * @param[in,out] rmw_options RMW initialization options that may receive a
     * discovered UDP endpoint.
     * @return `ContextStartError::kOk` when an Agent endpoint was selected and
     * verified, or a discovery-specific failure status.
     */
    ContextStartError discover_agent(rmw_init_options_t& rmw_options) noexcept
    {
        if (options_.udp_agent_address != nullptr) {
            return rmw_uros_ping_agent_options(
                       options_.discovery_timeout_ms,
                       options_.discovery_attempts,
                       &rmw_options) == RMW_RET_OK
                       ? ContextStartError::kOk
                       : ContextStartError::kStaticAgentPingFailed;
        }

        if (options_.agent_locator != nullptr) {
            char address[64] = {};
            char port[16] = {};
            if (!options_.agent_locator->resolve(
                    address, sizeof(address), port, sizeof(port))) {
                return ContextStartError::kAgentLocatorFailed;
            }

            if (address[0] == '\0') {
                return ContextStartError::kAgentLocatorNoAddress;
            }
            if (port[0] == '\0') {
                snprintf(port, sizeof(port), "%u", options_.udp_agent_port);
            }

            if (options_.use_custom_transport) {
                if ((options_.custom_transport == nullptr) ||
                    !options_.custom_transport->configure_agent_endpoint(address, port)) {
                    return ContextStartError::kAgentLocatorEndpointRejected;
                }
            } else if (rmw_uros_options_set_udp_address(address, port, &rmw_options) !=
                       RMW_RET_OK) {
                return ContextStartError::kDiscoveredUdpAddressSetFailed;
            }

            return rmw_uros_ping_agent_options(
                       options_.discovery_timeout_ms,
                       options_.discovery_attempts,
                       &rmw_options) == RMW_RET_OK
                       ? ContextStartError::kOk
                       : ContextStartError::kStaticAgentPingFailed;
        }

        if (options_.use_custom_transport) {
            return ContextStartError::kCustomTransportRequiresEndpoint;
        }

        if (!options_.auto_discover_agent) {
            return ContextStartError::kAutoDiscoveryDisabled;
        }

#if defined(RMW_UXRCE_TRANSPORT_UDP) || defined(RMW_UXRCE_TRANSPORT_TCP)
        return rmw_uros_discover_agent(&rmw_options) == RMW_RET_OK
               ? ContextStartError::kOk
               : ContextStartError::kAutoDiscoveryFailed;
#else
        return ContextStartError::kAutoDiscoveryDisabled;
#endif
    }

#if ROSETTE_MICRO_ROS_TRANSPORT_CUSTOM
    /**
     * @brief Opens the Rosette custom transport adapter for micro-ROS.
     *
     * `rmw_microxrcedds` invokes this C callback when creating the XRCE
     * transport. The callback recovers the `CustomTransport` object stored in
     * the transport arguments and delegates board-specific driver startup to
     * that adapter.
     *
     * @param[in,out] transport XRCE custom transport handle whose `args` field
     * points at a `CustomTransport` instance.
     * @return True when the board transport opened successfully.
     */
    static bool open_transport(::uxrCustomTransport* transport) noexcept
    {
        if (transport == nullptr || transport->args == nullptr) {
            return false;
        }
        auto* adapter = static_cast<CustomTransport*>(transport->args);
        return adapter->open();
    }

    /**
     * @brief Closes the Rosette custom transport adapter for micro-ROS.
     *
     * `rmw_microxrcedds` invokes this callback during XRCE transport teardown.
     * The callback delegates shutdown to the board adapter associated with the
     * transport handle.
     *
     * @param[in,out] transport XRCE custom transport handle whose `args` field
     * points at a `CustomTransport` instance.
     * @return True when the board transport closed successfully.
     */
    static bool close_transport(::uxrCustomTransport* transport) noexcept
    {
        if (transport == nullptr || transport->args == nullptr) {
            return false;
        }
        auto* adapter = static_cast<CustomTransport*>(transport->args);
        return adapter->close();
    }

    /**
     * @brief Writes an XRCE frame through the Rosette custom transport adapter.
     *
     * This callback is part of the micro-ROS custom transport table. It passes
     * the serialized XRCE bytes to the board adapter and returns the number of
     * bytes accepted by the underlying driver.
     *
     * @param[in,out] transport XRCE custom transport handle whose `args` field
     * points at a `CustomTransport` instance.
     * @param[in] data Serialized XRCE bytes to transmit.
     * @param[in] length Number of bytes available in `data`.
     * @param[out] error Transport-specific error byte written by the adapter
     * when a write fails.
     * @return Number of bytes written to the transport.
     */
    static size_t write_transport(::uxrCustomTransport* transport,
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

    /**
     * @brief Reads an XRCE frame through the Rosette custom transport adapter.
     *
     * This callback is part of the micro-ROS custom transport table. It asks
     * the board adapter to wait up to the requested timeout and copy received
     * bytes into the caller-provided buffer.
     *
     * @param[in,out] transport XRCE custom transport handle whose `args` field
     * points at a `CustomTransport` instance.
     * @param[out] data Buffer that receives XRCE bytes from the transport.
     * @param[in] length Maximum number of bytes that may be written to `data`.
     * @param[in] timeout_ms Maximum time to wait for incoming data.
     * @param[out] error Transport-specific error byte written by the adapter
     * when a read fails.
     * @return Number of bytes read from the transport.
     */
    static size_t read_transport(::uxrCustomTransport* transport,
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
#endif

    static constexpr size_t kMaxActionTickables = 8u;

    ContextOptions options_{};
    rcl_allocator_t allocator_{};
    rcl_init_options_t init_options_ = rcl_get_zero_initialized_init_options();
    rclc_support_t support_{};
    rclc_executor_t executor_ = rclc_executor_get_zero_initialized_executor();

    IActionTickable* action_tickables_[kMaxActionTickables] = {};
    size_t action_tickable_count_ = 0u;

    bool started_ = false;
    bool init_options_initialized_ = false;
    bool support_initialized_ = false;
    bool executor_initialized_ = false;
    ContextStartError last_start_error_ = ContextStartError::kOk;
};

}  // namespace milkros::micro_ros
