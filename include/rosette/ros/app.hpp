/**
 * @file ros/app.hpp
 * @brief `App` owns the micro-ROS session and drives a list of `Module`s.
 *
 * This is the layer that makes a ROS firmware trivial: construct an `App`, add
 * modules, call `spin()`. `App` runs the Agent connection state machine —
 * resolve/ping the Agent, build the context and node, register every module's
 * entities, spin the executor, detect Agent loss, tear down, and rebuild — so
 * application code never hand-rolls it.
 *
 *   rosette::App app({ .node_name = "controller",
 *                      .node_namespace = rosette::board_scoped_ns("mach") });
 *   static DeviceModule device;
 *   static MotionModule motion;
 *   app.add(device);
 *   app.add(motion);
 *   app.spin();                 // never returns
 *
 * On a bare loop or a FreeRTOS task, call `spin_once(now_ms)` yourself instead
 * of `spin()`.
 *
 * The executor is sized from the sum of `Module::executor_handles()` plus
 * `AppOptions::extra_executor_handles`, so applications do not hand-count slots.
 */

#pragma once

#include "rosette/ros/module.hpp"
#include "rosette/ros/node.hpp"
#include "rosette/ros/time_sync.hpp"

#include <cstdio>

#include "rosette/platform/host_runtime.hpp"

namespace rosette {

/// Connection state reported by `App::state()`.
enum class AppState : uint8_t {
    waiting_agent,   ///< No session; retrying on `connect_retry_ms`.
    connected,       ///< Session up; modules registered and publishing.
};

/**
 * @brief Configuration for an `App`.
 */
struct AppOptions {
    ContextOptions context{};          ///< Transport, domain, client key, agent endpoint.
    const char* node_name = "rosette";
    const char* node_namespace = "";

    uint32_t connect_retry_ms = 1000u;        ///< Delay between connection attempts.
    uint32_t agent_ping_period_ms = 2000u;    ///< Liveness ping interval once connected.
    int agent_ping_timeout_ms = 200;          ///< Per-ping reply timeout.
    uint8_t agent_ping_failures = 3u;         ///< Consecutive failures before declaring loss.

    bool sync_time_on_connect = true;         ///< Sync the epoch clock with the Agent on each connect.
    uint32_t time_resync_period_ms = 32u;     ///< Re-sync interval while connected; 0 = only on connect.
    int time_sync_timeout_ms = 200;           ///< Per-sync reply timeout.

    uint8_t extra_executor_handles = 2u;      ///< Headroom added to the module handle sum.
};

/**
 * @brief The micro-ROS session and its module list.
 */
class App {
public:
    static constexpr uint8_t kMaxModules = 12u;

    explicit App(const AppOptions& options = {}) noexcept
        : options_(options)
        , context_(options.context)
    {}

    App(const App&) = delete;
    App& operator=(const App&) = delete;

    /**
     * @brief Registers a module.
     *
     * The module is referenced, not owned — give it static or `App`-outliving
     * storage. Modules run in the order added.
     *
     * @param[in] module Module to add.
     * @return `true` when the module was added; `false` when the list is full.
     */
    bool add(Module& module) noexcept
    {
        if (module_count_ >= kMaxModules) {
            return false;
        }
        modules_[module_count_] = &module;
        last_publish_ms_[module_count_] = 0u;
        ++module_count_;
        return true;
    }

    /**
     * @brief Advances the state machine once.
     *
     * Call from a task loop or bare `for(;;)`. While `waiting_agent`, retries a
     * connection every `connect_retry_ms`. While `connected`, spins the
     * executor, runs due `Module::publish()` calls, and pings the Agent on
     * `agent_ping_period_ms` — dropping to `waiting_agent` after
     * `agent_ping_failures` misses.
     *
     * @param[in] now_ms Monotonic millisecond timestamp
     * (`rosette_platform_get_ms()`).
     * @return `true` while connected.
     */
    bool spin_once(uint32_t now_ms) noexcept
    {
        if (state_ == AppState::waiting_agent) {
            if (static_cast<int32_t>(now_ms - next_attempt_ms_) < 0) {
                return false;
            }
            next_attempt_ms_ = now_ms + options_.connect_retry_ms;
            if (build()) {
                state_ = AppState::connected;
                last_ping_ms_ = now_ms;
                last_time_sync_ms_ = now_ms;
                ping_failures_ = 0u;
                if (options_.sync_time_on_connect) {
                    (void)sync_time(options_.time_sync_timeout_ms);
                }
                for (uint8_t i = 0; i < module_count_; ++i) {
                    modules_[i]->on_connect();
                    last_publish_ms_[i] = now_ms;
                }
            }
            return state_ == AppState::connected;
        }

        // connected
        (void)context_.tick(now_ms);

        for (uint8_t i = 0; i < module_count_; ++i) {
            const uint32_t period = modules_[i]->period_ms();
            if (period != 0u &&
                static_cast<int32_t>(now_ms - (last_publish_ms_[i] + period)) >= 0) {
                last_publish_ms_[i] = now_ms;
                modules_[i]->publish();
            }
        }

        if (options_.time_resync_period_ms != 0u &&
            static_cast<int32_t>(now_ms - (last_time_sync_ms_ + options_.time_resync_period_ms)) >= 0) {
            last_time_sync_ms_ = now_ms;
            (void)sync_time(options_.time_sync_timeout_ms);
        }

        if (static_cast<int32_t>(now_ms - (last_ping_ms_ + options_.agent_ping_period_ms)) >= 0) {
            last_ping_ms_ = now_ms;
            const bool alive =
                ros_ok(rmw_uros_ping_agent(options_.agent_ping_timeout_ms, 1));
            if (alive) {
                ping_failures_ = 0u;
            } else if (++ping_failures_ >= options_.agent_ping_failures) {
                teardown();
                state_ = AppState::waiting_agent;
                next_attempt_ms_ = now_ms + options_.connect_retry_ms;
            }
        }

        return state_ == AppState::connected;
    }

    /**
     * @brief Runs `spin_once` forever, pacing with `rosette_platform_delay_ms`.
     */
    [[noreturn]] void spin() noexcept
    {
        for (;;) {
            (void)spin_once(rosette_platform_get_ms());
            rosette_platform_delay_ms(state_ == AppState::connected ? 10u : 200u);
        }
    }

    AppState state() const noexcept { return state_; }

    /// Whether the epoch clock is currently synchronized with the Agent.
    bool time_synced() const noexcept { return rosette::time_synced(); }
    bool connected() const noexcept { return state_ == AppState::connected; }
    Context& context() noexcept { return context_; }
    Node& node() noexcept { return node_; }

private:
    /**
     * @brief Starts the context, creates the node, and registers modules.
     *
     * The executor is sized from the module handle sum before the context
     * starts. A failure at any step leaves the App in `waiting_agent`.
     *
     * @return `true` when the session is up and every module is registered.
     */
    bool build() noexcept
    {
        size_t handles = options_.extra_executor_handles;
        for (uint8_t i = 0; i < module_count_; ++i) {
            handles += modules_[i]->executor_handles();
        }
        context_.options_mutable().executor_handles = handles;

        if (!context_.start_result().ok()) {
            std::printf("[app] context start failed: %s\n",
                        context_start_error_string(context_.last_start_error()));
            context_.fini();
            return false;
        }

        node_ = context_.create_node(options_.node_name, options_.node_namespace);
        if (!node_.valid()) {
            std::printf("[app] node '%s' (ns '%s') creation failed\n",
                        options_.node_name != nullptr ? options_.node_name : "",
                        options_.node_namespace != nullptr ? options_.node_namespace : "");
            context_.fini();
            return false;
        }

        for (uint8_t i = 0; i < module_count_; ++i) {
            modules_[i]->register_(node_);
        }
        return true;
    }

    /**
     * @brief Notifies modules of the loss and destroys the session.
     */
    void teardown() noexcept
    {
        for (uint8_t i = 0; i < module_count_; ++i) {
            modules_[i]->on_disconnect();
        }
        node_ = Node{};
        context_.fini();
    }

    AppOptions options_;
    Context context_;
    Node node_{};

    Module* modules_[kMaxModules] = {};
    uint32_t last_publish_ms_[kMaxModules] = {};
    uint8_t module_count_ = 0u;

    AppState state_ = AppState::waiting_agent;
    uint32_t next_attempt_ms_ = 0u;
    uint32_t last_ping_ms_ = 0u;
    uint32_t last_time_sync_ms_ = 0u;
    uint8_t ping_failures_ = 0u;
};

}  // namespace rosette
