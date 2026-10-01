#pragma once

/**
 * @file ros/agent_session_gate.hpp
 * @brief The micro-ROS agent backend's SessionGate: find the agent, keep pinging it, keep time.
 *
 * With the micro-ROS agent backend, the board's rcl session is an XRCE-DDS
 * session with one `micro_ros_agent`, which joins the DDS graph on the
 * board's behalf. A session can only be created while that agent answers,
 * and it is dead once the agent stops answering, whatever the board's own
 * network looks like. `AgentSessionGate` is the `SessionGate` that tells
 * `SessionSupervisor` both things:
 *
 * - `reachable()` (no session): resolve the agent's endpoint through an
 *   `AgentLocator` (a static address, mDNS, or the XRCE multicast probe),
 *   hand it to the Rosette socket transport the XRCE client speaks through,
 *   and ping the agent once.
 * - `alive()` (session running): ping the agent every `ping_period_ms` and
 *   report it gone after `ping_failures` consecutive misses; synchronize
 *   the epoch clock with the agent when the session comes up and every
 *   `time_resync_period_ms` after that (see ros/time_sync.hpp).
 *
 * The application's session is stock rclc: its `create` calls
 * `rclc_support_init()` with default options, because the gate installs
 * the socket transport as rmw_microxrcedds's process-wide default transport
 * (`rmw_uros_set_custom_transport()`) before the first ping, and
 * `rmw_init_options_init()` copies that default into every session.
 *
 *     static rosette::UdpSocketTransport transport(sockets, {}, any_local, 50u);
 *     static rosette::MdnsAgentLocator locator(sockets, "_microrosagent._udp.local");
 *     static rosette::ros::AgentSessionGate gate(transport, locator,
 *                                                rosette::ros::rmw_agent_link());
 *     rosette::ros::SessionSupervisor session(net::board_network_watch(),
 *                                             {create_session, destroy_session}, &gate);
 *
 * The gate itself calls no micro-ROS function: every agent operation goes
 * through an `AgentLink`, whose rmw_microxrcedds implementation
 * (`rmw_agent_link()`, src/ros/agent_rmw_link.cpp) lives in the
 * `rosette_agent` library. Host tests substitute a fake link.
 *
 * Concurrency: called from the thread that runs `SessionSupervisor::step()`;
 * not interrupt-safe.
 */

#include <cstdint>

#include "rosette/net/socket.hpp"
#include "rosette/ros/agent_locator.hpp"
#include "rosette/ros/custom_transport.hpp"
#include "rosette/ros/session_supervisor.hpp"

namespace rosette::ros {

/// The agent operations the gate performs, as plain functions over the XRCE client.
struct AgentLink {
    /// Installs `transport` as the XRCE client's transport for every later ping and session.
    bool (*attach)(CustomTransport& transport);
    /// One ping round trip, over the session when one exists, else over a transient transport.
    bool (*ping)(int timeout_ms);
    /// One time-synchronization round trip over the running session.
    bool (*sync_time)(int timeout_ms);
};

/**
 * @brief The rmw_microxrcedds implementation of `AgentLink`.
 *
 * `attach` registers the transport with `rmw_uros_set_custom_transport()`
 * (unframed: the transport carries whole datagrams), `ping` is
 * `rmw_uros_ping_agent(timeout_ms, 1)`, and `sync_time` is one
 * `rmw_uros_sync_session(timeout_ms)` exchange whose offset feeds
 * ros/time_sync.hpp; a failed call leaves no rcl or rmw error state behind.
 *
 * @return The link; defined only in the `rosette_agent` library.
 *
 * Verification:
 * - Correct means: after `attach`, a ping with no session reaches the agent
 *   through the given transport, and `rclc_support_init()` with default
 *   options creates its session over the same transport.
 * - Covered by: examples/led_files against a stock `micro_ros_agent udp4`.
 *
 * References:
 * - micro-ROS rmw_microxrcedds `rmw_microros/custom_transport.h`,
 *   `rmw_microros/ping.h`, `rmw_microros/time_sync.h` (Humble)
 */
AgentLink rmw_agent_link() noexcept;

/// Timing of the gate's pings and time synchronization.
struct AgentGatePolicy {
    /// Reply timeout of each ping, which bounds how long one gate call blocks.
    int ping_timeout_ms = 200;
    /// Interval between liveness pings while the session runs.
    uint32_t ping_period_ms = 2000u;
    /// Consecutive missed pings after which the agent is reported gone (at least 1).
    uint8_t ping_failures = 3u;
    /// Reply timeout of each time-synchronization exchange.
    int time_sync_timeout_ms = 200;
    /// Interval between resynchronizations while the session runs; 0 syncs once per session.
    uint32_t time_resync_period_ms = 60000u;
};

/// The micro-ROS agent backend's far-end check for `SessionSupervisor`.
class AgentSessionGate final : public SessionGate {
public:
    /**
     * @brief A gate that has not yet located the agent.
     *
     * @param[in] transport The Rosette socket transport the XRCE client
     *   speaks through; must outlive the gate and every session.
     * @param[in] locator   Where the agent's endpoint comes from (use
     *   `StaticAgentLocator` for a fixed address); must outlive the gate.
     * @param[in] link      The agent operations; `rmw_agent_link()` on firmware.
     * @param[in] policy    Ping and time-synchronization timing.
     */
    AgentSessionGate(CustomTransport& transport, AgentLocator& locator, AgentLink link,
                     AgentGatePolicy policy = {}) noexcept;

    /**
     * @brief Whether a session can be created now: the agent is located and answers a ping.
     *
     * The supervisor calls this in WAIT_NETWORK once the network is ready,
     * and again after its retry interval while it returns false. The first
     * call attaches the transport. An endpoint is resolved when the gate has
     * none, handed to the transport, and pinged once; a failed ping forgets
     * the endpoint, so the next call resolves again and follows an agent
     * that moved. A true return also arms `alive()` for a new session.
     *
     * @param[in] now_ms Monotonic time in milliseconds.
     * @return True when the agent answered; false when attaching the
     *   transport failed, the locator found nothing, the transport rejected
     *   the endpoint, or the ping went unanswered.
     *
     * Blocking: the locator's resolution (when the gate has no endpoint)
     * plus one ping timeout.
     *
     * Verification:
     * - Correct means: true only after a ping answered on the endpoint the
     *   transport now holds; the locator is consulted only when no endpoint
     *   is held or the last ping on it failed.
     * - Covered by: tests/host/agent_session_gate_test.cpp (TC-ROS-AGT-001,
     *   TC-ROS-AGT-002).
     *
     * References:
     * - micro-ROS "Handle reconnections and liveliness" tutorial (WAITING_AGENT)
     * - eProsima Micro XRCE-DDS client, `uxr_ping_agent_attempts()`
     */
    bool reachable(uint32_t now_ms) noexcept override;

    /**
     * @brief Whether the running session's agent still answers; also keeps the epoch clock synced.
     *
     * The supervisor calls this on every step while the session runs. The
     * first call after `reachable()` synchronizes the epoch clock; after
     * that, each call does at most one exchange: a resynchronization when
     * `time_resync_period_ms` has passed, else a ping when `ping_period_ms`
     * has passed, else nothing. Both exchanges are round trips with the
     * agent, so an answered synchronization counts as an answered ping and
     * an unanswered one as a miss; a failed synchronization is retried one
     * ping period later.
     *
     * @param[in] now_ms Monotonic time in milliseconds.
     * @return False once `ping_failures` consecutive exchanges went unanswered;
     *   true otherwise.
     *
     * Blocking: at most one ping or synchronization timeout per call.
     *
     * Verification:
     * - Correct means: exactly `ping_failures` consecutive unanswered
     *   exchanges report the agent gone and an answered one resets the
     *   count; exchanges are `ping_period_ms` apart; the clock is synced on
     *   the first call of every session and then every
     *   `time_resync_period_ms`, and a failed sync never starves the pings.
     * - Covered by: tests/host/agent_session_gate_test.cpp (TC-ROS-AGT-003
     *   to TC-ROS-AGT-005).
     *
     * References:
     * - micro-ROS "Handle reconnections and liveliness" tutorial (AGENT_CONNECTED)
     * - micro-ROS rmw_microxrcedds `rmw_uros_sync_session()` (Humble)
     */
    bool alive(uint32_t now_ms) noexcept override;

    /// @brief Whether the gate holds a located agent endpoint.
    bool located() const noexcept { return located_; }

    /// @brief The agent endpoint last handed to the transport.
    const net::SocketAddress& agent() const noexcept { return agent_; }

private:
    CustomTransport& transport_;
    AgentLocator& locator_;
    AgentLink link_;
    AgentGatePolicy policy_;

    net::SocketAddress agent_{};
    bool attached_ = false;
    bool located_ = false;
    bool session_fresh_ = false;
    bool sync_due_ = false;
    uint8_t missed_pings_ = 0u;
    uint32_t next_ping_ms_ = 0u;
    uint32_t next_sync_ms_ = 0u;
};

}  // namespace rosette::ros
