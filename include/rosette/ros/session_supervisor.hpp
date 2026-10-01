#pragma once

/**
 * @file ros/session_supervisor.hpp
 * @brief Keeps an rclc application's session matched to the network: wait, create, run, destroy.
 *
 * A firmware's ROS session (rcl support, node, entities, executor) is built
 * on the network as it stood at creation: rmw_rosette advertises the IPv4
 * address it read in rmw_init() as its locators, and the micro-ROS agent
 * backend holds a session with one agent. When that address moves (a
 * link-local address upgraded to a DHCP lease) or disappears (cable out,
 * lease lost), or the agent stops answering, the session has to be torn down
 * and built again. `SessionSupervisor` is that loop, the same for both
 * backends:
 *
 *     WAIT_NETWORK --(ready, gate reachable)--> CREATE_SESSION --(create ok)--> RUNNING
 *          ^                                          |                            |
 *          |                                    create failed     address moved, lost past
 *          |                                          v           the grace, or far end gone
 *          +---------------------------------- DESTROY_SESSION <-------------------+
 *
 * The network backend is serviced in every state, so DHCP renewal, SLAAC,
 * and carrier supervision never stop while the session is down.
 *
 * The application supplies two functions, as in the micro-ROS reconnection
 * pattern: `create` builds every rcl object the application uses and
 * returns false if any step fails; `destroy` finalizes them and must accept
 * a partly built session, because it also cleans up after a failed `create`.
 * rclc's `*_fini` calls on zero-initialized objects return an error and
 * change nothing, so an application that zero-initializes its objects at
 * the start of `create` gets that for free.
 *
 * A backend whose session has a far end supplies a `SessionGate`: the
 * micro-ROS agent backend discovers and pings its agent before a session is
 * created and keeps pinging while it runs. rmw_rosette has no far end and
 * passes no gate.
 *
 *     rosette::ros::SessionSupervisor session(net::board_network_watch(), {create, destroy});
 *     for (;;) {
 *         if (session.step(platform::monotonic_ms()) == rosette::ros::SessionState::running) {
 *             rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
 *         } else {
 *             platform::sleep_ms(10);
 *         }
 *     }
 *
 * Concurrency: step() and the application's functions run on the one thread
 * that owns the session; nothing here is interrupt-safe.
 */

#include <array>
#include <cstdint>

#include "rosette/net/network_watch.hpp"

namespace rosette::ros {

/// Where the supervisor is in the session's life.
enum class SessionState : uint8_t {
    wait_network,     ///< No session; waiting for the network and the gate.
    create_session,   ///< The next step() calls the application's `create`.
    running,          ///< The session is up; the application spins its executor.
    destroy_session,  ///< The next step() calls the application's `destroy`.
};

/// The application's session, as two plain functions (captureless lambdas work).
struct SessionCallbacks {
    /// Builds the session; false when any step failed (`destroy` then cleans up).
    bool (*create)();
    /// Finalizes the session, including one `create` built only in part.
    void (*destroy)();
};

/**
 * @brief A backend's check that the far end of its session is there.
 *
 * Both functions pace themselves from `now_ms` (for instance one agent ping
 * per period) and may block for at most one short reply timeout per call.
 */
class SessionGate {
public:
    /// @brief Whether a session can be created now (the agent is found and answering).
    virtual bool reachable(uint32_t now_ms) noexcept = 0;

    /// @brief Whether the running session's far end is still there.
    virtual bool alive(uint32_t now_ms) noexcept = 0;

protected:
    ~SessionGate() = default;
};

/// Timing of the supervisor's retries and tolerances.
struct SupervisorPolicy {
    /// After a failed `create` or an unreachable gate, how long to wait before trying again.
    uint32_t create_retry_ms = 1000u;
    /// How long the session survives the loss of its address (or of `ready`)
    /// before it is destroyed; the same address coming back in time keeps it.
    uint32_t address_loss_grace_ms = 5000u;
};

/// Runs an application's ROS session on a network, rebuilding it when the network moves under it.
class SessionSupervisor {
public:
    /**
     * @brief A supervisor in WAIT_NETWORK, with no session.
     *
     * @param[in] network The backend to service and watch; its functions must
     *   stay callable for the supervisor's life.
     * @param[in] app     The application's create and destroy functions; both non-null.
     * @param[in] gate    The backend's far-end check, or null when the
     *   backend has none (rmw_rosette); must outlive the supervisor.
     * @param[in] policy  Retry and grace timing.
     */
    SessionSupervisor(const net::NetworkWatch& network, SessionCallbacks app,
                      SessionGate* gate = nullptr, SupervisorPolicy policy = {}) noexcept;

    /**
     * @brief Services the network, then takes at most one transition of the session state machine.
     *
     * Call from the main loop as often as the executor is spun. Every call
     * polls the network backend first, whatever the state, and then acts on
     * the state:
     *
     * - WAIT_NETWORK: once `ready()` holds, the retry time has passed, and
     *   the gate (if any) is reachable, moves to CREATE_SESSION.
     * - CREATE_SESSION: records the IPv4 address the session is bound to and
     *   calls `create`; success moves to RUNNING, failure calls `destroy`
     *   and returns to WAIT_NETWORK for `create_retry_ms`.
     * - RUNNING: moves to DESTROY_SESSION when the gate reports the far end
     *   gone, when the board holds an IPv4 address other than the bound one,
     *   or when the bound address (or `ready()`) has been missing for
     *   `address_loss_grace_ms`.
     * - DESTROY_SESSION: calls `destroy` and returns to WAIT_NETWORK.
     *
     * @param[in] now_ms Monotonic time in milliseconds (platform::monotonic_ms()).
     * @return The state after this step; the application spins its executor
     *   only while it is `running`.
     *
     * Verification:
     * - Correct means: `create` runs only while `ready()` holds and the gate
     *   is reachable; a session always runs bound to the address the board
     *   holds, except for at most `address_loss_grace_ms` after losing it;
     *   every `create` is paired with exactly one `destroy`; and the network
     *   is polled on every step in every state.
     * - Covered by: tests/host/session_supervisor_test.cpp (TC-ROS-SUP-001 to
     *   TC-ROS-SUP-006); examples/rclc_chatter on the W6100-EVB-Pico against
     *   stock Fast DDS (the session rebuilt after an address conflict).
     *
     * References:
     * - micro-ROS "Handle reconnections and liveliness" tutorial (the
     *   WAITING_AGENT / AGENT_AVAILABLE / AGENT_CONNECTED / AGENT_DISCONNECTED
     *   pattern with create_entities() and destroy_entities())
     * - OMG DDSI-RTPS 2.5 §8.5.3.2 (a participant's announced unicast locators)
     */
    SessionState step(uint32_t now_ms) noexcept;

    /// @brief The current state.
    SessionState state() const noexcept { return state_; }

    /// @brief Sessions created successfully since construction.
    uint32_t sessions_created() const noexcept { return sessions_created_; }

    /// @brief The IPv4 address the current (or last) session was created on; zero for none.
    const std::array<uint8_t, 4>& session_address() const noexcept { return bound_v4_; }

private:
    void wait_for_network(uint32_t now_ms) noexcept;
    void create_session(uint32_t now_ms) noexcept;
    void supervise_session(uint32_t now_ms) noexcept;

    net::NetworkWatch network_;
    SessionCallbacks app_;
    SessionGate* gate_;
    SupervisorPolicy policy_;

    SessionState state_ = SessionState::wait_network;
    std::array<uint8_t, 4> bound_v4_{};
    uint32_t next_attempt_ms_ = 0u;
    uint32_t lost_since_ms_ = 0u;
    bool losing_ = false;
    uint32_t sessions_created_ = 0u;
};

}  // namespace rosette::ros
