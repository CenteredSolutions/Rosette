/**
 * @file tests/agent/agent_live_probe.cpp
 * @brief A stock rclc application on the micro-ROS agent backend, run on a host against a real
 * agent.
 *
 * This is the firmware's agent path with only the socket backend swapped:
 * rclc and rmw_microxrcedds talk XRCE-DDS through Rosette's
 * `UdpSocketTransport` (over `PosixSocketProvider` instead of the WIZnet
 * chip), `AgentSessionGate` pings the agent and keeps the clock, and
 * `SessionSupervisor` builds and rebuilds the session. The far side is all
 * stock: a `micro_ros_agent` joins the DDS graph for the probe, and
 * agent_echo_peer.py (rclpy on a stock RMW) echoes its messages and calls
 * its service. run_agent_live.sh starts both and runs this probe.
 *
 * Phases, each a test case:
 *
 * - TC-ROS-AGT-LIVE-001: the session comes up, the wall clock (set to
 *   1970 at start) is synchronized with the agent's host, and `--count`
 *   messages round-trip through the stock peer while the peer's SetBool
 *   calls are served.
 * - TC-ROS-AGT-LIVE-002: `--stop-cmd` stops the agent; the gate reports it
 *   gone within its ping budget, and the clock stays synchronized.
 * - TC-ROS-AGT-LIVE-003: `--start-cmd` restarts the agent; the supervisor
 *   builds a second session and traffic round-trips again.
 *
 * Exit status 0 only when every phase passed.
 *
 * References:
 * - micro-ROS "Handle reconnections and liveliness" tutorial
 * - rclc (ros2/rclc, Humble) executor and support API
 * - micro-ROS rmw_microxrcedds `rmw_microros/time_sync.h` (Humble)
 */

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>

#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/string.h>
#include <std_srvs/srv/set_bool.h>

#include "rosette/net/address_state.hpp"
#include "rosette/net/network_watch.hpp"
#include "rosette/net/posix/socket.hpp"
#include "rosette/platform/platform.hpp"
#include "rosette/platform/wall_clock.hpp"
#include "rosette/ros/agent_session_gate.hpp"
#include "rosette/ros/session_supervisor.hpp"
#include "rosette/ros/socket_transport.hpp"
#include "rosette/ros/time_sync.hpp"

namespace {

namespace pf = rosette::platform;
namespace ros = rosette::ros;

/// Command-line options; see run_agent_live.sh.
struct Options {
    const char* agent_ip = "127.0.0.1";
    const char* agent_port = "8888";
    const char* session = "agent_live";
    const char* stop_cmd = nullptr;
    const char* start_cmd = nullptr;
    uint32_t count = 5u;
    int domain = 42;
};

Options g_options;

rcl_allocator_t g_allocator;
rclc_support_t g_support;
rcl_node_t g_node;
rcl_publisher_t g_publisher;
rcl_subscription_t g_subscription;
rcl_service_t g_service;
rclc_executor_t g_executor;

std_msgs__msg__String g_outgoing;
char g_outgoing_text[64];
std_msgs__msg__String g_incoming;
char g_incoming_text[64];
std_srvs__srv__SetBool_Request g_request;
std_srvs__srv__SetBool_Response g_response;
char g_response_text[16] = "echo";

uint32_t g_sent = 0u;
uint32_t g_echoes = 0u;
uint32_t g_calls_served = 0u;
bool g_all_passed = true;

/// @brief Prints a failed rcl call and returns false; true for RCL_RET_OK.
bool ok(rcl_ret_t ret, const char* what) noexcept {
    if (ret == RCL_RET_OK) {
        return true;
    }

    std::printf("[probe] %s failed: %d (%s)\n", what, static_cast<int>(ret),
                rcl_get_error_string().str);
    rcl_reset_error();
    return false;
}

/// @brief Records one test case's verdict.
void verdict(const char* tc, bool passed, const char* what) noexcept {
    std::printf("[probe] %s %s: %s\n", tc, passed ? "PASS" : "FAIL", what);
    g_all_passed = g_all_passed && passed;
}

/// @brief Subscription callback: counts an echo of a message this probe sent.
void on_echo(const void* msg) {
    const auto* s = static_cast<const std_msgs__msg__String*>(msg);
    if (s->data.size > 0u && std::strncmp(s->data.data, "probe #", 7) == 0) {
        ++g_echoes;
    }
}

/// @brief SetBool service: replies `success = request.data`, which the peer checks.
void on_set_bool(const void* request, void* response) {
    const auto* req = static_cast<const std_srvs__srv__SetBool_Request*>(request);
    auto* res = static_cast<std_srvs__srv__SetBool_Response*>(response);
    res->success = req->data;
    res->message.data = g_response_text;
    res->message.size = std::strlen(g_response_text);
    res->message.capacity = sizeof(g_response_text);
    ++g_calls_served;
}

/**
 * @brief Builds the probe's rclc session in `--domain`: node, publisher, subscription, service.
 *
 * The init options come from `rcl_init_options_init()`, which copies
 * rmw_microxrcedds's process-wide default transport, the one
 * `AgentSessionGate` installed; so this session speaks through the Rosette
 * socket transport without naming it.
 *
 * @return True when every object was created.
 */
bool create_session() {
    g_support = rclc_support_t{};
    g_node = rcl_get_zero_initialized_node();
    g_publisher = rcl_get_zero_initialized_publisher();
    g_subscription = rcl_get_zero_initialized_subscription();
    g_service = rcl_get_zero_initialized_service();
    g_executor = rclc_executor_get_zero_initialized_executor();
    g_allocator = rcl_get_default_allocator();

    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
    if (!ok(rcl_init_options_init(&init_options, g_allocator), "rcl_init_options_init")) {
        return false;
    }

    const bool built =
        ok(rcl_init_options_set_domain_id(&init_options, static_cast<size_t>(g_options.domain)),
           "rcl_init_options_set_domain_id")
        && ok(rclc_support_init_with_options(&g_support, 0, nullptr, &init_options, &g_allocator),
              "rclc_support_init_with_options")
        && ok(rclc_node_init_default(&g_node, "agent_live_probe", g_options.session, &g_support),
              "rclc_node_init_default")
        && ok(rclc_publisher_init_default(&g_publisher, &g_node,
                                          ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
                                          "board_to_peer"),
              "rclc_publisher_init_default")
        && ok(rclc_subscription_init_default(&g_subscription, &g_node,
                                             ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
                                             "peer_to_board"),
              "rclc_subscription_init_default")
        && ok(rclc_service_init_default(&g_service, &g_node,
                                        ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, SetBool),
                                        "board_echo"),
              "rclc_service_init_default")
        && ok(rclc_executor_init(&g_executor, &g_support.context, 2, &g_allocator),
              "rclc_executor_init")
        && ok(rclc_executor_add_subscription(&g_executor, &g_subscription, &g_incoming, on_echo,
                                             ON_NEW_DATA),
              "rclc_executor_add_subscription")
        && ok(rclc_executor_add_service(&g_executor, &g_service, &g_request, &g_response,
                                        on_set_bool),
              "rclc_executor_add_service");

    // rcl marks this warn_unused_result, which a (void) cast does not satisfy in GCC.
    const rcl_ret_t discarded = rcl_init_options_fini(&init_options);
    (void)discarded;
    return built;
}

/// @brief Finalizes whatever create_session() built, in reverse order.
void destroy_session() {
    rcl_ret_t discarded = rclc_executor_fini(&g_executor);
    discarded = rcl_service_fini(&g_service, &g_node);
    discarded = rcl_subscription_fini(&g_subscription, &g_node);
    discarded = rcl_publisher_fini(&g_publisher, &g_node);
    discarded = rcl_node_fini(&g_node);
    discarded = rclc_support_fini(&g_support);
    (void)discarded;
    rcl_reset_error();
}

/// @brief The host's network, as the supervisor sees it: always ready, no address to track.
rosette::net::NetworkWatch host_network() noexcept {
    static const rosette::net::AddressSnapshot kNoAddress{};
    return rosette::net::NetworkWatch{
        [](uint32_t) {},
        []() { return true; },
        []() -> const rosette::net::AddressSnapshot& { return kNoAddress; },
    };
}

/// @brief One supervisor step, then a spin (running) or a short sleep (otherwise).
ros::SessionState step(ros::SessionSupervisor& session) noexcept {
    const ros::SessionState state = session.step(pf::monotonic_ms());
    if (state == ros::SessionState::running) {
        (void)rclc_executor_spin_some(&g_executor, RCL_MS_TO_NS(20));
    } else {
        pf::sleep_ms(10u);
    }

    return state;
}

/// @brief Publishes one numbered probe message while the session runs.
void publish_probe() noexcept {
    std::snprintf(g_outgoing_text, sizeof(g_outgoing_text), "probe #%lu",
                  static_cast<unsigned long>(++g_sent));
    g_outgoing.data.size = std::strlen(g_outgoing_text);
    (void)ok(rcl_publish(&g_publisher, &g_outgoing, nullptr), "rcl_publish");
}

/**
 * @brief Steps the session until it runs, then until `count` echoes and served calls arrive.
 *
 * Publishes one message every 200 ms while the session runs. The counters
 * are reset first, so only this phase's traffic counts.
 *
 * @param[in] session    The supervisor.
 * @param[in] timeout_ms How long the phase may take.
 * @return True when both counts were reached in time.
 */
bool run_traffic(ros::SessionSupervisor& session, uint32_t timeout_ms) noexcept {
    g_echoes = 0u;
    g_calls_served = 0u;
    const uint32_t deadline = pf::monotonic_ms() + timeout_ms;
    uint32_t next_publish = pf::monotonic_ms();
    while (static_cast<int32_t>(pf::monotonic_ms() - deadline) < 0) {
        if (step(session) == ros::SessionState::running
            && static_cast<int32_t>(pf::monotonic_ms() - next_publish) >= 0) {
            next_publish += 200u;
            publish_probe();
        }

        if (g_echoes >= g_options.count && g_calls_served >= g_options.count) {
            return true;
        }
    }

    std::printf("[probe] traffic timed out: echoes=%lu served=%lu sent=%lu\n",
                static_cast<unsigned long>(g_echoes), static_cast<unsigned long>(g_calls_served),
                static_cast<unsigned long>(g_sent));
    return false;
}

/// @brief Host wall clock in milliseconds since the Unix epoch.
int64_t host_epoch_ms() noexcept {
    timespec ts{};
    (void)clock_gettime(CLOCK_REALTIME, &ts);
    return static_cast<int64_t>(ts.tv_sec) * 1000 + ts.tv_nsec / 1000000;
}

/// @brief Whether the synchronized clock agrees with the host clock to within 50 ms.
bool clock_matches_host() noexcept {
    const int64_t skew_ms = ros::epoch_millis() - host_epoch_ms();
    std::printf("[probe] time_synced=%d skew=%lld ms\n", ros::time_synced() ? 1 : 0,
                static_cast<long long>(skew_ms));
    return ros::time_synced() && skew_ms > -50 && skew_ms < 50;
}

/**
 * @brief Stops the agent and steps until the supervisor leaves RUNNING.
 *
 * @param[in] session The supervisor, in RUNNING.
 * @param[in] budget_ms Longest acceptable detection time.
 * @return True when the session was torn down within `budget_ms`.
 */
bool run_outage(ros::SessionSupervisor& session, uint32_t budget_ms) noexcept {
    if (std::system(g_options.stop_cmd) != 0) {
        std::printf("[probe] stop command failed: %s\n", g_options.stop_cmd);
        return false;
    }

    const uint32_t stopped_at = pf::monotonic_ms();
    while (pf::monotonic_ms() - stopped_at < budget_ms) {
        if (step(session) != ros::SessionState::running) {
            std::printf("[probe] agent loss detected after %lu ms\n",
                        static_cast<unsigned long>(pf::monotonic_ms() - stopped_at));
            return true;
        }
    }

    return false;
}

/// @brief Parses `--name value` options into g_options; false on an unknown one.
bool parse_options(int argc, char** argv) noexcept {
    for (int i = 1; i + 1 < argc; i += 2) {
        const char* name = argv[i];
        const char* value = argv[i + 1];
        if (std::strcmp(name, "--agent-ip") == 0) {
            g_options.agent_ip = value;
        } else if (std::strcmp(name, "--agent-port") == 0) {
            g_options.agent_port = value;
        } else if (std::strcmp(name, "--session") == 0) {
            g_options.session = value;
        } else if (std::strcmp(name, "--stop-cmd") == 0) {
            g_options.stop_cmd = value;
        } else if (std::strcmp(name, "--start-cmd") == 0) {
            g_options.start_cmd = value;
        } else if (std::strcmp(name, "--count") == 0) {
            g_options.count = static_cast<uint32_t>(std::strtoul(value, nullptr, 10));
        } else if (std::strcmp(name, "--domain") == 0) {
            g_options.domain = std::atoi(value);
        } else {
            std::printf("[probe] unknown option %s\n", name);
            return false;
        }
    }

    return (argc % 2) == 1;
}

}  // namespace

int main(int argc, char** argv) {
    if (!parse_options(argc, argv)) {
        return 2;
    }

    g_outgoing.data.data = g_outgoing_text;
    g_outgoing.data.capacity = sizeof(g_outgoing_text);
    g_incoming.data.data = g_incoming_text;
    g_incoming.data.capacity = sizeof(g_incoming_text);

    static rosette::net::posix::PosixSocketProvider sockets;
    const rosette::net::SocketAddress any = rosette::net::make_socket_address_v4({0, 0, 0, 0}, 0u);
    static rosette::UdpSocketTransport transport(sockets, any, any, 50u);
    static rosette::StaticAgentLocator locator(g_options.agent_ip, g_options.agent_port);
    static ros::AgentSessionGate gate(transport, locator, ros::rmw_agent_link());
    ros::SessionSupervisor session(host_network(), {create_session, destroy_session}, &gate);

    // A host's wall clock starts on the OS clock; put it back to 1970 so only
    // the agent's time exchange can bring it within the skew check.
    pf::WallClock::set_offset_us(0);

    // Default gate policy: 3 missed pings 2 s apart, each a 200 ms timeout.
    constexpr uint32_t kLossBudgetMs = 3u * 2000u + 3u * 200u + 1500u;

    const bool traffic = run_traffic(session, 30000u);
    verdict("TC-ROS-AGT-LIVE-001", traffic && clock_matches_host(),
            "session up, clock synced to the agent host, messages and calls round-trip");

    if (g_options.stop_cmd != nullptr && g_options.start_cmd != nullptr) {
        const bool lost = run_outage(session, kLossBudgetMs);
        verdict("TC-ROS-AGT-LIVE-002", lost && clock_matches_host(),
                "agent loss detected within the ping budget; clock still synced");

        const bool restarted = std::system(g_options.start_cmd) == 0;
        const bool rebuilt = restarted && run_traffic(session, 30000u);
        verdict("TC-ROS-AGT-LIVE-003", rebuilt && session.sessions_created() == 2u,
                "second session built after the agent returned; traffic round-trips");
    }

    if (session.state() == ros::SessionState::running) {
        destroy_session();
    }

    return g_all_passed ? 0 : 1;
}
