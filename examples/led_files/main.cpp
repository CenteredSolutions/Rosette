/**
 * @file examples/led_files/main.cpp
 * @brief Stock rclc services on a WIZnet board through a micro-ROS agent: an LED and a file store.
 *
 * The firmware is an ordinary rclc application: the node, its five services
 * and the executor are built with the stock rclc API, and rmw_microxrcedds
 * carries them over XRCE-DDS to a `micro_ros_agent` on the LAN, which joins
 * the ROS 2 graph on the board's behalf. Rosette supplies what is below and
 * around rclc:
 *
 *   - the board network (DHCP, link-local fallback, SLAAC), brought up from
 *     `net::board_default_network()`;
 *   - the XRCE transport over the chip's hardware UDP sockets
 *     (`rosette::UdpSocketTransport`);
 *   - agent location: a fixed address, mDNS/DNS-SD, or the XRCE multicast
 *     probe, chosen at configure time;
 *   - `rosette::ros::AgentSessionGate`, which pings the agent before a
 *     session is created, keeps pinging while it runs, and syncs the epoch
 *     clock (ros/time_sync.hpp);
 *   - `rosette::ros::SessionSupervisor`, which builds the session when the
 *     network is ready and the agent answers, and rebuilds it when the
 *     address moves or the agent goes away;
 *   - the on-device file store (`rosette::storage::FileStore`).
 *
 * Services, under /led_files (rosette_msgs, this example's own package):
 *   ros2 service call /led_files/led/set    rosette_msgs/srv/SetLed    "{led: 0, state: true}"
 *   ros2 service call /led_files/files/put  rosette_msgs/srv/PutFile   "{key: k1, body: hello}"
 *   ros2 service call /led_files/files/get  rosette_msgs/srv/GetFile   "{key: k1}"
 *   ros2 service call /led_files/files/list rosette_msgs/srv/ListFiles "{}"
 *   ros2 service call /led_files/files/erase rosette_msgs/srv/EraseFile "{key: k1}"
 *
 * Agent on the host:
 *   docker run --rm --net=host microros/micro-ros-agent:humble udp4 --port 8888
 *
 * Verification: every service answers through a stock agent; stopping the
 * agent ends the session within three ping periods and starting it again
 * brings a new session up; the report line shows the epoch time matching the
 * host's `date +%s` after the first session; the heap in use returns to the
 * same value after every rebuild.
 *
 * References: rclc (ros2/rclc, Humble) service and executor API; micro-ROS
 * "Handle reconnections and liveliness" and "Custom transports" tutorials.
 */

#include <malloc.h>
#include <cstdio>
#include <cstring>

#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rosette_msgs/srv/erase_file.h>
#include <rosette_msgs/srv/get_file.h>
#include <rosette_msgs/srv/list_files.h>
#include <rosette_msgs/srv/put_file.h>
#include <rosette_msgs/srv/set_led.h>

#include "board.h"
#include "rosette/net/board_network.hpp"
#include "rosette/platform/platform.hpp"
#include "rosette/ros/agent_locator_mdns.hpp"
#include "rosette/ros/agent_locator_xrce.hpp"
#include "rosette/ros/agent_session_gate.hpp"
#include "rosette/ros/session_supervisor.hpp"
#include "rosette/ros/socket_transport.hpp"
#include "rosette/ros/time_sync.hpp"
#include "rosette/storage/file_store.hpp"
#include "rosette/storage/volume.hpp"

#include "pico/stdlib.h"

#define LED_FILES_STR2(x) #x
#define LED_FILES_STR(x) LED_FILES_STR2(x)

namespace {

namespace pf = rosette::platform;
namespace net = rosette::net;
namespace ros = rosette::ros;
namespace storage = rosette::storage;

constexpr uint32_t kReportPeriodMs = 5000u;
constexpr int64_t kSpinTimeoutNs = RCL_MS_TO_NS(50);
constexpr size_t kKeyCapacity = storage::kFileStoreKeyMax + 1u;
constexpr size_t kBodyCapacity = 4096u;
constexpr uint8_t kMaxListedKeys = 16u;
constexpr size_t kExecutorHandles = 5u;

storage::FileStore* g_store = nullptr;

rcl_allocator_t g_allocator;
rclc_support_t g_support;
rcl_node_t g_node;
rclc_executor_t g_executor;
rcl_service_t g_set_led_service;
rcl_service_t g_put_service;
rcl_service_t g_get_service;
rcl_service_t g_list_service;
rcl_service_t g_erase_service;

// Requests and responses live in static storage whose strings point at
// fixed buffers: the XRCE deserializer writes into caller-owned memory and
// allocates nothing, so every incoming string needs its capacity in advance.
rosette_msgs__srv__SetLed_Request g_set_led_request;
rosette_msgs__srv__SetLed_Response g_set_led_response;
rosette_msgs__srv__PutFile_Request g_put_request;
rosette_msgs__srv__PutFile_Response g_put_response;
rosette_msgs__srv__GetFile_Request g_get_request;
rosette_msgs__srv__GetFile_Response g_get_response;
rosette_msgs__srv__ListFiles_Request g_list_request;
rosette_msgs__srv__ListFiles_Response g_list_response;
rosette_msgs__srv__EraseFile_Request g_erase_request;
rosette_msgs__srv__EraseFile_Response g_erase_response;

char g_put_key[kKeyCapacity];
char g_put_body[kBodyCapacity];
char g_get_key[kKeyCapacity];
char g_get_body[kBodyCapacity];
char g_erase_key[kKeyCapacity];
char g_listed_key_text[kMaxListedKeys][kKeyCapacity];
rosidl_runtime_c__String g_listed_keys[kMaxListedKeys];

/// @brief Points a message string at a fixed buffer, empty.
void bind_string(rosidl_runtime_c__String& s, char* buffer, size_t capacity) noexcept {
    buffer[0] = '\0';
    s.data = buffer;
    s.size = 0u;
    s.capacity = capacity;
}

/// @brief Points a response string at constant text; serialization only reads it.
void set_text(rosidl_runtime_c__String& s, const char* text) noexcept {
    s.data = const_cast<char*>(text);
    s.size = std::strlen(text);
    s.capacity = s.size + 1u;
}

/// @brief Binds every request and response string to its buffer, once at boot.
void bind_messages() noexcept {
    bind_string(g_put_request.key, g_put_key, sizeof(g_put_key));
    bind_string(g_put_request.body, g_put_body, sizeof(g_put_body));
    bind_string(g_get_request.key, g_get_key, sizeof(g_get_key));
    bind_string(g_get_response.body, g_get_body, sizeof(g_get_body));
    bind_string(g_erase_request.key, g_erase_key, sizeof(g_erase_key));
    set_text(g_put_response.message, "");
    set_text(g_get_response.message, "");
    for (uint8_t i = 0; i < kMaxListedKeys; ++i) {
        bind_string(g_listed_keys[i], g_listed_key_text[i], kKeyCapacity);
    }

    g_list_response.keys.data = g_listed_keys;
    g_list_response.keys.size = 0u;
    g_list_response.keys.capacity = kMaxListedKeys;
}

/// @brief ~/led/set: drives the board LED (index 0 only).
void on_set_led(const void* request, void* response) {
    const auto* rq = static_cast<const rosette_msgs__srv__SetLed_Request*>(request);
    auto* rs = static_cast<rosette_msgs__srv__SetLed_Response*>(response);
    rs->success = rq->led == 0u;
    if (rs->success) {
        board_set_led(rq->state);
    }
}

/// @brief ~/files/put: stores the body under the key.
void on_put(const void* request, void* response) {
    const auto* rq = static_cast<const rosette_msgs__srv__PutFile_Request*>(request);
    auto* rs = static_cast<rosette_msgs__srv__PutFile_Response*>(response);
    const storage::FileStoreStatus st = g_store->put(rq->key.data, rq->body.data, rq->body.size);
    rs->success = st == storage::FileStoreStatus::ok;
    set_text(rs->message, storage::file_store_status_string(st));
}

/// @brief ~/files/get: returns the body stored under the key.
void on_get(const void* request, void* response) {
    const auto* rq = static_cast<const rosette_msgs__srv__GetFile_Request*>(request);
    auto* rs = static_cast<rosette_msgs__srv__GetFile_Response*>(response);
    size_t length = 0u;
    const storage::FileStoreStatus st =
        g_store->get(rq->key.data, rs->body.data, rs->body.capacity - 1u, length);
    rs->success = st == storage::FileStoreStatus::ok;
    rs->body.size = rs->success ? length : 0u;
    rs->body.data[rs->body.size] = '\0';
    set_text(rs->message, storage::file_store_status_string(st));
}

/// @brief ~/files/list: returns up to kMaxListedKeys stored keys.
void on_list(const void* /*request*/, void* response) {
    auto* rs = static_cast<rosette_msgs__srv__ListFiles_Response*>(response);
    const uint8_t count = g_store->list(g_listed_key_text, kMaxListedKeys);
    for (uint8_t i = 0; i < count; ++i) {
        g_listed_keys[i].size = std::strlen(g_listed_key_text[i]);
    }

    rs->keys.size = count;
}

/// @brief ~/files/erase: removes the key; succeeds when it is gone afterwards.
void on_erase(const void* request, void* response) {
    const auto* rq = static_cast<const rosette_msgs__srv__EraseFile_Request*>(request);
    auto* rs = static_cast<rosette_msgs__srv__EraseFile_Response*>(response);
    rs->success = g_store->erase(rq->key.data) == storage::FileStoreStatus::ok;
}

/**
 * @brief Reports a failed rcl call.
 *
 * @param[in] ret  The rcl return code.
 * @param[in] what The call that returned it.
 * @return True when `ret` is RCL_RET_OK.
 */
bool ok(rcl_ret_t ret, const char* what) noexcept {
    if (ret == RCL_RET_OK) {
        return true;
    }

    std::printf("[led_files] %s failed: %d (%s)\n", what, static_cast<int>(ret),
                rcl_get_error_string().str);
    rcl_reset_error();
    return false;
}

/**
 * @brief Builds the rclc session: support, node, five services, executor.
 *
 * The supervisor calls this once the network is ready and the agent answers
 * a ping, and again after every destroy_session(). The XRCE transport was
 * installed as the default by the agent gate, so support is initialized with
 * stock default options. Every object is zero-initialized first, so a
 * failure part way leaves destroy_session() finalizing only what was built.
 *
 * @return True when every object was created; false at the first failure (printed).
 *
 * Verification:
 * - Correct means: after a true return, each service answers through the
 *   agent; after a false one destroy_session() returns the heap to its
 *   level before the call.
 * - Covered by: the file's hardware check (agent stopped and restarted).
 *
 * References:
 * - rclc (ros2/rclc, Humble) rclc_support_init, rclc_service_init_default,
 *   rclc_executor_add_service
 */
bool create_session() {
    g_support = rclc_support_t{};
    g_node = rcl_get_zero_initialized_node();
    g_set_led_service = rcl_get_zero_initialized_service();
    g_put_service = rcl_get_zero_initialized_service();
    g_get_service = rcl_get_zero_initialized_service();
    g_list_service = rcl_get_zero_initialized_service();
    g_erase_service = rcl_get_zero_initialized_service();
    g_executor = rclc_executor_get_zero_initialized_executor();
    g_allocator = rcl_get_default_allocator();

    return ok(rclc_support_init(&g_support, 0, nullptr, &g_allocator), "rclc_support_init")
           && ok(rclc_node_init_default(&g_node, "board", "led_files", &g_support),
                 "rclc_node_init_default")
           && ok(rclc_service_init_default(&g_set_led_service, &g_node,
                                           ROSIDL_GET_SRV_TYPE_SUPPORT(rosette_msgs, srv, SetLed),
                                           "led/set"),
                 "service led/set")
           && ok(rclc_service_init_default(&g_put_service, &g_node,
                                           ROSIDL_GET_SRV_TYPE_SUPPORT(rosette_msgs, srv, PutFile),
                                           "files/put"),
                 "service files/put")
           && ok(rclc_service_init_default(&g_get_service, &g_node,
                                           ROSIDL_GET_SRV_TYPE_SUPPORT(rosette_msgs, srv, GetFile),
                                           "files/get"),
                 "service files/get")
           && ok(rclc_service_init_default(
                     &g_list_service, &g_node,
                     ROSIDL_GET_SRV_TYPE_SUPPORT(rosette_msgs, srv, ListFiles), "files/list"),
                 "service files/list")
           && ok(rclc_service_init_default(
                     &g_erase_service, &g_node,
                     ROSIDL_GET_SRV_TYPE_SUPPORT(rosette_msgs, srv, EraseFile), "files/erase"),
                 "service files/erase")
           && ok(
               rclc_executor_init(&g_executor, &g_support.context, kExecutorHandles, &g_allocator),
               "rclc_executor_init")
           && ok(rclc_executor_add_service(&g_executor, &g_set_led_service, &g_set_led_request,
                                           &g_set_led_response, on_set_led),
                 "add led/set")
           && ok(rclc_executor_add_service(&g_executor, &g_put_service, &g_put_request,
                                           &g_put_response, on_put),
                 "add files/put")
           && ok(rclc_executor_add_service(&g_executor, &g_get_service, &g_get_request,
                                           &g_get_response, on_get),
                 "add files/get")
           && ok(rclc_executor_add_service(&g_executor, &g_list_service, &g_list_request,
                                           &g_list_response, on_list),
                 "add files/list")
           && ok(rclc_executor_add_service(&g_executor, &g_erase_service, &g_erase_request,
                                           &g_erase_response, on_erase),
                 "add files/erase");
}

/**
 * @brief Finalizes whatever create_session() built, in reverse order.
 *
 * rcl's fini calls return an error and change nothing for an object that
 * was never initialized, so this is safe after a partial create. Their
 * errors are discarded: the objects are abandoned either way.
 */
void destroy_session() {
    // rcl marks these warn_unused_result, which a (void) cast does not satisfy in GCC.
    rcl_ret_t discarded = rclc_executor_fini(&g_executor);
    discarded = rcl_service_fini(&g_erase_service, &g_node);
    discarded = rcl_service_fini(&g_list_service, &g_node);
    discarded = rcl_service_fini(&g_get_service, &g_node);
    discarded = rcl_service_fini(&g_put_service, &g_node);
    discarded = rcl_service_fini(&g_set_led_service, &g_node);
    discarded = rcl_node_fini(&g_node);
    discarded = rclc_support_fini(&g_support);
    (void)discarded;
    rcl_reset_error();
}

/**
 * @brief The agent locator this build was configured with.
 *
 * ROSETTE_MICRO_ROS_AGENT_IP names a fixed agent; otherwise
 * ROSETTE_MICRO_ROS_DISCOVERY picks mDNS/DNS-SD (the default) or the XRCE
 * multicast GET_INFO probe (see cmake/micro_ros.cmake).
 *
 * @param[in] sockets The board's socket provider; outlives the locator.
 * @return A locator with static storage duration.
 */
rosette::AgentLocator& configured_locator(rosette::net::SocketProvider& sockets) {
    if (ROSETTE_MICRO_ROS_AGENT_IP[0] != '\0') {
        static rosette::StaticAgentLocator fixed(ROSETTE_MICRO_ROS_AGENT_IP,
                                                 LED_FILES_STR(ROSETTE_MICRO_ROS_AGENT_PORT));
        return fixed;
    }

#if ROSETTE_MICRO_ROS_DISCOVERY_MDNS
    static rosette::MdnsAgentLocator mdns(
        sockets, rosette::MdnsAgentLocatorConfig{
                     ROSETTE_MICRO_ROS_MDNS_SERVICE, ROSETTE_MICRO_ROS_MDNS_HOSTNAME,
                     static_cast<uint16_t>(ROSETTE_MICRO_ROS_AGENT_PORT),
                     ROSETTE_MICRO_ROS_MDNS_TIMEOUT_MS, ROSETTE_MICRO_ROS_MDNS_ATTEMPTS});
    return mdns;
#else
    static rosette::MulticastAgentLocator xrce(sockets);
    return xrce;
#endif
}

/// @brief Prints the session count, the agent's state, the epoch time, and the heap in use.
void report(const ros::SessionSupervisor& session) {
    const struct mallinfo heap = mallinfo();
    std::printf("[led_files] session=%lu%s epoch=%lld%s heap_in_use=%d\n",
                static_cast<unsigned long>(session.sessions_created()),
                session.state() == ros::SessionState::running ? "" : " (down)",
                static_cast<long long>(ros::epoch_millis() / 1000),
                ros::time_synced() ? "" : " (unsynced)", heap.uordblks);
}

}  // namespace

int main() {
    board_init();
    // Hold the boot log until a terminal attaches (up to 10 s).
    for (int i = 0; i < 100 && !stdio_usb_connected(); ++i) {
        sleep_ms(100);
    }

    std::printf("[led_files] rclc services through a micro-ROS agent\n");

    static storage::Volume volume;
    static storage::FileStore store(volume, "/files", "", kBodyCapacity - 1u);
    g_store = &store;
    bind_messages();

    // The supervisor does the waiting, so bring-up returns at once.
    rosette::net::NetworkConfig network = net::board_default_network();
    network.ready_timeout_ms = 0u;
    (void)net::bring_up_board_network("rosette-led-files", network);

    rosette::net::SocketProvider& sockets = net::board_socket_provider();
    static rosette::UdpSocketTransport transport(
        sockets, rosette::net::make_socket_address_v4({0u, 0u, 0u, 0u}, 0u),
        rosette::net::make_socket_address_v4({0u, 0u, 0u, 0u}, 0u), 50u);
    static ros::AgentSessionGate gate(transport, configured_locator(sockets),
                                      ros::rmw_agent_link());
    ros::SessionSupervisor session(net::board_network_watch(), {create_session, destroy_session},
                                   &gate);

    uint32_t next_report = pf::monotonic_ms() + kReportPeriodMs;
    for (;;) {
        const uint32_t now = pf::monotonic_ms();
        if (session.step(now) == ros::SessionState::running) {
            (void)rclc_executor_spin_some(&g_executor, kSpinTimeoutNs);
        } else {
            pf::sleep_ms(10u);
        }

        if (static_cast<int32_t>(now - next_report) >= 0) {
            next_report += kReportPeriodMs;
            report(session);
        }
    }
}
