/**
 * @file examples/rclc_chatter/main.cpp
 * @brief Stock rclc talker/listener on a WIZnet board over rmw_rosette, under a session supervisor.
 *
 * The firmware is an ordinary rclc application: rcl, rclc and the message
 * type supports are the stock C client libraries, and rmw_rosette is the
 * middleware, so the board joins the ROS 2 graph as a DDS participant with no
 * agent. It is the rclc counterpart of examples/rtps_chatter, which drives the
 * RTPS core directly; the two expose the same topics.
 *
 *   - publishes "hello from rclc #N" on /rosette/chatter every second (timer);
 *   - prints every std_msgs/String received on /rosette/command;
 *   - prints the heap in use every five seconds.
 *
 * The rcl objects are built in create_session() and finalized in
 * destroy_session(), and a `rosette::ros::SessionSupervisor` decides when:
 * it creates the session once the board holds an address, and destroys and
 * recreates it when that address moves (a DHCP lease replacing link-local,
 * a new lease after an address conflict), so the participant always
 * advertises the address peers can reach.
 *
 * From a PC in domain 0 with stock Fast DDS (no configuration):
 *   ros2 topic echo /rosette/chatter std_msgs/msg/String
 *   ros2 topic pub /rosette/command std_msgs/msg/String "{data: hi}"
 *
 * Verification: the PC echoes the board's messages and the board prints the
 * PC's, before and after the session is rebuilt; `ros2 topic info -v
 * /rosette/chatter` lists the board's publisher; the heap in use returns to
 * the same value after every rebuild.
 *
 * References: rclc (ros2/rclc, Humble) executor and support API; ROS 2 topic
 * name mangling (rt/ prefix, <pkg>::msg::dds_::<Type>_ names); micro-ROS
 * "Handle reconnections and liveliness" tutorial.
 */

#include <malloc.h>
#include <cstdio>
#include <cstring>

#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/string.h>

#include "board.h"
#include "rosette/net/wiznet/board_network.hpp"
#include "rosette/platform/platform.hpp"
#include "rosette/ros/session_supervisor.hpp"

#include "pico/stdlib.h"

namespace {

namespace pf = rosette::platform;
namespace wiz = rosette::net::wiznet;
namespace ros = rosette::ros;

constexpr uint64_t kPublishPeriodNs = RCL_MS_TO_NS(1000);
constexpr uint32_t kReportPeriodMs = 5000u;
constexpr int64_t kSpinTimeoutNs = RCL_MS_TO_NS(100);

rcl_allocator_t g_allocator;
rclc_support_t g_support;
rcl_node_t g_node;
rcl_publisher_t g_publisher;
rcl_subscription_t g_subscription;
rcl_timer_t g_timer;
rclc_executor_t g_executor;

std_msgs__msg__String g_outgoing;
char g_outgoing_text[48];
uint32_t g_published = 0u;

std_msgs__msg__String g_incoming;
char g_incoming_text[256];
uint32_t g_commands_received = 0u;

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

    std::printf("[rclc] %s failed: %d (%s)\n", what, static_cast<int>(ret),
                rcl_get_error_string().str);
    rcl_reset_error();
    return false;
}

/// @brief Timer callback: publishes one greeting.
void on_publish_timer(rcl_timer_t* timer, int64_t /*last_call_time*/) {
    if (timer == nullptr) {
        return;
    }

    std::snprintf(g_outgoing_text, sizeof(g_outgoing_text), "hello from rclc #%lu",
                  static_cast<unsigned long>(++g_published));
    g_outgoing.data.size = std::strlen(g_outgoing_text);
    (void)ok(rcl_publish(&g_publisher, &g_outgoing, nullptr), "rcl_publish");
}

/// @brief Subscription callback: prints the received command.
void on_command(const void* msg) {
    const auto* s = static_cast<const std_msgs__msg__String*>(msg);
    ++g_commands_received;
    std::printf("[rclc] /rosette/command: %.*s\n", static_cast<int>(s->data.size), s->data.data);
}

/**
 * @brief Builds the rclc session: support, node, publisher, subscription, timer, executor.
 *
 * The supervisor calls this once the board network is ready and again after
 * every destroy_session(). Every object is zero-initialized first, so a
 * failure part way leaves destroy_session() finalizing only what was built.
 *
 * @return True when every object was created; false at the first failure
 *   (printed).
 *
 * Verification:
 * - Correct means: after a true return the node publishes /rosette/chatter
 *   and receives /rosette/command; after a false one destroy_session()
 *   returns the heap to its level before the call.
 * - Covered by: the file's hardware check (rebuild after an address move).
 *
 * References:
 * - rclc (ros2/rclc, Humble) rclc_support_init, rclc_executor_init
 */
bool create_session() {
    g_support = rclc_support_t{};
    g_node = rcl_get_zero_initialized_node();
    g_publisher = rcl_get_zero_initialized_publisher();
    g_subscription = rcl_get_zero_initialized_subscription();
    g_timer = rcl_get_zero_initialized_timer();
    g_executor = rclc_executor_get_zero_initialized_executor();
    g_allocator = rcl_get_default_allocator();

    const rosidl_message_type_support_t* string_type =
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String);
    return ok(rclc_support_init(&g_support, 0, nullptr, &g_allocator), "rclc_support_init")
           && ok(rclc_node_init_default(&g_node, "rclc_chatter", "rosette", &g_support),
                 "rclc_node_init_default")
           && ok(rclc_publisher_init_default(&g_publisher, &g_node, string_type, "chatter"),
                 "rclc_publisher_init_default")
           && ok(rclc_subscription_init_default(&g_subscription, &g_node, string_type, "command"),
                 "rclc_subscription_init_default")
           && ok(rclc_timer_init_default(&g_timer, &g_support, kPublishPeriodNs, on_publish_timer),
                 "rclc_timer_init_default")
           && ok(rclc_executor_init(&g_executor, &g_support.context, 2, &g_allocator),
                 "rclc_executor_init")
           && ok(rclc_executor_add_timer(&g_executor, &g_timer), "rclc_executor_add_timer")
           && ok(rclc_executor_add_subscription(&g_executor, &g_subscription, &g_incoming,
                                                on_command, ON_NEW_DATA),
                 "rclc_executor_add_subscription");
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
    discarded = rcl_timer_fini(&g_timer);
    discarded = rcl_subscription_fini(&g_subscription, &g_node);
    discarded = rcl_publisher_fini(&g_publisher, &g_node);
    discarded = rcl_node_fini(&g_node);
    discarded = rclc_support_fini(&g_support);
    (void)discarded;
    rcl_reset_error();
}

/// @brief Prints the traffic counters, the heap in use, and the session count.
void report(const ros::SessionSupervisor& session) {
    const struct mallinfo heap = mallinfo();
    std::printf(
        "[rclc] session=%lu%s published=%lu commands=%lu heap_in_use=%d "
        "arp_timeouts=%lu\n",
        static_cast<unsigned long>(session.sessions_created()),
        session.state() == ros::SessionState::running ? "" : " (down)",
        static_cast<unsigned long>(g_published), static_cast<unsigned long>(g_commands_received),
        heap.uordblks, static_cast<unsigned long>(wiz::board_send_timeouts()));
}

}  // namespace

int main() {
    board_init();
    // Hold the boot log until a terminal attaches (up to 10 s).
    for (int i = 0; i < 100 && !stdio_usb_connected(); ++i) {
        sleep_ms(100);
    }

    std::printf("[rclc] rclc chatter over rmw_rosette\n");

    // Messages own fixed buffers; nothing is allocated per message.
    g_outgoing.data.data = g_outgoing_text;
    g_outgoing.data.capacity = sizeof(g_outgoing_text);
    g_incoming.data.data = g_incoming_text;
    g_incoming.data.capacity = sizeof(g_incoming_text);
    g_incoming.data.size = 0u;

    // The supervisor does the waiting, so bring-up returns at once.
    rosette::net::NetworkConfig network = wiz::board_default_network();
    network.ready_timeout_ms = 0u;
    (void)wiz::bring_up_board_network("rosette-rclc", network);
    ros::SessionSupervisor session(wiz::board_network_watch(), {create_session, destroy_session});

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
