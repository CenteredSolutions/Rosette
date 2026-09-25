/**
 * @file examples/rclc_chatter/main.cpp
 * @brief Stock rclc talker/listener on a WIZnet board over rmw_rosette.
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
 * From a PC in domain 0 with stock Fast DDS (no configuration):
 *   ros2 topic echo /rosette/chatter std_msgs/msg/String
 *   ros2 topic pub /rosette/command std_msgs/msg/String "{data: hi}"
 *
 * Verification: the PC echoes the board's messages and the board prints the
 * PC's; `ros2 topic info -v /rosette/chatter` lists the board's publisher.
 *
 * References: rclc (ros2/rclc, Humble) executor and support API; ROS 2 topic
 * name mangling (rt/ prefix, <pkg>::msg::dds_::<Type>_ names).
 */

#include <cstdio>
#include <cstring>
#include <malloc.h>

#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/string.h>

#include "board.h"
#include "rosette/platform/platform.hpp"
#include "wiznet_network.hpp"

#include "pico/stdlib.h"

namespace {

namespace pf = rosette::platform;

constexpr uint64_t kPublishPeriodNs = RCL_MS_TO_NS(1000);
constexpr uint32_t kReportPeriodMs = 5000u;
constexpr int64_t kSpinTimeoutNs = RCL_MS_TO_NS(100);

rcl_publisher_t g_publisher;
std_msgs__msg__String g_outgoing;
char g_outgoing_text[48];
uint32_t g_published = 0u;

std_msgs__msg__String g_incoming;
char g_incoming_text[256];
uint32_t g_commands_received = 0u;

/**
 * @brief Stops with a message when an rcl call fails.
 *
 * @param[in] ret  The rcl return code.
 * @param[in] what The call that returned it.
 */
void check(rcl_ret_t ret, const char* what) noexcept {
    if (ret == RCL_RET_OK) return;
    for (;;) {
        std::printf("[rclc] %s failed: %d (%s)\n", what, static_cast<int>(ret),
                    rcl_get_error_string().str);
        sleep_ms(2000);
    }
}

/// @brief Timer callback: publishes one greeting.
void on_publish_timer(rcl_timer_t* timer, int64_t /*last_call_time*/) {
    if (timer == nullptr) return;
    std::snprintf(g_outgoing_text, sizeof(g_outgoing_text), "hello from rclc #%lu",
                  static_cast<unsigned long>(++g_published));
    g_outgoing.data.size = std::strlen(g_outgoing_text);
    const rcl_ret_t ret = rcl_publish(&g_publisher, &g_outgoing, nullptr);
    if (ret != RCL_RET_OK) {
        std::printf("[rclc] publish failed: %d (%s)\n", static_cast<int>(ret),
                    rcl_get_error_string().str);
        rcl_reset_error();
    }
}

/// @brief Subscription callback: prints the received command.
void on_command(const void* msg) {
    const auto* s = static_cast<const std_msgs__msg__String*>(msg);
    ++g_commands_received;
    std::printf("[rclc] /rosette/command: %.*s\n", static_cast<int>(s->data.size), s->data.data);
}

}  // namespace

int main() {
    board_init();
    // Hold the boot log until a terminal attaches (up to 10 s).
    for (int i = 0; i < 100 && !stdio_usb_connected(); ++i) sleep_ms(100);
    std::printf("[rclc] rclc chatter over rmw_rosette\n");

    rclc_chatter::bring_up_wiznet_network("rosette-rclc");

    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    check(rclc_support_init(&support, 0, nullptr, &allocator), "rclc_support_init");

    rcl_node_t node = rcl_get_zero_initialized_node();
    check(rclc_node_init_default(&node, "rclc_chatter", "rosette", &support), "rclc_node_init_default");

    const rosidl_message_type_support_t* string_type =
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String);
    g_publisher = rcl_get_zero_initialized_publisher();
    check(rclc_publisher_init_default(&g_publisher, &node, string_type, "chatter"),
          "rclc_publisher_init_default");

    rcl_subscription_t subscription = rcl_get_zero_initialized_subscription();
    check(rclc_subscription_init_default(&subscription, &node, string_type, "command"),
          "rclc_subscription_init_default");

    rcl_timer_t timer = rcl_get_zero_initialized_timer();
    check(rclc_timer_init_default(&timer, &support, kPublishPeriodNs, on_publish_timer),
          "rclc_timer_init_default");

    // Messages own fixed buffers; nothing is allocated per message.
    g_outgoing.data.data = g_outgoing_text;
    g_outgoing.data.capacity = sizeof(g_outgoing_text);
    g_incoming.data.data = g_incoming_text;
    g_incoming.data.capacity = sizeof(g_incoming_text);
    g_incoming.data.size = 0u;

    rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
    check(rclc_executor_init(&executor, &support.context, 2, &allocator), "rclc_executor_init");
    check(rclc_executor_add_timer(&executor, &timer), "rclc_executor_add_timer");
    check(rclc_executor_add_subscription(&executor, &subscription, &g_incoming, on_command,
                                         ON_NEW_DATA),
          "rclc_executor_add_subscription");

    std::printf("[rclc] node /rosette/rclc_chatter up: /rosette/chatter, /rosette/command\n");

    uint32_t next_report = pf::monotonic_ms() + kReportPeriodMs;
    for (;;) {
        (void)rclc_executor_spin_some(&executor, kSpinTimeoutNs);
        const uint32_t now = pf::monotonic_ms();
        rclc_chatter::poll_wiznet_network(now);  // DHCP lease renewal
        if (static_cast<int32_t>(now - next_report) >= 0) {
            next_report += kReportPeriodMs;
            const struct mallinfo heap = mallinfo();
            std::printf("[rclc] published=%lu commands=%lu heap_in_use=%d arp_timeouts=%lu\n",
                        static_cast<unsigned long>(g_published),
                        static_cast<unsigned long>(g_commands_received), heap.uordblks,
                        static_cast<unsigned long>(rclc_chatter::wiznet_send_timeouts()));
        }
    }
}
