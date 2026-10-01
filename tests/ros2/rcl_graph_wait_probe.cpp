/**
 * @file rcl_graph_wait_probe.cpp
 * @brief Waits on the ROS graph through stock rcl, as rclcpp and rclc do.
 *
 Waiting for an endpoint to appear depends on both halves of an rmw's graph
 * support: endpoints are counted with rmw_count_publishers() and
 * rmw_count_subscribers(), and between counts the waiter blocks on the
 * node's graph guard condition, as rclcpp's GraphListener does. An rmw that
 * never triggers that guard makes the wait sleep out its whole timeout, and
 * one that miscounts gives the wrong answer, so the elapsed time and the
 * result together check both. (Humble's rcl_wait_for_publishers() is not
 * used: it does not return early even under stock rmw_fastrtps_cpp.)
 *
 * The probe runs under the RMW under test (RMW_IMPLEMENTATION) while
 * rmw_graph_peer.py, under a stock RMW, creates one publisher and one
 * subscription on the topic after a delay, holds them, then destroys them.
 * The probe:
 *   1. waits for one publisher, then one subscription (timeout 10 s each);
 *   2. checks both counts are exactly 1;
 *   3. polls until both counts fall to 0 once the peer withdraws them.
 *
 * Usage: rcl_graph_wait_probe <topic> <domain_id> <max_wait_ms>
 * Exit status 0 only when every step passes and each wait returns within
 * <max_wait_ms> of its start, which the harness sets below the timeout.
 *
 * References:
 * - rcl/graph.h (Humble), `rcl_count_publishers`, `rcl_node_get_graph_guard_condition`
 * - rclcpp/src/rclcpp/graph_listener.cpp (Humble), `GraphListener::run_loop`
 * - rmw/rmw.h (Humble), `rmw_node_get_graph_guard_condition`
 */

#include <rcl/error_handling.h>
#include <rcl/graph.h>
#include <rcl/rcl.h>
#include <rcutils/allocator.h>
#include <rmw/rmw.h>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <thread>

namespace {

using Clock = std::chrono::steady_clock;

long elapsed_ms(Clock::time_point since) {
    return static_cast<long>(
        std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - since).count());
}

/**
 * @brief Blocks on the node's graph guard condition until `topic` has at
 *        least one publisher (or subscription), as rclcpp's GraphListener does.
 *
 * Each pass counts, and when the count is short clears the wait set, adds
 * the graph guard condition again, and waits for the rest of the timeout.
 * Every wakeup before the timeout is a graph guard trigger, so a success
 * well inside the timeout proves the trigger arrived.
 *
 * @param[in] what        Label for the log line.
 * @param[in] node        Node whose graph guard condition is waited on.
 * @param[in] topic       Fully-qualified ROS topic name.
 * @param[in] publishers  `true` to wait for a publisher; `false` for a subscription.
 * @param[in] max_wait_ms Longest acceptable wait; the timeout is 10 s.
 * @return `true` when the count reached 1 within `max_wait_ms`.
 */
bool wait_for_endpoint(const char* what, rcl_node_t* node, const char* topic, bool publishers,
                       long max_wait_ms) {
    rcl_wait_set_t wait_set = rcl_get_zero_initialized_wait_set();
    if (rcl_wait_set_init(&wait_set, 0, 1, 0, 0, 0, 0, node->context,
                          rcl_get_default_allocator()) != RCL_RET_OK) {
        return false;
    }
    const rcl_guard_condition_t* graph = rcl_node_get_graph_guard_condition(node);
    const auto start = Clock::now();
    int wakeups = 0;
    size_t count = 0;
    bool ok = false;
    for (;;) {
        const rcl_ret_t counted = publishers ? rcl_count_publishers(node, topic, &count)
                                             : rcl_count_subscribers(node, topic, &count);
        if (counted != RCL_RET_OK) break;
        if (count >= 1u) {
            ok = true;
            break;
        }
        const long left_ms = 10000 - elapsed_ms(start);
        if (left_ms <= 0) break;
        if (rcl_wait_set_clear(&wait_set) != RCL_RET_OK ||
            rcl_wait_set_add_guard_condition(&wait_set, graph, nullptr) != RCL_RET_OK) {
            break;
        }
        const rcl_ret_t waited = rcl_wait(&wait_set, RCL_MS_TO_NS(left_ms));
        if (waited == RCL_RET_OK) ++wakeups;
        else if (waited != RCL_RET_TIMEOUT) break;
    }
    const long ms = elapsed_ms(start);
    std::printf("[rcl-graph-probe] %s: success=%d count=%zu graph_wakeups=%d elapsed_ms=%ld\n",
                what, ok ? 1 : 0, count, wakeups, ms);
    (void)!rcl_wait_set_fini(&wait_set);
    return ok && ms <= max_wait_ms;
}

/// Reads both counts; false when either query fails.
bool counts(rcl_node_t* node, const char* topic, size_t& pubs, size_t& subs) {
    return rcl_count_publishers(node, topic, &pubs) == RCL_RET_OK &&
           rcl_count_subscribers(node, topic, &subs) == RCL_RET_OK;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc != 4) {
        std::fprintf(stderr, "usage: %s <topic> <domain_id> <max_wait_ms>\n", argv[0]);
        return 2;
    }
    const char* topic = argv[1];
    const long max_wait_ms = std::atol(argv[3]);

    rcl_init_options_t options = rcl_get_zero_initialized_init_options();
    rcl_context_t context = rcl_get_zero_initialized_context();
    rcl_node_t node = rcl_get_zero_initialized_node();
    rcl_node_options_t node_options = rcl_node_get_default_options();
    if (rcl_init_options_init(&options, rcl_get_default_allocator()) != RCL_RET_OK ||
        rcl_init_options_set_domain_id(&options, std::strtoul(argv[2], nullptr, 10)) !=
            RCL_RET_OK ||
        rcl_init(0, nullptr, &options, &context) != RCL_RET_OK ||
        rcl_node_init(&node, "rosette_graph_probe", "/", &context, &node_options) !=
            RCL_RET_OK) {
        std::fprintf(stderr, "[rcl-graph-probe] init failed: %s\n", rcl_get_error_string().str);
        return 1;
    }
    std::printf("[rcl-graph-probe] rmw=%s topic=%s\n", rmw_get_implementation_identifier(),
                topic);

    bool ok = wait_for_endpoint("wait_for_publisher", &node, topic, true, max_wait_ms) &&
              wait_for_endpoint("wait_for_subscription", &node, topic, false, max_wait_ms);

    size_t pubs = 0, subs = 0;
    if (ok) {
        ok = counts(&node, topic, pubs, subs) && pubs == 1u && subs == 1u;
        std::printf("[rcl-graph-probe] counts while present: pubs=%zu subs=%zu\n", pubs, subs);
    }
    if (ok) {
        // Counting does not drive discovery on every rmw; waiting on the
        // graph guard does, so each pass waits up to 100 ms before counting.
        const auto start = Clock::now();
        rcl_wait_set_t wait_set = rcl_get_zero_initialized_wait_set();
        ok = rcl_wait_set_init(&wait_set, 0, 1, 0, 0, 0, 0, &context,
                               rcl_get_default_allocator()) == RCL_RET_OK;
        while (ok && (pubs != 0u || subs != 0u) && elapsed_ms(start) < 15000) {
            ok = rcl_wait_set_clear(&wait_set) == RCL_RET_OK &&
                 rcl_wait_set_add_guard_condition(
                     &wait_set, rcl_node_get_graph_guard_condition(&node), nullptr) ==
                     RCL_RET_OK;
            const rcl_ret_t waited = ok ? rcl_wait(&wait_set, RCL_MS_TO_NS(100)) : RCL_RET_ERROR;
            ok = (waited == RCL_RET_OK || waited == RCL_RET_TIMEOUT) &&
                 counts(&node, topic, pubs, subs);
        }
        (void)!rcl_wait_set_fini(&wait_set);
        ok = ok && pubs == 0u && subs == 0u;
        std::printf("[rcl-graph-probe] counts after withdrawal: pubs=%zu subs=%zu "
                    "elapsed_ms=%ld\n", pubs, subs, elapsed_ms(start));
    }

    ok = rcl_node_fini(&node) == RCL_RET_OK && ok;
    ok = rcl_shutdown(&context) == RCL_RET_OK && ok;
    ok = rcl_context_fini(&context) == RCL_RET_OK && ok;
    ok = rcl_init_options_fini(&options) == RCL_RET_OK && ok;
    std::printf("[rcl-graph-probe] %s\n", ok ? "PASS" : "FAIL");
    return ok ? 0 : 1;
}
