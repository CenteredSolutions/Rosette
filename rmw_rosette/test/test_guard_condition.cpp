/**
 * @file test_guard_condition.cpp
 * @brief Unit tests for guard conditions and their readiness in rmw_wait().
 *
 * rcl builds every executor wakeup (interrupts, SIGINT, graph changes) on
 * guard conditions, so these tests pin the contract rcl relies on: a trigger
 * wakes exactly one wait, is not lost when it lands between waits, and an
 * untriggered condition is reported not-ready. The graph guard condition is
 * the context's one guard, shared by its nodes and triggered by graph
 * changes, and rmw_count_publishers()/rmw_count_subscribers() see local
 * endpoints as soon as they exist.
 */

#include <gtest/gtest.h>

#include "rcl_default_init_options.hpp"

#include <rcutils/allocator.h>
#include <rmw/error_handling.h>
#include <rmw/rmw.h>
#include <std_msgs/msg/string.h>

#include <thread>

namespace {

/// One initialized context and node shared by the tests in this file.
class GuardConditionTest : public ::testing::Test {
protected:
    void SetUp() override {
        options_ = rmw_get_zero_initialized_init_options();
        ASSERT_EQ(rmw_rosette_test::init_options_as_rcl(&options_), RMW_RET_OK);
        options_.domain_id = 91;
        context_ = rmw_get_zero_initialized_context();
        ASSERT_EQ(rmw_init(&options_, &context_), RMW_RET_OK);
        wait_set_ = rmw_create_wait_set(&context_, 4);
        ASSERT_NE(wait_set_, nullptr);
    }

    void TearDown() override {
        rmw_destroy_wait_set(wait_set_);
        rmw_shutdown(&context_);
        rmw_context_fini(&context_);
        rmw_init_options_fini(&options_);
    }

    /// Waits on one guard condition; returns whether it was reported ready.
    bool wait_on(rmw_guard_condition_t* gc, rmw_time_t timeout) {
        void* conds[] = {gc->data};
        rmw_guard_conditions_t set{1, conds};
        const rmw_ret_t ret =
            rmw_wait(nullptr, &set, nullptr, nullptr, nullptr, wait_set_, &timeout);
        return ret == RMW_RET_OK && set.guard_conditions[0] != nullptr;
    }

    rmw_init_options_t options_{};
    rmw_context_t context_{};
    rmw_wait_set_t* wait_set_ = nullptr;
};

}  // namespace

TEST_F(GuardConditionTest, UntriggeredIsNotReady) {
    rmw_guard_condition_t* gc = rmw_create_guard_condition(&context_);
    ASSERT_NE(gc, nullptr);
    EXPECT_FALSE(wait_on(gc, rmw_time_t{0, 20000000}));
    EXPECT_EQ(rmw_destroy_guard_condition(gc), RMW_RET_OK);
}

TEST_F(GuardConditionTest, TriggerWakesExactlyOneWait) {
    rmw_guard_condition_t* gc = rmw_create_guard_condition(&context_);
    ASSERT_NE(gc, nullptr);
    ASSERT_EQ(rmw_trigger_guard_condition(gc), RMW_RET_OK);
    EXPECT_TRUE(wait_on(gc, rmw_time_t{0, 0}));
    EXPECT_FALSE(wait_on(gc, rmw_time_t{0, 0}));
    EXPECT_EQ(rmw_destroy_guard_condition(gc), RMW_RET_OK);
}

TEST_F(GuardConditionTest, TriggerFromAnotherThreadWakesBlockedWait) {
    rmw_guard_condition_t* gc = rmw_create_guard_condition(&context_);
    ASSERT_NE(gc, nullptr);
    std::thread trigger([gc] {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        rmw_trigger_guard_condition(gc);
    });
    EXPECT_TRUE(wait_on(gc, rmw_time_t{5, 0}));
    trigger.join();
    EXPECT_EQ(rmw_destroy_guard_condition(gc), RMW_RET_OK);
}

TEST_F(GuardConditionTest, NodeGraphGuardConditionIsStable) {
    rmw_node_t* node = rmw_create_node(&context_, "graph_guard_test", "/");
    ASSERT_NE(node, nullptr);
    const rmw_guard_condition_t* a = rmw_node_get_graph_guard_condition(node);
    const rmw_guard_condition_t* b = rmw_node_get_graph_guard_condition(node);
    ASSERT_NE(a, nullptr);
    EXPECT_EQ(a, b);
    EXPECT_EQ(rmw_destroy_node(node), RMW_RET_OK);
}

TEST_F(GuardConditionTest, NodesShareTheContextGraphGuardCondition) {
    rmw_node_t* a = rmw_create_node(&context_, "graph_guard_a", "/");
    rmw_node_t* b = rmw_create_node(&context_, "graph_guard_b", "/");
    ASSERT_NE(a, nullptr);
    ASSERT_NE(b, nullptr);
    EXPECT_EQ(rmw_node_get_graph_guard_condition(a), rmw_node_get_graph_guard_condition(b));
    EXPECT_EQ(rmw_destroy_node(b), RMW_RET_OK);
    EXPECT_EQ(rmw_destroy_node(a), RMW_RET_OK);
}

TEST_F(GuardConditionTest, LocalEndpointsTriggerGraphGuardAndCount) {
    rmw_node_t* node = rmw_create_node(&context_, "graph_count_test", "/");
    ASSERT_NE(node, nullptr);
    auto* graph = const_cast<rmw_guard_condition_t*>(rmw_node_get_graph_guard_condition(node));
    EXPECT_TRUE(wait_on(graph, rmw_time_t{0, 0})) << "node creation changes the graph";
    EXPECT_FALSE(wait_on(graph, rmw_time_t{0, 0}));

    size_t count = 99;
    ASSERT_EQ(rmw_count_publishers(node, "/graph_count", &count), RMW_RET_OK);
    EXPECT_EQ(count, 0u);

    const rosidl_message_type_support_t* ts = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String);
    rmw_qos_profile_t qos = rmw_qos_profile_default;
    rmw_publisher_options_t pub_options = rmw_get_default_publisher_options();
    rmw_publisher_t* pub = rmw_create_publisher(node, ts, "/graph_count", &qos, &pub_options);
    ASSERT_NE(pub, nullptr);
    EXPECT_TRUE(wait_on(graph, rmw_time_t{0, 0})) << "a new publisher changes the graph";

    ASSERT_EQ(rmw_count_publishers(node, "/graph_count", &count), RMW_RET_OK);
    EXPECT_EQ(count, 1u);
    ASSERT_EQ(rmw_count_subscribers(node, "/graph_count", &count), RMW_RET_OK);
    EXPECT_EQ(count, 0u);
    ASSERT_EQ(rmw_count_publishers(node, "/other", &count), RMW_RET_OK);
    EXPECT_EQ(count, 0u);

    EXPECT_EQ(rmw_count_publishers(node, "no_leading_slash", &count), RMW_RET_INVALID_ARGUMENT);
    rmw_reset_error();
    EXPECT_EQ(rmw_count_publishers(node, "/graph_count", nullptr), RMW_RET_INVALID_ARGUMENT);
    rmw_reset_error();

    EXPECT_EQ(rmw_destroy_publisher(node, pub), RMW_RET_OK);
    EXPECT_TRUE(wait_on(graph, rmw_time_t{0, 0})) << "a destroyed publisher changes the graph";
    ASSERT_EQ(rmw_count_publishers(node, "/graph_count", &count), RMW_RET_OK);
    EXPECT_EQ(count, 0u);
    EXPECT_EQ(rmw_destroy_node(node), RMW_RET_OK);
}
