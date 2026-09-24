/**
 * @file test_guard_condition.cpp
 * @brief Unit tests for guard conditions and their readiness in rmw_wait().
 *
 * rcl builds every executor wakeup (interrupts, SIGINT, graph changes) on
 * guard conditions, so these tests pin the contract rcl relies on: a trigger
 * wakes exactly one wait, is not lost when it lands between waits, and an
 * untriggered condition is reported not-ready.
 */

#include <gtest/gtest.h>

#include <rcutils/allocator.h>
#include <rmw/rmw.h>

#include <thread>

namespace {

/// One initialized context and node shared by the tests in this file.
class GuardConditionTest : public ::testing::Test {
protected:
    void SetUp() override {
        options_ = rmw_get_zero_initialized_init_options();
        ASSERT_EQ(rmw_init_options_init(&options_, rcutils_get_default_allocator()), RMW_RET_OK);
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
