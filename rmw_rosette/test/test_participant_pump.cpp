/**
 * @file test_participant_pump.cpp
 * @brief The host pump thread runs the participant with no thread in rmw_wait(),
 *        and wakes each waiting thread for its own readiness only.
 *
 * Verification:
 * - Correct means: discovery, publishing and delivery complete while no
 *   thread waits; concurrent rmw_wait() calls on separate wait sets each
 *   return for their own guard condition and no other; rmw_wait() honors a
 *   zero and a finite timeout.
 * - Covered here: TC-RMW-PUMP-001..003.
 *
 * References:
 * - rmw/rmw.h (Humble, Jazzy), `rmw_wait`, `rmw_publish`, `rmw_take`
 */

#include <gtest/gtest.h>

#include "rcl_default_init_options.hpp"

#include <rmw/rmw.h>

#include <rosidl_runtime_c/string_functions.h>
#include <std_msgs/msg/string.h>

#include <atomic>
#include <chrono>
#include <thread>

namespace {

using Clock = std::chrono::steady_clock;

/// @brief Milliseconds elapsed since `start`.
long long elapsed_ms(Clock::time_point start) {
    return std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - start).count();
}

/// @brief One initialized context with one node, torn down after each test.
class ParticipantPump : public ::testing::Test {
protected:
    void SetUp() override {
        rmw_init_options_t options = rmw_get_zero_initialized_init_options();
        ASSERT_EQ(rmw_rosette_test::init_options_as_rcl(&options), RMW_RET_OK);
        ASSERT_EQ(rmw_init(&options, &context_), RMW_RET_OK);
        ASSERT_EQ(rmw_init_options_fini(&options), RMW_RET_OK);
        node_ = rmw_create_node(&context_, "participant_pump_test", "/");
        ASSERT_NE(node_, nullptr);
    }

    void TearDown() override {
        if (node_ != nullptr) {
            EXPECT_EQ(rmw_destroy_node(node_), RMW_RET_OK);
        }

        EXPECT_EQ(rmw_shutdown(&context_), RMW_RET_OK);
        EXPECT_EQ(rmw_context_fini(&context_), RMW_RET_OK);
    }

    /// @brief Waits on one guard condition; returns rmw_wait()'s result.
    rmw_ret_t wait_on(rmw_wait_set_t* wait_set, rmw_guard_condition_t* guard, rmw_time_t timeout) {
        void*                  entries[] = {guard->data};
        rmw_guard_conditions_t guards{1, entries};
        return rmw_wait(nullptr, &guards, nullptr, nullptr, nullptr, wait_set, &timeout);
    }

    rmw_context_t context_ = rmw_get_zero_initialized_context();
    rmw_node_t*   node_ = nullptr;
};

// TC-RMW-PUMP-001: with no thread in rmw_wait(), the publisher and the
// subscription match, a published sample is sent, and it is delivered.
TEST_F(ParticipantPump, PublishedSampleArrivesWithoutAnyWait) {
    rmw_qos_profile_t qos = rmw_qos_profile_default;
    qos.depth = 1;
    const rosidl_message_type_support_t* ts = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String);
    rmw_publisher_options_t              pub_options = rmw_get_default_publisher_options();
    rmw_subscription_options_t           sub_options = rmw_get_default_subscription_options();
    rmw_publisher_t* pub = rmw_create_publisher(node_, ts, "/pump_topic", &qos, &pub_options);
    ASSERT_NE(pub, nullptr);
    rmw_subscription_t* sub = rmw_create_subscription(node_, ts, "/pump_topic", &qos, &sub_options);
    ASSERT_NE(sub, nullptr);

    size_t     matched = 0u;
    const auto start = Clock::now();
    while (matched == 0u && elapsed_ms(start) < 5000) {
        ASSERT_EQ(rmw_subscription_count_matched_publishers(sub, &matched), RMW_RET_OK);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    ASSERT_EQ(matched, 1u) << "the endpoints never matched without rmw_wait()";

    std_msgs__msg__String out_msg{};
    std_msgs__msg__String__init(&out_msg);
    rosidl_runtime_c__String__assign(&out_msg.data, "pumped");
    ASSERT_EQ(rmw_publish(pub, &out_msg, nullptr), RMW_RET_OK);

    std_msgs__msg__String in_msg{};
    std_msgs__msg__String__init(&in_msg);
    bool       taken = false;
    const auto sent = Clock::now();
    while (!taken && elapsed_ms(sent) < 2000) {
        ASSERT_EQ(rmw_take(sub, &in_msg, &taken, nullptr), RMW_RET_OK);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    ASSERT_TRUE(taken) << "the sample was not delivered without rmw_wait()";
    EXPECT_STREQ(in_msg.data.data, "pumped");

    std_msgs__msg__String__fini(&in_msg);
    std_msgs__msg__String__fini(&out_msg);
    EXPECT_EQ(rmw_destroy_subscription(node_, sub), RMW_RET_OK);
    EXPECT_EQ(rmw_destroy_publisher(node_, pub), RMW_RET_OK);
}

// TC-RMW-PUMP-002: two threads waiting on separate wait sets are each woken
// by their own guard condition, and a trigger of one does not end the other's wait.
TEST_F(ParticipantPump, ConcurrentWaitersWakeForTheirOwnGuardOnly) {
    rmw_guard_condition_t* main_guard = rmw_create_guard_condition(&context_);
    rmw_guard_condition_t* other_guard = rmw_create_guard_condition(&context_);
    rmw_wait_set_t*        main_set = rmw_create_wait_set(&context_, 1);
    rmw_wait_set_t*        other_set = rmw_create_wait_set(&context_, 1);
    ASSERT_NE(main_guard, nullptr);
    ASSERT_NE(other_guard, nullptr);
    ASSERT_NE(main_set, nullptr);
    ASSERT_NE(other_set, nullptr);

    std::atomic<bool> other_done{false};
    rmw_ret_t         other_ret = RMW_RET_ERROR;
    std::thread       other([&] {
        other_ret = wait_on(other_set, other_guard, rmw_time_t{5, 0});
        other_done.store(true);
    });

    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    std::thread trigger([&] {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        EXPECT_EQ(rmw_trigger_guard_condition(main_guard), RMW_RET_OK);
    });

    const auto start = Clock::now();
    EXPECT_EQ(wait_on(main_set, main_guard, rmw_time_t{5, 0}), RMW_RET_OK);
    EXPECT_LT(elapsed_ms(start), 1000);
    trigger.join();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    EXPECT_FALSE(other_done.load()) << "a trigger of another wait set's guard ended this wait";

    const auto other_start = Clock::now();
    EXPECT_EQ(rmw_trigger_guard_condition(other_guard), RMW_RET_OK);
    other.join();
    EXPECT_EQ(other_ret, RMW_RET_OK);
    EXPECT_LT(elapsed_ms(other_start), 1000);

    EXPECT_EQ(rmw_destroy_wait_set(other_set), RMW_RET_OK);
    EXPECT_EQ(rmw_destroy_wait_set(main_set), RMW_RET_OK);
    EXPECT_EQ(rmw_destroy_guard_condition(other_guard), RMW_RET_OK);
    EXPECT_EQ(rmw_destroy_guard_condition(main_guard), RMW_RET_OK);
}

// TC-RMW-PUMP-003: with nothing ready, a zero timeout returns at once and a
// finite one returns RMW_RET_TIMEOUT once it has passed.
TEST_F(ParticipantPump, WaitHonorsZeroAndFiniteTimeouts) {
    rmw_guard_condition_t* guard = rmw_create_guard_condition(&context_);
    rmw_wait_set_t*        wait_set = rmw_create_wait_set(&context_, 1);
    ASSERT_NE(guard, nullptr);
    ASSERT_NE(wait_set, nullptr);

    auto start = Clock::now();
    EXPECT_EQ(wait_on(wait_set, guard, rmw_time_t{0, 0}), RMW_RET_TIMEOUT);
    EXPECT_LT(elapsed_ms(start), 50);

    start = Clock::now();
    EXPECT_EQ(wait_on(wait_set, guard, rmw_time_t{0, 200000000}), RMW_RET_TIMEOUT);
    EXPECT_GE(elapsed_ms(start), 200);
    EXPECT_LT(elapsed_ms(start), 1000);

    EXPECT_EQ(rmw_destroy_wait_set(wait_set), RMW_RET_OK);
    EXPECT_EQ(rmw_destroy_guard_condition(guard), RMW_RET_OK);
}

}  // namespace
