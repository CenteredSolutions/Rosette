/**
 * @file test_creation_after_shutdown.cpp
 * @brief Refusals the entry points make on a live context: no creation after
 *        rmw_shutdown(), and no sample consumed by a refused take.
 *
 * rcl stops creating entities once its context is invalid, but the rmw API is
 * public, and a program may keep a node past rmw_shutdown(). Every factory
 * refuses such a context before it takes a pool slot, while destruction stays
 * allowed so the program can tear down. A take that refuses an argument
 * leaves the subscription's buffered samples as they were.
 */

#include <gtest/gtest.h>

#include "rcl_default_init_options.hpp"

#include <rcutils/allocator.h>
#include <rmw/error_handling.h>
#include <rmw/rmw.h>
#include <rosidl_runtime_c/string_functions.h>
#include <std_msgs/msg/string.h>
#include <std_srvs/srv/trigger.h>

#include <string>

namespace {

/// One initialized context and node, shut down or finalized by the test.
class LiveContextTest : public ::testing::Test {
protected:
    void SetUp() override {
        rmw_init_options_t options = rmw_get_zero_initialized_init_options();
        ASSERT_EQ(rmw_rosette_test::init_options_as_rcl(&options), RMW_RET_OK);
        options.domain_id = 94;
        context_ = rmw_get_zero_initialized_context();
        ASSERT_EQ(rmw_init(&options, &context_), RMW_RET_OK);
        ASSERT_EQ(rmw_init_options_fini(&options), RMW_RET_OK);
        node_ = rmw_create_node(&context_, "live_context_test", "/");
        ASSERT_NE(node_, nullptr);
    }

    void TearDown() override {
        if (node_ != nullptr) {
            EXPECT_EQ(rmw_destroy_node(node_), RMW_RET_OK);
        }

        EXPECT_EQ(rmw_shutdown(&context_), RMW_RET_OK);
        EXPECT_EQ(rmw_context_fini(&context_), RMW_RET_OK);
        rmw_reset_error();
    }

    rmw_context_t context_{};
    rmw_node_t*   node_ = nullptr;
};

}  // namespace

// TC-RMW-ARG-005: after rmw_shutdown(), a node kept from before creates no
// endpoint, and the context creates no guard condition or wait set; each
// refusal is INVALID_ARGUMENT's "has been shut down", and the node is still
// destroyed.
TEST_F(LiveContextTest, NothingIsCreatedAfterShutdown) {
    const rosidl_message_type_support_t* msg = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String);
    const rosidl_service_type_support_t* srv = ROSIDL_GET_SRV_TYPE_SUPPORT(std_srvs, srv, Trigger);
    const rmw_publisher_options_t        pub_options = rmw_get_default_publisher_options();
    const rmw_subscription_options_t     sub_options = rmw_get_default_subscription_options();
    ASSERT_EQ(rmw_shutdown(&context_), RMW_RET_OK);

    EXPECT_EQ(rmw_create_publisher(node_, msg, "/after", &rmw_qos_profile_default, &pub_options),
              nullptr);
    EXPECT_NE(std::string(rmw_get_error_string().str).find("has been shut down"),
              std::string::npos);
    rmw_reset_error();
    EXPECT_EQ(rmw_create_subscription(node_, msg, "/after", &rmw_qos_profile_default, &sub_options),
              nullptr);
    rmw_reset_error();
    EXPECT_EQ(rmw_create_client(node_, srv, "/after", &rmw_qos_profile_services_default), nullptr);
    rmw_reset_error();
    EXPECT_EQ(rmw_create_service(node_, srv, "/after", &rmw_qos_profile_services_default), nullptr);
    rmw_reset_error();
    EXPECT_EQ(rmw_create_guard_condition(&context_), nullptr);
    rmw_reset_error();
    EXPECT_EQ(rmw_create_wait_set(&context_, 1), nullptr);
    rmw_reset_error();
    EXPECT_EQ(rmw_create_node(&context_, "too_late", "/"), nullptr);
    rmw_reset_error();

    EXPECT_EQ(rmw_destroy_node(node_), RMW_RET_OK);
    node_ = nullptr;
}

// TC-RMW-ARG-006: rmw_take_sequence() with a null message slot is refused
// before it takes anything, so the buffered sample is still there for the
// next take.
TEST_F(LiveContextTest, RefusedTakeSequenceLeavesTheSampleBuffered) {
    const rosidl_message_type_support_t* ts = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String);
    rmw_qos_profile_t                    qos = rmw_qos_profile_default;
    qos.depth = 1;
    const rmw_publisher_options_t    pub_options = rmw_get_default_publisher_options();
    const rmw_subscription_options_t sub_options = rmw_get_default_subscription_options();
    rmw_publisher_t*    pub = rmw_create_publisher(node_, ts, "/refused_take", &qos, &pub_options);
    rmw_subscription_t* sub =
        rmw_create_subscription(node_, ts, "/refused_take", &qos, &sub_options);
    rmw_wait_set_t* wait_set = rmw_create_wait_set(&context_, 1);
    ASSERT_NE(pub, nullptr);
    ASSERT_NE(sub, nullptr);
    ASSERT_NE(wait_set, nullptr);

    std_msgs__msg__String out{};
    std_msgs__msg__String__init(&out);
    rosidl_runtime_c__String__assign(&out.data, "still here");
    bool ready = false;
    for (int attempt = 0; attempt < 50 && !ready; ++attempt) {
        ASSERT_EQ(rmw_publish(pub, &out, nullptr), RMW_RET_OK);
        void*               subs_data[] = {sub->data};
        rmw_subscriptions_t subs{1, subs_data};
        rmw_time_t          timeout{0, 100000000};
        const rmw_ret_t     waited =
            rmw_wait(&subs, nullptr, nullptr, nullptr, nullptr, wait_set, &timeout);
        ASSERT_TRUE(waited == RMW_RET_OK || waited == RMW_RET_TIMEOUT);
        ready = subs.subscribers[0] != nullptr;
    }

    ASSERT_TRUE(ready) << "the loopback sample never arrived";

    rcutils_allocator_t         allocator = rcutils_get_default_allocator();
    rmw_message_sequence_t      messages = rmw_get_zero_initialized_message_sequence();
    rmw_message_info_sequence_t infos = rmw_get_zero_initialized_message_info_sequence();
    ASSERT_EQ(rmw_message_sequence_init(&messages, 1, &allocator), RMW_RET_OK);
    ASSERT_EQ(rmw_message_info_sequence_init(&infos, 1, &allocator), RMW_RET_OK);
    messages.data[0] = nullptr;
    size_t taken = 0;
    EXPECT_EQ(rmw_take_sequence(sub, 1, &messages, &infos, &taken, nullptr),
              RMW_RET_INVALID_ARGUMENT);
    rmw_reset_error();

    std_msgs__msg__String in{};
    std_msgs__msg__String__init(&in);
    bool one = false;
    ASSERT_EQ(rmw_take(sub, &in, &one, nullptr), RMW_RET_OK);
    EXPECT_TRUE(one);
    EXPECT_STREQ(in.data.data, "still here");

    std_msgs__msg__String__fini(&in);
    std_msgs__msg__String__fini(&out);
    EXPECT_EQ(rmw_message_info_sequence_fini(&infos), RMW_RET_OK);
    EXPECT_EQ(rmw_message_sequence_fini(&messages), RMW_RET_OK);
    EXPECT_EQ(rmw_destroy_wait_set(wait_set), RMW_RET_OK);
    EXPECT_EQ(rmw_destroy_subscription(node_, sub), RMW_RET_OK);
    EXPECT_EQ(rmw_destroy_publisher(node_, pub), RMW_RET_OK);
}
