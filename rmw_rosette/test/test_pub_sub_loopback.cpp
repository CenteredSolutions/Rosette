/**
 * @file test_pub_sub_loopback.cpp
 * @brief One rmw_rosette context, publish -> wait -> take over real UDP
 *        loopback, using the generated std_msgs/String type support.
 *
 * Exercises the rmw entry points on the data path end to end inside one
 * participant: init, node, publisher, subscription, a discovery pump through
 * rmw_wait(), publish, wait for readiness, and take. Cross-vendor delivery is
 * covered by tests/ros2 against a stock RMW.
 *
 * Verification:
 * - Correct means: the taken message equals the published one, and the
 *   subscription is reported ready by rmw_wait() before the take.
 */

#include <gtest/gtest.h>

#include <rmw/rmw.h>

#include <rosidl_runtime_c/string_functions.h>
#include <std_msgs/msg/string.h>

TEST(RmwRosetteLoopback, PublishAndTake) {
    rmw_init_options_t init_options = rmw_get_zero_initialized_init_options();
    ASSERT_EQ(rmw_init_options_init(&init_options, rcutils_get_default_allocator()), RMW_RET_OK);

    rmw_context_t context = rmw_get_zero_initialized_context();
    ASSERT_EQ(rmw_init(&init_options, &context), RMW_RET_OK);

    rmw_node_t* node = rmw_create_node(&context, "loopback_test_node", "/");
    ASSERT_NE(node, nullptr);

    rmw_qos_profile_t qos = rmw_qos_profile_default;
    qos.depth = 1;
    qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;
    qos.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;

    const rosidl_message_type_support_t* ts =
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String);

    rmw_publisher_options_t pub_options = rmw_get_default_publisher_options();
    rmw_publisher_t* pub = rmw_create_publisher(node, ts, "/loopback_topic", &qos, &pub_options);
    ASSERT_NE(pub, nullptr);

    rmw_subscription_options_t sub_options = rmw_get_default_subscription_options();
    rmw_subscription_t* sub =
        rmw_create_subscription(node, ts, "/loopback_topic", &qos, &sub_options);
    ASSERT_NE(sub, nullptr);

    rmw_wait_set_t* wait_set = rmw_create_wait_set(&context, 4);
    ASSERT_NE(wait_set, nullptr);

    // Drive SPDP/SEDP to completion — matching endpoint discovery is not
    // instantaneous, and there's no dedicated "wait for discovery" rmw
    // call, so this leans on rmw_wait()'s own polling with zero
    // subscriptions as the discovery-pump loop.
    for (int i = 0; i < 50; ++i) {
        rmw_time_t t{0, 100000000};  // 100ms
        rmw_wait(nullptr, nullptr, nullptr, nullptr, nullptr, wait_set, &t);
    }

    std_msgs__msg__String out_msg{};
    std_msgs__msg__String__init(&out_msg);
    rosidl_runtime_c__String__assign(&out_msg.data, "hello from rmw_rosette");
    ASSERT_EQ(rmw_publish(pub, &out_msg, nullptr), RMW_RET_OK);

    void* subs_data[] = {sub->data};
    rmw_subscriptions_t subs{1, subs_data};
    rmw_time_t timeout{5, 0};
    ASSERT_EQ(rmw_wait(&subs, nullptr, nullptr, nullptr, nullptr, wait_set, &timeout), RMW_RET_OK);
    ASSERT_NE(subs.subscribers[0], nullptr) << "subscription never became ready";

    std_msgs__msg__String in_msg;
    std_msgs__msg__String__init(&in_msg);
    bool taken = false;
    ASSERT_EQ(rmw_take(sub, &in_msg, &taken, nullptr), RMW_RET_OK);
    ASSERT_TRUE(taken);
    EXPECT_STREQ(in_msg.data.data, "hello from rmw_rosette");

    std_msgs__msg__String__fini(&in_msg);
    std_msgs__msg__String__fini(&out_msg);
    rmw_destroy_wait_set(wait_set);
    rmw_destroy_subscription(node, sub);
    rmw_destroy_publisher(node, pub);
    rmw_destroy_node(node);
    rmw_context_fini(&context);
}
