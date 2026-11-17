// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file test_publish_fragment_limit.cpp
 * @brief A publish that would need more DATA_FRAGs than a reader can track
 *        fails, through every publish entry point.
 *
 * A reader tracks at most 256 fragments of one sample (a FragmentNumberSet,
 * DDSI-RTPS 2.5 §8.3.5.13). Over a 1472-octet datagram limit that caps a
 * sample near 358 KB, well below the hosted sample limit, so the writer must
 * refuse a larger one and report it rather than accept a sample no reader
 * could ever complete.
 */

#include <gtest/gtest.h>

#include "rcl_default_init_options.hpp"

#include <rcutils/allocator.h>
#include <rmw/error_handling.h>
#include <rmw/rmw.h>
#include <rmw/serialized_message.h>
#include <rosidl_runtime_c/string_functions.h>
#include <std_msgs/msg/string.h>

#include <cstdlib>
#include <cstring>
#include <string>

namespace {

/// Sample octets that need more than 256 fragments at a 1472-octet limit.
constexpr size_t kTooManyFragments = 400000u;
/// Sample octets above the largest pool block that still fit 256 fragments.
constexpr size_t kHeapSampleThatFits = 100000u;

/// A context limited to 1472-octet datagrams, with one publisher.
class PublishFragmentLimitTest : public ::testing::Test {
protected:
    void SetUp() override {
        ASSERT_EQ(setenv("RMW_ROSETTE_MAX_DATAGRAM_SIZE", "1472", 1), 0);
        rmw_init_options_t options = rmw_get_zero_initialized_init_options();
        ASSERT_EQ(rmw_rosette_test::init_options_as_rcl(&options), RMW_RET_OK);
        options.domain_id = 95;
        context_ = rmw_get_zero_initialized_context();
        ASSERT_EQ(rmw_init(&options, &context_), RMW_RET_OK);
        ASSERT_EQ(rmw_init_options_fini(&options), RMW_RET_OK);
        node_ = rmw_create_node(&context_, "fragment_limit_test", "/");
        ASSERT_NE(node_, nullptr);

        const rosidl_message_type_support_t* ts =
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String);
        rmw_qos_profile_t       qos = rmw_qos_profile_default;
        rmw_publisher_options_t options_pub = rmw_get_default_publisher_options();
        pub_ = rmw_create_publisher(node_, ts, "/fragment_limit", &qos, &options_pub);
        ASSERT_NE(pub_, nullptr);
    }

    void TearDown() override {
        if (pub_ != nullptr) {
            EXPECT_EQ(rmw_destroy_publisher(node_, pub_), RMW_RET_OK);
        }

        if (node_ != nullptr) {
            EXPECT_EQ(rmw_destroy_node(node_), RMW_RET_OK);
        }

        EXPECT_EQ(rmw_shutdown(&context_), RMW_RET_OK);
        EXPECT_EQ(rmw_context_fini(&context_), RMW_RET_OK);
        unsetenv("RMW_ROSETTE_MAX_DATAGRAM_SIZE");
        rmw_reset_error();
    }

    /// @brief Publishes `body_octets` of CDR body through rmw_publish_serialized_message().
    rmw_ret_t publish_serialized(size_t body_octets) {
        rcutils_allocator_t      allocator = rcutils_get_default_allocator();
        rmw_serialized_message_t msg = rmw_get_zero_initialized_serialized_message();
        EXPECT_EQ(rmw_serialized_message_init(&msg, body_octets + 4u, &allocator), RMW_RET_OK);
        std::memset(msg.buffer, 0, body_octets + 4u);
        msg.buffer[1] = 0x01u;  // CDR_LE encapsulation
        msg.buffer_length = body_octets + 4u;
        const rmw_ret_t ret = rmw_publish_serialized_message(pub_, &msg, nullptr);
        EXPECT_EQ(rmw_serialized_message_fini(&msg), RMW_RET_OK);
        return ret;
    }

    /// @brief Publishes a std_msgs/String of `octets` characters through rmw_publish().
    rmw_ret_t publish_string(size_t octets) {
        std_msgs__msg__String msg{};
        EXPECT_TRUE(std_msgs__msg__String__init(&msg));
        const std::string data(octets, 'x');
        EXPECT_TRUE(rosidl_runtime_c__String__assign(&msg.data, data.c_str()));
        const rmw_ret_t ret = rmw_publish(pub_, &msg, nullptr);
        std_msgs__msg__String__fini(&msg);
        return ret;
    }

    rmw_context_t    context_{};
    rmw_node_t*      node_ = nullptr;
    rmw_publisher_t* pub_ = nullptr;
};

TEST_F(PublishFragmentLimitTest, SerializedPublishBeyondTheFragmentLimitFails) {
    EXPECT_EQ(publish_serialized(kTooManyFragments), RMW_RET_ERROR);
    rmw_reset_error();
    EXPECT_EQ(publish_serialized(kHeapSampleThatFits), RMW_RET_OK);
}

TEST_F(PublishFragmentLimitTest, PublishBeyondTheFragmentLimitFails) {
    EXPECT_EQ(publish_string(kTooManyFragments), RMW_RET_ERROR);
    rmw_reset_error();
    EXPECT_EQ(publish_string(kHeapSampleThatFits), RMW_RET_OK);
}

}  // namespace
