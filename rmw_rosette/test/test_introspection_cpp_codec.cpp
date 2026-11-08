// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file test_introspection_cpp_codec.cpp
 * @brief Unit tests for the introspection_cpp CDR codec behind rclcpp messages.
 *
 * The C codec's golden bytes are the reference: a C++ message must encode to
 * exactly what the C mapping of the same contents encodes to, so a C++ node is
 * indistinguishable on the wire from an rclc or rclpy node. test_msgs carries
 * the shapes rclcpp brings that C does not: `std::vector<bool>`, `std::string`
 * sequences, bounded sequences, and `std::u16string`.
 */

#include <gtest/gtest.h>

#include <rcutils/error_handling.h>
#include <rosidl_runtime_c/string_functions.h>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <rosidl_typesupport_cpp/message_type_support.hpp>
#include <rosidl_typesupport_cpp/service_type_support.hpp>

#include <std_msgs/msg/string.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <test_msgs/msg/arrays.hpp>
#include <test_msgs/msg/bounded_sequences.hpp>
#include <test_msgs/msg/unbounded_sequences.hpp>
#include <test_msgs/msg/w_strings.hpp>

#include <cstdint>
#include <string>
#include <vector>

#include "rmw_rosette/message_codec.hpp"

namespace {

template <typename Message>
rmw_rosette::MessageCodec cpp_codec() {
    rmw_rosette::MessageCodec codec;
    EXPECT_TRUE(rmw_rosette::resolve_message_codec(
        rosidl_typesupport_cpp::get_message_type_support_handle<Message>(), codec));
    EXPECT_EQ(codec.kind, rmw_rosette::CodecKind::IntrospectionCpp);
    return codec;
}

/// Encodes through the codec and prepends the header the RTPS writer adds.
template <typename Message>
std::vector<uint8_t> encode_payload(const Message& msg) {
    std::vector<uint8_t> out(4096);
    out[0] = 0x00;
    out[1] = 0x01;
    const uint32_t n = rmw_rosette::serialize_message(cpp_codec<Message>(), &msg, out.data() + 4,
                                                      static_cast<uint32_t>(out.size() - 4));
    out.resize(n ? n + 4 : 0);
    return out;
}

/// Encodes a message and decodes it into a freshly constructed one.
template <typename Message>
Message round_trip(const Message& msg) {
    const std::vector<uint8_t> payload = encode_payload(msg);
    EXPECT_FALSE(payload.empty());
    Message decoded;
    EXPECT_TRUE(rmw_rosette::deserialize_message(cpp_codec<Message>(), payload.data(),
                                                 static_cast<uint32_t>(payload.size()), &decoded));
    return decoded;
}

}  // namespace

TEST(IntrospectionCppCodec, ResolvesCppHandleAndClearsErrorState) {
    using Message = std_msgs::msg::String;
    rmw_rosette::MessageCodec codec = cpp_codec<Message>();
    EXPECT_FALSE(rcutils_error_is_set());

    char name[64];
    ASSERT_TRUE(rmw_rosette::compose_codec_type_name(codec, name, sizeof(name)));
    EXPECT_STREQ(name, "std_msgs::msg::dds_::String_");
}

TEST(IntrospectionCppCodec, ResolvesServiceTypes) {
    rmw_rosette::MessageCodec request;
    rmw_rosette::MessageCodec response;
    ASSERT_TRUE(rmw_rosette::resolve_service_codecs(
        rosidl_typesupport_cpp::get_service_type_support_handle<std_srvs::srv::SetBool>(), request,
        response));
    EXPECT_EQ(request.kind, rmw_rosette::CodecKind::IntrospectionCpp);

    char name[64];
    ASSERT_TRUE(rmw_rosette::compose_codec_type_name(response, name, sizeof(name)));
    EXPECT_STREQ(name, "std_srvs::srv::dds_::SetBool_Response_");
}

TEST(IntrospectionCppCodec, MatchesTheCCodecByteForByte) {
    std_msgs::msg::String cpp_msg;
    cpp_msg.data = "hello rosette";

    std_msgs__msg__String c_msg;
    ASSERT_TRUE(std_msgs__msg__String__init(&c_msg));
    ASSERT_TRUE(rosidl_runtime_c__String__assign(&c_msg.data, "hello rosette"));

    rmw_rosette::MessageCodec c_codec;
    ASSERT_TRUE(rmw_rosette::resolve_message_codec(
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), c_codec));

    std::vector<uint8_t> c_bytes(128);
    const uint32_t       n =
        rmw_rosette::serialize_message(c_codec, &c_msg, c_bytes.data(), c_bytes.size());
    ASSERT_GT(n, 0u);
    c_bytes.resize(n);
    std_msgs__msg__String__fini(&c_msg);

    std::vector<uint8_t> cpp_payload = encode_payload(cpp_msg);
    ASSERT_GE(cpp_payload.size(), 4u);
    EXPECT_EQ(std::vector<uint8_t>(cpp_payload.begin() + 4, cpp_payload.end()), c_bytes);
}

TEST(IntrospectionCppCodec, RoundTripsNestedMessages) {
    geometry_msgs::msg::PoseStamped msg;
    msg.header.frame_id = "map";
    msg.header.stamp.sec = 42;
    msg.pose.position.x = 1.5;
    msg.pose.orientation.w = -0.25;
    EXPECT_EQ(round_trip(msg), msg);
}

TEST(IntrospectionCppCodec, RoundTripsUnboundedSequencesIncludingVectorBool) {
    test_msgs::msg::UnboundedSequences msg;
    msg.bool_values = {true, false, true, true, false, false, false, false, true};
    msg.int32_values = {-1, 2, -3};
    msg.float64_values = {0.5, -1.25};
    msg.string_values = {"", "alpha", "beta"};
    msg.basic_types_values.resize(2);
    msg.basic_types_values[1].bool_value = true;
    msg.basic_types_values[1].uint64_value = 77;
    EXPECT_EQ(round_trip(msg), msg);
}

TEST(IntrospectionCppCodec, VectorBoolEncodesOneByteEach) {
    test_msgs::msg::UnboundedSequences msg;
    msg.bool_values = {true, false, true};
    const std::vector<uint8_t> payload = encode_payload(msg);
    ASSERT_GE(payload.size(), 11u);

    // bool_values is the first field: count, then one normalized byte each.
    const std::vector<uint8_t> expected = {3, 0, 0, 0, 1, 0, 1};
    EXPECT_EQ(std::vector<uint8_t>(payload.begin() + 4, payload.begin() + 11), expected);
}

TEST(IntrospectionCppCodec, RoundTripsFixedArraysAndBoundedSequences) {
    test_msgs::msg::Arrays arrays;
    arrays.bool_values = {true, false, true};
    arrays.string_values = {"a", "bb", "ccc"};
    arrays.uint16_values = {1, 2, 3};
    EXPECT_EQ(round_trip(arrays), arrays);

    test_msgs::msg::BoundedSequences bounded;
    bounded.bool_values = {false, true};
    bounded.int64_values = {-9, 9};
    bounded.string_values = {"one"};
    EXPECT_EQ(round_trip(bounded), bounded);
}

TEST(IntrospectionCppCodec, RoundTripsWideStrings) {
    test_msgs::msg::WStrings msg;
    msg.wstring_value = u"héllo";
    msg.array_of_wstrings = {u"", u"a", u"世"};
    msg.bounded_sequence_of_wstrings = {u"x"};
    msg.unbounded_sequence_of_wstrings = {u"y", u"z"};
    EXPECT_EQ(round_trip(msg), msg);
}

TEST(IntrospectionCppCodec, RejectsTruncatedInputAtEveryLength) {
    test_msgs::msg::UnboundedSequences msg;
    msg.bool_values = {true, false};
    msg.string_values = {"truncate me"};
    const std::vector<uint8_t> payload = encode_payload(msg);
    ASSERT_GT(payload.size(), 8u);

    const auto codec = cpp_codec<test_msgs::msg::UnboundedSequences>();
    for (uint32_t len = 4; len < 40 && len < payload.size(); ++len) {
        test_msgs::msg::UnboundedSequences decoded;
        EXPECT_FALSE(rmw_rosette::deserialize_message(codec, payload.data(), len, &decoded))
            << "length " << len;
    }
}

TEST(IntrospectionCppCodec, RejectsAHugeSequenceLengthWithoutAllocating) {
    // bool_values claims four billion elements in a twelve-byte payload.
    const std::vector<uint8_t> payload = {0x00, 0x01, 0, 0, 0xFF, 0xFF, 0xFF, 0xFF, 1, 0, 1, 0};
    test_msgs::msg::UnboundedSequences decoded;
    EXPECT_FALSE(rmw_rosette::deserialize_message(cpp_codec<test_msgs::msg::UnboundedSequences>(),
                                                  payload.data(),
                                                  static_cast<uint32_t>(payload.size()), &decoded));
    EXPECT_TRUE(decoded.bool_values.empty());
}
