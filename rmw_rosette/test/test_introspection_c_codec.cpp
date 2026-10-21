/**
 * @file test_introspection_c_codec.cpp
 * @brief Unit tests for message_codec.hpp and the introspection_c CDR codec.
 *
 * Golden bytes come from the XCDR1 rules the codec implements and agree with
 * rmw_fastrtps_cpp's output (tests/ros2/rmw_serialization_parity.py checks
 * the full corpus against a live Fast DDS). These tests pin the rules that
 * need no second middleware: type resolution, DDS type naming, string and
 * alignment encoding, round trips, and rejection of malformed input.
 */

#include <gtest/gtest.h>

#include <rmw/rmw.h>
#include <rosidl_runtime_c/primitives_sequence_functions.h>
#include <rosidl_runtime_c/string_functions.h>

#include <geometry_msgs/msg/pose_stamped.h>
#include <sensor_msgs/msg/image.h>
#include <sensor_msgs/msg/joint_state.h>
#include <std_msgs/msg/string.h>

#include <cstring>
#include <vector>

#include "rmw_rosette/message_codec.hpp"
#include "rmw_rosette/topic_naming.hpp"

namespace {

const rosidl_message_type_support_t* string_ts() {
    return ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String);
}

/// Encodes through the codec and prepends the header the RTPS writer adds.
std::vector<uint8_t> encode_payload(const rmw_rosette::MessageCodec& codec, const void* msg) {
    std::vector<uint8_t> out(512);
    out[0] = 0x00; out[1] = 0x01;
    const uint32_t n = rmw_rosette::serialize_message(codec, msg, out.data() + 4,
                                                      static_cast<uint32_t>(out.size() - 4));
    out.resize(n ? n + 4 : 0);
    return out;
}

}  // namespace

TEST(MessageCodec, ResolvesStdMsgsString) {
    rmw_rosette::MessageCodec codec;
    ASSERT_TRUE(rmw_rosette::resolve_message_codec(string_ts(), codec));
    EXPECT_NE(codec.kind, rmw_rosette::CodecKind::None);
    EXPECT_FALSE(rmw_rosette::resolve_message_codec(nullptr, codec));
    EXPECT_EQ(codec.kind, rmw_rosette::CodecKind::None);
}

TEST(MessageCodec, ComposesDdsTypeName) {
    rmw_rosette::MessageCodec codec;
    ASSERT_TRUE(rmw_rosette::resolve_message_codec(string_ts(), codec));
    char name[128];
    EXPECT_TRUE(rmw_rosette::compose_codec_type_name(codec, name, sizeof(name)));
    EXPECT_STREQ(name, "std_msgs::msg::dds_::String_");

    ASSERT_TRUE(rmw_rosette::resolve_message_codec(
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, PoseStamped), codec));
    EXPECT_TRUE(rmw_rosette::compose_codec_type_name(codec, name, sizeof(name)));
    EXPECT_STREQ(name, "geometry_msgs::msg::dds_::PoseStamped_");

    char small[8];
    EXPECT_FALSE(rmw_rosette::compose_codec_type_name(codec, small, sizeof(small)))
        << "a type name that does not fit is reported, not silently truncated";
}

TEST(TopicNaming, ReportsNamesThatDoNotFit) {
    char out[8];
    EXPECT_TRUE(rmw_rosette::compose_topic_name(out, sizeof(out), "/abcd"));
    EXPECT_STREQ(out, "rt/abcd");
    EXPECT_FALSE(rmw_rosette::compose_topic_name(out, sizeof(out), "/abcde"));
    EXPECT_FALSE(rmw_rosette::compose_service_request_topic(out, sizeof(out), "/a"));
    EXPECT_FALSE(rmw_rosette::compose_service_reply_topic(out, sizeof(out), "/ab"));
}

TEST(MessageCodec, StringGoldenBytes) {
    rmw_rosette::MessageCodec codec;
    ASSERT_TRUE(rmw_rosette::resolve_message_codec(string_ts(), codec));
    std_msgs__msg__String msg;
    std_msgs__msg__String__init(&msg);
    rosidl_runtime_c__String__assign(&msg.data, "hi");

    // uint32 length including NUL, bytes, NUL.
    const std::vector<uint8_t> expected = {0x00, 0x01, 0x00, 0x00, 3, 0, 0, 0, 'h', 'i', 0};
    EXPECT_EQ(encode_payload(codec, &msg), expected);
    std_msgs__msg__String__fini(&msg);
}

TEST(MessageCodec, PoseStampedAlignsDoublesToEight) {
    rmw_rosette::MessageCodec codec;
    ASSERT_TRUE(rmw_rosette::resolve_message_codec(
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, PoseStamped), codec));
    geometry_msgs__msg__PoseStamped msg;
    geometry_msgs__msg__PoseStamped__init(&msg);
    rosidl_runtime_c__String__assign(&msg.header.frame_id, "map");  // 4 + 4 bytes
    msg.pose.position.x = 1.0;

    const std::vector<uint8_t> bytes = encode_payload(codec, &msg);
    // Body: stamp (8) + frame_id (4 + 4) = 16, already 8-aligned, so x starts at 16.
    ASSERT_EQ(bytes.size(), 4u + 16u + 7u * 8u);
    double x = 0.0;
    std::memcpy(&x, bytes.data() + 4 + 16, sizeof(x));
    EXPECT_EQ(x, 1.0);
    geometry_msgs__msg__PoseStamped__fini(&msg);
}

TEST(MessageCodec, JointStateRoundTrip) {
    rmw_rosette::MessageCodec codec;
    ASSERT_TRUE(rmw_rosette::resolve_message_codec(
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState), codec));

    sensor_msgs__msg__JointState in;
    sensor_msgs__msg__JointState__init(&in);
    rosidl_runtime_c__String__Sequence__init(&in.name, 2);
    rosidl_runtime_c__String__assign(&in.name.data[0], "shoulder");
    rosidl_runtime_c__String__assign(&in.name.data[1], "elbow");
    rosidl_runtime_c__double__Sequence__init(&in.position, 2);
    in.position.data[0] = 0.5;
    in.position.data[1] = -1.25;

    const std::vector<uint8_t> bytes = encode_payload(codec, &in);
    ASSERT_FALSE(bytes.empty());

    sensor_msgs__msg__JointState out;
    sensor_msgs__msg__JointState__init(&out);
    ASSERT_TRUE(rmw_rosette::deserialize_message(codec, bytes.data(),
                                                 static_cast<uint32_t>(bytes.size()), &out));
    EXPECT_TRUE(sensor_msgs__msg__JointState__are_equal(&in, &out));

    sensor_msgs__msg__JointState__fini(&in);
    sensor_msgs__msg__JointState__fini(&out);
}

TEST(MessageCodec, RejectsMalformedPayloads) {
    rmw_rosette::MessageCodec codec;
    ASSERT_TRUE(rmw_rosette::resolve_message_codec(string_ts(), codec));
    std_msgs__msg__String msg;
    std_msgs__msg__String__init(&msg);

    const uint8_t big_endian[] = {0x00, 0x00, 0x00, 0x00, 0, 0, 0, 3, 'h', 'i', 0};
    EXPECT_FALSE(rmw_rosette::deserialize_message(codec, big_endian, sizeof(big_endian), &msg));

    const uint8_t truncated[] = {0x00, 0x01, 0x00, 0x00, 9, 0, 0, 0, 'h', 'i'};
    EXPECT_FALSE(rmw_rosette::deserialize_message(codec, truncated, sizeof(truncated), &msg));

    // A hostile length must be rejected before any allocation is attempted.
    const uint8_t huge[] = {0x00, 0x01, 0x00, 0x00, 0xff, 0xff, 0xff, 0x7f};
    EXPECT_FALSE(rmw_rosette::deserialize_message(codec, huge, sizeof(huge), &msg));

    const uint8_t short_header[] = {0x00, 0x01};
    EXPECT_FALSE(rmw_rosette::deserialize_message(codec, short_header, sizeof(short_header), &msg));
    std_msgs__msg__String__fini(&msg);
}

TEST(MessageCodec, RmwSerializeRoundTrip) {
    std_msgs__msg__String in;
    std_msgs__msg__String__init(&in);
    rosidl_runtime_c__String__assign(&in.data, "rosette ünïcode");

    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    rmw_serialized_message_t serialized = rmw_get_zero_initialized_serialized_message();
    ASSERT_EQ(rmw_serialized_message_init(&serialized, 0, &allocator), RMW_RET_OK);
    ASSERT_EQ(rmw_serialize(&in, string_ts(), &serialized), RMW_RET_OK);
    ASSERT_GE(serialized.buffer_length, 4u);
    EXPECT_EQ(serialized.buffer[0], 0x00);
    EXPECT_EQ(serialized.buffer[1], 0x01);

    std_msgs__msg__String out;
    std_msgs__msg__String__init(&out);
    ASSERT_EQ(rmw_deserialize(&serialized, string_ts(), &out), RMW_RET_OK);
    EXPECT_STREQ(out.data.data, "rosette ünïcode");

    EXPECT_EQ(rmw_serialized_message_fini(&serialized), RMW_RET_OK);
    std_msgs__msg__String__fini(&in);
    std_msgs__msg__String__fini(&out);
}

TEST(MessageCodec, SerializeToPoolFitsLargeMessages) {
    rmw_rosette::MessageCodec codec;
    ASSERT_TRUE(rmw_rosette::resolve_message_codec(
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Image), codec));

    for (size_t size : {size_t{8}, size_t{1500}, size_t{20000}, size_t{60000}}) {
        sensor_msgs__msg__Image img;
        sensor_msgs__msg__Image__init(&img);
        rosidl_runtime_c__uint8__Sequence__init(&img.data, size);
        for (size_t i = 0; i < size; ++i) img.data.data[i] = static_cast<uint8_t>(i * 13u);

        std::vector<uint8_t> reference(size + 256u);
        const uint32_t ref_len = rmw_rosette::serialize_message(
            codec, &img, reference.data(), static_cast<uint32_t>(reference.size()));
        ASSERT_GT(ref_len, 0u);

        rosette::rtps::BufRef pooled = rmw_rosette::serialize_to_pool(codec, &img);
        ASSERT_TRUE(pooled.valid()) << "size " << size;
        ASSERT_EQ(pooled.size, ref_len);
        EXPECT_EQ(std::memcmp(pooled.data, reference.data(), ref_len), 0);
        // The message lands in the smallest pool class that holds it.
        uint32_t smallest_fit = 0;
        for (uint32_t cls : {RTPS_POOL_CLASS0_SIZE, RTPS_POOL_CLASS1_SIZE, RTPS_POOL_CLASS2_SIZE,
                             RTPS_POOL_CLASS3_SIZE, RTPS_POOL_CLASS4_SIZE}) {
            if (cls >= ref_len) { smallest_fit = cls; break; }
        }
        EXPECT_EQ(pooled.capacity(), smallest_fit) << "size " << size;
        sensor_msgs__msg__Image__fini(&img);
    }
}
