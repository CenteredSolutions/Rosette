/**
 * @file rmw_rosette/introspection_c_codec.hpp
 * @brief CDR encoding of C ROS messages by walking their introspection metadata.
 *
 * Every rosidl-generated message type ships `rosidl_typesupport_introspection_c`
 * metadata: a `MessageMembers` table describing each field's type, offset,
 * bounds, and accessor functions. This codec walks that table to encode a C
 * message struct as XCDR1 and to decode XCDR1 back into one, which lets
 * rmw_rosette carry any message type a stock ROS 2 installation provides
 * (rclpy and rclc both hand rmw C message structs). rmw_cyclonedds_cpp takes
 * the same approach.
 *
 * Wire rules follow rosidl_typesupport_fastrtps_c, the reference serializer
 * for ROS 2 Humble, so payloads are byte-identical to rmw_fastrtps_cpp's:
 * - `string`: uint32 length including the NUL terminator, bytes, NUL.
 * - `wstring`: uint32 character count (no terminator), then each UTF-16
 *   code unit widened to uint32 (Fast-CDR serializes `wchar_t` as 4 bytes).
 * - `wchar`: one uint32.
 * - fixed arrays: elements back to back; sequences: uint32 count, then elements.
 * - nested messages: fields inline, no padding of their own.
 * `long double` fields have no interoperable CDR mapping in ROS 2 and are
 * rejected.
 */

#pragma once

#include <rosidl_typesupport_introspection_c/message_introspection.h>

#include "cdr_stream.hpp"

namespace rmw_rosette {

using IntrospectionMembers = rosidl_typesupport_introspection_c__MessageMembers;

/**
 * @brief Encodes a C ROS message as an XCDR1 body.
 *
 * rmw_publish() reaches this through message_codec.hpp when a type resolves to
 * introspection metadata. The output carries no encapsulation header; the
 * RTPS writer prepends it.
 *
 * Visits `members` in declaration order. Primitive arrays and sequences are
 * copied in one block after a single alignment step, since C lays them out
 * contiguously and this codec requires a little-endian host; strings,
 * wide characters, and nested messages are encoded element by element.
 *
 * @param[in]     members     Introspection table for the message type.
 * @param[in]     ros_message Message instance laid out as `members` describes.
 * @param[in,out] out         Stream positioned at the start of the CDR body.
 * @return `true` when every field was written; `false` when `out` overflowed
 *   or a field has a type with no CDR mapping (`long double`).
 *
 * Verification:
 * - Correct means: output is byte-identical to rosidl_typesupport_fastrtps_c
 *   for the same message, and decode_introspection_c() restores an equal
 *   message.
 * - Covered by: rmw_rosette/test/test_introspection_c_codec.cpp (golden bytes
 *   for std_msgs/String, round trips for sensor_msgs/JointState and
 *   geometry_msgs/PoseStamped).
 *
 * References:
 * - OMG CDR, CORBA 3.4 Part 2 §9.3.1 (primitive alignment and encoding)
 * - OMG DDS-XTypes 1.3 §7.4.3.4 (PLAIN_CDR / XCDR1)
 * - rosidl_typesupport_fastrtps_c/resource/msg__type_support_c.cpp.em (Humble)
 */
bool encode_introspection_c(const IntrospectionMembers& members, const void* ros_message,
                            CdrWriter& out) noexcept;

/**
 * @brief Decodes an XCDR1 body into an initialized C ROS message.
 *
 * rmw_take_with_info() reaches this through message_codec.hpp. The caller
 * passes a message already initialized by its generated `__init` function
 * (rcl does this), so sequences and strings may be resized in place.
 *
 * Mirrors encode_introspection_c(). Sequence and string lengths are checked
 * against the remaining input before anything is allocated, so a corrupt or
 * hostile length cannot trigger a huge allocation, and bounded sequences and
 * strings reject lengths beyond their declared bound.
 *
 * @param[in]     members     Introspection table for the message type.
 * @param[in,out] in          Stream positioned at the start of the CDR body.
 * @param[out]    ros_message Initialized message instance to overwrite.
 * @return `true` on success; `false` on truncated input, a bound violation,
 *   an allocation failure, or an unsupported field type. `ros_message` may be
 *   partially written on failure but stays valid for its `__fini` function.
 *
 * Verification:
 * - Correct means: decodes every payload rmw_fastrtps_cpp produces for the
 *   type, and rejects truncated or over-bound input without over-reading.
 * - Covered by: rmw_rosette/test/test_introspection_c_codec.cpp.
 *
 * References:
 * - OMG CDR, CORBA 3.4 Part 2 §9.3.1
 * - OMG DDS-XTypes 1.3 §7.4.3.4
 */
bool decode_introspection_c(const IntrospectionMembers& members, CdrReader& in,
                            void* ros_message) noexcept;

}  // namespace rmw_rosette
