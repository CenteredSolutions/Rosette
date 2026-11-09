// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file rmw_rosette/introspection_cpp_codec.hpp
 * @brief CDR encoding of C++ ROS messages by walking their introspection metadata.
 *
 * rclcpp hands rmw a `rosidl_typesupport_cpp` handle whose messages are C++
 * structs, with `std::string` and `std::vector` members. This codec is the
 * counterpart of introspection_c_codec.hpp for that mapping and shares its
 * walk and wire rules, so a C++ node's payloads are byte-identical to a C
 * node's.
 *
 * C++ message containers allocate from the heap whenever a sample is decoded.
 * The codec is therefore compiled only into builds that define
 * `RMW_ROSETTE_HAVE_INTROSPECTION_CPP_TYPESUPPORT`, which CMake sets from the
 * `RMW_ROSETTE_CXX_TYPESUPPORT` option on hosted builds. Firmware runs stock
 * rclc over C message structs and never compiles or links this file.
 */

#pragma once

#include <rosidl_typesupport_introspection_cpp/message_introspection.hpp>

#include "cdr_stream.hpp"

namespace rmw_rosette {

using IntrospectionCppMembers = rosidl_typesupport_introspection_cpp::MessageMembers;

/**
 * @brief Encodes a C++ ROS message as an XCDR1 body.
 *
 * rmw_publish() reaches this through message_codec.hpp when a type resolves to
 * C++ introspection metadata. The output carries no encapsulation header; the
 * RTPS writer prepends it.
 *
 * @param[in]     members     Introspection table for the message type.
 * @param[in]     ros_message Message instance laid out as `members` describes.
 * @param[in,out] out         Stream positioned at the start of the CDR body.
 * @return `true` when every field was written; `false` when `out` overflowed
 *   or a field has a type with no CDR mapping (`long double`).
 *
 * Verification:
 * - Correct means: output is byte-identical to encode_introspection_c() for
 *   the same message contents, and decode_introspection_cpp() restores an
 *   equal message.
 * - Covered by: rmw_rosette/test/test_introspection_cpp_codec.cpp, and the
 *   test_communication rclcpp cases in the rmw-rosette conformance job.
 *
 * References:
 * - OMG DDS-XTypes 1.3 §7.4.3.4 (PLAIN_CDR / XCDR1)
 * - rosidl_typesupport_fastrtps_cpp/resource/msg__type_support.cpp.em (Humble)
 */
bool encode_introspection_cpp(const IntrospectionCppMembers& members, const void* ros_message,
                              CdrWriter& out) noexcept;

/**
 * @brief Decodes an XCDR1 body into a constructed C++ ROS message.
 *
 * rmw_take_with_info() reaches this through message_codec.hpp. Lengths are
 * checked against the remaining input and declared bounds before any
 * container is resized, so a corrupt length cannot request a huge allocation.
 * An exception from a container is caught here and reported as a failure.
 *
 * @param[in]     members     Introspection table for the message type.
 * @param[in,out] in          Stream positioned at the start of the CDR body.
 * @param[out]    ros_message Constructed message instance to overwrite.
 * @return `true` on success; `false` on truncated input, a bound violation,
 *   an allocation failure, or an unsupported field type. `ros_message` may be
 *   partially written on failure but stays valid for destruction.
 *
 * Verification:
 * - Correct means: decodes the bytes the C codec and rmw_fastrtps_cpp produce
 *   for the type, and rejects truncated or over-bound input without
 *   over-reading.
 * - Covered by: rmw_rosette/test/test_introspection_cpp_codec.cpp.
 *
 * References:
 * - OMG DDS-XTypes 1.3 §7.4.3.4
 */
bool decode_introspection_cpp(const IntrospectionCppMembers& members, CdrReader& in,
                              void* ros_message) noexcept;

}  // namespace rmw_rosette
