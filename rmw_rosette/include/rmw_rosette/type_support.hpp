/**
 * @file rmw_rosette/type_support.hpp
 * @brief Glue between rosidl-generated type support and rtps writers/readers.
 *
 * micro_ros_setup already generates, for every ROS message type, Micro-CDR
 * (ucdr)-based serialize/deserialize callbacks via
 * rosidl_typesupport_microxrcedds_c. Those callbacks produce standard CDR
 * on the wire — the same bytes a real Agent would forward to DDS — so
 * rmw_rosette reuses them directly instead of writing a second
 * serializer. This header extracts the function pointers and type-name
 * strings from the rosidl_message_type_support_t handle rcl hands to
 * rmw_create_publisher/rmw_create_subscription, and wraps them around a
 * plain byte buffer so the rest of rmw_rosette never has to touch
 * ucdrBuffer directly.
 *
 * Struct layout confirmed against upstream sources (not written from
 * memory — see the git history of this file for the correction):
 *   rosidl_runtime_c/message_type_support_struct.h  (ros2/rosidl, humble)
 *   rosidl_typesupport_microxrcedds_c/message_type_support.h
 *     (micro-ROS/rosidl_typesupport_microxrcedds)
 * Both are pulled in via this package's package.xml dependencies
 * (rosidl_runtime_c, rosidl_typesupport_microxrcedds_c) at real build
 * time; nothing here is vendored, to avoid a second, possibly-diverging
 * copy of structs those packages own.
 */

#pragma once

#include <rosidl_runtime_c/message_type_support_struct.h>
#include <rosidl_typesupport_microxrcedds_c/identifier.h>
#include <rosidl_typesupport_microxrcedds_c/message_type_support.h>

#include <cstdint>
#include <cstring>

namespace rmw_rosette {

/// Walks the type support's identifier chain (a rosidl_message_type_support_t
/// can wrap more than one typesupport family — see
/// get_message_typesupport_handle()'s doc comment in rosidl_runtime_c) down
/// to the microxrcedds_c one, and returns its callback struct. Returns
/// nullptr if this type wasn't compiled with microxrcedds_c support at
/// all — e.g. rcl was linked against a different typesupport, which is a
/// configuration bug worth surfacing loudly rather than silently no-op'ing.
inline const message_type_support_callbacks_t* get_message_typesupport(
    const rosidl_message_type_support_t* handle) noexcept {
    if (!handle) return nullptr;
    const rosidl_message_type_support_t* ts = get_message_typesupport_handle(
        handle, rosidl_typesupport_microxrcedds_c__identifier);
    if (!ts) return nullptr;
    return static_cast<const message_type_support_callbacks_t*>(ts->data);
}

/// Serializes `ros_message` into `out_buf` (capacity `out_cap`) using the
/// resolved microxrcedds callbacks. Returns the encoded length, or 0 on
/// failure (buffer too small, or the callback itself rejected the
/// message — ucdr_init_buffer()/cdr_serialize() both signal errors this
/// way rather than through exceptions, matching the no-exceptions posture
/// of the rest of rosette::rtps).
inline uint32_t serialize_message(const message_type_support_callbacks_t& callbacks,
                                   const void* ros_message,
                                   uint8_t* out_buf, uint32_t out_cap) noexcept {
    ucdrBuffer cdr;
    ucdr_init_buffer(&cdr, out_buf, out_cap);
    if (!callbacks.cdr_serialize(ros_message, &cdr)) {
        return 0;
    }
    return static_cast<uint32_t>(ucdr_buffer_length(&cdr));
}

/// Bytes rtps::MessageBuilder::append_data() always prepends to a DATA
/// submessage's payload (representation id CDR_LE + 2 zero option bytes —
/// confirmed against tests/spec's TC-BLD-10.2-001). It does this
/// unconditionally, regardless of what serialize_message() above produced,
/// so every CacheChange this stack ever delivers carries it — there is no
/// path where in_buf arrives without it.
constexpr uint32_t kRtpsEncapsulationHeaderSize = 4;

/// Deserializes `in_buf` (length `in_len`, as delivered in a
/// CacheChange::payload — see kRtpsEncapsulationHeaderSize) into
/// `ros_message`, which the caller must have already allocated/
/// initialized as an instance of the message type these callbacks belong
/// to (same contract rcl itself relies on — rmw_rosette does not know how
/// to default-construct an arbitrary ROS message type; that's
/// rosidl-generated code the caller already has).
inline bool deserialize_message(const message_type_support_callbacks_t& callbacks,
                                 const uint8_t* in_buf, uint32_t in_len,
                                 void* ros_message) noexcept {
    if (in_len < kRtpsEncapsulationHeaderSize) return false;
    ucdrBuffer cdr;
    ucdr_init_buffer(&cdr, const_cast<uint8_t*>(in_buf) + kRtpsEncapsulationHeaderSize,
                     in_len - kRtpsEncapsulationHeaderSize);
    return callbacks.cdr_deserialize(&cdr, ros_message);
}

}  // namespace rmw_rosette
