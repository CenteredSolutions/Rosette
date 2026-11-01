// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file rmw_serialize.cpp
 * @brief rmw_serialize / rmw_deserialize / rmw_get_serialized_message_size.
 *
 * An `rmw_serialized_message_t` holds a complete serialized payload: the
 * 4-byte CDR encapsulation header followed by the CDR body, exactly the
 * bytes that follow a DATA submessage's inline QoS on the wire. This is the
 * representation every stock RMW uses, so bags recorded on any of them
 * replay here. message_codec.hpp's serialize_message() emits only the body
 * (the RTPS writer prepends the header on the publish path), so
 * rmw_serialize() writes the header itself; deserialize_message() already
 * expects and checks the header, so rmw_deserialize() passes the buffer
 * through unchanged.
 */

#include "rmw_rosette/identifier.h"
#include "rmw_rosette/message_codec.hpp"

#include <rmw/rmw.h>
#include <rmw/error_handling.h>
#include <rcutils/types/uint8_array.h>

namespace {

/// Starting capacity for rmw_serialize() when the caller supplies none.
constexpr size_t kInitialSerializeCapacity = 256;
/// Upper bound on rmw_serialize()'s buffer growth, far above any payload
/// rosette::rtps can transmit.
constexpr size_t kMaxSerializeCapacity = 64u * 1024u * 1024u;

}  // namespace

extern "C" {

/**
 * @brief Encodes a ROS message into a standalone rmw_serialized_message_t.
 *
 * Used by rclpy's `serialize_message()`, rosbag2 recording, and any rcl
 * user that publishes pre-serialized data. The result is interchangeable
 * with the buffers stock RMWs produce.
 *
 * Writes the CDR_LE encapsulation header, then encodes the body with the
 * type's resolved codec. The codec reports overflow rather than a size, so
 * the buffer grows by doubling (through the message's own allocator) until
 * the body fits or kMaxSerializeCapacity is reached.
 *
 * @param[in]     ros_message        Message to encode.
 * @param[in]     type_support       Type support handle for the message.
 * @param[in,out] serialized_message Initialized byte array; resized as
 *   needed, `buffer_length` set to header plus body on success.
 * @return `RMW_RET_OK` on success; `RMW_RET_INVALID_ARGUMENT` for null
 *   arguments; `RMW_RET_UNSUPPORTED` when the type resolves to no codec;
 *   `RMW_RET_BAD_ALLOC` when resizing fails; `RMW_RET_ERROR` when the
 *   message cannot be encoded within kMaxSerializeCapacity.
 *
 * Verification:
 * - Correct means: output equals rmw_fastrtps_cpp's rmw_serialize() for the
 *   same message, and rmw_deserialize() restores an equal message.
 * - Covered by: rmw_rosette/test/test_introspection_c_codec.cpp
 *   (RmwSerializeRoundTrip).
 *
 * References:
 * - rmw/rmw.h (Humble), `rmw_serialize`
 * - OMG DDSI-RTPS 2.5 §10.2 (SerializedPayloadHeader)
 */
rmw_ret_t rmw_serialize(const void* ros_message,
                        const rosidl_message_type_support_t* type_support,
                        rmw_serialized_message_t* serialized_message) {
    if (!ros_message || !type_support || !serialized_message) return RMW_RET_INVALID_ARGUMENT;

    rmw_rosette::MessageCodec codec;
    if (!rmw_rosette::resolve_message_codec(type_support, codec)) {
        RMW_SET_ERROR_MSG("rmw_rosette: rmw_serialize: unsupported type support");
        return RMW_RET_UNSUPPORTED;
    }

    constexpr size_t hdr = rmw_rosette::kRtpsEncapsulationHeaderSize;
    size_t capacity = serialized_message->buffer_capacity;
    if (capacity < kInitialSerializeCapacity) capacity = kInitialSerializeCapacity;

    while (capacity <= kMaxSerializeCapacity) {
        if (serialized_message->buffer_capacity < capacity &&
            rcutils_uint8_array_resize(serialized_message, capacity) != RCUTILS_RET_OK) {
            return RMW_RET_BAD_ALLOC;
        }
        uint8_t* buf = serialized_message->buffer;
        buf[0] = 0x00; buf[1] = 0x01; buf[2] = 0x00; buf[3] = 0x00;  // CDR_LE, no options
        const uint32_t body = rmw_rosette::serialize_message(
            codec, ros_message, buf + hdr, static_cast<uint32_t>(capacity - hdr));
        if (body > 0) {
            serialized_message->buffer_length = hdr + body;
            return RMW_RET_OK;
        }
        capacity *= 2;
    }
    RMW_SET_ERROR_MSG("rmw_rosette: rmw_serialize: message could not be encoded");
    return RMW_RET_ERROR;
}

/**
 * @brief Decodes a standalone rmw_serialized_message_t into a ROS message.
 *
 * The inverse of rmw_serialize(); also accepts buffers produced by any stock
 * RMW, provided they use the CDR_LE representation.
 *
 * @param[in]  serialized_message Complete payload, header included.
 * @param[in]  type_support       Type support handle for the message.
 * @param[out] ros_message        Initialized message instance to overwrite.
 * @return `RMW_RET_OK` on success; `RMW_RET_INVALID_ARGUMENT` for null
 *   arguments; `RMW_RET_UNSUPPORTED` when the type resolves to no codec;
 *   `RMW_RET_ERROR` for a malformed or non-CDR_LE payload.
 *
 * Verification:
 * - Correct means: decodes every rmw_fastrtps_cpp serialized buffer of the
 *   type into an equal message.
 * - Covered by: rmw_rosette/test/test_introspection_c_codec.cpp
 *   (RmwSerializeRoundTrip).
 *
 * References:
 * - rmw/rmw.h (Humble), `rmw_deserialize`
 */
rmw_ret_t rmw_deserialize(const rmw_serialized_message_t* serialized_message,
                          const rosidl_message_type_support_t* type_support,
                          void* ros_message) {
    if (!serialized_message || !type_support || !ros_message) return RMW_RET_INVALID_ARGUMENT;

    rmw_rosette::MessageCodec codec;
    if (!rmw_rosette::resolve_message_codec(type_support, codec)) {
        RMW_SET_ERROR_MSG("rmw_rosette: rmw_deserialize: unsupported type support");
        return RMW_RET_UNSUPPORTED;
    }
    if (!rmw_rosette::deserialize_message(codec, serialized_message->buffer,
                                          static_cast<uint32_t>(serialized_message->buffer_length),
                                          ros_message)) {
        RMW_SET_ERROR_MSG("rmw_rosette: rmw_deserialize: malformed payload");
        return RMW_RET_ERROR;
    }
    return RMW_RET_OK;
}

/**
 * @brief Computes the maximum serialized size of a bounded message type.
 *
 * Unsupported, as in rmw_fastrtps_cpp and rmw_cyclonedds_cpp; rcl treats
 * `RMW_RET_UNSUPPORTED` as "size unknown".
 *
 * @param[in] type_support Unused.
 * @param[in] message_bounds Unused.
 * @param[out] size Always set to 0.
 * @return `RMW_RET_UNSUPPORTED`, or `RMW_RET_INVALID_ARGUMENT` if `size`
 *   is null.
 */
rmw_ret_t rmw_get_serialized_message_size(const rosidl_message_type_support_t* type_support,
                                           const rosidl_runtime_c__Sequence__bound* message_bounds,
                                           size_t* size) {
    (void)type_support; (void)message_bounds;
    if (!size) return RMW_RET_INVALID_ARGUMENT;
    *size = 0;
    RMW_SET_ERROR_MSG("rmw_rosette: rmw_get_serialized_message_size is unsupported");
    return RMW_RET_UNSUPPORTED;
}

}  // extern "C"
