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
 * Struct layouts are taken from the upstream headers this package
 * depends on (package.xml: rosidl_runtime_c, rosidl_typesupport_microxrcedds_c):
 *   rosidl_runtime_c/message_type_support_struct.h
 *   rosidl_typesupport_microxrcedds_c/message_type_support.h
 * Neither is vendored here, so there is exactly one copy of each struct
 * in the build.
 */

#pragma once

#include <rosidl_runtime_c/message_type_support_struct.h>
#include <rosidl_typesupport_microxrcedds_c/identifier.h>
#include <rosidl_typesupport_microxrcedds_c/message_type_support.h>

#include <cstdint>
#include <cstring>

namespace rmw_rosette {

/**
 * @brief Resolves the microxrcedds_c serialization callbacks for a ROS
 * message type.
 *
 * `rmw_create_publisher()` and `rmw_create_subscription()` call this
 * once, at entity-creation time, to get the callback struct that
 * serialize_message()/deserialize_message() then use on every
 * publish/take for that entity's lifetime.
 *
 * A `rosidl_message_type_support_t` can wrap more than one typesupport
 * family (see `get_message_typesupport_handle()`'s doc comment in
 * rosidl_runtime_c); this walks that chain down to the
 * `rosidl_typesupport_microxrcedds_c` entry specifically and returns its
 * `data` pointer, already cast to the callbacks struct it's known to
 * hold for that identifier.
 *
 * @param[in] handle Type support handle as rcl passes it to
 *   `rmw_create_publisher()`/`rmw_create_subscription()`.
 * @return The resolved callbacks, or `nullptr` when `handle` is null or
 *   the type was not compiled with microxrcedds_c support (rcl linked
 *   against a different typesupport family).
 */
inline const message_type_support_callbacks_t* get_message_typesupport(
    const rosidl_message_type_support_t* handle) noexcept {
    if (!handle) return nullptr;
    const rosidl_message_type_support_t* ts = get_message_typesupport_handle(
        handle, rosidl_typesupport_microxrcedds_c__identifier);
    if (!ts) return nullptr;
    return static_cast<const message_type_support_callbacks_t*>(ts->data);
}

/**
 * @brief Encodes a ROS message into standard CDR using its resolved
 * microxrcedds callbacks.
 *
 * `rmw_publish()` calls this to turn the application's message struct
 * into the byte buffer it then hands to `IStatefulWriter::write_reliable()`.
 * The result carries no DDS encapsulation header (see
 * kRtpsEncapsulationHeaderSize below) — `MessageBuilder::append_data()`
 * adds that automatically when the writer places the DATA submessage on
 * the wire.
 *
 * Initializes a `ucdrBuffer` view over `out_buf` and delegates to
 * `callbacks.cdr_serialize()`, which walks the message's fields and
 * writes each one through Micro-CDR.
 *
 * @param[in] callbacks Resolved via get_message_typesupport().
 * @param[in] ros_message Message instance to encode.
 * @param[out] out_buf Destination buffer.
 * @param[in] out_cap Capacity of `out_buf`.
 * @return Encoded length in bytes, or 0 when `out_buf` was too small or
 *   `cdr_serialize()` itself rejected the message (both signaled through
 *   the return value rather than exceptions, matching the no-exceptions
 *   posture of the rest of rosette::rtps).
 */
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

/// Size, in bytes, of the DDS CDR encapsulation header (representation id
/// CDR_LE + 2 zero option bytes) that `rtps::MessageBuilder::append_data()`
/// unconditionally prepends to every DATA submessage's payload (see
/// `tests/spec`'s `TC-BLD-10.2-001`). Every `CacheChange::payload` this
/// stack delivers to a reader therefore carries this header ahead of the
/// application's CDR content — deserialize_message() below skips past it
/// before invoking the microxrcedds callbacks, which know nothing about
/// it and expect to start reading application fields immediately.
constexpr uint32_t kRtpsEncapsulationHeaderSize = 4;

/**
 * @brief Decodes a `CacheChange` payload into a ROS message using its
 * resolved microxrcedds callbacks.
 *
 * `rmw_take_with_info()` calls this once it has popped a buffered change
 * for a subscription, turning the received bytes back into the
 * application's message struct.
 *
 * Skips the leading kRtpsEncapsulationHeaderSize bytes (see that
 * constant), initializes a `ucdrBuffer` view over the remainder, and
 * delegates to `callbacks.cdr_deserialize()`.
 *
 * @param[in] callbacks Resolved via get_message_typesupport().
 * @param[in] in_buf Payload bytes as delivered in a `CacheChange::payload`.
 * @param[in] in_len Length of `in_buf`.
 * @param[out] ros_message Message instance to decode into; the caller
 *   must have already allocated/initialized it as an instance of the
 *   message type `callbacks` belongs to — rmw_rosette has no way to
 *   default-construct an arbitrary ROS message type itself, that being
 *   rosidl-generated code the caller already has.
 * @return `true` on success; `false` when `in_len` is smaller than the
 *   encapsulation header or `cdr_deserialize()` itself failed.
 */
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
