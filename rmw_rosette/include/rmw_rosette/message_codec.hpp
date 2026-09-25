/**
 * @file rmw_rosette/message_codec.hpp
 * @brief Chooses and runs the CDR serializer for a ROS message type.
 *
 * rcl hands rmw a `rosidl_message_type_support_t` handle that may front
 * several typesupport families. rmw_rosette can serialize through two of them:
 *
 * - `rosidl_typesupport_microxrcedds_c`: generated Micro-CDR callbacks that
 *   micro-ROS firmware builds already link. Compiled in only when the package
 *   is found at build time (`RMW_ROSETTE_HAVE_MICROXRCEDDS_TYPESUPPORT`), and
 *   preferred when present because it is the smallest code path on a
 *   microcontroller.
 * - `rosidl_typesupport_introspection_c`: field metadata that every stock
 *   ROS 2 installation generates, walked by introspection_c_codec.hpp. This
 *   is what rclpy and rclc applications on a desktop resolve to.
 *
 * A MessageCodec records which family resolved and is resolved once per
 * publisher or subscription, then used on every publish and take. Both
 * families produce the same XCDR1 bytes, so peers cannot tell them apart.
 *
 * Payload framing: every `CacheChange::payload` delivered by rosette::rtps
 * starts with the 4-byte encapsulation header that
 * `rtps::MessageBuilder::append_data()` prepends (TC-BLD-10.2-001).
 * serialize_message() therefore emits only the CDR body, and
 * deserialize_message() checks and strips the header.
 */

#pragma once

#include <rosidl_runtime_c/message_type_support_struct.h>
#include <rosidl_runtime_c/service_type_support_struct.h>

#include "rosette/rtps/buf_pool.hpp"

#include <cstddef>
#include <cstdint>

namespace rmw_rosette {

/// @brief Typesupport family a MessageCodec dispatches to.
enum class CodecKind : uint8_t {
    None,            ///< Unresolved; every operation fails.
    MicroXrceDds,    ///< rosidl_typesupport_microxrcedds_c callbacks.
    IntrospectionC,  ///< rosidl_typesupport_introspection_c metadata.
};

/// @brief A resolved serializer for one message type.
struct MessageCodec {
    CodecKind   kind = CodecKind::None;
    const void* impl = nullptr;  ///< Family-specific table; owned by the typesupport library.
};

/// Size in bytes of the CDR encapsulation header (representation identifier
/// plus two option bytes) at the start of every RTPS serialized payload.
constexpr uint32_t kRtpsEncapsulationHeaderSize = 4;

/**
 * @brief Resolves the serializer for a message type support handle.
 *
 * rmw_create_publisher() and rmw_create_subscription() call this once per
 * entity. It asks the handle for the microxrcedds_c family first (when
 * compiled in), then introspection_c, using
 * `get_message_typesupport_handle()`, which also triggers the lazy library
 * loading done by `rosidl_typesupport_c` dispatch handles.
 *
 * @param[in]  handle Type support handle as rcl passes it; may be null.
 * @param[out] codec  Receives the resolved family and table.
 * @return `true` when a supported family resolved; `false` when `handle` is
 *   null or offers neither family, leaving `codec` as CodecKind::None.
 *
 * Verification:
 * - Correct means: stock desktop `std_msgs` types resolve to IntrospectionC,
 *   and micro-ROS builds resolve to MicroXrceDds.
 * - Covered by: rmw_rosette/test/test_introspection_c_codec.cpp
 *   (ResolvesStdMsgsString).
 *
 * References:
 * - rosidl_runtime_c/message_type_support_struct.h (Humble),
 *   `get_message_typesupport_handle`
 */
bool resolve_message_codec(const rosidl_message_type_support_t* handle,
                           MessageCodec& codec) noexcept;

/**
 * @brief Resolves the request and response serializers of a service type.
 *
 * rmw_create_service() and rmw_create_client() call this once per entity.
 * It prefers microxrcedds_c when compiled in, then introspection_c, like
 * resolve_message_codec(). Both families describe a service as a pair of
 * message types, so the resulting codecs work with serialize_message() and
 * deserialize_message() unchanged, and compose_codec_type_name() yields the
 * `<pkg>::srv::dds_::<Srv>_Request_` / `_Response_` names SEDP announces.
 *
 * @param[in]  handle   Service type support handle as rcl passes it.
 * @param[out] request  Receives the request message codec.
 * @param[out] response Receives the response message codec.
 * @return `true` when both resolved; `false` otherwise (both left
 *   CodecKind::None).
 *
 * Verification:
 * - Correct means: example_interfaces/srv/AddTwoInts resolves, and its
 *   request type name is `example_interfaces::srv::dds_::AddTwoInts_Request_`.
 * - Covered by: rmw_rosette/test/test_introspection_c_codec.cpp
 *   (ResolvesServiceCodecs).
 *
 * References:
 * - rosidl_runtime_c/service_type_support_struct.h (Humble)
 * - rosidl_typesupport_introspection_c/service_introspection.h (Humble)
 */
bool resolve_service_codecs(const rosidl_service_type_support_t* handle,
                            MessageCodec& request, MessageCodec& response) noexcept;

/**
 * @brief Writes the DDS type name for the codec's message type.
 *
 * SEDP announces this name, and stock RMWs match only endpoints whose type
 * name follows the ROS 2 DDS convention `<pkg>::<subfolder>::dds_::<Name>_`
 * (for example `std_msgs::msg::dds_::String_`).
 *
 * @param[in]  codec Resolved codec.
 * @param[out] out   Destination buffer, always NUL-terminated when `cap > 0`.
 * @param[in]  cap   Capacity of `out` in bytes.
 * @return `false` when the name did not fit `cap` and was truncated, or the
 *   codec is unresolved.
 *
 * Verification:
 * - Correct means: output equals the type name rmw_fastrtps_cpp announces.
 * - Covered by: rmw_rosette/test/test_introspection_c_codec.cpp
 *   (ComposesDdsTypeName).
 *
 * References:
 * - ROS 2 design, "Topic and Service name mapping to DDS"
 *   (design.ros2.org/articles/topic_and_service_names.html)
 */
bool compose_codec_type_name(const MessageCodec& codec, char* out, size_t cap) noexcept;

/**
 * @brief Encodes a ROS message as a CDR body for an RTPS writer.
 *
 * rmw_publish() calls this and passes the result to
 * `StatefulWriter::write_async()`, which adds the encapsulation header.
 *
 * @param[in]  codec       Resolved codec for the message's type.
 * @param[in]  ros_message Message instance to encode.
 * @param[out] out_buf     Destination buffer.
 * @param[in]  out_cap     Capacity of `out_buf` in bytes.
 * @return Encoded length in bytes, or 0 when the buffer was too small, the
 *   codec is unresolved, or the message contains an unencodable field.
 *
 * Verification:
 * - Correct means: bytes equal rosidl_typesupport_fastrtps_c's output.
 * - Covered by: rmw_rosette/test/test_introspection_c_codec.cpp.
 *
 * References:
 * - OMG DDS-XTypes 1.3 §7.4.3.4
 */
uint32_t serialize_message(const MessageCodec& codec, const void* ros_message,
                           uint8_t* out_buf, uint32_t out_cap) noexcept;

/**
 * @brief Encodes a ROS message into a pool buffer sized to fit it.
 *
 * rmw_publish(), rmw_send_request() and rmw_send_response() call this and
 * hand the buffer to StatefulWriter::write_async_owned(), so a sample is
 * serialized once and never copied again, and no stack buffer bounds its
 * size. The smallest pool class is tried first, doubling until the message
 * fits, so small messages hold small blocks.
 *
 * @param[in] codec       Resolved codec for the message's type.
 * @param[in] ros_message Message instance to encode.
 * @return A buffer holding the CDR body (payload.size set), or an invalid
 *   buffer when the message does not fit the largest pool block
 *   (RMW_ROSETTE_MAX_SAMPLE_SIZE) or the pool is exhausted.
 *
 * Verification:
 * - Correct means: equals serialize_message() into a large enough buffer.
 * - Covered by: rmw_rosette/test/test_introspection_c_codec.cpp
 *   (SerializeToPoolFitsLargeMessages).
 */
rtps::BufRef serialize_to_pool(const MessageCodec& codec, const void* ros_message) noexcept;

/**
 * @brief Decodes an RTPS serialized payload into a ROS message.
 *
 * rmw_take_with_info() calls this on each buffered change. The payload must
 * start with a CDR_LE encapsulation header; any other representation
 * (big-endian CDR, XCDR2, parameter lists) is rejected rather than misread.
 *
 * @param[in]  codec       Resolved codec for the message's type.
 * @param[in]  in_buf      Payload including the encapsulation header.
 * @param[in]  in_len      Length of `in_buf` in bytes.
 * @param[out] ros_message Initialized message instance to overwrite.
 * @return `true` on success; `false` for a short or non-CDR_LE payload or a
 *   decode failure.
 *
 * Verification:
 * - Correct means: payloads from rmw_fastrtps_cpp decode to equal messages;
 *   CDR_BE and truncated payloads return false.
 * - Covered by: rmw_rosette/test/test_introspection_c_codec.cpp.
 *
 * References:
 * - OMG DDSI-RTPS 2.5 §10.2 (SerializedPayloadHeader)
 * - OMG DDS-XTypes 1.3 §7.6.3.1.2 (encapsulation identifiers)
 */
bool deserialize_message(const MessageCodec& codec, const uint8_t* in_buf, uint32_t in_len,
                         void* ros_message) noexcept;

}  // namespace rmw_rosette
