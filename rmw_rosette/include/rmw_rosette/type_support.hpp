/**
 * @file rmw_rosette/type_support.hpp
 * @brief Glue between rosidl-generated type support and rtps writers/readers.
 *
 * micro_ros_setup already generates, for every ROS message type,
 * Micro-CDR-based serialize/deserialize callbacks via
 * rosidl_typesupport_microxrcedds_c. Those callbacks produce standard CDR
 * on the wire (the same thing a real Agent would forward to DDS), so
 * rmw_rosette reuses them directly instead of writing a second
 * serializer — this header is *only* the extraction shim that pulls the
 * function pointers and type-name string out of the
 * rosidl_message_type_support_t handle rcl hands to rmw_create_publisher/
 * rmw_create_subscription.
 *
 * STATUS: unverified against the real header.
 * -------------------------------------------
 * The struct shape below is written from memory/best understanding of
 * rosidl_typesupport_microxrcedds_c's `message_type_support_callbacks_t`
 * (roughly: message_name, message_namespace, a ucdrBuffer-based
 * serialize/deserialize pair, and a size function). This has NOT been
 * checked against the actual generated header in this repo's
 * micro_ros_setup vendor tree — do that before writing real code against
 * it. Treat every field name here as a placeholder to confirm, not a
 * fact.
 */

#pragma once

#include <cstdint>

// TODO: #include the real header once located, e.g.
//   #include <rosidl_typesupport_microxrcedds_c/message_type_support.h>
// and the Micro-CDR buffer type it uses, e.g. <ucdr/microcdr.h>.

namespace rmw_rosette {

/// Placeholder mirror of rosidl_typesupport_microxrcedds_c's callback
/// struct — confirm field names/order against the vendored header before
/// relying on this.
struct MessageTypeSupportCallbacksPlaceholder {
    const char* message_name;
    const char* message_namespace;
    // bool (*serialize)(const void*, ucdrBuffer*);
    // bool (*deserialize)(ucdrBuffer*, void*);
    // uint32_t (*size_of)(const void*);
    void* serialize_fn_placeholder;
    void* deserialize_fn_placeholder;
    void* size_of_fn_placeholder;
};

/// Extracts the microxrcedds typesupport callbacks from an opaque
/// rosidl_message_type_support_t*, matching typesupport_identifier
/// "rosidl_typesupport_microxrcedds_c". Returns nullptr on mismatch (e.g.
/// if rcl was linked against a different typesupport, which would itself
/// be a configuration bug worth surfacing loudly, not silently ignoring).
inline const MessageTypeSupportCallbacksPlaceholder*
get_message_typesupport(const void* /*rosidl_message_type_support*/) noexcept {
    // TODO: real implementation walks rosidl_message_type_support_t's
    // typesupport_identifier chain (a type support can wrap another via
    // ->func(handle, target_identifier)) looking for
    // rosidl_typesupport_microxrcedds_c_identifier, then casts ->data.
    return nullptr;
}

// TODO: compose_dds_type_name() in topic_naming.hpp needs
// message_namespace + message_name from here to build
// "<pkg>::msg::dds_::<Type>_" — confirm whether the generated struct
// already stores the mangled form or the plain ROS name.

}  // namespace rmw_rosette
