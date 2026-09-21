/**
 * @file rmw_serialize.cpp
 * @brief rmw_serialize / rmw_deserialize / rmw_get_serialized_message_size.
 *
 * These are standalone entry points onto the same serialization
 * rmw_publish()/rmw_take_with_info() perform internally, but under a
 * different header-accounting contract: `rmw_serialized_message_t` holds
 * the full on-wire CDR representation, encapsulation header included, per
 * the same contract `rmw_publish_serialized_message()`/
 * `rmw_take_serialized_message_with_info()` (rmw_publisher.cpp/
 * rmw_subscription.cpp) already rely on. `type_support.hpp`'s
 * serialize_message()/deserialize_message() operate one level below
 * that: serialize_message() produces bytes with no header (the writer
 * adds the real one automatically via `MessageBuilder` — see
 * `kRtpsEncapsulationHeaderSize`), and deserialize_message() expects the
 * header already stripped before it's called. Implementing rmw_serialize()
 * therefore means prepending the header explicitly (there is no writer
 * downstream to add it), and rmw_deserialize() means skipping it before
 * calling into the callbacks — neither can delegate straight through to
 * type_support.hpp's helpers the way rmw_publish()/rmw_take_with_info() do.
 */

#include "rmw_rosette/identifier.h"
#include "rmw_rosette/type_support.hpp"

#include <rmw/rmw.h>
#include <rmw/error_handling.h>

extern "C" {

/**
 * @brief Encodes a ROS message into a standalone rmw_serialized_message_t.
 *
 * @param[in] ros_message Unused.
 * @param[in] type_support Unused.
 * @param[out] serialized_message Unused.
 * @return `RMW_RET_UNSUPPORTED`. See the file comment for what
 *   implementing this requires beyond calling type_support.hpp's
 *   serialize_message().
 */
rmw_ret_t rmw_serialize(const void* ros_message,
                         const rosidl_message_type_support_t* type_support,
                         rmw_serialized_message_t* serialized_message) {
    (void)ros_message; (void)type_support; (void)serialized_message;
    RMW_SET_ERROR_MSG("rmw_rosette: rmw_serialize not implemented");
    return RMW_RET_UNSUPPORTED;
}

/**
 * @brief Decodes a standalone rmw_serialized_message_t into a ROS message.
 *
 * @param[in] serialized_message Unused.
 * @param[in] type_support Unused.
 * @param[out] ros_message Unused.
 * @return `RMW_RET_UNSUPPORTED`. See the file comment for what
 *   implementing this requires beyond calling type_support.hpp's
 *   deserialize_message().
 */
rmw_ret_t rmw_deserialize(const rmw_serialized_message_t* serialized_message,
                           const rosidl_message_type_support_t* type_support,
                           void* ros_message) {
    (void)serialized_message; (void)type_support; (void)ros_message;
    RMW_SET_ERROR_MSG("rmw_rosette: rmw_deserialize not implemented");
    return RMW_RET_UNSUPPORTED;
}

/**
 * @brief Computes the maximum serialized size of a bounded message type.
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
    RMW_SET_ERROR_MSG("rmw_rosette: rmw_get_serialized_message_size not implemented");
    return RMW_RET_UNSUPPORTED;
}

}  // extern "C"
