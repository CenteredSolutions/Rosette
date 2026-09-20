/**
 * @file rmw_serialize.cpp
 * @brief rmw_serialize / rmw_deserialize / rmw_get_serialized_message_size.
 *
 * Thin wrappers over the rosidl_typesupport_microxrcedds_c callbacks
 * (type_support.hpp) — no new serialization logic, just exposing what
 * rmw_publish()/rmw_take() already need to do internally as standalone
 * entry points, per the rmw ABI.
 */

#include "rmw_rosette/identifier.h"
#include "rmw_rosette/type_support.hpp"

#include <rmw/rmw.h>
#include <rmw/error_handling.h>

extern "C" {

rmw_ret_t rmw_serialize(const void* ros_message,
                         const rosidl_message_type_support_t* type_support,
                         rmw_serialized_message_t* serialized_message) {
    (void)ros_message; (void)type_support; (void)serialized_message;
    RMW_SET_ERROR_MSG("rmw_rosette: rmw_serialize not implemented");
    return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_deserialize(const rmw_serialized_message_t* serialized_message,
                           const rosidl_message_type_support_t* type_support,
                           void* ros_message) {
    (void)serialized_message; (void)type_support; (void)ros_message;
    RMW_SET_ERROR_MSG("rmw_rosette: rmw_deserialize not implemented");
    return RMW_RET_UNSUPPORTED;
}

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
