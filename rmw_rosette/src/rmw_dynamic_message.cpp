// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file rmw_dynamic_message.cpp
 * @brief Runtime (dynamic) type support entry points, which rmw_rosette refuses.
 *
 * rmw/dynamic_message_type_support.h lets a subscriber take messages whose
 * type it learns at run time, through a serialization library
 * (rosidl_dynamic_typesupport_fastrtps and the like) that builds and reads
 * dynamic data. rmw_rosette serializes only through compiled type supports
 * (message_codec.hpp) and carries no serialization library, so every entry
 * point reports RMW_RET_UNSUPPORTED and takes nothing. Subscriptions made
 * with compiled type supports are unaffected.
 *
 * The build compiles this file only against headers that declare these
 * functions (RMW_ROSETTE_HAS_DYNAMIC_TYPESUPPORT, rmw_rosette_ros_features.cmake).
 *
 * References:
 * - rmw/dynamic_message_type_support.h (Jazzy)
 * - rosidl_dynamic_typesupport/types.h (Jazzy), the dynamic data and
 *   serialization support handles
 */

#include <rmw/dynamic_message_type_support.h>
#include <rmw/error_handling.h>

extern "C" {

/// @brief Unsupported: rmw_rosette carries no dynamic serialization library.
rmw_ret_t rmw_serialization_support_init(
    const char* serialization_lib_name, rcutils_allocator_t* allocator,
    rosidl_dynamic_typesupport_serialization_support_t* serialization_support) {
    (void)serialization_lib_name;
    (void)allocator;
    (void)serialization_support;
    RMW_SET_ERROR_MSG("rmw_rosette: dynamic type support is not available");
    return RMW_RET_UNSUPPORTED;
}

/// @brief Unsupported: nothing is taken and `*taken` is cleared.
rmw_ret_t rmw_take_dynamic_message(const rmw_subscription_t*                  subscription,
                                   rosidl_dynamic_typesupport_dynamic_data_t* dynamic_message,
                                   bool* taken, rmw_subscription_allocation_t* allocation) {
    (void)subscription;
    (void)dynamic_message;
    (void)allocation;
    if (taken) {
        *taken = false;
    }

    RMW_SET_ERROR_MSG("rmw_rosette: dynamic type support is not available");
    return RMW_RET_UNSUPPORTED;
}

/// @brief Unsupported: nothing is taken and `*taken` is cleared.
rmw_ret_t rmw_take_dynamic_message_with_info(
    const rmw_subscription_t*                  subscription,
    rosidl_dynamic_typesupport_dynamic_data_t* dynamic_message, bool* taken,
    rmw_message_info_t* message_info, rmw_subscription_allocation_t* allocation) {
    (void)subscription;
    (void)dynamic_message;
    (void)message_info;
    (void)allocation;
    if (taken) {
        *taken = false;
    }

    RMW_SET_ERROR_MSG("rmw_rosette: dynamic type support is not available");
    return RMW_RET_UNSUPPORTED;
}

}  // extern "C"
