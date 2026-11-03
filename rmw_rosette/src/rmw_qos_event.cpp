// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file rmw_qos_event.cpp
 * @brief QoS status events and new-data listener callbacks.
 *
 * QoS events (deadline missed, liveliness changed, incompatible QoS,
 * message lost) are DDS status conditions. rosette::rtps does not track
 * those statuses, so event initialization reports `RMW_RET_UNSUPPORTED`.
 * rcl passes that through, and both client libraries treat it as "this
 * middleware has no such event": rclcpp catches
 * `UnsupportedEventTypeException` for its default incompatible-QoS handler,
 * and rclpy skips the handler.
 *
 * The listener callbacks (`rmw_*_set_on_new_*_callback`) serve rclcpp's
 * EventsExecutor, which needs the middleware to call out on arrival.
 * rmw_rosette delivers data only while rmw_wait() polls, so it cannot honor
 * them and reports `RMW_RET_UNSUPPORTED`; wait-set based executors are
 * unaffected.
 *
 * References:
 * - rmw/event.h, rmw/rmw.h (Humble, Jazzy)
 * - rclcpp/src/rclcpp/publisher_base.cpp (Humble), `bind_event_callbacks`
 */

#include "rmw_rosette/ros_features.h"

#include <rmw/event.h>
#include <rmw/event_callback_type.h>
#include <rmw/rmw.h>

extern "C" {

/// @brief Unsupported: no DDS status tracking (see file comment).
rmw_ret_t rmw_publisher_event_init(rmw_event_t* rmw_event, const rmw_publisher_t* publisher,
                                   rmw_event_type_t event_type) {
    (void)rmw_event; (void)publisher; (void)event_type;
    return RMW_RET_UNSUPPORTED;
}

/// @brief Unsupported: no DDS status tracking (see file comment).
rmw_ret_t rmw_subscription_event_init(rmw_event_t* rmw_event,
                                      const rmw_subscription_t* subscription,
                                      rmw_event_type_t event_type) {
    (void)rmw_event; (void)subscription; (void)event_type;
    return RMW_RET_UNSUPPORTED;
}

#if RMW_ROSETTE_HAS(EVENT_TYPE_QUERY)
/**
 * @brief Reports that no QoS event type is supported.
 *
 * rcl asks before initializing an event, and the answer agrees with
 * rmw_publisher_event_init() and rmw_subscription_event_init(), which refuse
 * every type.
 *
 * @param[in] rmw_event_type Event type asked about; every value gets the same answer.
 * @return `false`.
 *
 * References:
 * - rmw/event.h (Jazzy), `rmw_event_type_is_supported`
 */
bool rmw_event_type_is_supported(rmw_event_type_t rmw_event_type) {
    (void)rmw_event_type;
    return false;
}
#endif

/// @brief Unsupported: no event can be initialized, so none can be taken.
rmw_ret_t rmw_take_event(const rmw_event_t* event_handle, void* event_info, bool* taken) {
    (void)event_handle; (void)event_info;
    if (taken) *taken = false;
    return RMW_RET_UNSUPPORTED;
}

/// @brief Unsupported: data arrives only while rmw_wait() polls (see file comment).
rmw_ret_t rmw_event_set_callback(rmw_event_t* event, rmw_event_callback_t callback,
                                 const void* user_data) {
    (void)event; (void)callback; (void)user_data;
    return RMW_RET_UNSUPPORTED;
}

/// @brief Unsupported: data arrives only while rmw_wait() polls (see file comment).
rmw_ret_t rmw_subscription_set_on_new_message_callback(rmw_subscription_t* subscription,
                                                       rmw_event_callback_t callback,
                                                       const void* user_data) {
    (void)subscription; (void)callback; (void)user_data;
    return RMW_RET_UNSUPPORTED;
}

/// @brief Unsupported: data arrives only while rmw_wait() polls (see file comment).
rmw_ret_t rmw_service_set_on_new_request_callback(rmw_service_t* service,
                                                  rmw_event_callback_t callback,
                                                  const void* user_data) {
    (void)service; (void)callback; (void)user_data;
    return RMW_RET_UNSUPPORTED;
}

/// @brief Unsupported: data arrives only while rmw_wait() polls (see file comment).
rmw_ret_t rmw_client_set_on_new_response_callback(rmw_client_t* client,
                                                  rmw_event_callback_t callback,
                                                  const void* user_data) {
    (void)client; (void)callback; (void)user_data;
    return RMW_RET_UNSUPPORTED;
}

}  // extern "C"
