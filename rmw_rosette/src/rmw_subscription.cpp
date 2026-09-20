/**
 * @file rmw_subscription.cpp
 * @brief rmw_create_subscription / rmw_take(_with_info) family.
 */

#include "rmw_rosette/identifier.h"
#include "rmw_rosette/qos.hpp"
#include "rmw_rosette/topic_naming.hpp"
#include "rmw_rosette/type_support.hpp"
#include "rmw_rosette/types.hpp"

#include <rmw/rmw.h>
#include <rmw/error_handling.h>

extern "C" {

rmw_subscription_t* rmw_create_subscription(
    const rmw_node_t* node,
    const rosidl_message_type_support_t* type_support,
    const char* topic_name,
    const rmw_qos_profile_t* qos_policies,
    const rmw_subscription_options_t* subscription_options) {
    (void)node; (void)type_support; (void)topic_name;
    (void)qos_policies; (void)subscription_options;
    // TODO: mirror rmw_create_publisher — compose the mangled topic name,
    // resolve type support, rtps::ReaderConfig via fill_reader_config(),
    // then participant->create_reader(cfg, on_change_available_trampoline,
    // /*user_data=*/this_subscription). The trampoline needs to feed
    // whatever buffering scheme wait_set.hpp ends up using (see its TODO
    // on Subscription in types.hpp) since it runs from tick()/
    // parse_received() context, not from rmw_take().
    RMW_SET_ERROR_MSG("rmw_rosette: rmw_create_subscription not implemented");
    return nullptr;
}

rmw_ret_t rmw_destroy_subscription(rmw_node_t* node, rmw_subscription_t* subscription) {
    (void)node; (void)subscription;
    RMW_SET_ERROR_MSG("rmw_rosette: rmw_destroy_subscription not implemented");
    return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_take_with_info(const rmw_subscription_t* subscription, void* ros_message,
                              bool* taken, rmw_message_info_t* message_info,
                              rmw_subscription_allocation_t* allocation) {
    (void)subscription; (void)ros_message; (void)message_info; (void)allocation;
    if (!taken) return RMW_RET_INVALID_ARGUMENT;
    *taken = false;
    // TODO:
    //   1. Pop the oldest buffered CacheChange for this subscription (see
    //      types.hpp TODO).
    //   2. Deserialize via the microxrcedds _cdr_deserialize callback into
    //      ros_message.
    //   3. Fill message_info (publisher_gid via guid_to_rmw_gid(),
    //      source_timestamp, received_timestamp).
    RMW_SET_ERROR_MSG("rmw_rosette: rmw_take_with_info not implemented");
    return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_take(const rmw_subscription_t* subscription, void* ros_message, bool* taken,
                    rmw_subscription_allocation_t* allocation) {
    return rmw_take_with_info(subscription, ros_message, taken, nullptr, allocation);
}

rmw_ret_t rmw_take_serialized_message_with_info(
    const rmw_subscription_t* subscription, rmw_serialized_message_t* serialized_message,
    bool* taken, rmw_message_info_t* message_info, rmw_subscription_allocation_t* allocation) {
    (void)subscription; (void)serialized_message; (void)message_info; (void)allocation;
    if (!taken) return RMW_RET_INVALID_ARGUMENT;
    *taken = false;
    // Same buffered-change lookup as rmw_take_with_info(), minus the
    // deserialize step — copy the raw CDR bytes out instead.
    RMW_SET_ERROR_MSG("rmw_rosette: rmw_take_serialized_message_with_info not implemented");
    return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_subscription_count_matched_publishers(const rmw_subscription_t* subscription,
                                                     size_t* publisher_count) {
    (void)subscription;
    if (!publisher_count) return RMW_RET_INVALID_ARGUMENT;
    *publisher_count = 0;
    return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_get_gid_for_subscription(const rmw_subscription_t* subscription, rmw_gid_t* gid) {
    (void)subscription; (void)gid;
    RMW_SET_ERROR_MSG("rmw_rosette: rmw_get_gid_for_subscription not implemented");
    return RMW_RET_UNSUPPORTED;
}

// Deferred: rmw_take_sequence, rmw_take_loaned_message(_with_info),
// rmw_subscription_get_actual_qos, rmw_subscription_set_content_filter
// (no content-filter support in rtps), rmw_subscription_get_network_flow_endpoints.

}  // extern "C"
