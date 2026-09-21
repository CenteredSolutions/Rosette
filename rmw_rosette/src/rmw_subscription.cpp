/**
 * @file rmw_subscription.cpp
 * @brief rmw_create_subscription / rmw_take(_with_info) family.
 */

#include "rmw_rosette/guid.hpp"
#include "rmw_rosette/identifier.h"
#include "rmw_rosette/pool.hpp"
#include "rmw_rosette/qos.hpp"
#include "rmw_rosette/topic_naming.hpp"
#include "rmw_rosette/type_support.hpp"
#include "rmw_rosette/types.hpp"

#include <rmw/rmw.h>
#include <rmw/error_handling.h>

#ifndef RMW_ROSETTE_MAX_SUBSCRIPTIONS
#  define RMW_ROSETTE_MAX_SUBSCRIPTIONS 8
#endif

namespace {

struct SubscriptionSlot {
    rmw_subscription_t sub{};
    rmw_rosette::Subscription impl{};
};

rmw_rosette::Pool<SubscriptionSlot, RMW_ROSETTE_MAX_SUBSCRIPTIONS>& subscription_pool() {
    static rmw_rosette::Pool<SubscriptionSlot, RMW_ROSETTE_MAX_SUBSCRIPTIONS> pool;
    return pool;
}

SubscriptionSlot* slot_from(const rmw_subscription_t* sub) noexcept {
    return reinterpret_cast<SubscriptionSlot*>(const_cast<rmw_subscription_t*>(sub));
}

/// Bridges StatefulReader::OnChangeAvailable (fired from
/// tick()/parse_received(), where `change` is only valid for the
/// duration of the call — see stateful_reader.hpp) into the
/// Subscription's pending ring, which rmw_take() drains later from
/// whatever context calls it.
void on_change_available(const rtps::CacheChange& change, void* user_data) noexcept {
    auto* impl = static_cast<rmw_rosette::Subscription*>(user_data);
    impl->push_change(change.payload.data, change.payload.size);
}

}  // namespace

extern "C" {

rmw_subscription_t* rmw_create_subscription(
    const rmw_node_t* node,
    const rosidl_message_type_support_t* type_support,
    const char* topic_name,
    const rmw_qos_profile_t* qos_policies,
    const rmw_subscription_options_t* subscription_options) {
    if (!node || !node->data || !type_support || !topic_name || !qos_policies) {
        RMW_SET_ERROR_MSG("rmw_rosette: rmw_create_subscription: null argument");
        return nullptr;
    }

    const auto* callbacks = rmw_rosette::get_message_typesupport(type_support);
    if (!callbacks) {
        RMW_SET_ERROR_MSG("rmw_rosette: rmw_create_subscription: type was not compiled "
                          "with rosidl_typesupport_microxrcedds_c support");
        return nullptr;
    }

    auto* node_impl = static_cast<rmw_rosette::Node*>(node->data);

    SubscriptionSlot* slot = subscription_pool().acquire();
    if (!slot) {
        RMW_SET_ERROR_MSG("rmw_rosette: rmw_create_subscription: subscription pool exhausted "
                          "(RMW_ROSETTE_MAX_SUBSCRIPTIONS)");
        return nullptr;
    }

    rtps::ReaderConfig cfg{};
    rmw_rosette::fill_reader_config(cfg, *qos_policies);
    rmw_rosette::compose_topic_name(slot->impl.mangled_topic_name,
                                     sizeof(slot->impl.mangled_topic_name),
                                     node_impl->namespace_, topic_name);
    cfg.topicName.assign(slot->impl.mangled_topic_name);

    char type_name[RTPS_MAX_TOPIC_NAME_LEN];
    rmw_rosette::compose_dds_type_name(type_name, sizeof(type_name),
                                        callbacks->message_namespace_,
                                        callbacks->message_name_);
    cfg.typeName.assign(type_name);

    auto* reader = node_impl->context->participant->create_reader(
        cfg, on_change_available, &slot->impl);
    if (!reader) {
        RMW_SET_ERROR_MSG("rmw_rosette: rmw_create_subscription: create_reader failed "
                          "(participant reader table full?)");
        subscription_pool().release(slot);
        return nullptr;
    }

    slot->impl.node = node_impl;
    slot->impl.reader = reader;
    slot->impl.type_support = callbacks;

    slot->sub.implementation_identifier = rmw_rosette_identifier;
    slot->sub.data = &slot->impl;
    slot->sub.topic_name = slot->impl.mangled_topic_name;
    if (subscription_options) slot->sub.options = *subscription_options;
    slot->sub.can_loan_messages = false;
    slot->sub.is_cft_enabled = false;

    return &slot->sub;
}

rmw_ret_t rmw_destroy_subscription(rmw_node_t* node, rmw_subscription_t* subscription) {
    (void)node;
    if (!subscription) return RMW_RET_INVALID_ARGUMENT;
    subscription_pool().release(slot_from(subscription));
    return RMW_RET_OK;
}

rmw_ret_t rmw_take_with_info(const rmw_subscription_t* subscription, void* ros_message,
                              bool* taken, rmw_message_info_t* message_info,
                              rmw_subscription_allocation_t* allocation) {
    (void)allocation;
    if (!subscription || !subscription->data || !ros_message || !taken) {
        return RMW_RET_INVALID_ARGUMENT;
    }
    *taken = false;

    auto* impl = static_cast<rmw_rosette::Subscription*>(subscription->data);
    rmw_rosette::PendingChange change;
    if (!impl->pop_change(change)) {
        return RMW_RET_OK;  // Nothing buffered — not an error, per rmw_take()'s contract.
    }

    if (!rmw_rosette::deserialize_message(*impl->type_support, change.bytes, change.size,
                                          ros_message)) {
        RMW_SET_ERROR_MSG("rmw_rosette: rmw_take_with_info: deserialize failed");
        return RMW_RET_ERROR;
    }

    if (message_info) {
        *message_info = rmw_get_zero_initialized_message_info();
        message_info->publisher_gid.implementation_identifier = rmw_rosette_identifier;
    }
    *taken = true;
    return RMW_RET_OK;
}

rmw_ret_t rmw_take(const rmw_subscription_t* subscription, void* ros_message, bool* taken,
                    rmw_subscription_allocation_t* allocation) {
    return rmw_take_with_info(subscription, ros_message, taken, nullptr, allocation);
}

rmw_ret_t rmw_take_serialized_message_with_info(
    const rmw_subscription_t* subscription, rmw_serialized_message_t* serialized_message,
    bool* taken, rmw_message_info_t* message_info, rmw_subscription_allocation_t* allocation) {
    (void)allocation;
    if (!subscription || !subscription->data || !serialized_message || !taken) {
        return RMW_RET_INVALID_ARGUMENT;
    }
    *taken = false;

    auto* impl = static_cast<rmw_rosette::Subscription*>(subscription->data);
    rmw_rosette::PendingChange change;
    if (!impl->pop_change(change)) {
        return RMW_RET_OK;
    }

    if (serialized_message->buffer_capacity < change.size) {
        RMW_SET_ERROR_MSG("rmw_rosette: rmw_take_serialized_message_with_info: "
                          "serialized_message buffer too small");
        return RMW_RET_ERROR;
    }
    for (uint32_t i = 0; i < change.size; ++i) serialized_message->buffer[i] = change.bytes[i];
    serialized_message->buffer_length = change.size;

    if (message_info) {
        *message_info = rmw_get_zero_initialized_message_info();
        message_info->publisher_gid.implementation_identifier = rmw_rosette_identifier;
    }
    *taken = true;
    return RMW_RET_OK;
}

rmw_ret_t rmw_subscription_count_matched_publishers(const rmw_subscription_t* subscription,
                                                     size_t* publisher_count) {
    (void)subscription;
    if (!publisher_count) return RMW_RET_INVALID_ARGUMENT;
    *publisher_count = 0;
    return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_get_gid_for_subscription(const rmw_subscription_t* subscription, rmw_gid_t* gid) {
    if (!subscription || !subscription->data || !gid) return RMW_RET_INVALID_ARGUMENT;
    auto* impl = static_cast<rmw_rosette::Subscription*>(subscription->data);
    rmw_rosette::guid_to_rmw_gid(impl->reader->guid(), *gid);
    return RMW_RET_OK;
}

// Deferred: rmw_take_sequence, rmw_take_loaned_message(_with_info),
// rmw_subscription_get_actual_qos, rmw_subscription_set_content_filter
// (no content-filter support in rtps), rmw_subscription_get_network_flow_endpoints.

}  // extern "C"
