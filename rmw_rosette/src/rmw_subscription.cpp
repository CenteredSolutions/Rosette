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

/// `rmw_subscription_t` is always the first member of `SubscriptionSlot`,
/// so this reinterpret is well-defined (standard-layout, same address as
/// the struct itself).
SubscriptionSlot* slot_from(const rmw_subscription_t* sub) noexcept {
    return reinterpret_cast<SubscriptionSlot*>(const_cast<rmw_subscription_t*>(sub));
}

/**
 * @brief Trampoline registered as a reader's OnChangeAvailable callback.
 *
 * Bridges `StatefulReader::OnChangeAvailable` — fired from
 * `RTPSParticipant::tick()`/`parse_received()`, where `change` is only
 * valid for the duration of the call (see stateful_reader.hpp) — into
 * `Subscription::push_change()`, so the payload survives until
 * `rmw_take()` drains it from whatever context calls that.
 *
 * @param[in] change Received change; its payload is copied out
 *   immediately since it does not outlive this call.
 * @param[in] user_data The owning `Subscription*`, passed as the reader's
 *   user data at `create_reader()` time.
 */
void on_change_available(const rtps::CacheChange& change, void* user_data) noexcept {
    auto* impl = static_cast<rmw_rosette::Subscription*>(user_data);
    impl->push_change(change.payload.data, change.payload.size);
}

}  // namespace

extern "C" {

/**
 * @brief Creates a subscription backed by an RTPS reader.
 *
 * Resolves the message type's microxrcedds callbacks, mangles the ROS
 * topic name and DDS wire type name (topic_naming.hpp), builds a
 * `ReaderConfig` from `qos_policies`, and creates the reader with
 * on_change_available() registered as its change callback so incoming
 * data lands in the pooled `Subscription`'s pending ring
 * (`Subscription::push_change()`, types.hpp) ahead of any `rmw_take()` call.
 *
 * @param[in] node Node to scope this subscription to.
 * @param[in] type_support Message type support from rosidl-generated code.
 * @param[in] topic_name ROS 2 topic name (unmangled).
 * @param[in] qos_policies Requested QoS profile.
 * @param[in] subscription_options Copied verbatim into the resulting
 *   `rmw_subscription_t::options`.
 * @return The new subscription, or `nullptr` if any required argument is
 *   null, the type support has no microxrcedds_c callbacks, the
 *   subscription pool is exhausted, or the participant's reader table is
 *   full.
 */
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

/**
 * @brief Destroys a subscription and returns its slot to the pool.
 *
 * Does not remove the underlying reader from the participant's reader
 * table; the reader's storage is reused only when the participant itself
 * is torn down.
 *
 * @param[in] node Unused; kept for ABI symmetry with `rmw_create_subscription()`.
 * @param[in] subscription Subscription to destroy.
 * @return `RMW_RET_OK`, or `RMW_RET_INVALID_ARGUMENT` if `subscription` is null.
 */
rmw_ret_t rmw_destroy_subscription(rmw_node_t* node, rmw_subscription_t* subscription) {
    (void)node;
    if (!subscription) return RMW_RET_INVALID_ARGUMENT;
    subscription_pool().release(slot_from(subscription));
    return RMW_RET_OK;
}

/**
 * @brief Pops and deserializes the oldest buffered change for a subscription.
 *
 * Pops one entry from the subscription's pending ring
 * (`Subscription::pop_change()`) and decodes it via deserialize_message()
 * into `ros_message`. Per `rmw_take()`'s contract, an empty ring is not
 * an error: `*taken` is left `false` and `RMW_RET_OK` is returned.
 *
 * @param[in] subscription Subscription to take from.
 * @param[out] ros_message Message instance to decode into; must already
 *   be allocated/initialized as an instance of the subscription's
 *   message type.
 * @param[out] taken Set to `true` if a change was taken and decoded,
 *   `false` if none was buffered.
 * @param[out] message_info When non-null, filled with metadata about the
 *   taken change (currently only `publisher_gid.implementation_identifier`).
 * @param[in] allocation Unused; this path performs no loaned-message allocation.
 * @return `RMW_RET_OK` on success (including the nothing-buffered case);
 *   `RMW_RET_INVALID_ARGUMENT` if any required argument is null;
 *   `RMW_RET_ERROR` if deserialization failed.
 */
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

/// Equivalent to rmw_take_with_info() with `message_info` omitted.
rmw_ret_t rmw_take(const rmw_subscription_t* subscription, void* ros_message, bool* taken,
                    rmw_subscription_allocation_t* allocation) {
    return rmw_take_with_info(subscription, ros_message, taken, nullptr, allocation);
}

/**
 * @brief Pops the oldest buffered change for a subscription without
 * deserializing it.
 *
 * The serialized counterpart to rmw_take_with_info(): pops one entry
 * from the subscription's pending ring and copies its raw bytes into
 * `serialized_message` rather than decoding them, leaving the caller to
 * deserialize later (e.g. via `rmw_deserialize()`).
 *
 * @param[in] subscription Subscription to take from.
 * @param[out] serialized_message Receives the raw CDR bytes of the taken
 *   change, encapsulation header included (this is the full on-wire
 *   representation — see the caution comment in rmw_serialize.cpp).
 * @param[out] taken Set to `true` if a change was taken, `false` if none
 *   was buffered.
 * @param[out] message_info When non-null, filled with metadata about the
 *   taken change.
 * @param[in] allocation Unused; this path performs no loaned-message allocation.
 * @return `RMW_RET_OK` on success (including the nothing-buffered case);
 *   `RMW_RET_INVALID_ARGUMENT` if any required argument is null;
 *   `RMW_RET_ERROR` if `serialized_message`'s buffer was too small to
 *   hold the change.
 */
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

/**
 * @brief Reports how many matched publishers a subscription currently has.
 *
 * `rtps::StatefulReader` (stateful_reader.hpp) exposes
 * `matched_writer_count()`, but this is not yet wired to it; always
 * reports zero and `RMW_RET_UNSUPPORTED`.
 *
 * @param[in] subscription Unused.
 * @param[out] publisher_count Always set to 0.
 * @return `RMW_RET_UNSUPPORTED`, or `RMW_RET_INVALID_ARGUMENT` if
 *   `publisher_count` is null.
 */
rmw_ret_t rmw_subscription_count_matched_publishers(const rmw_subscription_t* subscription,
                                                     size_t* publisher_count) {
    (void)subscription;
    if (!publisher_count) return RMW_RET_INVALID_ARGUMENT;
    *publisher_count = 0;
    return RMW_RET_UNSUPPORTED;
}

/**
 * @brief Reports a subscription's reader GUID as an rmw_gid_t.
 *
 * @param[in] subscription Subscription to query.
 * @param[out] gid Receives the reader's GUID via guid_to_rmw_gid().
 * @return `RMW_RET_OK`, or `RMW_RET_INVALID_ARGUMENT` if any required
 *   argument is null.
 */
rmw_ret_t rmw_get_gid_for_subscription(const rmw_subscription_t* subscription, rmw_gid_t* gid) {
    if (!subscription || !subscription->data || !gid) return RMW_RET_INVALID_ARGUMENT;
    auto* impl = static_cast<rmw_rosette::Subscription*>(subscription->data);
    rmw_rosette::guid_to_rmw_gid(impl->reader->guid(), *gid);
    return RMW_RET_OK;
}

// This file does not define rmw_take_sequence(), rmw_take_loaned_message()/
// rmw_take_loaned_message_with_info(), rmw_subscription_get_actual_qos(),
// rmw_subscription_set_content_filter() (the underlying rtps core has no
// content-filter support), or rmw_subscription_get_network_flow_endpoints().

}  // extern "C"
