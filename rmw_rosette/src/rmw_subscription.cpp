/**
 * @file rmw_subscription.cpp
 * @brief rmw_create_subscription / rmw_take(_with_info) family.
 */

#include "rmw_rosette/guid.hpp"
#include "rmw_rosette/identifier.h"
#include "rmw_rosette/pool.hpp"
#include "rmw_rosette/qos.hpp"
#include "rmw_rosette/topic_naming.hpp"
#include "rmw_rosette/message_codec.hpp"
#include "rmw_rosette/types.hpp"

#include <rcutils/types/uint8_array.h>
#include <rmw/rmw.h>

#include <cstring>
#include <rmw/error_handling.h>
#include <rmw/get_network_flow_endpoints.h>

namespace {

struct SubscriptionSlot {
    rmw_subscription_t sub{};
    rmw_rosette::Subscription impl{};
};

rmw_rosette::Pool<SubscriptionSlot, rmw_rosette::kSubscriptionCapacity>& subscription_pool() {
    static rmw_rosette::Pool<SubscriptionSlot, rmw_rosette::kSubscriptionCapacity> pool;
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
 * the subscription's ChangeRing (change_ring.hpp), so the payload
 * survives until `rmw_take()` drains it.
 *
 * @param[in] change Received change; its payload is copied out
 *   immediately since it does not outlive this call.
 * @param[in] user_data The owning `Subscription*`, passed as the reader's
 *   user data at `create_reader()` time.
 */
void on_change_available(const rosette::rtps::CacheChange& change, void* user_data) noexcept {
    auto* impl = static_cast<rmw_rosette::Subscription*>(user_data);
    impl->received.push(change);
}

}  // namespace

extern "C" {

/**
 * @brief Creates a subscription backed by an RTPS reader.
 *
 * Resolves the message type's serializer (message_codec.hpp), mangles the
 * fully-qualified ROS topic name and DDS wire type name (topic_naming.hpp), builds a
 * `ReaderConfig` from `qos_policies`, and creates the reader with
 * on_change_available() registered as its change callback so incoming
 * data lands in the pooled `Subscription`'s pending ring
 * (ChangeRing, change_ring.hpp) ahead of any `rmw_take()` call. The reader
 * is associated with `node` on `ros_discovery_info`
 * (ros_discovery_info_writer.hpp), so stock peers list the subscription
 * under its node.
 *
 * @param[in] node Node to scope this subscription to.
 * @param[in] type_support Message type support from rosidl-generated code.
 * @param[in] topic_name Fully-qualified ROS 2 topic name, as rcl passes it.
 * @param[in] qos_policies Requested QoS profile.
 * @param[in] subscription_options Copied verbatim into the resulting
 *   `rmw_subscription_t::options`.
 * @return The new subscription, or `nullptr` if any required argument is
 *   null, the type support offers no supported typesupport family, the
 *   subscription pool is exhausted, the participant's reader table is
 *   full, or the updated `ros_discovery_info` message cannot be written.
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

    rmw_rosette::MessageCodec codec;
    if (!rmw_rosette::resolve_message_codec(type_support, codec)) {
        RMW_SET_ERROR_MSG("rmw_rosette: rmw_create_subscription: type support offers neither "
                          "introspection_c nor microxrcedds_c");
        return nullptr;
    }

    auto* node_impl = static_cast<rmw_rosette::Node*>(node->data);
    rmw_rosette::ContextLock lock(*node_impl->context);

    SubscriptionSlot* slot = subscription_pool().acquire();
    if (!slot) {
        RMW_SET_ERROR_MSG("rmw_rosette: rmw_create_subscription: subscription pool exhausted "
                          "(RMW_ROSETTE_MAX_SUBSCRIPTIONS)");
        return nullptr;
    }

    rosette::rtps::ReaderConfig cfg{};
    rmw_rosette::fill_reader_config(cfg, *qos_policies);
    char type_name[RTPS_MAX_TOPIC_NAME_LEN];
    if (!rmw_rosette::compose_topic_name(slot->impl.mangled_topic_name,
                                         sizeof(slot->impl.mangled_topic_name), topic_name) ||
        !rmw_rosette::compose_codec_type_name(codec, type_name, sizeof(type_name))) {
        RMW_SET_ERROR_MSG("rmw_rosette: rmw_create_subscription: topic or type name longer than "
                          "RTPS_MAX_TOPIC_NAME_LEN - 1");
        subscription_pool().release(slot);
        return nullptr;
    }
    cfg.topicName.assign(slot->impl.mangled_topic_name);
    cfg.typeName.assign(type_name);

    auto* reader = node_impl->context->participant->create_reader(
        cfg, on_change_available, &slot->impl);
    if (!reader) {
        RMW_SET_ERROR_MSG("rmw_rosette: rmw_create_subscription: create_reader failed "
                          "(participant reader table full?)");
        subscription_pool().release(slot);
        return nullptr;
    }
    if (!node_impl->context->discovery_info.add_endpoints(*node_impl, &reader->guid(), nullptr)) {
        RMW_SET_ERROR_MSG("rmw_rosette: rmw_create_subscription: cannot publish "
                          "ros_discovery_info (buffer pool exhausted)");
        node_impl->context->participant->delete_reader(reader);
        subscription_pool().release(slot);
        return nullptr;
    }

    slot->impl.node = node_impl;
    slot->impl.reader = reader;
    slot->impl.codec = codec;
    slot->impl.qos = rmw_rosette::effective_qos(*qos_policies, false,
                                                RMW_ROSETTE_MAX_PENDING_CHANGES);
    // The context lock is held, so no change reaches the ring before it has
    // the depth the application is told it has.
    slot->impl.received.set_depth(static_cast<uint32_t>(slot->impl.qos.depth));

    slot->sub.implementation_identifier = rmw_rosette_identifier;
    slot->sub.data = &slot->impl;
    slot->sub.topic_name = slot->impl.mangled_topic_name;
    if (subscription_options) slot->sub.options = *subscription_options;
    slot->sub.can_loan_messages = false;
    slot->sub.is_cft_enabled = false;

    return &slot->sub;
}

/**
 * @brief Destroys a subscription, its RTPS reader, and its pool slot.
 *
 * The reader is dissociated from its node on `ros_discovery_info`, and
 * deleting it through the participant withdraws it from SEDP, so remote
 * publishers stop sending to it, and frees its reader-table slot.
 *
 * @param[in] node Unused; kept for ABI symmetry with `rmw_create_subscription()`.
 * @param[in] subscription Subscription to destroy.
 * @return `RMW_RET_OK`, or `RMW_RET_INVALID_ARGUMENT` if `subscription` is null.
 *
 * References:
 * - rmw/rmw.h (Humble), `rmw_destroy_subscription`
 */
rmw_ret_t rmw_destroy_subscription(rmw_node_t* node, rmw_subscription_t* subscription) {
    (void)node;
    if (!subscription || !subscription->data) return RMW_RET_INVALID_ARGUMENT;
    auto* impl = static_cast<rmw_rosette::Subscription*>(subscription->data);
    rmw_rosette::ContextLock lock(*impl->node->context);
    if (impl->node && impl->reader) {
        impl->node->context->discovery_info.remove_endpoints(&impl->reader->guid(), nullptr);
        impl->node->context->participant->delete_reader(impl->reader);
    }
    subscription_pool().release(slot_from(subscription));
    return RMW_RET_OK;
}

/**
 * @brief Pops and deserializes the oldest buffered change for a subscription.
 *
 * Pops one entry from the subscription's pending ring
 * (ChangeRing::pop()) and decodes it via deserialize_message()
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
    rmw_rosette::ContextLock lock(*impl->node->context);
    rmw_rosette::PendingChange change;
    if (!impl->received.pop(change)) {
        return RMW_RET_OK;  // Nothing buffered — not an error, per rmw_take()'s contract.
    }

    if (!rmw_rosette::deserialize_message(impl->codec, change.payload.data, change.payload.size,
                                          ros_message)) {
        RMW_SET_ERROR_MSG("rmw_rosette: rmw_take_with_info: deserialize failed");
        return RMW_RET_ERROR;
    }

    if (message_info) {
        *message_info = rmw_get_zero_initialized_message_info();
        message_info->source_timestamp = rmw_rosette::timestamp_to_ns(change.source_timestamp);
        rmw_rosette::guid_to_rmw_gid(change.writer_guid, message_info->publisher_gid);
        message_info->from_intra_process = false;
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
 *   representation described in rmw_serialize.cpp).
 * @param[out] taken Set to `true` if a change was taken, `false` if none
 *   was buffered.
 * @param[out] message_info When non-null, filled with metadata about the
 *   taken change.
 * @param[in] allocation Unused; this path performs no loaned-message allocation.
 * @return `RMW_RET_OK` on success (including the nothing-buffered case);
 *   `RMW_RET_INVALID_ARGUMENT` if any required argument is null;
 *   `RMW_RET_BAD_ALLOC` if `serialized_message` could not be grown (with
 *   its own allocator) to hold the change.
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
    rmw_rosette::ContextLock lock(*impl->node->context);
    rmw_rosette::PendingChange change;
    if (!impl->received.pop(change)) {
        return RMW_RET_OK;
    }

    const uint32_t size = change.payload.size;
    if (serialized_message->buffer_capacity < size &&
        rcutils_uint8_array_resize(serialized_message, size) != RCUTILS_RET_OK) {
        RMW_SET_ERROR_MSG("rmw_rosette: rmw_take_serialized_message_with_info: "
                          "could not grow serialized_message");
        return RMW_RET_BAD_ALLOC;
    }
    std::memcpy(serialized_message->buffer, change.payload.data, size);
    serialized_message->buffer_length = size;

    if (message_info) {
        *message_info = rmw_get_zero_initialized_message_info();
        message_info->source_timestamp = rmw_rosette::timestamp_to_ns(change.source_timestamp);
        rmw_rosette::guid_to_rmw_gid(change.writer_guid, message_info->publisher_gid);
        message_info->from_intra_process = false;
    }
    *taken = true;
    return RMW_RET_OK;
}

/**
 * @brief Reports how many remote publishers the subscription's reader is matched with.
 *
 * @param[in]  subscription Subscription to query.
 * @param[out] publisher_count Receives the matched writer count.
 * @return `RMW_RET_OK`, or `RMW_RET_INVALID_ARGUMENT` for null arguments.
 *
 * References:
 * - rmw/rmw.h (Humble), `rmw_subscription_count_matched_publishers`
 */
rmw_ret_t rmw_subscription_count_matched_publishers(const rmw_subscription_t* subscription,
                                                     size_t* publisher_count) {
    if (!subscription || !subscription->data || !publisher_count) return RMW_RET_INVALID_ARGUMENT;
    const auto* impl = static_cast<const rmw_rosette::Subscription*>(subscription->data);
    rmw_rosette::ContextLock lock(*impl->node->context);
    *publisher_count = impl->reader ? impl->reader->matched_writer_count() : 0u;
    return RMW_RET_OK;
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
    rmw_rosette::ContextLock lock(*impl->node->context);
    rmw_rosette::guid_to_rmw_gid(impl->reader->guid(), *gid);
    return RMW_RET_OK;
}

// This file does not define rmw_take_sequence(), rmw_take_loaned_message()/
// rmw_take_loaned_message_with_info(), rmw_subscription_get_actual_qos(),
// rmw_subscription_set_content_filter() (the underlying rtps core has no
// content-filter support), or rmw_subscription_get_network_flow_endpoints().

/**
 * @brief Takes the oldest buffered change as a serialized message.
 *
 * Equivalent to rmw_take_serialized_message_with_info() without the info.
 *
 * References:
 * - rmw/rmw.h (Humble), `rmw_take_serialized_message`
 */
rmw_ret_t rmw_take_serialized_message(const rmw_subscription_t* subscription,
                                      rmw_serialized_message_t* serialized_message, bool* taken,
                                      rmw_subscription_allocation_t* allocation) {
    return rmw_take_serialized_message_with_info(subscription, serialized_message, taken, nullptr,
                                                 allocation);
}

/**
 * @brief Takes up to `count` buffered changes in arrival order.
 *
 * Used by rclcpp's `take` of message sequences. Takes one message at a
 * time through rmw_take_with_info() into the caller's pre-initialized
 * message slots and stops at the first empty take.
 *
 * @param[in]     subscription          Subscription to drain.
 * @param[in]     count                 Maximum number of messages to take.
 * @param[in,out] message_sequence      Pre-initialized messages; `size`
 *   is set to the number taken. Capacity must be at least `count`.
 * @param[in,out] message_info_sequence Receives one info per message;
 *   capacity must be at least `count`.
 * @param[out]    taken                 Number of messages taken.
 * @param[in]     allocation            Unused.
 * @return `RMW_RET_OK` (including when nothing was available);
 *   `RMW_RET_INVALID_ARGUMENT` for null arguments, `count == 0`, or
 *   insufficient capacity; `RMW_RET_ERROR` when a message fails to decode.
 *
 * Verification:
 * - Correct means: after N changes are buffered, a take of count >= N
 *   returns exactly N messages in arrival order.
 *
 * References:
 * - rmw/rmw.h (Humble), `rmw_take_sequence`
 */
rmw_ret_t rmw_take_sequence(const rmw_subscription_t* subscription, size_t count,
                            rmw_message_sequence_t* message_sequence,
                            rmw_message_info_sequence_t* message_info_sequence, size_t* taken,
                            rmw_subscription_allocation_t* allocation) {
    if (!subscription || !message_sequence || !message_info_sequence || !taken || count == 0 ||
        message_sequence->capacity < count || message_info_sequence->capacity < count) {
        return RMW_RET_INVALID_ARGUMENT;
    }
    *taken = 0;
    for (size_t i = 0; i < count; ++i) {
        bool one = false;
        const rmw_ret_t ret = rmw_take_with_info(subscription, message_sequence->data[i], &one,
                                                 &message_info_sequence->data[i], allocation);
        if (ret != RMW_RET_OK) return ret;
        if (!one) break;
        ++*taken;
    }
    message_sequence->size = *taken;
    message_info_sequence->size = *taken;
    return RMW_RET_OK;
}

/**
 * @brief Reports the QoS profile the subscription's RTPS reader runs with.
 *
 * @param[in]  subscription Subscription to query.
 * @param[out] qos          Receives the effective profile (qos.hpp, effective_qos()).
 * @return `RMW_RET_OK`, or `RMW_RET_INVALID_ARGUMENT` for null arguments.
 *
 * References:
 * - rmw/rmw.h (Humble), `rmw_subscription_get_actual_qos`
 */
rmw_ret_t rmw_subscription_get_actual_qos(const rmw_subscription_t* subscription,
                                          rmw_qos_profile_t* qos) {
    if (!subscription || !subscription->data || !qos) return RMW_RET_INVALID_ARGUMENT;
    *qos = static_cast<const rmw_rosette::Subscription*>(subscription->data)->qos;
    return RMW_RET_OK;
}

/// @brief Content-filtered topics are not offered (`is_cft_enabled` is false).
rmw_ret_t rmw_subscription_set_content_filter(
    rmw_subscription_t* subscription, const rmw_subscription_content_filter_options_t* options) {
    (void)subscription; (void)options;
    return RMW_RET_UNSUPPORTED;
}

/// @brief Content-filtered topics are not offered (`is_cft_enabled` is false).
rmw_ret_t rmw_subscription_get_content_filter(const rmw_subscription_t* subscription,
                                              rcutils_allocator_t* allocator,
                                              rmw_subscription_content_filter_options_t* options) {
    (void)subscription; (void)allocator; (void)options;
    return RMW_RET_UNSUPPORTED;
}

/// @brief Preallocated take buffers are not offered.
rmw_ret_t rmw_init_subscription_allocation(const rosidl_message_type_support_t* type_support,
                                           const rosidl_runtime_c__Sequence__bound* message_bounds,
                                           rmw_subscription_allocation_t* allocation) {
    (void)type_support; (void)message_bounds; (void)allocation;
    return RMW_RET_UNSUPPORTED;
}

/// @brief Counterpart of rmw_init_subscription_allocation(); unsupported.
rmw_ret_t rmw_fini_subscription_allocation(rmw_subscription_allocation_t* allocation) {
    (void)allocation;
    return RMW_RET_UNSUPPORTED;
}

/// @brief Network flow endpoints (unique-network-flow QoS) are not offered.
rmw_ret_t rmw_subscription_get_network_flow_endpoints(
    const rmw_subscription_t* subscription, rcutils_allocator_t* allocator,
    rmw_network_flow_endpoint_array_t* network_flow_endpoint_array) {
    (void)subscription; (void)allocator; (void)network_flow_endpoint_array;
    return RMW_RET_UNSUPPORTED;
}

}  // extern "C"
