/**
 * @file rmw_publisher.cpp
 * @brief rmw_create_publisher / rmw_publish / matched-subscription queries.
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

#ifndef RMW_ROSETTE_MAX_PUBLISHERS
#  define RMW_ROSETTE_MAX_PUBLISHERS 8
#endif

namespace {

struct PublisherSlot {
    rmw_publisher_t pub{};
    rmw_rosette::Publisher impl{};
};

rmw_rosette::Pool<PublisherSlot, RMW_ROSETTE_MAX_PUBLISHERS>& publisher_pool() {
    static rmw_rosette::Pool<PublisherSlot, RMW_ROSETTE_MAX_PUBLISHERS> pool;
    return pool;
}

/// `rmw_publisher_t` is always the first member of `PublisherSlot`, so
/// this reinterpret is well-defined (standard-layout, same address as
/// the struct itself).
PublisherSlot* slot_from(const rmw_publisher_t* pub) noexcept {
    return reinterpret_cast<PublisherSlot*>(const_cast<rmw_publisher_t*>(pub));
}

/// Publish-side wire buffer. Sized for the max fragment-free datagram
/// this stack targets — see RTPS_BUF_BLOCK_SIZE (buf_pool.hpp), which the
/// reader side already budgets against for the same reason.
constexpr uint32_t kSerializeBufSize = RTPS_BUF_BLOCK_SIZE;

}  // namespace

extern "C" {

/**
 * @brief Creates a publisher backed by a runtime-depth-selected RTPS writer.
 *
 * Resolves the message type's microxrcedds callbacks, mangles the ROS
 * topic name and DDS wire type name (topic_naming.hpp), builds a
 * `WriterConfig` from `qos_policies`, and creates the writer through
 * create_writer_for_qos() so its `CacheDepth` matches the requested
 * history depth. The resulting `IStatefulWriter*` and callbacks are
 * stored in a pooled `Publisher`, and the `rmw_publisher_t` returned to
 * the caller points at it.
 *
 * @param[in] node Node to scope this publisher to.
 * @param[in] type_support Message type support from rosidl-generated code.
 * @param[in] topic_name ROS 2 topic name (unmangled).
 * @param[in] qos_policies Requested QoS profile.
 * @param[in] publisher_options Copied verbatim into the resulting
 *   `rmw_publisher_t::options`.
 * @return The new publisher, or `nullptr` if any required argument is
 *   null, the type support has no microxrcedds_c callbacks, the
 *   participant's writer table is full, or the publisher pool is
 *   exhausted.
 */
rmw_publisher_t* rmw_create_publisher(
    const rmw_node_t* node,
    const rosidl_message_type_support_t* type_support,
    const char* topic_name,
    const rmw_qos_profile_t* qos_policies,
    const rmw_publisher_options_t* publisher_options) {
    if (!node || !node->data || !type_support || !topic_name || !qos_policies) {
        RMW_SET_ERROR_MSG("rmw_rosette: rmw_create_publisher: null argument");
        return nullptr;
    }

    const auto* callbacks = rmw_rosette::get_message_typesupport(type_support);
    if (!callbacks) {
        RMW_SET_ERROR_MSG("rmw_rosette: rmw_create_publisher: type was not compiled "
                          "with rosidl_typesupport_microxrcedds_c support");
        return nullptr;
    }

    auto* node_impl = static_cast<rmw_rosette::Node*>(node->data);

    rtps::WriterConfig cfg{};
    rmw_rosette::fill_writer_config(cfg, *qos_policies);

    char mangled_topic[RTPS_MAX_TOPIC_NAME_LEN];
    rmw_rosette::compose_topic_name(mangled_topic, sizeof(mangled_topic),
                                     node_impl->namespace_, topic_name);
    cfg.topicName.assign(mangled_topic);

    char type_name[RTPS_MAX_TOPIC_NAME_LEN];
    rmw_rosette::compose_dds_type_name(type_name, sizeof(type_name),
                                        callbacks->message_namespace_,
                                        callbacks->message_name_);
    cfg.typeName.assign(type_name);

    auto* writer = rmw_rosette::create_writer_for_qos(
        *node_impl->context->participant, cfg, *qos_policies);
    if (!writer) {
        RMW_SET_ERROR_MSG("rmw_rosette: rmw_create_publisher: create_writer failed "
                          "(participant writer table full?)");
        return nullptr;
    }

    PublisherSlot* slot = publisher_pool().acquire();
    if (!slot) {
        RMW_SET_ERROR_MSG("rmw_rosette: rmw_create_publisher: publisher pool exhausted "
                          "(RMW_ROSETTE_MAX_PUBLISHERS)");
        return nullptr;
    }

    slot->impl.node = node_impl;
    slot->impl.writer = writer;
    slot->impl.type_support = callbacks;
    for (uint32_t i = 0; i < sizeof(mangled_topic); ++i) {
        slot->impl.mangled_topic_name[i] = mangled_topic[i];
    }

    slot->pub.implementation_identifier = rmw_rosette_identifier;
    slot->pub.data = &slot->impl;
    slot->pub.topic_name = slot->impl.mangled_topic_name;
    if (publisher_options) slot->pub.options = *publisher_options;
    slot->pub.can_loan_messages = false;

    return &slot->pub;
}

/**
 * @brief Destroys a publisher and returns its slot to the pool.
 *
 * Does not remove the underlying writer from the participant's writer
 * table; the writer's storage is reused only when the participant itself
 * is torn down.
 *
 * @param[in] node Unused; kept for ABI symmetry with `rmw_create_publisher()`.
 * @param[in] publisher Publisher to destroy.
 * @return `RMW_RET_OK`, or `RMW_RET_INVALID_ARGUMENT` if `publisher` is null.
 */
rmw_ret_t rmw_destroy_publisher(rmw_node_t* node, rmw_publisher_t* publisher) {
    (void)node;
    if (!publisher) return RMW_RET_INVALID_ARGUMENT;
    publisher_pool().release(slot_from(publisher));
    return RMW_RET_OK;
}

/**
 * @brief Serializes and reliably publishes a ROS message.
 *
 * Encodes `ros_message` via serialize_message() into a stack buffer,
 * then hands the resulting bytes to the publisher's
 * `IStatefulWriter::write_reliable()`, which prepends the RTPS DATA
 * submessage's CDR encapsulation header and delivers it through the
 * normal RTPS reliability machinery (heartbeat/acknack).
 *
 * @param[in] publisher Publisher to send through.
 * @param[in] ros_message Message instance to encode and send.
 * @param[in] allocation Unused; this path performs no loaned-message allocation.
 * @return `RMW_RET_OK` on success; `RMW_RET_INVALID_ARGUMENT` if any
 *   required argument is null; `RMW_RET_ERROR` if serialization or the
 *   underlying write failed.
 */
rmw_ret_t rmw_publish(const rmw_publisher_t* publisher, const void* ros_message,
                       rmw_publisher_allocation_t* allocation) {
    (void)allocation;
    if (!publisher || !publisher->data || !ros_message) return RMW_RET_INVALID_ARGUMENT;

    auto* impl = static_cast<rmw_rosette::Publisher*>(publisher->data);

    uint8_t buf[kSerializeBufSize];
    const uint32_t len = rmw_rosette::serialize_message(*impl->type_support, ros_message,
                                                          buf, sizeof(buf));
    if (len == 0) {
        RMW_SET_ERROR_MSG("rmw_rosette: rmw_publish: serialization failed "
                          "(message too large for the wire buffer?)");
        return RMW_RET_ERROR;
    }

    const bool ok = impl->writer->write_reliable(rtps::ChangeKind::ALIVE, buf, len);
    if (!ok) {
        RMW_SET_ERROR_MSG("rmw_rosette: rmw_publish: write_reliable failed");
        return RMW_RET_ERROR;
    }
    return RMW_RET_OK;
}

/**
 * @brief Reliably publishes an already-serialized message.
 *
 * The same delivery path as rmw_publish(), minus the serialize step:
 * `serialized_message`'s bytes are already CDR-encoded application
 * content (no encapsulation header — see kRtpsEncapsulationHeaderSize in
 * type_support.hpp), so they're handed to `write_reliable()` directly.
 *
 * @param[in] publisher Publisher to send through.
 * @param[in] serialized_message Pre-serialized message bytes.
 * @param[in] allocation Unused; this path performs no loaned-message allocation.
 * @return `RMW_RET_OK` on success; `RMW_RET_INVALID_ARGUMENT` if any
 *   required argument is null; `RMW_RET_ERROR` if the underlying write
 *   failed.
 */
rmw_ret_t rmw_publish_serialized_message(const rmw_publisher_t* publisher,
                                          const rmw_serialized_message_t* serialized_message,
                                          rmw_publisher_allocation_t* allocation) {
    (void)allocation;
    if (!publisher || !publisher->data || !serialized_message) return RMW_RET_INVALID_ARGUMENT;

    auto* impl = static_cast<rmw_rosette::Publisher*>(publisher->data);
    const bool ok = impl->writer->write_reliable(
        rtps::ChangeKind::ALIVE,
        reinterpret_cast<const rtps::Octet*>(serialized_message->buffer),
        static_cast<uint32_t>(serialized_message->buffer_length));
    if (!ok) {
        RMW_SET_ERROR_MSG("rmw_rosette: rmw_publish_serialized_message: write_reliable failed");
        return RMW_RET_ERROR;
    }
    return RMW_RET_OK;
}

/**
 * @brief Reports how many matched subscriptions a publisher currently has.
 *
 * `IStatefulWriter` (stateful_writer.hpp) exposes `matched_reader_add()`/
 * `matched_reader_remove()` but no accessor for the current count; this
 * always reports zero and `RMW_RET_UNSUPPORTED` until one is added
 * (directly on `IStatefulWriter`, or tracked alongside GraphCache).
 *
 * @param[in] publisher Unused.
 * @param[out] subscription_count Always set to 0.
 * @return `RMW_RET_UNSUPPORTED`, or `RMW_RET_INVALID_ARGUMENT` if
 *   `subscription_count` is null.
 */
rmw_ret_t rmw_publisher_count_matched_subscriptions(const rmw_publisher_t* publisher,
                                                     size_t* subscription_count) {
    (void)publisher;
    if (!subscription_count) return RMW_RET_INVALID_ARGUMENT;
    *subscription_count = 0;
    return RMW_RET_UNSUPPORTED;
}

/**
 * @brief Reports a publisher's writer GUID as an rmw_gid_t.
 *
 * @param[in] publisher Publisher to query.
 * @param[out] gid Receives the writer's GUID via guid_to_rmw_gid().
 * @return `RMW_RET_OK`, or `RMW_RET_INVALID_ARGUMENT` if any required
 *   argument is null.
 */
rmw_ret_t rmw_get_gid_for_publisher(const rmw_publisher_t* publisher, rmw_gid_t* gid) {
    if (!publisher || !publisher->data || !gid) return RMW_RET_INVALID_ARGUMENT;
    auto* impl = static_cast<rmw_rosette::Publisher*>(publisher->data);
    rmw_rosette::guid_to_rmw_gid(impl->writer->guid(), *gid);
    return RMW_RET_OK;
}

// This file does not define rmw_publisher_assert_liveliness() (no
// LIVELINESS QoS in the underlying rtps core), rmw_publisher_wait_for_all_acked()
// (write_reliable() already blocks on acknowledgment internally, but under
// a different contract than this call), rmw_publisher_get_actual_qos(),
// rmw_borrow_loaned_message()/rmw_publish_loaned_message() (loaned-message
// support assumes a memory model this stack doesn't provide), or
// rmw_publisher_get_network_flow_endpoints().

}  // extern "C"
