/**
 * @file rmw_publisher.cpp
 * @brief rmw_create_publisher / rmw_publish / matched-subscription queries.
 */

#include "rmw_rosette/guid.hpp"
#include "rmw_rosette/identifier.h"
#include "rmw_rosette/pool.hpp"
#include "rmw_rosette/qos.hpp"
#include "rmw_rosette/topic_naming.hpp"
#include "rmw_rosette/message_codec.hpp"
#include "rmw_rosette/types.hpp"

#include <rmw/rmw.h>
#include <rmw/time.h>
#include <rmw/error_handling.h>
#include <rmw/get_network_flow_endpoints.h>

namespace {

struct PublisherSlot {
    rmw_publisher_t pub{};
    rmw_rosette::Publisher impl{};
};

rmw_rosette::Pool<PublisherSlot, rmw_rosette::kPublisherCapacity>& publisher_pool() {
    static rmw_rosette::Pool<PublisherSlot, rmw_rosette::kPublisherCapacity> pool;
    return pool;
}

/// `rmw_publisher_t` is always the first member of `PublisherSlot`, so
/// this reinterpret is well-defined (standard-layout, same address as
/// the struct itself).
PublisherSlot* slot_from(const rmw_publisher_t* pub) noexcept {
    return reinterpret_cast<PublisherSlot*>(const_cast<rmw_publisher_t*>(pub));
}


}  // namespace

extern "C" {

/**
 * @brief Creates a publisher backed by a runtime-depth-selected RTPS writer.
 *
 * Resolves the message type's serializer (message_codec.hpp), mangles the
 * fully-qualified ROS topic name and DDS wire type name (topic_naming.hpp), builds a
 * `WriterConfig` from `qos_policies`, and creates the writer through
 * create_writer_for_qos() so its `CacheDepth` matches the requested
 * history depth. The resulting `StatefulWriter*` and codec are
 * stored in a pooled `Publisher`, and the `rmw_publisher_t` returned to
 * the caller points at it.
 *
 * @param[in] node Node to scope this publisher to.
 * @param[in] type_support Message type support from rosidl-generated code.
 * @param[in] topic_name Fully-qualified ROS 2 topic name, as rcl passes it.
 * @param[in] qos_policies Requested QoS profile.
 * @param[in] publisher_options Copied verbatim into the resulting
 *   `rmw_publisher_t::options`.
 * @return The new publisher, or `nullptr` if any required argument is
 *   null, the type support offers no supported typesupport family, the
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

    rmw_rosette::MessageCodec codec;
    if (!rmw_rosette::resolve_message_codec(type_support, codec)) {
        RMW_SET_ERROR_MSG("rmw_rosette: rmw_create_publisher: type support offers neither "
                          "introspection_c nor microxrcedds_c");
        return nullptr;
    }

    auto* node_impl = static_cast<rmw_rosette::Node*>(node->data);
    rmw_rosette::ContextLock lock(*node_impl->context);

    rtps::WriterConfig cfg{};
    rmw_rosette::fill_writer_config(cfg, *qos_policies);

    char mangled_topic[RTPS_MAX_TOPIC_NAME_LEN];
    char type_name[RTPS_MAX_TOPIC_NAME_LEN];
    if (!rmw_rosette::compose_topic_name(mangled_topic, sizeof(mangled_topic), topic_name) ||
        !rmw_rosette::compose_codec_type_name(codec, type_name, sizeof(type_name))) {
        RMW_SET_ERROR_MSG("rmw_rosette: rmw_create_publisher: topic or type name longer than "
                          "RTPS_MAX_TOPIC_NAME_LEN - 1");
        return nullptr;
    }
    cfg.topicName.assign(mangled_topic);
    cfg.typeName.assign(type_name);

    PublisherSlot* slot = publisher_pool().acquire();
    if (!slot) {
        RMW_SET_ERROR_MSG("rmw_rosette: rmw_create_publisher: publisher pool exhausted "
                          "(RMW_ROSETTE_MAX_PUBLISHERS)");
        return nullptr;
    }

    auto* writer = rmw_rosette::create_writer_for_qos(
        *node_impl->context->participant, cfg, *qos_policies);
    if (!writer) {
        RMW_SET_ERROR_MSG("rmw_rosette: rmw_create_publisher: create_writer failed "
                          "(participant writer table full)");
        publisher_pool().release(slot);
        return nullptr;
    }

    slot->impl.node = node_impl;
    slot->impl.writer = writer;
    slot->impl.codec = codec;
    slot->impl.qos = rmw_rosette::effective_qos(*qos_policies, true, 0);
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
 * @brief Destroys a publisher, its RTPS writer, and its pool slot.
 *
 * Deleting the writer through the participant withdraws it from SEDP, so
 * remote subscriptions unmatch it promptly instead of at lease expiry, and
 * frees its writer-table slot for reuse.
 *
 * @param[in] node Unused; kept for ABI symmetry with `rmw_create_publisher()`.
 * @param[in] publisher Publisher to destroy.
 * @return `RMW_RET_OK`, or `RMW_RET_INVALID_ARGUMENT` if `publisher` is null.
 *
 * Verification:
 * - Correct means: after destruction a stock subscriber's matched count
 *   drops, and a new publisher can reuse the writer slot.
 * - Covered by: TC-PART-8.5.1-005 (participant side).
 *
 * References:
 * - rmw/rmw.h (Humble), `rmw_destroy_publisher`
 */
rmw_ret_t rmw_destroy_publisher(rmw_node_t* node, rmw_publisher_t* publisher) {
    (void)node;
    if (!publisher || !publisher->data) return RMW_RET_INVALID_ARGUMENT;
    auto* impl = static_cast<rmw_rosette::Publisher*>(publisher->data);
    rmw_rosette::ContextLock lock(*impl->node->context);
    if (impl->node && impl->writer) {
        impl->node->context->participant->delete_writer(impl->writer);
    }
    publisher_pool().release(slot_from(publisher));
    return RMW_RET_OK;
}

/**
 * @brief Serializes and publishes a ROS message.
 *
 * Encodes `ros_message` into a pool buffer (serialize_to_pool()) and hands
 * it to the publisher's `StatefulWriter::write_async_owned()`, which adds
 * the change to the writer history, prepends the CDR encapsulation header,
 * and sends it to every matched reader, as DATA_FRAG when it exceeds the
 * transport's datagram limit. The call does
 * not wait for acknowledgments: RELIABLE delivery is completed by
 * HEARTBEAT/ACKNACK repair while rmw_wait() drives the participant.
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
    rmw_rosette::ContextLock lock(*impl->node->context);

    rtps::BufRef payload = rmw_rosette::serialize_to_pool(impl->codec, ros_message);
    if (!payload.valid()) {
        RMW_SET_ERROR_MSG("rmw_rosette: rmw_publish: serialization failed (message larger "
                          "than RMW_ROSETTE_MAX_SAMPLE_SIZE, or buffer pool exhausted)");
        return RMW_RET_ERROR;
    }

    // A DDS write with KEEP_LAST history does not wait for acknowledgments;
    // reliability comes from the history plus HEARTBEAT/ACKNACK repair driven
    // by rmw_wait(). Samples over the datagram limit go out as DATA_FRAG.
    const bool ok = impl->writer->write_async_owned(rtps::ChangeKind::ALIVE,
                                                    static_cast<rtps::BufRef&&>(payload));
    if (!ok) {
        RMW_SET_ERROR_MSG("rmw_rosette: rmw_publish: write failed");
        return RMW_RET_ERROR;
    }
    return RMW_RET_OK;
}

/**
 * @brief Reliably publishes an already-serialized message.
 *
 * The same delivery path as rmw_publish(), minus the encode step. Per the
 * rmw contract `serialized_message` holds a complete payload, header
 * included (see rmw_serialize.cpp). The RTPS writer always prepends its own
 * CDR_LE header, so this checks that the caller's header is CDR_LE and
 * hands the writer only the body; forwarding the whole buffer would put two
 * headers on the wire.
 *
 * @param[in] publisher Publisher to send through.
 * @param[in] serialized_message Complete serialized payload, header included.
 * @param[in] allocation Unused; this path performs no loaned-message allocation.
 * @return `RMW_RET_OK` on success; `RMW_RET_INVALID_ARGUMENT` if any
 *   required argument is null or the payload is not CDR_LE; `RMW_RET_ERROR`
 *   if the underlying write failed.
 *
 * Verification:
 * - Correct means: a buffer from rmw_serialize() arrives at a stock
 *   subscriber as the original message, with exactly one encapsulation header.
 *
 * References:
 * - rmw/rmw.h (Humble), `rmw_publish_serialized_message`
 * - OMG DDSI-RTPS 2.5 §10.2
 */
rmw_ret_t rmw_publish_serialized_message(const rmw_publisher_t* publisher,
                                          const rmw_serialized_message_t* serialized_message,
                                          rmw_publisher_allocation_t* allocation) {
    (void)allocation;
    if (!publisher || !publisher->data || !serialized_message) return RMW_RET_INVALID_ARGUMENT;

    constexpr size_t hdr = rmw_rosette::kRtpsEncapsulationHeaderSize;
    const uint8_t* buf = serialized_message->buffer;
    if (serialized_message->buffer_length < hdr || buf[0] != 0x00 || buf[1] != 0x01) {
        RMW_SET_ERROR_MSG("rmw_rosette: rmw_publish_serialized_message: payload is not CDR_LE");
        return RMW_RET_INVALID_ARGUMENT;
    }

    auto* impl = static_cast<rmw_rosette::Publisher*>(publisher->data);
    rmw_rosette::ContextLock lock(*impl->node->context);
    const bool ok = impl->writer->write_async(
        rtps::ChangeKind::ALIVE,
        reinterpret_cast<const rtps::Octet*>(buf + hdr),
        static_cast<uint32_t>(serialized_message->buffer_length - hdr));
    if (!ok) {
        RMW_SET_ERROR_MSG("rmw_rosette: rmw_publish_serialized_message: write_async failed");
        return RMW_RET_ERROR;
    }
    return RMW_RET_OK;
}

/**
 * @brief Reports how many remote subscriptions the publisher's writer is matched with.
 *
 * @param[in]  publisher Publisher to query.
 * @param[out] subscription_count Receives the matched reader count.
 * @return `RMW_RET_OK`, or `RMW_RET_INVALID_ARGUMENT` for null arguments.
 *
 * References:
 * - rmw/rmw.h (Humble), `rmw_publisher_count_matched_subscriptions`
 */
rmw_ret_t rmw_publisher_count_matched_subscriptions(const rmw_publisher_t* publisher,
                                                     size_t* subscription_count) {
    if (!publisher || !publisher->data || !subscription_count) return RMW_RET_INVALID_ARGUMENT;
    const auto* impl = static_cast<const rmw_rosette::Publisher*>(publisher->data);
    rmw_rosette::ContextLock lock(*impl->node->context);
    *subscription_count = impl->writer ? impl->writer->matched_reader_count() : 0u;
    return RMW_RET_OK;
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
    rmw_rosette::ContextLock lock(*impl->node->context);
    rmw_rosette::guid_to_rmw_gid(impl->writer->guid(), *gid);
    return RMW_RET_OK;
}

/**
 * @brief Reports the QoS profile the publisher's RTPS writer runs with.
 *
 * @param[in]  publisher Publisher to query.
 * @param[out] qos       Receives the effective profile (qos.hpp, effective_qos()).
 * @return `RMW_RET_OK`, or `RMW_RET_INVALID_ARGUMENT` for null arguments.
 *
 * References:
 * - rmw/rmw.h (Humble), `rmw_publisher_get_actual_qos`
 */
rmw_ret_t rmw_publisher_get_actual_qos(const rmw_publisher_t* publisher, rmw_qos_profile_t* qos) {
    if (!publisher || !publisher->data || !qos) return RMW_RET_INVALID_ARGUMENT;
    *qos = static_cast<const rmw_rosette::Publisher*>(publisher->data)->qos;
    return RMW_RET_OK;
}

/**
 * @brief Asserts liveliness for a MANUAL_BY_TOPIC publisher.
 *
 * The RTPS writer announces liveliness implicitly through its heartbeats,
 * which satisfies the AUTOMATIC liveliness kind stock subscribers default
 * to, so there is nothing further to send.
 *
 * @param[in] publisher Publisher asserting liveliness.
 * @return `RMW_RET_OK`, or `RMW_RET_INVALID_ARGUMENT` if null.
 *
 * References:
 * - rmw/rmw.h (Humble), `rmw_publisher_assert_liveliness`
 * - OMG DDSI-RTPS 2.5 §8.4.13 (writer liveliness)
 */
rmw_ret_t rmw_publisher_assert_liveliness(const rmw_publisher_t* publisher) {
    if (!publisher || !publisher->data) return RMW_RET_INVALID_ARGUMENT;
    return RMW_RET_OK;
}

/**
 * @brief Waits until every matched subscription acknowledged every sample.
 *
 * The opt-in "block until delivered" for applications that need it (rclcpp
 * `Publisher::wait_for_all_acked()`, DDS `wait_for_acknowledgments()`):
 * publishing itself never blocks. The wait runs the participant through its
 * driver, so the acknowledgments being waited for are received and processed
 * during the wait, on any platform, including bare metal.
 *
 * @param[in] publisher    Publisher to wait on.
 * @param[in] wait_timeout Longest wait; {0,0} checks once; RMW_DURATION_INFINITE
 *   waits indefinitely.
 * @return `RMW_RET_OK` once all samples are acknowledged (immediately for a
 *   BEST_EFFORT publisher or with no matched subscriptions);
 *   `RMW_RET_TIMEOUT` otherwise; `RMW_RET_INVALID_ARGUMENT` for a null argument.
 *
 * Verification:
 * - Correct means: returns OK after a stock reliable subscriber has
 *   acknowledged the last sample, TIMEOUT while one has not.
 * - Covered by: TC-SWR-8.4.7-001 (writer acknowledgment state).
 *
 * References:
 * - rmw/rmw.h (Humble), `rmw_publisher_wait_for_all_acked`
 * - OMG DDS 1.4 §2.2.2.4.1.12 (DataWriter::wait_for_acknowledgments)
 */
rmw_ret_t rmw_publisher_wait_for_all_acked(const rmw_publisher_t* publisher,
                                           rmw_time_t wait_timeout) {
    if (!publisher || !publisher->data) return RMW_RET_INVALID_ARGUMENT;
    auto* impl = static_cast<rmw_rosette::Publisher*>(publisher->data);

    uint32_t timeout_ms = rosette::platform::kWaitForever;
    const rmw_time_t infinite = RMW_DURATION_INFINITE;
    if (!rmw_time_equal(wait_timeout, infinite)) {
        const uint64_t ms = rmw_time_total_nsec(wait_timeout) / 1000000u;
        timeout_ms = ms >= rosette::platform::kWaitForever
            ? rosette::platform::kWaitForever - 1u : static_cast<uint32_t>(ms);
    }
    rtps::StatefulWriter* writer = impl->writer;
    const bool acked = impl->node->context->driver->spin_until(
        [writer]() noexcept { return writer->acknowledged_by_all(); }, timeout_ms);
    return acked ? RMW_RET_OK : RMW_RET_TIMEOUT;
}

/// @brief Preallocated publish buffers are not offered; rmw_publish() uses a stack buffer.
rmw_ret_t rmw_init_publisher_allocation(const rosidl_message_type_support_t* type_support,
                                        const rosidl_runtime_c__Sequence__bound* message_bounds,
                                        rmw_publisher_allocation_t* allocation) {
    (void)type_support; (void)message_bounds; (void)allocation;
    return RMW_RET_UNSUPPORTED;
}

/// @brief Counterpart of rmw_init_publisher_allocation(); unsupported.
rmw_ret_t rmw_fini_publisher_allocation(rmw_publisher_allocation_t* allocation) {
    (void)allocation;
    return RMW_RET_UNSUPPORTED;
}

/// @brief Network flow endpoints (unique-network-flow QoS) are not offered.
rmw_ret_t rmw_publisher_get_network_flow_endpoints(
    const rmw_publisher_t* publisher, rcutils_allocator_t* allocator,
    rmw_network_flow_endpoint_array_t* network_flow_endpoint_array) {
    (void)publisher; (void)allocator; (void)network_flow_endpoint_array;
    return RMW_RET_UNSUPPORTED;
}

}  // extern "C"
