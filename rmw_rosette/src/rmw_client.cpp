/**
 * @file rmw_client.cpp
 * @brief rmw_create_client / rmw_send_request / rmw_take_response.
 *
 * A client is a writer on the request topic and a reader on the reply topic
 * (service_endpoints.hpp). Each request carries PID_RELATED_SAMPLE_IDENTITY
 * naming this client's reply reader, the convention rmw_fastrtps_cpp servers
 * expect. Every server reply goes to every client of the service; this client
 * keeps only those whose related sample identity names its request writer or
 * its reply reader, and reports the related sequence number, which
 * rmw_send_request() returned as the request's sequence id, so rcl can pair
 * reply with request.
 */

#include "rmw_rosette/guid.hpp"
#include "rmw_rosette/identifier.h"
#include "rmw_rosette/message_codec.hpp"
#include "rmw_rosette/pool.hpp"
#include "rmw_rosette/qos.hpp"
#include "rmw_rosette/service_endpoints.hpp"
#include "rmw_rosette/types.hpp"

#include <rmw/error_handling.h>
#include <rmw/rmw.h>

#include <cstring>

namespace {

struct ClientSlot {
    rmw_client_t handle{};
    rmw_rosette::Client impl{};
};

rmw_rosette::Pool<ClientSlot, rmw_rosette::kClientCapacity>& client_pool() {
    static rmw_rosette::Pool<ClientSlot, rmw_rosette::kClientCapacity> pool;
    return pool;
}

/// `rmw_client_t` is the first member of `ClientSlot` (standard layout).
ClientSlot* slot_from(rmw_client_t* client) noexcept {
    return reinterpret_cast<ClientSlot*>(client);
}

/// Reply-reader callback: buffers only replies correlated with this client,
/// so rmw_wait() wakes only for this client's replies. A server names the
/// request either by the request writer's GUID or, when the request carried
/// one, by the client's reply-reader GUID (rmw_fastrtps_cpp servers do the
/// latter), so both identify this client.
void on_reply(const rtps::CacheChange& change, void* user_data) noexcept {
    auto* impl = static_cast<rmw_rosette::Client*>(user_data);
    if (!impl->request_writer || !impl->reply_reader ||
        !change.inlineQos.hasRelatedSampleIdentity) {
        return;
    }
    const rtps::Guid& target = change.inlineQos.relatedSampleIdentity.writerGuid;
    if (target != impl->request_writer->guid() && target != impl->reply_reader->guid()) return;
    impl->replies.push(change);
}

/// Serialization buffer for a request; one RTPS datagram without fragmentation.
constexpr uint32_t kRequestBufSize = RTPS_BUF_BLOCK_SIZE;

}  // namespace

extern "C" {

/**
 * @brief Creates a service client backed by a request writer and a reply reader.
 *
 * @param[in] node         Node the client belongs to.
 * @param[in] type_support Service type support from rosidl-generated code.
 * @param[in] service_name Fully-qualified service name, as rcl passes it.
 * @param[in] qos_policies QoS for both endpoints.
 * @return The new client, or `nullptr` when an argument is null, the type
 *   resolves to no codec, the client pool (kClientCapacity) is exhausted,
 *   or the participant tables are full.
 *
 * Verification:
 * - Correct means: calls to a stock rclpy/rclcpp service on
 *   rmw_fastrtps_cpp return that service's replies.
 * - Covered by: tests/ros2 service interop.
 *
 * References:
 * - rmw/rmw.h (Humble), `rmw_create_client`
 */
rmw_client_t* rmw_create_client(const rmw_node_t* node,
                                const rosidl_service_type_support_t* type_support,
                                const char* service_name,
                                const rmw_qos_profile_t* qos_policies) {
    if (!node || !node->data || !type_support || !service_name || !qos_policies) {
        RMW_SET_ERROR_MSG("rmw_rosette: rmw_create_client: null argument");
        return nullptr;
    }
    auto* node_impl = static_cast<rmw_rosette::Node*>(node->data);

    ClientSlot* slot = client_pool().acquire();
    if (!slot) {
        RMW_SET_ERROR_MSG("rmw_rosette: rmw_create_client: client pool exhausted "
                          "(RMW_ROSETTE_MAX_CLIENTS)");
        return nullptr;
    }
    rmw_rosette::Client& impl = slot->impl;
    if (!rmw_rosette::resolve_service_codecs(type_support, impl.request_codec,
                                             impl.response_codec)) {
        RMW_SET_ERROR_MSG("rmw_rosette: rmw_create_client: unsupported type support");
        client_pool().release(slot);
        return nullptr;
    }
    if (!rmw_rosette::create_service_endpoints(
            *node_impl, rmw_rosette::ServiceRole::Client, service_name, *qos_policies,
            impl.request_codec, impl.response_codec, on_reply, &impl, impl.request_writer,
            impl.reply_reader)) {
        RMW_SET_ERROR_MSG("rmw_rosette: rmw_create_client: participant endpoint tables full");
        client_pool().release(slot);
        return nullptr;
    }
    impl.node = node_impl;
    impl.qos = *qos_policies;
    std::strncpy(impl.service_name, service_name, sizeof(impl.service_name) - 1);

    slot->handle.implementation_identifier = rmw_rosette_identifier;
    slot->handle.data = &impl;
    slot->handle.service_name = impl.service_name;
    return &slot->handle;
}

/**
 * @brief Destroys a client, withdrawing both endpoints from discovery.
 *
 * @param[in] node   Unused; kept for ABI symmetry.
 * @param[in] client Client to destroy.
 * @return `RMW_RET_OK`, or `RMW_RET_INVALID_ARGUMENT` if `client` is null.
 *
 * References:
 * - rmw/rmw.h (Humble), `rmw_destroy_client`
 */
rmw_ret_t rmw_destroy_client(rmw_node_t* node, rmw_client_t* client) {
    (void)node;
    if (!client || !client->data) return RMW_RET_INVALID_ARGUMENT;
    auto* impl = static_cast<rmw_rosette::Client*>(client->data);
    if (impl->node) {
        rmw_rosette::destroy_service_endpoints(*impl->node, impl->request_writer,
                                               impl->reply_reader);
    }
    client_pool().release(slot_from(client));
    return RMW_RET_OK;
}

/**
 * @brief Sends a request and returns the sequence id its reply will carry.
 *
 * @param[in]  client      Client sending the request.
 * @param[in]  ros_request Request message to encode.
 * @param[out] sequence_id The request DATA's sequence number; the matching
 *   reply's `request_id.sequence_number` equals it.
 * @return `RMW_RET_OK`; `RMW_RET_INVALID_ARGUMENT` for null arguments;
 *   `RMW_RET_ERROR` when encoding or the write fails.
 *
 * Verification:
 * - Correct means: a stock server replies with related sample identity
 *   {this client's request writer GUID, `*sequence_id`}.
 *
 * References:
 * - rmw/rmw.h (Humble), `rmw_send_request`
 * - rmw_fastrtps_shared_cpp (Humble), rmw_request.cpp
 */
rmw_ret_t rmw_send_request(const rmw_client_t* client, const void* ros_request,
                           int64_t* sequence_id) {
    if (!client || !client->data || !ros_request || !sequence_id) {
        return RMW_RET_INVALID_ARGUMENT;
    }
    auto* impl = static_cast<rmw_rosette::Client*>(client->data);

    uint8_t buf[kRequestBufSize];
    const uint32_t len =
        rmw_rosette::serialize_message(impl->request_codec, ros_request, buf, sizeof(buf));
    if (len == 0) {
        RMW_SET_ERROR_MSG("rmw_rosette: rmw_send_request: serialization failed");
        return RMW_RET_ERROR;
    }

    // Name this client's reply reader, as rmw_fastrtps_cpp clients do; the
    // sequence number is unknown until the writer assigns it.
    rtps::InlineQos qos{};
    qos.hasRelatedSampleIdentity = true;
    qos.relatedSampleIdentity.writerGuid = impl->reply_reader->guid();
    qos.relatedSampleIdentity.sequenceNumber = rtps::SEQUENCENUMBER_UNKNOWN;

    rtps::SequenceNumber sn = rtps::SEQUENCENUMBER_UNKNOWN;
    if (!impl->request_writer->write_async(rtps::ChangeKind::ALIVE, buf, len, qos, &sn)) {
        RMW_SET_ERROR_MSG("rmw_rosette: rmw_send_request: write failed");
        return RMW_RET_ERROR;
    }
    *sequence_id = sn.to_int64();
    return RMW_RET_OK;
}

/**
 * @brief Takes the oldest reply addressed to this client.
 *
 * @param[in]  client         Client to take from.
 * @param[out] request_header `request_id` of the request this replies to:
 *   the GUID the server correlated on (this client's request writer or reply
 *   reader) and the sequence id rmw_send_request() returned.
 * @param[out] ros_response   Initialized response message to decode into.
 * @param[out] taken          `true` when a reply was taken.
 * @return `RMW_RET_OK` (including when nothing was waiting);
 *   `RMW_RET_INVALID_ARGUMENT` for null arguments; `RMW_RET_ERROR` when the
 *   reply fails to decode.
 *
 * References:
 * - rmw/rmw.h (Humble), `rmw_take_response`
 */
rmw_ret_t rmw_take_response(const rmw_client_t* client, rmw_service_info_t* request_header,
                            void* ros_response, bool* taken) {
    if (!client || !client->data || !request_header || !ros_response || !taken) {
        return RMW_RET_INVALID_ARGUMENT;
    }
    *taken = false;
    auto* impl = static_cast<rmw_rosette::Client*>(client->data);
    rmw_rosette::PendingChange change;
    if (!impl->replies.pop(change)) return RMW_RET_OK;

    if (!rmw_rosette::deserialize_message(impl->response_codec, change.bytes, change.size,
                                          ros_response)) {
        RMW_SET_ERROR_MSG("rmw_rosette: rmw_take_response: malformed reply");
        return RMW_RET_ERROR;
    }
    rmw_rosette::guid_to_request_writer(change.related_sample_identity.writerGuid,
                                        request_header->request_id.writer_guid);
    request_header->request_id.sequence_number =
        change.related_sample_identity.sequenceNumber.to_int64();
    request_header->source_timestamp = rmw_rosette::timestamp_to_ns(change.source_timestamp);
    request_header->received_timestamp = 0;
    *taken = true;
    return RMW_RET_OK;
}

/**
 * @brief Reports whether a server for the client's service is reachable.
 *
 * rclcpp's `wait_for_service()` and rclpy's `Client.wait_for_service()` poll
 * this. A server is available once the request writer has matched a server's
 * request reader and the reply reader has matched its reply writer; before
 * then a request could be sent but its reply would be lost.
 *
 * @param[in]  node         Unused; the client carries its endpoints.
 * @param[in]  client       Client to check.
 * @param[out] is_available `true` when both endpoints are matched.
 * @return `RMW_RET_OK`, or `RMW_RET_INVALID_ARGUMENT` for null arguments.
 *
 * References:
 * - rmw/rmw.h (Humble), `rmw_service_server_is_available`
 */
rmw_ret_t rmw_service_server_is_available(const rmw_node_t* node, const rmw_client_t* client,
                                          bool* is_available) {
    (void)node;
    if (!client || !client->data || !is_available) return RMW_RET_INVALID_ARGUMENT;
    const auto* impl = static_cast<const rmw_rosette::Client*>(client->data);
    *is_available = impl->request_writer && impl->reply_reader &&
                    impl->request_writer->matched_reader_count() > 0u &&
                    impl->reply_reader->matched_writer_count() > 0u;
    return RMW_RET_OK;
}

/// @brief Reports the effective QoS of the request writer.
rmw_ret_t rmw_client_request_publisher_get_actual_qos(const rmw_client_t* client,
                                                      rmw_qos_profile_t* qos) {
    if (!client || !client->data || !qos) return RMW_RET_INVALID_ARGUMENT;
    *qos = rmw_rosette::effective_qos(static_cast<const rmw_rosette::Client*>(client->data)->qos,
                                      true, 0);
    return RMW_RET_OK;
}

/// @brief Reports the effective QoS of the reply reader.
rmw_ret_t rmw_client_response_subscription_get_actual_qos(const rmw_client_t* client,
                                                          rmw_qos_profile_t* qos) {
    if (!client || !client->data || !qos) return RMW_RET_INVALID_ARGUMENT;
    *qos = rmw_rosette::effective_qos(static_cast<const rmw_rosette::Client*>(client->data)->qos,
                                      false, RMW_ROSETTE_MAX_PENDING_CHANGES);
    return RMW_RET_OK;
}

}  // extern "C"
