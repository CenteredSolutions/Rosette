/**
 * @file rmw_service.cpp
 * @brief rmw_create_service / rmw_take_request / rmw_send_response.
 *
 * A service is a reader on the request topic and a writer on the reply topic
 * (service_endpoints.hpp). A request's identity is the GUID of the writer that
 * sent it plus that DATA's sequence number; rmw_take_request() reports it as
 * the rmw_request_id_t, rcl hands it back to rmw_send_response(), and the
 * reply carries it as PID_RELATED_SAMPLE_IDENTITY so the calling client, and
 * only that client, accepts the reply. This is the wire contract
 * rmw_fastrtps_cpp uses, so stock clients call rosette services directly.
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

namespace rtps = rosette::rtps;

namespace {

struct ServiceSlot {
    rmw_service_t handle{};
    rmw_rosette::Service impl{};
};

rmw_rosette::Pool<ServiceSlot, rmw_rosette::kServiceCapacity>& service_pool() {
    static rmw_rosette::Pool<ServiceSlot, rmw_rosette::kServiceCapacity> pool;
    return pool;
}

/// `rmw_service_t` is the first member of `ServiceSlot` (standard layout).
ServiceSlot* slot_from(rmw_service_t* service) noexcept {
    return reinterpret_cast<ServiceSlot*>(service);
}

/// Request-reader callback: buffers the request with its writer GUID and SN.
void on_request(const rtps::CacheChange& change, void* user_data) noexcept {
    static_cast<rmw_rosette::Service*>(user_data)->requests.push(change);
}

}  // namespace

extern "C" {

/**
 * @brief Creates a service server backed by a request reader and a reply writer.
 *
 * @param[in] node         Node the service belongs to.
 * @param[in] type_support Service type support from rosidl-generated code.
 * @param[in] service_name Fully-qualified service name, as rcl passes it.
 * @param[in] qos_policies QoS for both endpoints (rcl defaults to
 *   rmw_qos_profile_services_default: RELIABLE, VOLATILE, KEEP_LAST(10)).
 * @return The new service, or `nullptr` when an argument is null, the type
 *   resolves to no codec, the service pool (kServiceCapacity) is exhausted,
 *   or the participant tables are full.
 *
 * Verification:
 * - Correct means: a stock rclpy/rclcpp client on rmw_fastrtps_cpp calls the
 *   service and receives exactly its own replies.
 * - Covered by: tests/ros2 service interop.
 *
 * References:
 * - rmw/rmw.h (Humble), `rmw_create_service`
 */
rmw_service_t* rmw_create_service(const rmw_node_t* node,
                                  const rosidl_service_type_support_t* type_support,
                                  const char* service_name,
                                  const rmw_qos_profile_t* qos_policies) {
    if (!node || !node->data || !type_support || !service_name || !qos_policies) {
        RMW_SET_ERROR_MSG("rmw_rosette: rmw_create_service: null argument");
        return nullptr;
    }
    auto* node_impl = static_cast<rmw_rosette::Node*>(node->data);
    rmw_rosette::ContextLock lock(*node_impl->context);

    ServiceSlot* slot = service_pool().acquire();
    if (!slot) {
        RMW_SET_ERROR_MSG("rmw_rosette: rmw_create_service: service pool exhausted "
                          "(RMW_ROSETTE_MAX_SERVICES + parameter services per node)");
        return nullptr;
    }
    rmw_rosette::Service& impl = slot->impl;
    if (!rmw_rosette::resolve_service_codecs(type_support, impl.request_codec,
                                             impl.response_codec)) {
        RMW_SET_ERROR_MSG("rmw_rosette: rmw_create_service: unsupported type support");
        service_pool().release(slot);
        return nullptr;
    }
    const rmw_rosette::ServiceEndpointsStatus status = rmw_rosette::create_service_endpoints(
            *node_impl, rmw_rosette::ServiceRole::Server, service_name, *qos_policies,
            impl.request_codec, impl.response_codec, on_request, &impl, impl.reply_writer,
            impl.request_reader);
    if (status != rmw_rosette::ServiceEndpointsStatus::Ok) {
        RMW_SET_ERROR_MSG(
            status == rmw_rosette::ServiceEndpointsStatus::NameTooLong
                ? "rmw_rosette: rmw_create_service: topic or type name longer than "
                  "RTPS_MAX_TOPIC_NAME_LEN - 1"
            : status == rmw_rosette::ServiceEndpointsStatus::TablesFull
                ? "rmw_rosette: rmw_create_service: participant endpoint tables full"
                : "rmw_rosette: rmw_create_service: cannot publish ros_discovery_info "
                  "(buffer pool exhausted)");
        service_pool().release(slot);
        return nullptr;
    }
    impl.node = node_impl;
    impl.qos = *qos_policies;
    impl.requests.set_depth(static_cast<uint32_t>(
        rmw_rosette::effective_qos(impl.qos, false, RMW_ROSETTE_MAX_PENDING_CHANGES).depth));
    std::strncpy(impl.service_name, service_name, sizeof(impl.service_name) - 1);

    slot->handle.implementation_identifier = rmw_rosette_identifier;
    slot->handle.data = &impl;
    slot->handle.service_name = impl.service_name;
    return &slot->handle;
}

/**
 * @brief Destroys a service, withdrawing both endpoints from discovery.
 *
 * @param[in] node    Unused; kept for ABI symmetry.
 * @param[in] service Service to destroy.
 * @return `RMW_RET_OK`, or `RMW_RET_INVALID_ARGUMENT` if `service` is null.
 *
 * References:
 * - rmw/rmw.h (Humble), `rmw_destroy_service`
 */
rmw_ret_t rmw_destroy_service(rmw_node_t* node, rmw_service_t* service) {
    (void)node;
    if (!service || !service->data) return RMW_RET_INVALID_ARGUMENT;
    auto* impl = static_cast<rmw_rosette::Service*>(service->data);
    rmw_rosette::ContextLock lock(*impl->node->context);
    if (impl->node) {
        rmw_rosette::destroy_service_endpoints(*impl->node, impl->reply_writer,
                                               impl->request_reader);
    }
    service_pool().release(slot_from(service));
    return RMW_RET_OK;
}

/**
 * @brief Takes the oldest buffered request and reports its identity.
 *
 * @param[in]  service        Service to take from.
 * @param[out] request_header Receives the request writer's GUID and the
 *   request's SN (`request_id`) and its source timestamp.
 * @param[out] ros_request    Initialized request message to decode into.
 * @param[out] taken          `true` when a request was taken.
 * @return `RMW_RET_OK` (including when nothing was waiting);
 *   `RMW_RET_INVALID_ARGUMENT` for null arguments; `RMW_RET_ERROR` when the
 *   request fails to decode.
 *
 * Verification:
 * - Correct means: `request_id` equals the {writer GUID, SN} of the DATA
 *   that carried the request, the identity a stock client correlates on.
 *
 * References:
 * - rmw/rmw.h (Humble), `rmw_take_request`
 */
rmw_ret_t rmw_take_request(const rmw_service_t* service, rmw_service_info_t* request_header,
                           void* ros_request, bool* taken) {
    if (!service || !service->data || !request_header || !ros_request || !taken) {
        return RMW_RET_INVALID_ARGUMENT;
    }
    *taken = false;
    auto* impl = static_cast<rmw_rosette::Service*>(service->data);
    rmw_rosette::ContextLock lock(*impl->node->context);
    rmw_rosette::PendingChange change;
    if (!impl->requests.pop(change)) return RMW_RET_OK;

    if (!rmw_rosette::deserialize_message(impl->request_codec, change.payload.data, change.payload.size,
                                          ros_request)) {
        RMW_SET_ERROR_MSG("rmw_rosette: rmw_take_request: malformed request");
        return RMW_RET_ERROR;
    }
    rmw_rosette::guid_to_request_writer(change.writer_guid,
                                        request_header->request_id.writer_guid);
    request_header->request_id.sequence_number = change.sequence_number.to_int64();
    request_header->source_timestamp = rmw_rosette::timestamp_to_ns(change.source_timestamp);
    request_header->received_timestamp = 0;
    *taken = true;
    return RMW_RET_OK;
}

/**
 * @brief Sends a reply correlated with the request identified by `request_header`.
 *
 * The reply goes to every matched client reader; each client keeps only
 * replies whose related sample identity names its own request writer.
 *
 * A client that has left is not replied to. The reply writer keeps a reply
 * whose client has no matched reader yet, so it reaches a reply reader that
 * matches late (StatefulWriter::release_replies_on_delivery()); a reply to a
 * client that is gone would instead stay in the history until KEEP_LAST
 * evicts it and be replayed to every later client. The request writer named
 * in `request_header` tells the two apart: it stays matched to this
 * service's request reader for as long as the client exists, and a new
 * client has a writer of its own, so the test is exact per client. The
 * response is then dropped before it is encoded, and the call succeeds, as
 * rmw_fastrtps_cpp does for a client it reports gone.
 *
 * @param[in] service        Service replying.
 * @param[in] request_header The `request_id` rmw_take_request() reported.
 * @param[in] ros_response   Response message to encode.
 * @return `RMW_RET_OK` when the reply is written, or dropped because its
 *   client has left; `RMW_RET_INVALID_ARGUMENT` for null arguments;
 *   `RMW_RET_ERROR` when encoding or the write fails.
 *
 * Verification:
 * - Correct means: the reply's PID_RELATED_SAMPLE_IDENTITY equals
 *   `request_header`, so the stock client that sent the request accepts it;
 *   a reply whose request writer is no longer matched leaves nothing in the
 *   reply writer's history.
 * - Covered by: rmw_rosette/test/test_service_departed_client.cpp.
 *
 * References:
 * - rmw/rmw.h (Humble), `rmw_send_response`
 * - OMG DDS-RPC 1.0 §7.8.2 (related sample identity)
 * - OMG DDSI-RTPS 2.5 §8.4.7.4 (matched_writer_lookup)
 */
rmw_ret_t rmw_send_response(const rmw_service_t* service, rmw_request_id_t* request_header,
                            void* ros_response) {
    if (!service || !service->data || !request_header || !ros_response) {
        return RMW_RET_INVALID_ARGUMENT;
    }
    auto* impl = static_cast<rmw_rosette::Service*>(service->data);
    rmw_rosette::ContextLock lock(*impl->node->context);
    const rtps::Guid request_writer =
        rmw_rosette::guid_from_request_writer(request_header->writer_guid);
    if (impl->request_reader->matched_writer_lookup(request_writer) == nullptr) {
        return RMW_RET_OK;
    }

    rtps::BufRef payload = rmw_rosette::serialize_to_pool(impl->response_codec, ros_response);
    if (!payload.valid()) {
        RMW_SET_ERROR_MSG("rmw_rosette: rmw_send_response: serialization failed");
        return RMW_RET_ERROR;
    }

    rtps::InlineQos qos{};
    qos.hasRelatedSampleIdentity = true;
    qos.relatedSampleIdentity.writerGuid = request_writer;
    qos.relatedSampleIdentity.sequenceNumber =
        rtps::SequenceNumber::from_int64(request_header->sequence_number);
    if (!impl->reply_writer->write_async_owned(rtps::ChangeKind::ALIVE,
                                               static_cast<rtps::BufRef&&>(payload), qos)) {
        RMW_SET_ERROR_MSG("rmw_rosette: rmw_send_response: write failed");
        return RMW_RET_ERROR;
    }
    return RMW_RET_OK;
}

/// @brief Reports the effective QoS of the request reader.
rmw_ret_t rmw_service_request_subscription_get_actual_qos(const rmw_service_t* service,
                                                          rmw_qos_profile_t* qos) {
    if (!service || !service->data || !qos) return RMW_RET_INVALID_ARGUMENT;
    *qos = rmw_rosette::effective_qos(static_cast<const rmw_rosette::Service*>(service->data)->qos,
                                      false, RMW_ROSETTE_MAX_PENDING_CHANGES);
    return RMW_RET_OK;
}

/**
 * @brief Reports the effective QoS of the reply writer.
 *
 * The reply writer is TRANSIENT_LOCAL with KEEP_LAST(RMW_ROSETTE_REPLY_DEPTH)
 * history whatever the service requested (create_service_endpoints()), so
 * those replace the requested durability and depth.
 *
 * @param[in]  service The service.
 * @param[out] qos     Receives the reply writer's QoS.
 * @return RMW_RET_OK, or RMW_RET_INVALID_ARGUMENT for a null argument.
 *
 * Verification:
 * - Correct means: depth and durability are those of the reply writer's
 *   history, for any requested profile.
 * - Covered by: rmw_rosette/test/test_service_reply_depth.cpp.
 *
 * References:
 * - rmw/rmw.h (Humble), `rmw_service_response_publisher_get_actual_qos`
 */
rmw_ret_t rmw_service_response_publisher_get_actual_qos(const rmw_service_t* service,
                                                        rmw_qos_profile_t* qos) {
    if (!service || !service->data || !qos) {
        return RMW_RET_INVALID_ARGUMENT;
    }

    *qos = rmw_rosette::effective_qos(static_cast<const rmw_rosette::Service*>(service->data)->qos,
                                      true, 0);
    qos->durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
    qos->depth = RMW_ROSETTE_REPLY_DEPTH;
    return RMW_RET_OK;
}

}  // extern "C"
