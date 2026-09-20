/**
 * @file rmw_service.cpp
 * @brief rmw_create_service / rmw_take_request / rmw_send_response.
 *
 * A ROS 2 service is two RTPS endpoints: a reader on "rq/.../<Srv>Request"
 * and a writer on "rr/.../<Srv>Reply", correlated by SampleIdentity in the
 * DATA submessage's inline QoS — already implemented in rtps/types.hpp
 * (SampleIdentity, InlineQos, inline_qos_pid::RELATED_SAMPLE_IDENTITY) and
 * carried by write_reliable()'s InlineQos parameter (see
 * stateful_writer.hpp). This file is mostly wiring two endpoints together
 * plus that correlation, not new protocol work.
 */

#include "rmw_rosette/identifier.h"
#include "rmw_rosette/types.hpp"

#include <rmw/rmw.h>
#include <rmw/error_handling.h>

extern "C" {

rmw_service_t* rmw_create_service(const rmw_node_t* node,
                                   const rosidl_service_type_support_t* type_support,
                                   const char* service_name,
                                   const rmw_qos_profile_t* qos_policies) {
    (void)node; (void)type_support; (void)service_name; (void)qos_policies;
    // TODO: compose_service_request_topic() / compose_service_reply_topic(),
    // create_reader() for the request side, create_writer<>() for the
    // reply side. request_type_support/response_type_support come from
    // rosidl_service_type_support_t (a pair of message type supports).
    RMW_SET_ERROR_MSG("rmw_rosette: rmw_create_service not implemented");
    return nullptr;
}

rmw_ret_t rmw_destroy_service(rmw_node_t* node, rmw_service_t* service) {
    (void)node; (void)service;
    RMW_SET_ERROR_MSG("rmw_rosette: rmw_destroy_service not implemented");
    return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_take_request(const rmw_service_t* service, rmw_service_info_t* request_header,
                            void* ros_request, bool* taken) {
    (void)service; (void)ros_request;
    if (!taken) return RMW_RET_INVALID_ARGUMENT;
    *taken = false;
    // TODO: pop from the request reader's buffered changes, deserialize,
    // and stash the incoming SampleIdentity (writerGuid + sequenceNumber)
    // in request_header->request_id so rmw_send_response() can echo it
    // back as relatedSampleIdentity.
    (void)request_header;
    RMW_SET_ERROR_MSG("rmw_rosette: rmw_take_request not implemented");
    return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_send_response(const rmw_service_t* service, rmw_request_id_t* request_header,
                             void* ros_response) {
    (void)service; (void)ros_response;
    if (!request_header) return RMW_RET_INVALID_ARGUMENT;
    // TODO: serialize ros_response, build an InlineQos with
    // hasRelatedSampleIdentity=true set from request_header, and call
    // reply_writer->write_reliable(ChangeKind::ALIVE, buf, len, qos, ...).
    RMW_SET_ERROR_MSG("rmw_rosette: rmw_send_response not implemented");
    return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_service_server_is_available(const rmw_node_t* node, const rmw_client_t* client,
                                           bool* is_available) {
    (void)node; (void)client;
    if (!is_available) return RMW_RET_INVALID_ARGUMENT;
    // TODO: true once the client's request-writer has at least one
    // matched reader (GraphCache or a matched-count accessor — same gap
    // noted in rmw_publisher_count_matched_subscriptions).
    *is_available = false;
    return RMW_RET_UNSUPPORTED;
}

}  // extern "C"
