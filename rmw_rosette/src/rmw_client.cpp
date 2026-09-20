/**
 * @file rmw_client.cpp
 * @brief rmw_create_client / rmw_send_request / rmw_take_response.
 *
 * Mirror image of rmw_service.cpp: a writer on the "rq/..." request topic,
 * a reader on the "rr/..." reply topic, correlated the same way via
 * SampleIdentity.
 */

#include "rmw_rosette/identifier.h"
#include "rmw_rosette/types.hpp"

#include <rmw/rmw.h>
#include <rmw/error_handling.h>

extern "C" {

rmw_client_t* rmw_create_client(const rmw_node_t* node,
                                 const rosidl_service_type_support_t* type_support,
                                 const char* service_name,
                                 const rmw_qos_profile_t* qos_policies) {
    (void)node; (void)type_support; (void)service_name; (void)qos_policies;
    RMW_SET_ERROR_MSG("rmw_rosette: rmw_create_client not implemented");
    return nullptr;
}

rmw_ret_t rmw_destroy_client(rmw_node_t* node, rmw_client_t* client) {
    (void)node; (void)client;
    RMW_SET_ERROR_MSG("rmw_rosette: rmw_destroy_client not implemented");
    return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_send_request(const rmw_client_t* client, const void* ros_request,
                            int64_t* sequence_id) {
    (void)client; (void)ros_request;
    if (!sequence_id) return RMW_RET_INVALID_ARGUMENT;
    // TODO: serialize + request_writer->write_reliable(); *sequence_id
    // should be the writer's assigned SequenceNumber so rmw_take_response()
    // can match the reply's relatedSampleIdentity back to this call.
    RMW_SET_ERROR_MSG("rmw_rosette: rmw_send_request not implemented");
    return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_take_response(const rmw_client_t* client, rmw_service_info_t* request_header,
                             void* ros_response, bool* taken) {
    (void)client; (void)ros_response; (void)request_header;
    if (!taken) return RMW_RET_INVALID_ARGUMENT;
    *taken = false;
    RMW_SET_ERROR_MSG("rmw_rosette: rmw_take_response not implemented");
    return RMW_RET_UNSUPPORTED;
}

}  // extern "C"
