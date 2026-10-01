/**
 * @file rmw_graph.cpp
 * @brief rmw graph queries: names and types, endpoint counts, per-node lists.
 *
 * The graph an rmw implementation reports has two sources. RTPS discovery
 * (SPDP and SEDP) knows every participant and endpoint; the participant's
 * EndpointMatchTable holds those endpoints and answers per-topic counts
 * (EndpointMatchTable::count()). Which ROS node owns an endpoint, and which
 * nodes exist at all, is carried only by the rmw_dds_common
 * ParticipantEntitiesInfo messages each participant publishes on
 * `ros_discovery_info`; queries that need node names return
 * RMW_RET_UNSUPPORTED, because this participant publishes that topic but
 * does not read it.
 *
 * Every query takes the context lock: the tables it reads are updated by
 * the thread driving the participant.
 */

#include "rmw_rosette/identifier.h"
#include "rmw_rosette/topic_naming.hpp"
#include "rmw_rosette/types.hpp"

#include <rmw/rmw.h>
#include <rmw/error_handling.h>
#include <rmw/get_node_info_and_types.h>
#include <rmw/get_topic_names_and_types.h>
#include <rmw/names_and_types.h>
#include <rmw/topic_endpoint_info_array.h>
#include <rmw/validate_full_topic_name.h>

#include <cstring>

namespace {

/**
 * @brief Counts the writers or readers the graph knows on one ROS topic.
 *
 * The shared body of rmw_count_publishers() and rmw_count_subscribers().
 * The ROS name is mangled to its DDS topic name ("/chatter" becomes
 * "rt/chatter", as publishers and subscriptions announce it) and counted in
 * the participant's EndpointMatchTable, which holds this participant's own
 * endpoints and every remote one SEDP has announced.
 *
 * @param[in]  node       Node the query is made through; selects the context.
 * @param[in]  topic_name Fully-qualified ROS topic name.
 * @param[in]  writers    `true` to count publishers; `false` for subscriptions.
 * @param[out] count      Endpoints on the topic, written on RMW_RET_OK.
 * @param[in]  caller     rmw function name, for error messages.
 * @return RMW_RET_OK with `count` set; RMW_RET_INVALID_ARGUMENT for a null
 *   argument or a topic name that is not a valid fully-qualified name;
 *   RMW_RET_INCORRECT_RMW_IMPLEMENTATION for another rmw's node. A name too
 *   long to be announced counts zero endpoints.
 *
 * Verification:
 * - Correct means: the count equals what a stock rmw reports for the same
 *   topic: every publisher (or subscription) on it, local or remote,
 *   whatever its type; a withdrawn endpoint no longer counts.
 * - Covered by: TC-SEDP-8.5.4-014 (the table's count), and
 *   tests/ros2/run_rmw_graph_loopback.sh (against stock Fast DDS).
 *
 * References:
 * - rmw/rmw.h (Humble), `rmw_count_publishers`, `rmw_count_subscribers`
 * - rmw_fastrtps_shared_cpp/src/rmw_count.cpp (Humble), `__rmw_count_publishers`
 */
rmw_ret_t count_endpoints(const rmw_node_t* node, const char* topic_name, bool writers,
                          size_t* count, const char* caller) noexcept {
    if (!node || !node->data || !topic_name || !count) {
        RMW_SET_ERROR_MSG_WITH_FORMAT_STRING("rmw_rosette: %s: null argument", caller);
        return RMW_RET_INVALID_ARGUMENT;
    }
    if (!node->implementation_identifier ||
        std::strcmp(node->implementation_identifier, rmw_rosette_identifier) != 0) {
        RMW_SET_ERROR_MSG_WITH_FORMAT_STRING("rmw_rosette: %s: node from another rmw", caller);
        return RMW_RET_INCORRECT_RMW_IMPLEMENTATION;
    }
    int validation = RMW_TOPIC_VALID;
    if (rmw_validate_full_topic_name(topic_name, &validation, nullptr) != RMW_RET_OK) {
        return RMW_RET_ERROR;
    }
    if (validation != RMW_TOPIC_VALID) {
        RMW_SET_ERROR_MSG_WITH_FORMAT_STRING("rmw_rosette: %s: invalid topic name: %s", caller,
                                             rmw_full_topic_name_validation_result_string(
                                                 validation));
        return RMW_RET_INVALID_ARGUMENT;
    }

    char dds_topic[RTPS_MAX_TOPIC_NAME_LEN];
    if (!rmw_rosette::compose_topic_name(dds_topic, sizeof(dds_topic), topic_name)) {
        *count = 0u;
        return RMW_RET_OK;
    }
    auto& ctx = *static_cast<const rmw_rosette::Node*>(node->data)->context;
    rmw_rosette::ContextLock lock(ctx);
    *count = ctx.participant->match_table().count(dds_topic, writers);
    return RMW_RET_OK;
}

}  // namespace

extern "C" {

/// @brief Returns RMW_RET_UNSUPPORTED: the match table keeps remote endpoints
/// by topic key, without their names (endpoint_match_table.hpp).
rmw_ret_t rmw_get_topic_names_and_types(const rmw_node_t* node, rcutils_allocator_t* allocator,
                                         bool no_demangle,
                                         rmw_names_and_types_t* topic_names_and_types) {
    (void)node; (void)allocator; (void)no_demangle; (void)topic_names_and_types;
    RMW_SET_ERROR_MSG("rmw_rosette: rmw_get_topic_names_and_types not implemented");
    return RMW_RET_UNSUPPORTED;
}

/// @brief Returns RMW_RET_UNSUPPORTED (see the file comment).
rmw_ret_t rmw_get_node_names(const rmw_node_t* node, rcutils_string_array_t* node_names,
                              rcutils_string_array_t* node_namespaces) {
    (void)node; (void)node_names; (void)node_namespaces;
    RMW_SET_ERROR_MSG("rmw_rosette: rmw_get_node_names not implemented");
    return RMW_RET_UNSUPPORTED;
}

/// @brief Counts publishers on a ROS topic across the graph; see count_endpoints().
rmw_ret_t rmw_count_publishers(const rmw_node_t* node, const char* topic_name, size_t* count) {
    return count_endpoints(node, topic_name, /*writers=*/true, count, "rmw_count_publishers");
}

/// @brief Counts subscriptions on a ROS topic across the graph; see count_endpoints().
rmw_ret_t rmw_count_subscribers(const rmw_node_t* node, const char* topic_name, size_t* count) {
    return count_endpoints(node, topic_name, /*writers=*/false, count, "rmw_count_subscribers");
}

/// @brief Returns RMW_RET_UNSUPPORTED (see the file comment).
rmw_ret_t rmw_get_node_names_with_enclaves(const rmw_node_t* node,
                                           rcutils_string_array_t* node_names,
                                           rcutils_string_array_t* node_namespaces,
                                           rcutils_string_array_t* enclaves) {
    (void)node; (void)node_names; (void)node_namespaces; (void)enclaves;
    return RMW_RET_UNSUPPORTED;
}

/// @brief Returns RMW_RET_UNSUPPORTED (see the file comment).
rmw_ret_t rmw_get_service_names_and_types(const rmw_node_t* node, rcutils_allocator_t* allocator,
                                          rmw_names_and_types_t* service_names_and_types) {
    (void)node; (void)allocator; (void)service_names_and_types;
    return RMW_RET_UNSUPPORTED;
}

/// @brief Returns RMW_RET_UNSUPPORTED (see the file comment).
rmw_ret_t rmw_get_publisher_names_and_types_by_node(const rmw_node_t* node,
                                                    rcutils_allocator_t* allocator,
                                                    const char* node_name,
                                                    const char* node_namespace, bool no_demangle,
                                                    rmw_names_and_types_t* topic_names_and_types) {
    (void)node; (void)allocator; (void)node_name; (void)node_namespace; (void)no_demangle;
    (void)topic_names_and_types;
    return RMW_RET_UNSUPPORTED;
}

/// @brief Returns RMW_RET_UNSUPPORTED (see the file comment).
rmw_ret_t rmw_get_subscriber_names_and_types_by_node(const rmw_node_t* node,
                                                     rcutils_allocator_t* allocator,
                                                     const char* node_name,
                                                     const char* node_namespace, bool no_demangle,
                                                     rmw_names_and_types_t* topic_names_and_types) {
    (void)node; (void)allocator; (void)node_name; (void)node_namespace; (void)no_demangle;
    (void)topic_names_and_types;
    return RMW_RET_UNSUPPORTED;
}

/// @brief Returns RMW_RET_UNSUPPORTED (see the file comment).
rmw_ret_t rmw_get_service_names_and_types_by_node(const rmw_node_t* node,
                                                  rcutils_allocator_t* allocator,
                                                  const char* node_name,
                                                  const char* node_namespace,
                                                  rmw_names_and_types_t* service_names_and_types) {
    (void)node; (void)allocator; (void)node_name; (void)node_namespace;
    (void)service_names_and_types;
    return RMW_RET_UNSUPPORTED;
}

/// @brief Returns RMW_RET_UNSUPPORTED (see the file comment).
rmw_ret_t rmw_get_client_names_and_types_by_node(const rmw_node_t* node,
                                                 rcutils_allocator_t* allocator,
                                                 const char* node_name,
                                                 const char* node_namespace,
                                                 rmw_names_and_types_t* service_names_and_types) {
    (void)node; (void)allocator; (void)node_name; (void)node_namespace;
    (void)service_names_and_types;
    return RMW_RET_UNSUPPORTED;
}

/// @brief Returns RMW_RET_UNSUPPORTED (see the file comment).
rmw_ret_t rmw_get_publishers_info_by_topic(const rmw_node_t* node, rcutils_allocator_t* allocator,
                                           const char* topic_name, bool no_mangle,
                                           rmw_topic_endpoint_info_array_t* publishers_info) {
    (void)node; (void)allocator; (void)topic_name; (void)no_mangle; (void)publishers_info;
    return RMW_RET_UNSUPPORTED;
}

/// @brief Returns RMW_RET_UNSUPPORTED (see the file comment).
rmw_ret_t rmw_get_subscriptions_info_by_topic(const rmw_node_t* node,
                                              rcutils_allocator_t* allocator,
                                              const char* topic_name, bool no_mangle,
                                              rmw_topic_endpoint_info_array_t* subscriptions_info) {
    (void)node; (void)allocator; (void)topic_name; (void)no_mangle; (void)subscriptions_info;
    return RMW_RET_UNSUPPORTED;
}

}  // extern "C"
