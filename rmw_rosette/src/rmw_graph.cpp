/**
 * @file rmw_graph.cpp
 * @brief rmw_get_{node,topic,service}_names_and_types, counts, gids.
 *
 * Backed by graph_cache.hpp's GraphCache, once something actually feeds
 * it from discovery events (see that header's open question about where
 * the feed hooks in).
 */

#include "rmw_rosette/identifier.h"
#include "rmw_rosette/graph_cache.hpp"

#include <rmw/rmw.h>
#include <rmw/error_handling.h>

extern "C" {

rmw_ret_t rmw_get_topic_names_and_types(const rmw_node_t* node, rcutils_allocator_t* allocator,
                                         bool no_demangle,
                                         rmw_names_and_types_t* topic_names_and_types) {
    (void)node; (void)allocator; (void)no_demangle; (void)topic_names_and_types;
    // TODO: iterate GraphCache, group by topic name, demangle "rt/..." back
    // to the ROS name (unless no_demangle) via the inverse of
    // topic_naming.hpp's compose_topic_name().
    RMW_SET_ERROR_MSG("rmw_rosette: rmw_get_topic_names_and_types not implemented");
    return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_get_node_names(const rmw_node_t* node, rcutils_string_array_t* node_names,
                              rcutils_string_array_t* node_namespaces) {
    (void)node; (void)node_names; (void)node_namespaces;
    // TODO: this needs each remote participant's node list, which SEDP
    // alone doesn't carry — real rmw_fastrtps_cpp gets it from a
    // ParticipantEntitiesInfo message published on a builtin ROS-specific
    // topic. rosette::rtps has no equivalent yet; this may be the single
    // largest remaining protocol-level gap, not just glue.
    RMW_SET_ERROR_MSG("rmw_rosette: rmw_get_node_names not implemented");
    return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_count_publishers(const rmw_node_t* node, const char* topic_name, size_t* count) {
    (void)node; (void)topic_name;
    if (!count) return RMW_RET_INVALID_ARGUMENT;
    *count = 0;
    return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_count_subscribers(const rmw_node_t* node, const char* topic_name, size_t* count) {
    (void)node; (void)topic_name;
    if (!count) return RMW_RET_INVALID_ARGUMENT;
    *count = 0;
    return RMW_RET_UNSUPPORTED;
}

// Deferred: rmw_get_service_names_and_types, rmw_get_publisher_names_and_types_by_node,
// rmw_get_subscriber_names_and_types_by_node, rmw_get_service_names_and_types_by_node,
// rmw_get_client_names_and_types_by_node, rmw_get_publishers_info_by_topic,
// rmw_get_subscriptions_info_by_topic. All the same shape as
// rmw_get_topic_names_and_types once GraphCache is real.

}  // extern "C"
