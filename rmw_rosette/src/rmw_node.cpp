/**
 * @file rmw_node.cpp
 * @brief rmw_create_node / rmw_destroy_node.
 *
 * RTPS has no first-class "node" entity; this exists purely to satisfy
 * rcl's node lifecycle, provide the namespace context topic_naming.hpp
 * needs, and give graph-cache queries something to scope by. All nodes
 * created against one rmw_context_t share the same underlying
 * RTPSParticipant (a participant is a process-wide thing; nodes are a ROS
 * concept layered on top).
 */

#include "rmw_rosette/identifier.h"
#include "rmw_rosette/types.hpp"

#include <rmw/rmw.h>
#include <rmw/error_handling.h>

extern "C" {

rmw_node_t* rmw_create_node(rmw_context_t* context, const char* name,
                             const char* namespace_) {
    (void)context; (void)name; (void)namespace_;
    // TODO: allocate an rmw_rosette::Node (see types.hpp), bind it to
    // context->impl->rosette_context, allocate + fill an rmw_node_t whose
    // ->data points at it and ->implementation_identifier is
    // rmw_rosette_identifier. RMW_ROSETTE_MAX_NODES bounds how many can
    // exist at once (types.hpp) — most firmware images need exactly one.
    RMW_SET_ERROR_MSG("rmw_rosette: rmw_create_node not implemented");
    return nullptr;
}

rmw_ret_t rmw_destroy_node(rmw_node_t* node) {
    (void)node;
    RMW_SET_ERROR_MSG("rmw_rosette: rmw_destroy_node not implemented");
    return RMW_RET_UNSUPPORTED;
}

const rmw_guard_condition_t* rmw_node_get_graph_guard_condition(const rmw_node_t* node) {
    (void)node;
    // TODO: one guard condition per node, triggered whenever GraphCache
    // changes (endpoint match/unmatch) — this is what lets
    // rclpy/rclcpp graph-change callbacks wake up.
    return nullptr;
}

}  // extern "C"
