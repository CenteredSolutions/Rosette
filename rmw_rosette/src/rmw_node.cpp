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
#include "rmw_rosette/pool.hpp"
#include "rmw_rosette/types.hpp"

#include <rmw/rmw.h>
#include <rmw/error_handling.h>

#include <cstring>

namespace {

/// strncpy(dst, src, cap - 1) doesn't null-terminate when src is >= cap - 1
/// bytes long — the one sharp edge in an otherwise-standard bounded copy,
/// so it still needs the explicit dst[cap-1] afterward.
void copy_bounded(char* dst, size_t cap, const char* src) noexcept {
    std::strncpy(dst, src, cap - 1);
    dst[cap - 1] = '\0';
}

/// The rmw_node_t the caller sees, plus the payload it points at, in one
/// pooled slot — see pool.hpp.
struct NodeSlot {
    rmw_node_t node{};
    rmw_rosette::Node impl{};
};

rmw_rosette::Pool<NodeSlot, RMW_ROSETTE_MAX_NODES>& node_pool() {
    static rmw_rosette::Pool<NodeSlot, RMW_ROSETTE_MAX_NODES> pool;
    return pool;
}

NodeSlot* slot_from(rmw_node_t* node) noexcept {
    // rmw_node_t is always the first member of NodeSlot, so this
    // reinterpret is well-defined (standard-layout, same address as the
    // struct itself) — the same trick rmw_fastrtps_cpp/rmw_cyclonedds_cpp
    // use for their own wrapper structs.
    return reinterpret_cast<NodeSlot*>(node);
}

}  // namespace

extern "C" {

rmw_node_t* rmw_create_node(rmw_context_t* context, const char* name,
                             const char* namespace_) {
    if (!context || !context->impl || !name || !namespace_) {
        RMW_SET_ERROR_MSG("rmw_rosette: rmw_create_node: null argument");
        return nullptr;
    }

    NodeSlot* slot = node_pool().acquire();
    if (!slot) {
        RMW_SET_ERROR_MSG("rmw_rosette: rmw_create_node: node pool exhausted "
                          "(RMW_ROSETTE_MAX_NODES)");
        return nullptr;
    }

    slot->impl.context = &context->impl->rosette_context;
    copy_bounded(slot->impl.name, sizeof(slot->impl.name), name);
    copy_bounded(slot->impl.namespace_, sizeof(slot->impl.namespace_), namespace_);

    slot->node.implementation_identifier = rmw_rosette_identifier;
    slot->node.data = &slot->impl;
    slot->node.name = slot->impl.name;
    slot->node.namespace_ = slot->impl.namespace_;
    slot->node.context = context;

    return &slot->node;
}

rmw_ret_t rmw_destroy_node(rmw_node_t* node) {
    if (!node) return RMW_RET_INVALID_ARGUMENT;
    node_pool().release(slot_from(node));
    return RMW_RET_OK;
}

const rmw_guard_condition_t* rmw_node_get_graph_guard_condition(const rmw_node_t* node) {
    (void)node;
    // TODO: one guard condition per node, triggered whenever GraphCache
    // changes (endpoint match/unmatch) — this is what lets
    // rclpy/rclcpp graph-change callbacks wake up. Depends on
    // graph_cache.hpp's still-open question about where discovery events
    // feed in (see that header).
    return nullptr;
}

}  // extern "C"
