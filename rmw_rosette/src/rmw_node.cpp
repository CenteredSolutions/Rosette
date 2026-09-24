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

/**
 * @brief Copies a NUL-terminated string into a fixed buffer, truncating
 * rather than overflowing.
 *
 * `rmw_create_node()` uses this to copy rcl's `name`/`namespace_`
 * arguments into `Node`'s fixed buffers, since the caller makes no
 * promise those pointers stay valid past the call.
 *
 * `strncpy(dst, src, cap - 1)` does not NUL-terminate `dst` when `src` is
 * `cap - 1` bytes or longer, so the explicit `dst[cap - 1] = '\0'`
 * afterward is required, not defensive.
 *
 * @param[out] dst Destination buffer.
 * @param[in] cap Capacity of `dst`, including the terminating NUL.
 * @param[in] src Source string.
 */
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

/// `rmw_node_t` is always the first member of `NodeSlot`, so this
/// reinterpret is well-defined (standard-layout, same address as the
/// struct itself) — the same technique rmw_fastrtps_cpp/rmw_cyclonedds_cpp
/// use for their own wrapper structs.
NodeSlot* slot_from(rmw_node_t* node) noexcept {
    return reinterpret_cast<NodeSlot*>(node);
}

}  // namespace

extern "C" {

/**
 * @brief Creates a node scoped to a context's RTPS participant.
 *
 * RTPS has no node concept of its own; this exists to satisfy rcl's node
 * lifecycle, own the graph guard condition rcl_node_init() requires, and
 * give graph-cache queries something to scope by. Every
 * node created against one `rmw_context_t` shares that context's single
 * `RTPSParticipant` — a participant is process-wide; nodes are a ROS
 * concept layered on top of it.
 *
 * Draws a slot from node_pool(), copies `name`/`namespace_` into the
 * slot's fixed buffers via copy_bounded(), and fills the `rmw_node_t`
 * fields to point at those buffers. The node's graph guard condition lives
 * in the same slot, so it shares the node's lifetime.
 *
 * @param[in] context Context whose participant this node is scoped to.
 * @param[in] name Node name.
 * @param[in] namespace_ Node namespace.
 * @return The new node, or `nullptr` if any argument is null or the node
 *   pool is exhausted.
 */
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

    slot->impl.graph_guard_handle.implementation_identifier = rmw_rosette_identifier;
    slot->impl.graph_guard_handle.data = &slot->impl.graph_guard;
    slot->impl.graph_guard_handle.context = context;

    return &slot->node;
}

/**
 * @brief Destroys a node and returns its slot to the pool.
 *
 * @param[in] node Node to destroy.
 * @return `RMW_RET_OK`, or `RMW_RET_INVALID_ARGUMENT` if `node` is null.
 */
rmw_ret_t rmw_destroy_node(rmw_node_t* node) {
    if (!node) return RMW_RET_INVALID_ARGUMENT;
    node_pool().release(slot_from(node));
    return RMW_RET_OK;
}

/**
 * @brief Returns the guard condition rcl waits on for graph changes.
 *
 * rcl_node_init() fails when this returns null, and rcl's graph helpers
 * (`rcl_wait_for_publishers`, `rclpy` `wait_for_service`) add it to their
 * wait sets. The condition is embedded in the node's pool slot; it is
 * triggered when discovery changes the graph this node observes.
 *
 * @param[in] node Node whose graph guard condition is requested.
 * @return The node's guard condition, valid until rmw_destroy_node();
 *   `nullptr` only if `node` or its data is null.
 *
 * Verification:
 * - Correct means: rcl_node_init() succeeds, and the returned handle is
 *   stable for the node's lifetime.
 * - Covered by: rmw_rosette/test/test_guard_condition.cpp.
 *
 * References:
 * - rmw/rmw.h (Humble), `rmw_node_get_graph_guard_condition`
 * - rcl/src/rcl/node.c (Humble), `rcl_node_init`
 */
const rmw_guard_condition_t* rmw_node_get_graph_guard_condition(const rmw_node_t* node) {
    if (!node || !node->data) return nullptr;
    return &static_cast<rmw_rosette::Node*>(node->data)->graph_guard_handle;
}

}  // extern "C"
