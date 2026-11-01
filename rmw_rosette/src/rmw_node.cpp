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

#include "rmw_rosette/argument_checks.hpp"
#include "rmw_rosette/identifier.h"
#include "rmw_rosette/pool.hpp"
#include "rmw_rosette/types.hpp"

#include <rmw/error_handling.h>
#include <rmw/rmw.h>
#include <rmw/validate_namespace.h>
#include <rmw/validate_node_name.h>

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

/**
 * @brief Checks a node's name and namespace against the ROS naming rules.
 *
 * rcl_node_init() validates both before it calls rmw_create_node(), but
 * the rmw entry point is public and must refuse an invalid name itself, as
 * the stock rmws do, before it takes a node slot.
 *
 * @param[in] name       Node name; may be null.
 * @param[in] namespace_ Node namespace; may be null.
 * @return RMW_RET_OK when both are valid; RMW_RET_INVALID_ARGUMENT, with the error state set,
 *   when either is null or breaks the rules; RMW_RET_ERROR when the validation itself fails.
 *
 * Verification:
 * - Correct means: "foo bar" is refused as a name and as a namespace, and
 *   "my_node" in "/my_ns" is accepted.
 * - Covered by: test_rmw_implementation's
 * TestNodeConstructionDestruction.create_with_bad_arguments.
 *
 * References:
 * - rmw/validate_node_name.h, rmw/validate_namespace.h (Humble, Jazzy)
 * - ROS 2 design, "Topic and Service name mapping to DDS", node name rules
 */
rmw_ret_t check_node_names(const char* name, const char* namespace_) noexcept {
    using rmw_rosette::ArgumentFault;
    if (name == nullptr || namespace_ == nullptr) {
        return rmw_rosette::refuse_argument(
            "rmw_create_node", name == nullptr ? "name" : "namespace_", ArgumentFault::null);
    }

    int    validation = RMW_NODE_NAME_VALID;
    size_t invalid_index = 0;
    if (rmw_validate_node_name(name, &validation, &invalid_index) != RMW_RET_OK) {
        return RMW_RET_ERROR;
    }

    if (validation != RMW_NODE_NAME_VALID) {
        return rmw_rosette::refuse_argument("rmw_create_node", "name", ArgumentFault::invalid);
    }

    if (rmw_validate_namespace(namespace_, &validation, &invalid_index) != RMW_RET_OK) {
        return RMW_RET_ERROR;
    }

    if (validation != RMW_NAMESPACE_VALID) {
        return rmw_rosette::refuse_argument("rmw_create_node", "namespace_",
                                            ArgumentFault::invalid);
    }

    return RMW_RET_OK;
}

}  // namespace

extern "C" {

/**
 * @brief Creates a node scoped to a context's RTPS participant.
 *
 * RTPS has no node concept of its own; this exists to satisfy rcl's node
 * lifecycle and give graph-cache queries something to scope by. Every
 * node created against one `rmw_context_t` shares that context's single
 * `RTPSParticipant` — a participant is process-wide; nodes are a ROS
 * concept layered on top of it.
 *
 * Draws a slot from node_pool(), copies `name`/`namespace_` into the
 * slot's fixed buffers via copy_bounded(), and fills the `rmw_node_t`
 * fields to point at those buffers. Finally the node is published on
 * `ros_discovery_info`, which is how stock peers learn that it exists (a
 * node that cannot be published is not created), and the context's graph
 * guard condition is triggered for local observers.
 *
 * @param[in] context Context whose participant this node is scoped to;
 *   initialized by rmw_init() and not shut down.
 * @param[in] name Node name; a valid ROS node name.
 * @param[in] namespace_ Node namespace; a valid ROS namespace.
 * @return The new node, or `nullptr` with the error state set if any
 *   argument is null, the context is not initialized, is another rmw's or
 *   was shut down, a name is invalid, the node pool is exhausted, or the
 *   updated `ros_discovery_info` message cannot be written. A refused call
 *   takes no node slot.
 *
 * Verification:
 * - Correct means: a stock `ros2 node list` shows `<namespace>/<name>` for
 *   as long as the node exists, and a refused call leaves the node pool as
 *   it was.
 * - Covered by: tests/ros2/run_rmw_interop_loopback.sh (node list check);
 *   test_rmw_implementation's TestNodeConstructionDestruction.
 *
 * References:
 * - rmw/rmw.h (Humble), `rmw_create_node`
 * - rmw_dds_common/msg/ParticipantEntitiesInfo.msg (Humble)
 */
rmw_node_t* rmw_create_node(rmw_context_t* context, const char* name,
                             const char* namespace_) {
    if (rmw_rosette::check_context(context, __func__, true) != RMW_RET_OK
        || check_node_names(name, namespace_) != RMW_RET_OK) {
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

    rmw_rosette::ContextUpdateLock lock(*slot->impl.context);
    if (!slot->impl.context->discovery_info.add_node(slot->impl)) {
        RMW_SET_ERROR_MSG("rmw_rosette: rmw_create_node: cannot publish ros_discovery_info "
                          "(buffer pool exhausted)");
        node_pool().release(slot);
        return nullptr;
    }

    slot->impl.context->graph_guard.trigger();
    return &slot->node;
}

/**
 * @brief Withdraws a node from `ros_discovery_info` and returns its slot to the pool.
 *
 * Triggers the context's graph guard condition, since the node list changed.
 *
 * @param[in] node Node to destroy; allowed after the context was shut down.
 * @return `RMW_RET_OK`; `RMW_RET_INVALID_ARGUMENT` if `node` is null or not
 *   initialized; `RMW_RET_INCORRECT_RMW_IMPLEMENTATION` if it is another
 *   rmw's, in which case the node is untouched and still owned by the caller.
 *
 * Verification:
 * - Correct means: the node leaves `ros2 node list`, and its slot returns
 *   to the pool exactly once.
 * - Covered by: test_rmw_implementation's
 * TestNodeConstructionDestruction.destroy_with_bad_arguments.
 *
 * References:
 * - rmw/rmw.h (Humble, Jazzy), `rmw_destroy_node`
 */
rmw_ret_t rmw_destroy_node(rmw_node_t* node) {
    const rmw_ret_t ret = rmw_rosette::check_handle(node, __func__, "node");
    if (ret != RMW_RET_OK) {
        return ret;
    }

    auto* impl = static_cast<rmw_rosette::Node*>(node->data);
    {
        rmw_rosette::ContextUpdateLock lock(*impl->context);
        impl->context->discovery_info.remove_node(*impl);
        impl->context->graph_guard.trigger();
    }

    node_pool().release(slot_from(node));
    return RMW_RET_OK;
}

/**
 * @brief Returns the guard condition rcl waits on for graph changes.
 *
 * rcl_node_init() fails when this returns null, and rcl's graph helpers
 * (`rcl_wait_for_publishers`, `rclpy` `wait_for_service`) add it to their
 * wait sets. Every node of a context shares the context's one graph guard
 * condition, as stock DDS-based rmw implementations do: the participant
 * triggers it whenever discovery changes the graph (rmw_init()), and node
 * creation and destruction trigger it for local nodes.
 *
 * @param[in] node Node whose graph guard condition is requested.
 * @return The context's graph guard condition, valid until
 *   rmw_context_fini(); `nullptr`, with the error state set, if `node` is
 *   null, not initialized or another rmw's.
 *
 * Verification:
 * - Correct means: rcl_node_init() succeeds, the returned handle is stable
 *   for the node's lifetime, and a wait on it returns when a publisher
 *   appears on the graph.
 * - Covered by: rmw_rosette/test/test_guard_condition.cpp.
 *
 * References:
 * - rmw/rmw.h (Humble), `rmw_node_get_graph_guard_condition`
 * - rcl/src/rcl/node.c (Humble), `rcl_node_init`
 */
const rmw_guard_condition_t* rmw_node_get_graph_guard_condition(const rmw_node_t* node) {
    if (rmw_rosette::check_handle(node, __func__, "node") != RMW_RET_OK) {
        return nullptr;
    }

    return &static_cast<rmw_rosette::Node*>(node->data)->context->graph_guard_handle;
}

}  // extern "C"
