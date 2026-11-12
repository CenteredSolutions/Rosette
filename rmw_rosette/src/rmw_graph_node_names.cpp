// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file rmw_graph_node_names.cpp
 * @brief rmw_get_node_names(), answered from local nodes and the GraphCache.
 *
 * Compiled into hosted builds only (RMW_ROSETTE_GRAPH_CACHE). A build without
 * the cache gets the RMW_RET_UNSUPPORTED version in rmw_graph.cpp.
 */

#include <rcutils/allocator.h>
#include <rcutils/strdup.h>
#include <rcutils/types/string_array.h>
#include <rmw/error_handling.h>
#include <rmw/rmw.h>
#include <rmw/sanity_checks.h>

#include <new>
#include <vector>

#include "rmw_rosette/argument_checks.hpp"
#include "rmw_rosette/graph_cache.hpp"
#include "rmw_rosette/types.hpp"

namespace {

/**
 * @brief Copies node names and namespaces into the caller's zero-initialized string arrays.
 *
 * On RMW_RET_BAD_ALLOC both arrays are left zero-initialized and nothing leaks.
 */
rmw_ret_t copy_node_names(const std::vector<rmw_rosette::NodeName>& nodes,
                          rcutils_string_array_t*                   names,
                          rcutils_string_array_t*                   namespaces) noexcept {
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    bool ok = rcutils_string_array_init(names, nodes.size(), &allocator) == RCUTILS_RET_OK;
    if (ok) {
        ok = rcutils_string_array_init(namespaces, nodes.size(), &allocator) == RCUTILS_RET_OK;
        if (!ok) {
            (void)rcutils_string_array_fini(names);
        }
    }

    if (!ok) {
        rcutils_reset_error();
        RMW_SET_ERROR_MSG("rmw_rosette: out of memory listing node names");
        return RMW_RET_BAD_ALLOC;
    }

    for (size_t i = 0; i < nodes.size(); ++i) {
        names->data[i] = rcutils_strdup(nodes[i].name.c_str(), allocator);
        namespaces->data[i] = rcutils_strdup(nodes[i].namespace_.c_str(), allocator);
        if (names->data[i] == nullptr || namespaces->data[i] == nullptr) {
            (void)rcutils_string_array_fini(names);
            (void)rcutils_string_array_fini(namespaces);
            rcutils_reset_error();
            RMW_SET_ERROR_MSG("rmw_rosette: out of memory listing node names");
            return RMW_RET_BAD_ALLOC;
        }
    }

    return RMW_RET_OK;
}

}  // namespace

extern "C" {

/**
 * @brief Lists every node in the graph with its namespace.
 *
 * A participant never reads its own `ros_discovery_info` writer, so the
 * context's own nodes come from its node table and every other participant's
 * from the GraphCache. Nodes sharing a name are all listed, as stock rmws do,
 * which is how `ros2 node list` warns about duplicates.
 *
 * @param[in]  node            Node whose context is queried.
 * @param[out] node_names      Zero-initialized array receiving the names.
 * @param[out] node_namespaces Zero-initialized array receiving the namespaces.
 * @return RMW_RET_OK; RMW_RET_INVALID_ARGUMENT for a null argument or an array
 *   already in use; RMW_RET_INCORRECT_RMW_IMPLEMENTATION for another rmw's
 *   node; RMW_RET_BAD_ALLOC when the heap is exhausted.
 *
 * Verification:
 * - Correct means: the result equals what rmw_fastrtps_cpp reports for the same graph.
 * - Covered by: test_graph_api in test_rmw_implementation, and
 *   rmw_rosette/test/test_graph_cache.cpp (RmwNodeNames).
 *
 * References:
 * - rmw/rmw.h (Humble), `rmw_get_node_names`
 * - rmw_dds_common/graph_cache.hpp (Humble), `get_node_names`
 */
rmw_ret_t rmw_get_node_names(const rmw_node_t* node, rcutils_string_array_t* node_names,
                             rcutils_string_array_t* node_namespaces) {
    const rmw_ret_t checked = rmw_rosette::check_handle(
        node, __func__, "node", {{node_names, "node_names"}, {node_namespaces, "node_namespaces"}});
    if (checked != RMW_RET_OK) {
        return checked;
    }

    if (rmw_check_zero_rmw_string_array(node_names) != RMW_RET_OK
        || rmw_check_zero_rmw_string_array(node_namespaces) != RMW_RET_OK) {
        return RMW_RET_INVALID_ARGUMENT;
    }

    auto& ctx = *static_cast<const rmw_rosette::Node*>(node->data)->context;
    try {
        std::vector<rmw_rosette::NodeName> nodes;
        {
            rmw_rosette::ContextLock lock(ctx);
            nodes = ctx.graph_cache->node_names();

            const rmw_rosette::LocalNodeEntities& local = ctx.discovery_info.local_entities();
            for (uint32_t i = 0; i < local.node_count(); ++i) {
                nodes.push_back({local.node(i).namespace_, local.node(i).name});
            }
        }

        return copy_node_names(nodes, node_names, node_namespaces);
    } catch (const std::bad_alloc&) {
        RMW_SET_ERROR_MSG("rmw_rosette: out of memory listing node names");
        return RMW_RET_BAD_ALLOC;
    }
}

}  // extern "C"
