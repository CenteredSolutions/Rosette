// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file rmw_graph_node_names.cpp
 * @brief rmw_get_node_names() and rmw_get_node_names_with_enclaves(), answered
 * from local nodes and the GraphCache.
 *
 * Compiled into hosted builds only (RMW_ROSETTE_GRAPH_CACHE). A build without
 * the cache gets the RMW_RET_UNSUPPORTED versions in rmw_graph.cpp.
 *
 * Stock rmws read a remote participant's enclave from its SPDP USER_DATA
 * (`enclave=/;`). Rosette's SPDP keeps no USER_DATA, so a remote node is
 * listed with an empty enclave, and only the context's own nodes carry theirs.
 */

#include <rcutils/allocator.h>
#include <rcutils/strdup.h>
#include <rcutils/types/string_array.h>
#include <rmw/error_handling.h>
#include <rmw/rmw.h>
#include <rmw/sanity_checks.h>

#include <new>
#include <string>
#include <vector>

#include "rmw_rosette/argument_checks.hpp"
#include "rmw_rosette/graph_cache.hpp"
#include "rmw_rosette/types.hpp"

namespace {

/// @brief Every node in the graph, as the parallel columns the rmw listings return.
struct NodeListing {
    std::vector<std::string> names;
    std::vector<std::string> namespaces;
    std::vector<std::string> enclaves;
};

/**
 * @brief Lists the context's own nodes, then every other participant's.
 *
 * The context's own nodes come from its node table: its `ros_discovery_info`
 * reader never pairs with its own writer (ReaderConfig::ignoresLocalWriters).
 *
 * @throws std::bad_alloc when the heap is exhausted.
 */
NodeListing list_nodes(rmw_rosette::Context& ctx) {
    NodeListing out;
    rmw_rosette::ContextLock lock(ctx);

    const rmw_rosette::LocalNodeEntities& local = ctx.discovery_info.local_entities();
    for (uint32_t i = 0; i < local.node_count(); ++i) {
        out.names.emplace_back(local.node(i).name);
        out.namespaces.emplace_back(local.node(i).namespace_);
        out.enclaves.emplace_back(ctx.enclave);
    }

    for (const rmw_rosette::NodeName& node : ctx.graph_cache->node_names()) {
        out.names.push_back(node.name);
        out.namespaces.push_back(node.namespace_);
        out.enclaves.emplace_back();
    }

    return out;
}

/**
 * @brief Copies strings into the caller's zero-initialized string array.
 *
 * @return `true` on success; `false` on allocation failure, with `out` left
 *   zero-initialized and nothing leaked.
 */
bool copy_strings(const std::vector<std::string>& in, rcutils_string_array_t* out) noexcept {
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    if (rcutils_string_array_init(out, in.size(), &allocator) != RCUTILS_RET_OK) {
        return false;
    }

    for (size_t i = 0; i < in.size(); ++i) {
        out->data[i] = rcutils_strdup(in[i].c_str(), allocator);
        if (out->data[i] == nullptr) {
            (void)rcutils_string_array_fini(out);
            return false;
        }
    }

    return true;
}

/**
 * @brief Lists the graph's nodes into the caller's arrays.
 *
 * @param[in]  node       Validated node handle; selects the context.
 * @param[out] names      Validated, zero-initialized array.
 * @param[out] namespaces Validated, zero-initialized array.
 * @param[out] enclaves   Validated, zero-initialized array, or null to list none.
 * @return RMW_RET_OK, or RMW_RET_BAD_ALLOC with every array left zero-initialized.
 */
rmw_ret_t answer_node_names(const rmw_node_t* node, rcutils_string_array_t* names,
                            rcutils_string_array_t* namespaces,
                            rcutils_string_array_t* enclaves) noexcept {
    auto& ctx = *static_cast<const rmw_rosette::Node*>(node->data)->context;
    try {
        const NodeListing nodes = list_nodes(ctx);
        bool ok = copy_strings(nodes.names, names);
        ok = ok && copy_strings(nodes.namespaces, namespaces);
        ok = ok && (enclaves == nullptr || copy_strings(nodes.enclaves, enclaves));
        if (ok) {
            return RMW_RET_OK;
        }

        (void)rcutils_string_array_fini(names);
        (void)rcutils_string_array_fini(namespaces);
        rcutils_reset_error();
    } catch (const std::bad_alloc&) {
    }

    RMW_SET_ERROR_MSG("rmw_rosette: out of memory listing node names");
    return RMW_RET_BAD_ALLOC;
}

}  // namespace

extern "C" {

/**
 * @brief Lists every node in the graph with its namespace.
 *
 * The context's own nodes come from its node table, and every other
 * participant's from the GraphCache: the context's `ros_discovery_info` reader
 * never pairs with its own writer (ReaderConfig::ignoresLocalWriters).
 * Nodes sharing a name are all listed, as stock rmws do, which is how
 * `ros2 node list` warns about duplicates.
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

    return answer_node_names(node, node_names, node_namespaces, nullptr);
}

/**
 * @brief Lists every node in the graph with its namespace and enclave.
 *
 * The variant `ros2 node list` calls. Listed as rmw_get_node_names() lists
 * them; the context's own nodes carry the enclave its init options named, and
 * every other node an empty one (see the file comment).
 *
 * @param[in]  node            Node whose context is queried.
 * @param[out] node_names      Zero-initialized array receiving the names.
 * @param[out] node_namespaces Zero-initialized array receiving the namespaces.
 * @param[out] enclaves        Zero-initialized array receiving the enclaves.
 * @return RMW_RET_OK; RMW_RET_INVALID_ARGUMENT for a null argument or an array
 *   already in use; RMW_RET_INCORRECT_RMW_IMPLEMENTATION for another rmw's
 *   node; RMW_RET_BAD_ALLOC when the heap is exhausted. On failure every array
 *   is left as it was passed.
 *
 * Verification:
 * - Correct means: the names and namespaces equal rmw_get_node_names()'s, in
 *   the same order, and a local node's enclave is its context's.
 * - Covered by: test_graph_api in test_rmw_implementation
 *   (get_node_names_with_enclaves_with_bad_arguments), and
 *   rmw_rosette/test/test_graph_cache.cpp (RmwGraphQueries.ListsTheLocalEnclave).
 *
 * References:
 * - rmw/rmw.h (Humble, Jazzy), `rmw_get_node_names_with_enclaves`
 * - rmw_dds_common/graph_cache.hpp (Humble), `get_node_names` with enclaves
 */
rmw_ret_t rmw_get_node_names_with_enclaves(const rmw_node_t* node,
                                           rcutils_string_array_t* node_names,
                                           rcutils_string_array_t* node_namespaces,
                                           rcutils_string_array_t* enclaves) {
    const rmw_ret_t checked = rmw_rosette::check_handle(
        node, __func__, "node",
        {{node_names, "node_names"}, {node_namespaces, "node_namespaces"}, {enclaves, "enclaves"}});
    if (checked != RMW_RET_OK) {
        return checked;
    }

    if (rmw_check_zero_rmw_string_array(node_names) != RMW_RET_OK
        || rmw_check_zero_rmw_string_array(node_namespaces) != RMW_RET_OK
        || rmw_check_zero_rmw_string_array(enclaves) != RMW_RET_OK) {
        return RMW_RET_INVALID_ARGUMENT;
    }

    return answer_node_names(node, node_names, node_namespaces, enclaves);
}

}  // extern "C"
