// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file rmw_graph_names.cpp
 * @brief rmw topic and service name queries, graph-wide and per node,
 * answered from the GraphCache.
 *
 * A per-node query finds the node's endpoints first: the context's own nodes
 * in its LocalNodeEntities table, every other node in the GraphCache, which
 * keeps the endpoint GUIDs each peer's `ros_discovery_info` lists. It then
 * lists those endpoints' names from the GraphCache, as the graph-wide queries
 * do.
 *
 * Compiled into hosted builds only (RMW_ROSETTE_GRAPH_CACHE). A build without
 * the cache gets the RMW_RET_UNSUPPORTED versions in rmw_graph.cpp.
 */

#include <rcutils/strdup.h>
#include <rmw/error_handling.h>
#include <rmw/get_service_names_and_types.h>
#include <rmw/get_topic_names_and_types.h>
#include <rmw/names_and_types.h>
#include <rmw/rmw.h>
#include <rmw/validate_namespace.h>
#include <rmw/validate_node_name.h>

#include <cstring>
#include <new>
#include <vector>

#include "rmw_rosette/argument_checks.hpp"
#include "rmw_rosette/graph_cache.hpp"
#include "rmw_rosette/types.hpp"

namespace {

/**
 * @brief Copies names and types into the caller's rmw_names_and_types_t.
 *
 * @param[in]  names     Names with their types.
 * @param[in]  allocator Allocator the result is made with; valid.
 * @param[out] out       Zero-initialized result, left zero-initialized on failure.
 * @return RMW_RET_OK, or RMW_RET_BAD_ALLOC when allocation failed.
 *
 * Verification:
 * - Correct means: rmw_names_and_types_fini() releases the result, and a
 *   failure part way through leaks nothing.
 * - Covered by: test_graph_api's *_names_and_types cases in test_rmw_implementation.
 *
 * References:
 * - rmw/names_and_types.h (Humble)
 */
rmw_ret_t copy_names_and_types(const rmw_rosette::NamesAndTypes& names,
                               rcutils_allocator_t*              allocator,
                               rmw_names_and_types_t*            out) noexcept {
    if (names.empty()) {
        return RMW_RET_OK;
    }

    if (rmw_names_and_types_init(out, names.size(), allocator) != RMW_RET_OK) {
        return RMW_RET_BAD_ALLOC;
    }

    size_t index = 0;
    for (const auto& entry : names) {
        out->names.data[index] = rcutils_strdup(entry.first.c_str(), *allocator);
        bool ok = out->names.data[index] != nullptr
                  && rcutils_string_array_init(&out->types[index], entry.second.size(), allocator)
                         == RCUTILS_RET_OK;

        size_t type_index = 0;
        for (const std::string& type : entry.second) {
            if (!ok) {
                break;
            }

            out->types[index].data[type_index] = rcutils_strdup(type.c_str(), *allocator);
            ok = out->types[index].data[type_index] != nullptr;
            ++type_index;
        }

        if (!ok) {
            rcutils_reset_error();
            (void)rmw_names_and_types_fini(out);
            RMW_SET_ERROR_MSG("rmw_rosette: out of memory listing graph names");
            return RMW_RET_BAD_ALLOC;
        }

        ++index;
    }

    return RMW_RET_OK;
}

/**
 * @brief Checks the arguments every names-and-types query shares.
 *
 * @param[in] node      Node handle.
 * @param[in] allocator Allocator for the result.
 * @param[in] out       Result, which must be zero-initialized.
 * @param[in] caller    rmw function name, for error messages.
 * @return RMW_RET_OK; RMW_RET_INVALID_ARGUMENT for a null argument, an invalid
 *   allocator, or a result already in use; RMW_RET_INCORRECT_RMW_IMPLEMENTATION
 *   for another rmw's node.
 */
rmw_ret_t check_query(const rmw_node_t* node, rcutils_allocator_t* allocator,
                      rmw_names_and_types_t* out, const char* caller) noexcept {
    const rmw_ret_t checked = rmw_rosette::check_handle(
        node, caller, "node", {{allocator, "allocator"}, {out, "names_and_types"}});
    if (checked != RMW_RET_OK) {
        return checked;
    }

    if (!rcutils_allocator_is_valid(allocator)) {
        RMW_SET_ERROR_MSG("rmw_rosette: allocator is invalid");
        return RMW_RET_INVALID_ARGUMENT;
    }

    return rmw_names_and_types_check_zero(out);
}

/**
 * @brief Runs one query against the context's cache and copies out the result.
 *
 * The cache is read under the context lock, and the result is copied after
 * the lock is released so a large listing does not hold up discovery.
 *
 * @tparam Query          Callable taking a `const GraphCache&`, returning NamesAndTypes.
 * @param[in]  node       Validated node handle.
 * @param[in]  allocator  Validated allocator.
 * @param[out] out        Zero-initialized result.
 * @param[in]  query      Selects what to list.
 * @return RMW_RET_OK, or RMW_RET_BAD_ALLOC when the heap is exhausted.
 */
template <typename Query>
rmw_ret_t answer_query(const rmw_node_t* node, rcutils_allocator_t* allocator,
                       rmw_names_and_types_t* out, Query query) noexcept {
    auto& ctx = *static_cast<const rmw_rosette::Node*>(node->data)->context;
    try {
        rmw_rosette::NamesAndTypes names;
        {
            rmw_rosette::ContextLock lock(ctx);
            names = query(*ctx.graph_cache);
        }

        return copy_names_and_types(names, allocator, out);
    } catch (const std::bad_alloc&) {
        RMW_SET_ERROR_MSG("rmw_rosette: out of memory listing graph names");
        return RMW_RET_BAD_ALLOC;
    }
}

/**
 * @brief Checks the node name and namespace a per-node query names.
 *
 * @return RMW_RET_OK; RMW_RET_INVALID_ARGUMENT for a null, or not valid, name
 *   or namespace; RMW_RET_ERROR when validation itself failed.
 */
rmw_ret_t check_node_name(const char* node_name, const char* node_namespace,
                          const char* caller) noexcept {
    if (node_name == nullptr || node_namespace == nullptr) {
        return rmw_rosette::refuse_argument(caller,
                                            node_name == nullptr ? "node_name" : "node_namespace",
                                            rmw_rosette::ArgumentFault::null);
    }

    int name_result = RMW_NODE_NAME_VALID;
    int namespace_result = RMW_NAMESPACE_VALID;
    if (rmw_validate_node_name(node_name, &name_result, nullptr) != RMW_RET_OK
        || rmw_validate_namespace(node_namespace, &namespace_result, nullptr) != RMW_RET_OK) {
        return RMW_RET_ERROR;
    }

    if (name_result != RMW_NODE_NAME_VALID) {
        RMW_SET_ERROR_MSG_WITH_FORMAT_STRING("rmw_rosette: %s: invalid node_name: %s", caller,
                                             rmw_node_name_validation_result_string(name_result));
        return RMW_RET_INVALID_ARGUMENT;
    }

    if (namespace_result != RMW_NAMESPACE_VALID) {
        RMW_SET_ERROR_MSG_WITH_FORMAT_STRING(
            "rmw_rosette: %s: invalid node_namespace: %s", caller,
            rmw_namespace_validation_result_string(namespace_result));
        return RMW_RET_INVALID_ARGUMENT;
    }

    return RMW_RET_OK;
}

/**
 * @brief Collects the endpoint GUIDs of one node, local or remote.
 *
 * The context's own nodes are searched first; then every other participant's.
 * Requires the context lock.
 *
 * @param[out] endpoints Receives the node's endpoint GUIDs.
 * @return `true` when the node exists.
 * @throws std::bad_alloc when the heap is exhausted.
 */
bool find_node_endpoints(const rmw_rosette::Context& ctx, const char* node_name,
                         const char* node_namespace, std::vector<rosette::rtps::Guid>& endpoints) {
    const rmw_rosette::LocalNodeEntities& local = ctx.discovery_info.local_entities();
    for (uint32_t i = 0; i < local.node_count(); ++i) {
        const rmw_rosette::Node& candidate = local.node(i);
        if (std::strcmp(candidate.name, node_name) != 0
            || std::strcmp(candidate.namespace_, node_namespace) != 0) {
            continue;
        }

        for (uint32_t e = 0; e < local.endpoint_count(); ++e) {
            if (local.endpoint(e).node == &candidate) {
                endpoints.push_back(local.endpoint(e).guid);
            }
        }

        return true;
    }

    const rmw_rosette::NodeEntities* remote =
        ctx.graph_cache->find_node({node_namespace, node_name});
    if (remote == nullptr) {
        return false;
    }

    endpoints = remote->endpoints;
    return true;
}

/**
 * @brief The shared body of the four rmw_get_*_names_and_types_by_node() queries.
 *
 * @param[in]  role        Which of the node's endpoints to list.
 * @param[in]  no_demangle `true` to list raw DDS names (publishers and subscriptions only).
 * @param[out] out         Zero-initialized result; left so on failure.
 * @param[in]  caller      rmw function name, for error messages.
 * @return RMW_RET_OK; RMW_RET_INVALID_ARGUMENT for a null argument, an invalid
 *   allocator, node name, or namespace, or a result already in use;
 *   RMW_RET_INCORRECT_RMW_IMPLEMENTATION for another rmw's node;
 *   RMW_RET_NODE_NAME_NON_EXISTENT when no node of that name is in the graph;
 *   RMW_RET_BAD_ALLOC when the heap is exhausted.
 *
 * Verification:
 * - Correct means: the result equals what rmw_fastrtps_cpp reports for the
 *   same node, including the return code for each bad argument.
 * - Covered by: test_graph_api in test_rmw_implementation (the four
 *   *_names_and_types_by_node_with_bad_arguments cases), and
 *   rmw_rosette/test/test_graph_cache.cpp (GraphCache.ListsByNode,
 *   RmwGraphQueries.ListsALocalNodesPublisher).
 *
 * References:
 * - rmw/get_node_info_and_types.h, rmw/get_service_names_and_types.h (Humble, Jazzy)
 * - rmw_fastrtps_shared_cpp/src/rmw_node_info_and_types.cpp (Humble),
 * `__get_names_and_types_by_node`
 */
rmw_ret_t answer_by_node(const rmw_node_t* node, rcutils_allocator_t* allocator,
                         const char* node_name, const char* node_namespace,
                         rmw_rosette::EndpointRole role, bool no_demangle,
                         rmw_names_and_types_t* out, const char* caller) noexcept {
    rmw_ret_t checked = check_query(node, allocator, out, caller);
    if (checked == RMW_RET_OK) {
        checked = check_node_name(node_name, node_namespace, caller);
    }

    if (checked != RMW_RET_OK) {
        return checked;
    }

    auto& ctx = *static_cast<const rmw_rosette::Node*>(node->data)->context;
    try {
        rmw_rosette::NamesAndTypes names;
        {
            rmw_rosette::ContextLock lock(ctx);
            std::vector<rosette::rtps::Guid> endpoints;
            if (!find_node_endpoints(ctx, node_name, node_namespace, endpoints)) {
                RMW_SET_ERROR_MSG_WITH_FORMAT_STRING(
                    "rmw_rosette: %s: no node '%s' in namespace '%s'", caller, node_name,
                    node_namespace);
                return RMW_RET_NODE_NAME_NON_EXISTENT;
            }

            names = ctx.graph_cache->names_and_types_of(endpoints, role, no_demangle);
        }

        return copy_names_and_types(names, allocator, out);
    } catch (const std::bad_alloc&) {
        RMW_SET_ERROR_MSG("rmw_rosette: out of memory listing graph names");
        return RMW_RET_BAD_ALLOC;
    }
}

}  // namespace

extern "C" {

/// @brief Lists the topics known to the graph, with their types; see GraphCache.
rmw_ret_t rmw_get_topic_names_and_types(const rmw_node_t* node, rcutils_allocator_t* allocator,
                                        bool                   no_demangle,
                                        rmw_names_and_types_t* topic_names_and_types) {
    const rmw_ret_t checked = check_query(node, allocator, topic_names_and_types, __func__);
    if (checked != RMW_RET_OK) {
        return checked;
    }

    return answer_query(node, allocator, topic_names_and_types,
                        [no_demangle](const rmw_rosette::GraphCache& cache) {
                            return cache.topic_names_and_types(no_demangle);
                        });
}

/// @brief Lists the services known to the graph, with their types; see GraphCache.
rmw_ret_t rmw_get_service_names_and_types(const rmw_node_t* node, rcutils_allocator_t* allocator,
                                          rmw_names_and_types_t* service_names_and_types) {
    const rmw_ret_t checked = check_query(node, allocator, service_names_and_types, __func__);
    if (checked != RMW_RET_OK) {
        return checked;
    }

    return answer_query(
        node, allocator, service_names_and_types,
        [](const rmw_rosette::GraphCache& cache) { return cache.service_names_and_types(); });
}

/// @brief Lists one node's publishers' topics, with their types; see answer_by_node().
rmw_ret_t rmw_get_publisher_names_and_types_by_node(const rmw_node_t* node,
                                                    rcutils_allocator_t* allocator,
                                                    const char* node_name,
                                                    const char* node_namespace, bool no_demangle,
                                                    rmw_names_and_types_t* topic_names_and_types) {
    return answer_by_node(node, allocator, node_name, node_namespace,
                          rmw_rosette::EndpointRole::publisher, no_demangle, topic_names_and_types,
                          __func__);
}

/// @brief Lists one node's subscriptions' topics, with their types; see answer_by_node().
rmw_ret_t rmw_get_subscriber_names_and_types_by_node(const rmw_node_t* node,
                                                     rcutils_allocator_t* allocator,
                                                     const char* node_name,
                                                     const char* node_namespace, bool no_demangle,
                                                     rmw_names_and_types_t* topic_names_and_types) {
    return answer_by_node(node, allocator, node_name, node_namespace,
                          rmw_rosette::EndpointRole::subscription, no_demangle,
                          topic_names_and_types, __func__);
}

/// @brief Lists the services one node serves, with their types; see answer_by_node().
rmw_ret_t rmw_get_service_names_and_types_by_node(const rmw_node_t* node,
                                                  rcutils_allocator_t* allocator,
                                                  const char* node_name, const char* node_namespace,
                                                  rmw_names_and_types_t* service_names_and_types) {
    return answer_by_node(node, allocator, node_name, node_namespace,
                          rmw_rosette::EndpointRole::server, false, service_names_and_types,
                          __func__);
}

/// @brief Lists the services one node is a client of, with their types; see answer_by_node().
rmw_ret_t rmw_get_client_names_and_types_by_node(const rmw_node_t* node,
                                                 rcutils_allocator_t* allocator,
                                                 const char* node_name, const char* node_namespace,
                                                 rmw_names_and_types_t* service_names_and_types) {
    return answer_by_node(node, allocator, node_name, node_namespace,
                          rmw_rosette::EndpointRole::client, false, service_names_and_types,
                          __func__);
}

}  // extern "C"
