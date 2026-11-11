// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file rmw_graph_names.cpp
 * @brief rmw topic and service name queries, answered from the GraphCache.
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

#include <new>

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

}  // extern "C"
