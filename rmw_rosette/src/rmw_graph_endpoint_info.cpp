// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file rmw_graph_endpoint_info.cpp
 * @brief rmw_get_publishers_info_by_topic() and
 * rmw_get_subscriptions_info_by_topic(), answered from the GraphCache.
 *
 * `ros2 topic echo`, `ros2 topic info -v`, and `ros2 bag record` call these
 * to learn each endpoint's owner and QoS. A row joins two sources: SEDP gives
 * the endpoint's GUID, type, reliability, and durability, and the owner is
 * the context's own node table for a local endpoint, or the node whose
 * `ros_discovery_info` lists the GUID for a remote one. An endpoint no node
 * claims is reported under `_NODE_NAME_UNKNOWN_` in
 * `_NODE_NAMESPACE_UNKNOWN_`, as stock rmws report it.
 *
 * Rosette's SEDP parses reliability and durability only, so every other QoS
 * policy is reported as rmw_qos_profile_unknown has it.
 *
 * Compiled into hosted builds only (RMW_ROSETTE_GRAPH_CACHE). A build without
 * the cache gets the RMW_RET_UNSUPPORTED versions in rmw_graph.cpp.
 */

#include <rcutils/allocator.h>
#include <rmw/error_handling.h>
#include <rmw/qos_profiles.h>
#include <rmw/rmw.h>
#include <rmw/topic_endpoint_info_array.h>

#include <cstring>
#include <new>
#include <string>
#include <vector>

#include "rmw_rosette/argument_checks.hpp"
#include "rmw_rosette/graph_cache.hpp"
#include "rmw_rosette/topic_naming.hpp"
#include "rmw_rosette/types.hpp"

namespace rtps = rosette::rtps;

namespace {

/// Owner names stock rmws report for an endpoint no node claims
/// (rmw_dds_common/graph_cache.cpp).
constexpr const char* kNodeNameUnknown = "_NODE_NAME_UNKNOWN_";
constexpr const char* kNodeNamespaceUnknown = "_NODE_NAMESPACE_UNKNOWN_";

/// @brief One endpoint on the topic with the node that owns it.
struct EndpointRow {
    rmw_rosette::TopicEndpoint endpoint;
    std::string node_name;
    std::string node_namespace;
};

/**
 * @brief Finds the node that owns an endpoint: the context's own first, then
 * every other participant's. Requires the context lock.
 */
void find_owner(const rmw_rosette::Context& ctx, const rtps::Guid& guid, EndpointRow& row) {
    const rmw_rosette::LocalNodeEntities& local = ctx.discovery_info.local_entities();
    for (uint32_t i = 0; i < local.endpoint_count(); ++i) {
        if (local.endpoint(i).guid == guid) {
            row.node_name = local.endpoint(i).node->name;
            row.node_namespace = local.endpoint(i).node->namespace_;
            return;
        }
    }

    const rmw_rosette::NodeName* remote = ctx.graph_cache->owner_of(guid);
    row.node_name = remote != nullptr ? remote->name : kNodeNameUnknown;
    row.node_namespace = remote != nullptr ? remote->namespace_ : kNodeNamespaceUnknown;
}

/**
 * @brief Lists the endpoints on one DDS topic, with their owners.
 *
 * @throws std::bad_alloc when the heap is exhausted.
 */
std::vector<EndpointRow> list_endpoints(rmw_rosette::Context& ctx, const char* dds_topic,
                                        bool is_writer, bool no_demangle) {
    rmw_rosette::ContextLock lock(ctx);
    std::vector<EndpointRow> rows;
    for (rmw_rosette::TopicEndpoint& endpoint :
         ctx.graph_cache->endpoints_on(dds_topic, is_writer, no_demangle)) {
        rows.push_back({std::move(endpoint), {}, {}});
        find_owner(ctx, rows.back().endpoint.guid, rows.back());
    }

    return rows;
}

/// @brief Maps SEDP's reliability and durability onto an otherwise unknown profile.
rmw_qos_profile_t qos_of(const rmw_rosette::TopicEndpoint& endpoint) noexcept {
    rmw_qos_profile_t qos = rmw_qos_profile_unknown;
    qos.reliability = endpoint.reliability == rtps::ReliabilityKind::RELIABLE
                          ? RMW_QOS_POLICY_RELIABILITY_RELIABLE
                          : RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT;
    switch (endpoint.durability) {
        case rtps::DurabilityKind::VOLATILE:
            qos.durability = RMW_QOS_POLICY_DURABILITY_VOLATILE;
            break;
        case rtps::DurabilityKind::TRANSIENT_LOCAL:
            qos.durability = RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
            break;
        default:
            // ROS 2 has no TRANSIENT or PERSISTENT durability; rmw_dds_common
            // reports them as unknown too.
            qos.durability = RMW_QOS_POLICY_DURABILITY_UNKNOWN;
            break;
    }

    return qos;
}

/**
 * @brief Fills one rmw row; `false` on allocation failure.
 *
 * The 16-byte GUID is the leading part of the GID, zero-padded to
 * RMW_GID_STORAGE_SIZE as rmw_fastrtps_cpp pads it.
 */
bool fill_info(const EndpointRow& row, bool is_writer, rcutils_allocator_t* allocator,
               rmw_topic_endpoint_info_t* info) noexcept {
    uint8_t gid[RMW_GID_STORAGE_SIZE] = {};
    static_assert(sizeof(gid) >= sizeof(rtps::Guid), "a GID holds a GUID");
    std::memcpy(gid, &row.endpoint.guid, sizeof(rtps::Guid));

    const rmw_qos_profile_t qos = qos_of(row.endpoint);
    return rmw_topic_endpoint_info_set_node_name(info, row.node_name.c_str(), allocator)
               == RMW_RET_OK
           && rmw_topic_endpoint_info_set_node_namespace(info, row.node_namespace.c_str(),
                                                         allocator)
                  == RMW_RET_OK
           && rmw_topic_endpoint_info_set_topic_type(info, row.endpoint.type.c_str(), allocator)
                  == RMW_RET_OK
           && rmw_topic_endpoint_info_set_endpoint_type(
                  info, is_writer ? RMW_ENDPOINT_PUBLISHER : RMW_ENDPOINT_SUBSCRIPTION)
                  == RMW_RET_OK
           && rmw_topic_endpoint_info_set_gid(info, gid, sizeof(gid)) == RMW_RET_OK
           && rmw_topic_endpoint_info_set_qos_profile(info, &qos) == RMW_RET_OK;
}

/**
 * @brief The shared body of both info_by_topic queries.
 *
 * @param[in]  topic_name ROS topic name, or the DDS name when `no_mangle`.
 * @param[in]  no_mangle  `true` to look `topic_name` up verbatim and list raw DDS type names.
 * @param[in]  is_writer  `true` for publishers; `false` for subscriptions.
 * @param[out] out        Zero-initialized result; left so on failure.
 * @return RMW_RET_OK; RMW_RET_INVALID_ARGUMENT for a null argument, an invalid
 *   allocator, or a result already in use; RMW_RET_INCORRECT_RMW_IMPLEMENTATION
 *   for another rmw's node; RMW_RET_BAD_ALLOC when the heap is exhausted. A
 *   name too long to be announced lists no endpoints.
 *
 * Verification:
 * - Correct means: one row per endpoint on the topic, local or remote, with
 *   its owner's name and namespace, its demangled type, its GUID as the GID,
 *   and the reliability and durability it announced; the bad-argument codes
 *   match rmw_fastrtps_cpp's.
 * - Covered by: test_graph_api in test_rmw_implementation
 *   (get_*_info_by_topic_with_bad_arguments), and
 *   rmw_rosette/test/test_graph_cache.cpp (GraphCache.ListsEndpointsOnTopic,
 *   RmwGraphQueries.ReportsALocalPublishersInfo).
 *
 * References:
 * - rmw/get_topic_endpoint_info.h (Humble, Jazzy)
 * - rmw_dds_common/graph_cache.hpp (Humble, Jazzy), `get_writers_info_by_topic`
 */
rmw_ret_t answer_info_by_topic(const rmw_node_t* node, rcutils_allocator_t* allocator,
                               const char* topic_name, bool no_mangle, bool is_writer,
                               rmw_topic_endpoint_info_array_t* out, const char* caller) noexcept {
    const rmw_ret_t checked = rmw_rosette::check_handle(
        node, caller, "node",
        {{allocator, "allocator"}, {topic_name, "topic_name"}, {out, "endpoints_info"}});
    if (checked != RMW_RET_OK) {
        return checked;
    }

    if (!rcutils_allocator_is_valid(allocator)) {
        RMW_SET_ERROR_MSG("rmw_rosette: allocator is invalid");
        return RMW_RET_INVALID_ARGUMENT;
    }

    if (rmw_topic_endpoint_info_array_check_zero(out) != RMW_RET_OK) {
        return RMW_RET_INVALID_ARGUMENT;
    }

    char mangled[RTPS_MAX_TOPIC_NAME_LEN];
    const char* dds_topic = topic_name;
    if (!no_mangle) {
        if (!rmw_rosette::compose_topic_name(mangled, sizeof(mangled), topic_name)) {
            return RMW_RET_OK;
        }

        dds_topic = mangled;
    }

    auto& ctx = *static_cast<const rmw_rosette::Node*>(node->data)->context;
    try {
        const std::vector<EndpointRow> rows = list_endpoints(ctx, dds_topic, is_writer, no_mangle);
        if (rows.empty()) {
            return RMW_RET_OK;
        }

        if (rmw_topic_endpoint_info_array_init_with_size(out, rows.size(), allocator)
            == RMW_RET_OK) {
            bool ok = true;
            for (size_t i = 0; ok && i < rows.size(); ++i) {
                ok = fill_info(rows[i], is_writer, allocator, &out->info_array[i]);
            }

            if (ok) {
                return RMW_RET_OK;
            }

            (void)rmw_topic_endpoint_info_array_fini(out, allocator);
        }

        rmw_reset_error();
    } catch (const std::bad_alloc&) {
    }

    RMW_SET_ERROR_MSG("rmw_rosette: out of memory listing topic endpoints");
    return RMW_RET_BAD_ALLOC;
}

}  // namespace

extern "C" {

/// @brief Lists the publishers on a topic with their owners and QoS; see answer_info_by_topic().
rmw_ret_t rmw_get_publishers_info_by_topic(const rmw_node_t* node, rcutils_allocator_t* allocator,
                                           const char* topic_name, bool no_mangle,
                                           rmw_topic_endpoint_info_array_t* publishers_info) {
    return answer_info_by_topic(node, allocator, topic_name, no_mangle, /*is_writer=*/true,
                                publishers_info, __func__);
}

/// @brief Lists the subscriptions on a topic with their owners and QoS; see answer_info_by_topic().
rmw_ret_t rmw_get_subscriptions_info_by_topic(const rmw_node_t* node,
                                              rcutils_allocator_t* allocator,
                                              const char* topic_name, bool no_mangle,
                                              rmw_topic_endpoint_info_array_t* subscriptions_info) {
    return answer_info_by_topic(node, allocator, topic_name, no_mangle, /*is_writer=*/false,
                                subscriptions_info, __func__);
}

}  // extern "C"
