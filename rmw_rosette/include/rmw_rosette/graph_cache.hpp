// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file rmw_rosette/graph_cache.hpp
 * @brief Host-side record of every endpoint's names, for rmw graph queries.
 *
 * EndpointMatchTable keeps remote endpoints as 32-bit name keys so a board
 * spends no RAM on names it never needs. Graph queries such as `ros2 topic
 * list` need those names back, so a hosted build registers this cache as the
 * table's GraphEndpointObserver and keeps a copy of each name.
 *
 * The cache lives on the heap and grows with the network, unlike the table,
 * and is therefore compiled only into builds with `RMW_ROSETTE_GRAPH_CACHE=1`,
 * which CMake sets on POSIX. A firmware build has no cache, allocates nothing
 * for graph queries, and reports them as RMW_RET_UNSUPPORTED.
 *
 * Concurrency: every method requires the context lock. The observer runs on
 * the thread driving the participant, which holds that lock, and queries take
 * it through ContextLock.
 */

#pragma once

#ifndef RMW_ROSETTE_GRAPH_CACHE
#  define RMW_ROSETTE_GRAPH_CACHE 0
#endif

#if RMW_ROSETTE_GRAPH_CACHE

#  include <array>
#  include <cstdint>
#  include <map>
#  include <set>
#  include <string>

#  include "rosette/rtps/endpoint_match_table.hpp"

namespace rmw_rosette {

/// @brief ROS or DDS names, each with the sorted set of its types.
using NamesAndTypes = std::map<std::string, std::set<std::string>>;

/**
 * @brief Every endpoint this participant has discovered or created, by GUID.
 *
 * Verification:
 * - Correct means: an endpoint is listed from its first announcement until it
 *   is withdrawn or its participant leaves, a repeated announcement changes
 *   nothing, and an endpoint the match table refused for space is still listed.
 * - Covered by: rmw_rosette/test/test_graph_cache.cpp.
 *
 * References:
 * - OMG DDSI-RTPS 2.5 §8.5.4 (SEDP)
 * - rmw_dds_common/graph_cache.hpp (Humble), the equivalent in stock rmws
 *
 * Example:
 * @code{.cpp}
 * rmw_rosette::GraphCache cache;
 * participant.match_table().set_graph_endpoint_observer(&cache.observer());
 * // ... the participant runs discovery under the context lock ...
 * const rmw_rosette::NamesAndTypes topics = cache.topic_names_and_types(false);
 * @endcode
 * Full program: rmw_rosette/test/test_graph_cache.cpp
 */
class GraphCache {
public:
    GraphCache() noexcept;

    /// @brief Returns the observer to register with the participant's match table.
    const rosette::rtps::GraphEndpointObserver& observer() const noexcept { return observer_; }

    /**
     * @brief Records an endpoint, or refreshes one already recorded.
     *
     * @param[in] guid      Endpoint GUID.
     * @param[in] topic     DDS topic name, NUL-terminated; copied.
     * @param[in] type      DDS type name, NUL-terminated; copied.
     * @param[in] is_writer `true` for a writer; `false` for a reader.
     * @throws std::bad_alloc when the heap is exhausted.
     */
    void add(const rosette::rtps::Guid& guid, const char* topic, const char* type, bool is_writer);

    /// @brief Forgets one endpoint; an unknown GUID is ignored.
    void remove(const rosette::rtps::Guid& guid) noexcept;

    /// @brief Forgets every endpoint of a participant.
    void remove_participant(const rosette::rtps::GuidPrefix& prefix) noexcept;

    /**
     * @brief Lists topics and their types, as rmw_get_topic_names_and_types() reports them.
     *
     * With demangling, only ROS topics (`rt/`) are listed, as `/chatter` with
     * type `std_msgs/msg/String`. Without it, every DDS topic is listed
     * verbatim.
     *
     * @param[in] no_demangle `true` to list raw DDS names.
     * @return Names with their types.
     * @throws std::bad_alloc when the heap is exhausted.
     *
     * Verification:
     * - Correct means: names and types equal what rmw_fastrtps_cpp reports.
     * - Covered by: rmw_rosette/test/test_graph_cache.cpp (DemanglesRosTopics).
     *
     * References:
     * - ROS 2 design, "Topic and Service name mapping to DDS"
     * - rmw_dds_common/graph_cache.hpp, rmw_fastrtps_shared_cpp/demangle.cpp (Humble)
     */
    NamesAndTypes topic_names_and_types(bool no_demangle) const;

    /**
     * @brief Lists services and their types, as rmw_get_service_names_and_types() reports them.
     *
     * A service is recognized by its request (`rq/<name>Request`) or reply
     * (`rr/<name>Reply`) topic, so a service that has only clients is listed
     * too, as in stock rmws.
     *
     * @return Service names such as `/add` with types such as `example_interfaces/srv/AddTwoInts`.
     * @throws std::bad_alloc when the heap is exhausted.
     *
     * Verification:
     * - Correct means: names and types equal what rmw_fastrtps_cpp reports.
     * - Covered by: rmw_rosette/test/test_graph_cache.cpp (DemanglesServices).
     *
     * References:
     * - rmw_fastrtps_shared_cpp/demangle.cpp (Humble), `_demangle_service_from_topic`
     */
    NamesAndTypes service_names_and_types() const;

private:
    using GuidBytes = std::array<uint8_t, 16>;

    struct Endpoint {
        std::string topic;
        std::string type;
        bool        is_writer = false;
    };

    static GuidBytes key_of(const rosette::rtps::Guid& guid) noexcept;

    std::map<GuidBytes, Endpoint>        endpoints_;
    rosette::rtps::GraphEndpointObserver observer_{};
};

}  // namespace rmw_rosette

#endif  // RMW_ROSETTE_GRAPH_CACHE
