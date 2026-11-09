// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file rmw_rosette/graph_cache.hpp
 * @brief Host-side record of every endpoint's names and every peer's nodes, for rmw graph
 * queries.
 *
 * EndpointMatchTable keeps remote endpoints as 32-bit name keys so a board
 * spends no RAM on names it never needs. Graph queries such as `ros2 topic
 * list` need those names back, so a hosted build registers this cache as the
 * table's GraphEndpointObserver and keeps a copy of each name.
 *
 * Node names travel in no SEDP message. RosDiscoveryInfoReader stores each
 * peer's `ros_discovery_info` node list here; the context's own nodes come from
 * its LocalNodeEntities table instead (rmw_graph_node_names.cpp).
 *
 * The cache lives on the heap and grows with the network, unlike the table,
 * and is therefore compiled only into builds with `RMW_ROSETTE_GRAPH_CACHE=1`,
 * which CMake sets on POSIX. A firmware build has no cache, allocates nothing
 * for graph queries, and reports them as RMW_RET_UNSUPPORTED.
 *
 * The cache, not the match table, tells the context's graph guard condition
 * that the listings changed: the table reports only the endpoints it has room
 * for, while the cache lists every endpoint, so only the cache knows when what
 * a query returns has changed.
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
#  include <vector>

#  include "rosette/rtps/endpoint_match_table.hpp"

namespace rmw_rosette {

/// @brief ROS or DDS names, each with the sorted set of its types.
using NamesAndTypes = std::map<std::string, std::set<std::string>>;

/// @brief One ROS node, as `ros_discovery_info` names it.
struct NodeName {
    std::string namespace_;  ///< Namespace, such as `/` or `/robot`.
    std::string name;        ///< Name within the namespace.

    /// @brief Whether both name the same node.
    bool operator==(const NodeName& other) const {
        return namespace_ == other.namespace_ && name == other.name;
    }
};

/**
 * @brief Every endpoint this participant has discovered or created, by GUID, and
 * every other participant's nodes.
 *
 * Verification:
 * - Correct means: an endpoint is listed from its first announcement until it
 *   is withdrawn or its participant leaves, a participant's nodes are listed
 *   from its first `ros_discovery_info` sample until a later one replaces them
 *   or the participant leaves, a repeated announcement changes nothing, an
 *   endpoint the match table refused for space is still listed, a parked
 *   participant's endpoints stay listed and those it withdrew while parked
 *   leave once its replay completes, and the change callback fires exactly
 *   when a listing could differ.
 * - Covered by: rmw_rosette/test/test_graph_cache.cpp.
 *
 * References:
 * - OMG DDSI-RTPS 2.5 §8.5.4 (SEDP)
 * - rmw_dds_common/graph_cache.hpp (Humble), the equivalent in stock rmws
 *
 * Example:
 * @code{.cpp}
 * rmw_rosette::GraphCache cache;
 * cache.set_on_change(trigger_graph_guard, &graph_guard);
 * participant.match_table().set_graph_endpoint_observer(&cache.observer());
 * // ... the participant runs discovery under the context lock ...
 * const rmw_rosette::NamesAndTypes topics = cache.topic_names_and_types(false);
 * @endcode
 * Full program: rmw_rosette/test/test_graph_cache.cpp
 */
class GraphCache {
public:
    /// @brief Called, under the context lock, after the recorded endpoints or nodes change.
    using OnChange = void (*)(void* user) noexcept;

    GraphCache() noexcept;

    /// @brief Returns the observer to register with the participant's match table.
    const rosette::rtps::GraphEndpointObserver& observer() const noexcept { return observer_; }

    /**
     * @brief Registers the callback told that the recorded endpoints or nodes changed.
     *
     * rmw_init() passes the trigger of the context's graph guard condition, so
     * a waiter on rmw_node_get_graph_guard_condition() wakes for every change a
     * graph query would report, and for none that it would not.
     *
     * @param[in] on_change Callback, or null for none.
     * @param[in] user      Passed to `on_change`; must outlive the cache.
     */
    void set_on_change(OnChange on_change, void* user) noexcept {
        on_change_ = on_change;
        on_change_user_ = user;
    }

    /**
     * @brief Records an endpoint, or refreshes one already recorded.
     *
     * @param[in] guid      Endpoint GUID.
     * @param[in] topic     DDS topic name, NUL-terminated; copied.
     * @param[in] type      DDS type name, NUL-terminated; copied.
     * @param[in] is_writer `true` for a writer; `false` for a reader.
     * @return `true`, after the change callback ran, when the endpoint is new or
     *   its names or kind differ from the recorded ones; `false` for a repeated
     *   identical announcement.
     * @throws std::bad_alloc when the heap is exhausted; nothing is recorded.
     */
    bool add(const rosette::rtps::Guid& guid, const char* topic, const char* type, bool is_writer);

    /// @brief Forgets one endpoint; `true`, after the change callback ran, when it was recorded.
    bool remove(const rosette::rtps::Guid& guid) noexcept;

    /// @brief Forgets every endpoint and node of a participant; `true`, after the change
    /// callback ran, when it had any.
    bool remove_participant(const rosette::rtps::GuidPrefix& prefix) noexcept;

    /**
     * @brief Marks every endpoint of a reactivated participant unconfirmed.
     *
     * The participant replays its live endpoints after reactivation; add()
     * confirms each one it replays. Nothing leaves the listing yet.
     *
     * @param[in] prefix GUID prefix of the participant being refreshed.
     *
     * Verification:
     * - Correct means: the listings and the change callback are unaffected.
     * - Covered by: rmw_rosette/test/test_graph_cache.cpp (RefreshForgetsOnlyUnconfirmed).
     */
    void begin_refresh(const rosette::rtps::GuidPrefix& prefix) noexcept;

    /**
     * @brief Forgets every endpoint of a refreshed participant that its replay did not confirm.
     *
     * @param[in] prefix GUID prefix of the participant whose replay is complete.
     * @return `true`, after the change callback ran, when an endpoint was forgotten.
     *
     * Verification:
     * - Correct means: exactly the endpoints of `prefix` not passed to add()
     *   since begin_refresh() are forgotten; other participants are untouched.
     * - Covered by: rmw_rosette/test/test_graph_cache.cpp (RefreshForgetsOnlyUnconfirmed).
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.5.4.1 (a late joiner receives only live announcements)
     */
    bool end_refresh(const rosette::rtps::GuidPrefix& prefix) noexcept;

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

    /**
     * @brief Replaces everything known about one participant's nodes.
     *
     * Each `ros_discovery_info` sample carries the participant's complete node
     * list, so an empty list means it hosts no node.
     *
     * @param[in] prefix Participant the list came from.
     * @param[in] nodes  Its complete node list.
     * @return `true`, after the change callback ran, when the list differs from
     *   the recorded one; `false` for a repeated identical list.
     * @throws std::bad_alloc when the heap is exhausted.
     *
     * Verification:
     * - Correct means: a later list replaces an earlier one, the nodes last until
     *   replaced or until their participant leaves, and the change callback fires
     *   exactly when node_names() could differ.
     * - Covered by: rmw_rosette/test/test_graph_cache.cpp (ReplacesAndForgetsNodes).
     *
     * References:
     * - rmw_dds_common/graph_cache.hpp (Humble), `update_participant_entities`
     */
    bool set_participant_nodes(const rosette::rtps::GuidPrefix& prefix,
                               std::vector<NodeName>            nodes);

    /**
     * @brief Lists the nodes of every other participant.
     *
     * @return Each node with its namespace; duplicates are kept.
     * @throws std::bad_alloc when the heap is exhausted.
     */
    std::vector<NodeName> node_names() const;

private:
    using GuidBytes = std::array<uint8_t, 16>;
    using PrefixBytes = std::array<uint8_t, 12>;

    struct Endpoint {
        std::string topic;
        std::string type;
        bool        is_writer = false;
        /// Cleared by begin_refresh(), set again when the endpoint is announced.
        bool        confirmed = true;
    };

    using Endpoints = std::map<GuidBytes, Endpoint>;

    static GuidBytes   key_of(const rosette::rtps::Guid& guid) noexcept;
    static PrefixBytes key_of(const rosette::rtps::GuidPrefix& prefix) noexcept;

    /// @brief The first endpoint of participant `prefix`; its endpoints are adjacent from there.
    Endpoints::iterator first_of(const rosette::rtps::GuidPrefix& prefix) noexcept;

    /// @brief Whether `it` is an endpoint of participant `prefix`.
    bool belongs_to(Endpoints::const_iterator        it,
                    const rosette::rtps::GuidPrefix& prefix) const noexcept;

    /// @brief Runs the change callback, when one is registered.
    void changed() const noexcept;

    Endpoints                                    endpoints_;
    std::map<PrefixBytes, std::vector<NodeName>> nodes_;
    rosette::rtps::GraphEndpointObserver         observer_{};
    OnChange                                     on_change_ = nullptr;
    void*                                        on_change_user_ = nullptr;
};

}  // namespace rmw_rosette

#endif  // RMW_ROSETTE_GRAPH_CACHE
