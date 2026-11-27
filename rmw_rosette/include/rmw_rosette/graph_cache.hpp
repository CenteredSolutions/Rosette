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
 * peer's `ros_discovery_info` node list here, with the GUIDs of the endpoints
 * each node owns; the context's own nodes come from its LocalNodeEntities
 * table instead. Per-node listings and endpoint details join the two: an
 * endpoint's names and QoS come from SEDP, its owner from `ros_discovery_info`.
 *
 * The cache lives on the heap and grows with the network, unlike the table,
 * and is therefore compiled only into builds with `RMW_ROSETTE_GRAPH_CACHE=1`,
 * which CMake sets on POSIX. A firmware build has no cache, allocates nothing
 * for graph queries, and reports them as RMW_RET_UNSUPPORTED.
 *
 * The cache answers every graph query of a hosted build, the topic and
 * service listings and the endpoint counts alike: the match table holds only
 * the endpoints it has room for and drops a parked peer's, while the cache
 * lists every endpoint, so only the cache knows when what a query returns has
 * changed. It triggers the context's graph guard condition on each such
 * change. The participant triggers the guard too, because a match can form
 * without any listing changing: a parked server's endpoints match again when
 * the peer is reactivated, which turns rmw_service_server_is_available() true
 * and must wake a client blocked in `wait_for_service()`.
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
#  include <cstddef>
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

/// @brief One remote ROS node and the endpoints `ros_discovery_info` says it owns.
struct NodeEntities {
    NodeName node;
    /// GUIDs of its readers and writers: the first 16 octets of each GID.
    std::vector<rosette::rtps::Guid> endpoints;

    /// @brief Whether both describe the same node with the same endpoints.
    bool operator==(const NodeEntities& other) const {
        return node == other.node && endpoints == other.endpoints;
    }
};

/// @brief Which endpoints a per-node listing selects, and how it names them.
enum class EndpointRole {
    publisher,     ///< Writers on `rt/` topics.
    subscription,  ///< Readers on `rt/` topics.
    server,        ///< Readers on `rq/<service>Request` topics.
    client,        ///< Readers on `rr/<service>Reply` topics.
};

/// @brief One endpoint on a topic, as rmw_get_publishers_info_by_topic() reports it.
struct TopicEndpoint {
    rosette::rtps::Guid guid;
    std::string type;  ///< Demangled unless the query asked for raw names.
    rosette::rtps::ReliabilityKind reliability;
    rosette::rtps::DurabilityKind durability;
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
     * @param[in] reliability Reliability the announcement offers or requests.
     * @param[in] durability  Durability the announcement offers or requests.
     * An endpoint already recorded is confirmed (see begin_refresh()) before
     * its new names are copied, so a refresh never forgets an endpoint its
     * replay announced, even when the copy fails.
     *
     * @return `true`, after the change callback ran, when the endpoint is new or
     *   its names, kind, or QoS differ from the recorded ones; `false` for a repeated
     *   identical announcement.
     * @throws std::bad_alloc when the heap is exhausted; a new endpoint is not
     *   recorded, and one already recorded keeps its previous names, confirmed.
     */
    bool add(const rosette::rtps::Guid& guid, const char* topic, const char* type, bool is_writer,
             rosette::rtps::ReliabilityKind reliability, rosette::rtps::DurabilityKind durability);

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
     * type `std_msgs/msg/String`; a type that is not a mangled ROS 2 message
     * type is listed as announced. Without demangling, every DDS topic is
     * listed verbatim.
     *
     * @param[in] no_demangle `true` to list raw DDS names.
     * @return Names with their types.
     * @throws std::bad_alloc when the heap is exhausted.
     *
     * Verification:
     * - Correct means: names and types equal what rmw_fastrtps_cpp reports.
     * - Covered by: rmw_rosette/test/test_graph_cache.cpp (DemanglesRosTopics,
     *   KeepsForeignTypesOnRosTopics).
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
     * @param[in] nodes  Its complete node list, with each node's endpoints.
     * @return `true`, after the change callback ran, when the list differs from
     *   the recorded one; `false` for a repeated identical list.
     * @throws std::bad_alloc when the heap is exhausted.
     *
     * Verification:
     * - Correct means: a later list replaces an earlier one, the nodes last until
     *   replaced or until their participant leaves, and the change callback fires
     *   exactly when node_names() or a per-node listing could differ.
     * - Covered by: rmw_rosette/test/test_graph_cache.cpp (ReplacesAndForgetsNodes).
     *
     * References:
     * - rmw_dds_common/graph_cache.hpp (Humble), `update_participant_entities`
     */
    bool set_participant_nodes(const rosette::rtps::GuidPrefix& prefix,
                               std::vector<NodeEntities> nodes);

    /**
     * @brief Lists the nodes of every other participant.
     *
     * @return Each node with its namespace; duplicates are kept.
     * @throws std::bad_alloc when the heap is exhausted.
     */
    std::vector<NodeName> node_names() const;

    /**
     * @brief Finds a remote node by name.
     *
     * @param[in] node Namespace and name to look for.
     * @return The first participant's record of that node, or null when no
     *   participant lists it. Valid until the cache next changes.
     */
    const NodeEntities* find_node(const NodeName& node) const noexcept;

    /**
     * @brief Names the remote node that owns an endpoint.
     *
     * @param[in] guid Endpoint GUID.
     * @return The node whose `ros_discovery_info` entry lists `guid`, or null
     *   when its participant lists it under no node. Valid until the cache next changes.
     */
    const NodeName* owner_of(const rosette::rtps::Guid& guid) const noexcept;

    /**
     * @brief Lists the topics or services of a node's endpoints in one role.
     *
     * The per-node counterpart of topic_names_and_types() and
     * service_names_and_types(). An endpoint the node lists that SEDP has not
     * announced is skipped, as stock rmws skip one their graph cache lacks.
     *
     * @param[in] endpoints   The node's endpoint GUIDs, local or remote.
     * @param[in] role        Which endpoints to list.
     * @param[in] no_demangle `true` to list raw DDS names; honored for
     *   publishers and subscriptions only, as rmw.h defines.
     * @return Names with their types.
     * @throws std::bad_alloc when the heap is exhausted.
     *
     * Verification:
     * - Correct means: names and types equal what rmw_fastrtps_cpp reports from
     *   rmw_get_{publisher,subscriber,service,client}_names_and_types_by_node()
     *   for the same node: a server is a reader on its request topic, a client a
     *   reader on its reply topic.
     * - Covered by: rmw_rosette/test/test_graph_cache.cpp (ListsByNode).
     *
     * References:
     * - rmw_dds_common/graph_cache.hpp (Humble, Jazzy), `get_writer_names_and_types_by_node`,
     *   `get_reader_names_and_types_by_node`
     * - rmw_fastrtps_shared_cpp/src/rmw_node_info_and_types.cpp (Humble, Jazzy)
     */
    NamesAndTypes names_and_types_of(const std::vector<rosette::rtps::Guid>& endpoints,
                                     EndpointRole role, bool no_demangle) const;

    /**
     * @brief Lists the writers or readers on one DDS topic, with their QoS.
     *
     * @param[in] topic       DDS topic name, such as `rt/chatter`; NUL-terminated.
     * @param[in] is_writer   `true` for writers; `false` for readers.
     * @param[in] no_demangle `true` to report types as announced; otherwise a
     *   ROS 2 message type is demangled to `pkg/msg/Name`.
     * @return One entry per endpoint, in GUID order.
     * @throws std::bad_alloc when the heap is exhausted.
     *
     * Verification:
     * - Correct means: the entries are exactly the endpoints count() counts.
     * - Covered by: rmw_rosette/test/test_graph_cache.cpp (ListsEndpointsOnTopic).
     *
     * References:
     * - rmw_dds_common/graph_cache.hpp (Humble, Jazzy), `get_writers_info_by_topic`
     */
    std::vector<TopicEndpoint> endpoints_on(const char* topic, bool is_writer,
                                            bool no_demangle) const;

    /**
     * @brief Counts the writers or readers recorded on one DDS topic.
     *
     * rmw_count_publishers() and its siblings mangle the ROS name to its DDS
     * topic and count here, so a count agrees with the listings: a parked
     * participant's endpoints and those the match table had no room for count.
     *
     * @param[in] topic     DDS topic name, such as `rt/chatter`; NUL-terminated.
     * @param[in] is_writer `true` to count writers; `false` for readers.
     * @return Recorded endpoints of that kind on `topic`, whatever their type.
     *
     * Verification:
     * - Correct means: equals what rmw_fastrtps_cpp's `__rmw_count_publishers`
     *   (`__rmw_count_subscribers`) reports for the same graph.
     * - Covered by: rmw_rosette/test/test_graph_cache.cpp (CountsMatchListings).
     *
     * References:
     * - rmw_dds_common/graph_cache.hpp (Humble), `get_writer_count`, `get_reader_count`
     */
    size_t count(const char* topic, bool is_writer) const noexcept;

private:
    using GuidBytes = std::array<uint8_t, 16>;
    using PrefixBytes = std::array<uint8_t, 12>;

    struct Endpoint {
        std::string topic;
        std::string type;
        bool        is_writer = false;
        rosette::rtps::ReliabilityKind reliability = rosette::rtps::ReliabilityKind::BEST_EFFORT;
        rosette::rtps::DurabilityKind durability = rosette::rtps::DurabilityKind::VOLATILE;
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
    std::map<PrefixBytes, std::vector<NodeEntities>> nodes_;
    rosette::rtps::GraphEndpointObserver         observer_{};
    OnChange                                     on_change_ = nullptr;
    void*                                        on_change_user_ = nullptr;
};

}  // namespace rmw_rosette

#endif  // RMW_ROSETTE_GRAPH_CACHE
