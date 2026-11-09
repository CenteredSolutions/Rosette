// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file rmw_rosette/ros_discovery_info_reader.hpp
 * @brief Reads other participants' nodes from the `ros_discovery_info` topic.
 *
 * SEDP announces endpoints and says nothing about ROS nodes. Each participant
 * therefore publishes its nodes as a `rmw_dds_common/msg/ParticipantEntitiesInfo`
 * on `ros_discovery_info` (RosDiscoveryInfoWriter), and this reader stores what
 * peers publish in the GraphCache so that `ros2 node list` can show them.
 *
 * A message replaces everything known about its participant. The reader is
 * RELIABLE and TRANSIENT_LOCAL, so a peer's current message arrives however
 * late this participant joins.
 *
 * Compiled into builds with `RMW_ROSETTE_GRAPH_CACHE=1` only. A firmware build
 * has no reader, spends no endpoint or proxy on it, and reports node queries
 * as RMW_RET_UNSUPPORTED.
 */

#pragma once

#ifndef RMW_ROSETTE_GRAPH_CACHE
#  define RMW_ROSETTE_GRAPH_CACHE 0
#endif

#if RMW_ROSETTE_GRAPH_CACHE

#  include <cstddef>
#  include <cstdint>
#  include <vector>

#  include "graph_cache.hpp"
#  include "rosette/rtps/participant.hpp"

namespace rmw_rosette {

/**
 * @brief Decodes the node names of one serialized ParticipantEntitiesInfo.
 *
 * Every length comes from a remote peer, so each is checked against the bytes
 * that remain before anything is read or stored. The endpoint GID sequences
 * are skipped, because node listings do not use them.
 *
 * @param[in]  payload Serialized payload, starting at the CDR encapsulation header.
 * @param[in]  size    Length of `payload` in octets.
 * @param[out] nodes   Replaced with the decoded nodes on success; unchanged on failure.
 * @return `true` when the whole message decoded; `false` for an unsupported
 *   encapsulation or a malformed or truncated message.
 * @throws std::bad_alloc when the heap is exhausted.
 *
 * Verification:
 * - Correct means: a message LocalNodeEntities encodes decodes to the same
 *   names, and a message cut at any length is refused without a read past its end.
 * - Covered by: rmw_rosette/test/test_graph_cache.cpp (DecodesNodeNames,
 *   RefusesTruncatedNodeMessages).
 *
 * References:
 * - rmw_dds_common/msg/ParticipantEntitiesInfo.msg, NodeEntitiesInfo.msg (Humble, Jazzy)
 * - OMG CDR 3.1, CORBA 3.4 Part 2 §9.3 (primitive alignment, strings, sequences)
 */
bool decode_participant_node_names(const uint8_t* payload, size_t size,
                                   std::vector<NodeName>& nodes);

/**
 * @brief The context's `ros_discovery_info` reader, feeding its GraphCache.
 *
 * Not thread-safe; create() runs under the context lock during rmw_init(), and
 * deliveries run on the thread driving the participant, which holds that lock.
 *
 * Verification:
 * - Correct means: `ros2 node list` on rmw_rosette shows a stock peer's nodes
 *   while they exist.
 * - Covered by: test_graph_api in test_rmw_implementation, and the rmw_rosette
 *   unit test RmwNodeNames.ListsALocalNodeUntilDestroyed.
 *
 * References:
 * - rmw_fastrtps_cpp/src/init_rmw_context_impl.cpp (Humble), `init_context_impl`
 * - rmw_dds_common/graph_cache.hpp (Humble), `update_participant_entities`
 */
class RosDiscoveryInfoReader {
public:
    /**
     * @brief Creates the RTPS reader on `ros_discovery_info`.
     *
     * rmw_init() calls this after the GraphCache exists and before any node.
     * The reader is deleted with the participant. Every active peer writes this
     * topic, so the reader's floor and cap in the WriterProxy pool are both
     * RTPS_MAX_PEERS, and capacity.hpp proves the pool holds them.
     *
     * @param[in,out] participant Participant that owns the reader; outlives this object.
     * @param[in,out] cache       Receives each peer's nodes, and tells the graph guard
     *   condition when they change; outlives this object.
     * @return `true` on success; `false` when the participant's reader table
     *   is full or its WriterProxy pool cannot reserve RTPS_MAX_PEERS.
     */
    bool create(rosette::rtps::RTPSParticipant& participant, GraphCache& cache) noexcept;

private:
    /// @brief Stores one delivered message; a message that cannot be used is dropped.
    static void on_change(const rosette::rtps::CacheChange& change, void* self) noexcept;

    GraphCache* cache_ = nullptr;
};

}  // namespace rmw_rosette

#endif  // RMW_ROSETTE_GRAPH_CACHE
