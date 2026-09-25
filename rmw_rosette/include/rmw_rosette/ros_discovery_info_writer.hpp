/**
 * @file rmw_rosette/ros_discovery_info_writer.hpp
 * @brief Publishes this participant's nodes on the `ros_discovery_info` topic.
 *
 * Stock DDS-based rmw implementations learn which ROS nodes a participant
 * hosts, and which endpoints belong to each, only from the
 * `rmw_dds_common/msg/ParticipantEntitiesInfo` that participant publishes on
 * the DDS topic `ros_discovery_info`. Without it, a stock `ros2 node list`
 * reports none of this participant's nodes, even though SEDP has announced
 * every endpoint.
 *
 * RosDiscoveryInfoWriter owns that one writer per context. Every rmw entry
 * point that creates or destroys a node or an endpoint reports the change
 * here, and the writer publishes the whole updated message, as rmw_fastrtps_cpp
 * and rmw_cyclonedds_cpp do. The writer is RELIABLE, TRANSIENT_LOCAL,
 * KEEP_LAST(1), so a peer that joins later receives the current message and
 * never a stale one.
 *
 * The topic name is used verbatim: stock rmw implementations create it with
 * `avoid_ros_namespace_conventions`, so it carries no `rt/` prefix.
 */

#pragma once

#include "local_node_entities.hpp"
#include "rosette/rtps/participant.hpp"
#include "rosette/rtps/stateful_writer.hpp"

namespace rmw_rosette {

/// DDS topic every stock DDS-based rmw uses for ParticipantEntitiesInfo.
constexpr const char* kRosDiscoveryInfoTopic = "ros_discovery_info";
/// DDS type name of `rmw_dds_common/msg/ParticipantEntitiesInfo`.
constexpr const char* kParticipantEntitiesInfoType =
    "rmw_dds_common::msg::dds_::ParticipantEntitiesInfo_";

/**
 * @brief The context's `ros_discovery_info` writer and the node table it publishes.
 *
 * Not thread-safe; callers hold the context lock (types.hpp, ContextLock).
 * Default-constructed state publishes nothing until create() succeeds.
 */
class RosDiscoveryInfoWriter {
public:
    /**
     * @brief Creates the RTPS writer on `ros_discovery_info`.
     *
     * rmw_init() calls this once the participant exists, before any node.
     * The writer is announced through SEDP like any other endpoint and is
     * deleted with the participant.
     *
     * @param[in,out] participant Participant that owns the writer; must outlive this object.
     * @return `true` on success; `false` when the participant's writer table is full.
     *
     * References:
     * - rmw_fastrtps_cpp/src/init_rmw_context_impl.cpp (Humble), `init_context_impl`
     */
    bool create(rtps::RTPSParticipant& participant) noexcept;

    /**
     * @brief Records a new node and publishes the updated message.
     *
     * @param[in] node Node being created; borrowed until remove_node().
     * @return `true` when recorded and published; on `false` nothing is
     *   recorded and the caller fails the node's creation, as stock rmw
     *   implementations do.
     *
     * Verification:
     * - Correct means: a stock `ros2 node list` shows the node while it exists.
     * - Covered by: tests/ros2/run_rmw_interop_loopback.sh (node list check).
     *
     * References:
     * - rmw_dds_common/graph_cache.hpp (Humble), `GraphCache::add_node`
     */
    bool add_node(const Node& node) noexcept;

    /**
     * @brief Forgets a node and its endpoints and publishes the updated message.
     *
     * @param[in] node Node being destroyed.
     */
    void remove_node(const Node& node) noexcept;

    /**
     * @brief Associates the endpoints of one rmw entity with its node and
     *        publishes the updated message once.
     *
     * A publisher has only a writer, a subscription only a reader, and a
     * service or client one of each; the absent one is null.
     *
     * @param[in] node   Owning node, already added.
     * @param[in] reader GUID of the entity's RTPS reader, or null.
     * @param[in] writer GUID of the entity's RTPS writer, or null.
     * @return `true` when recorded and published; on `false` nothing is
     *   recorded and the caller fails the entity's creation.
     *
     * Verification:
     * - Correct means: a stock `ros2 node info` lists the entity under its node.
     * - Covered by: tests/ros2/run_rmw_interop_loopback.sh (node info check).
     *
     * References:
     * - rmw_dds_common/graph_cache.hpp (Humble), `associate_reader`, `associate_writer`
     */
    bool add_endpoints(const Node& node, const rtps::Guid* reader,
                       const rtps::Guid* writer) noexcept;

    /**
     * @brief Removes the endpoints of one rmw entity and publishes the updated message.
     *
     * @param[in] reader GUID of the entity's RTPS reader, or null.
     * @param[in] writer GUID of the entity's RTPS writer, or null.
     */
    void remove_endpoints(const rtps::Guid* reader, const rtps::Guid* writer) noexcept;

private:
    /**
     * @brief Encodes the current table into a pool buffer and writes it.
     *
     * Starts from the smallest pool class and moves up one class whenever the
     * message does not fit, as serialize_to_pool() does for user messages.
     *
     * @return `true` when the writer accepted the change; `false` when no
     *   pool block could hold the message or create() never succeeded.
     */
    bool publish() noexcept;

    rtps::RTPSParticipant* participant_ = nullptr;
    rtps::StatefulWriter*  writer_      = nullptr;
    LocalNodeEntities      entities_;
};

}  // namespace rmw_rosette
