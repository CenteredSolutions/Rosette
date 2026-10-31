/**
 * @file rmw_rosette/local_node_entities.hpp
 * @brief This participant's nodes and the endpoints each one owns, encoded
 *        as an `rmw_dds_common/msg/ParticipantEntitiesInfo`.
 *
 * RTPS discovery (SPDP/SEDP) announces participants and endpoints, but has no
 * notion of a ROS node. Every DDS-based rmw closes that gap the same way: each
 * participant publishes one ParticipantEntitiesInfo on the `ros_discovery_info`
 * topic, listing its nodes and the reader and writer GIDs each node owns.
 * `ros2 node list`, `ros2 node info`, and every `rmw_get_*_by_node()` query on
 * a stock peer are answered from those messages.
 *
 * LocalNodeEntities is the local half of that picture: the table the rmw
 * entry points update as nodes and endpoints come and go, and the encoder that
 * turns it into the message body. It owns no RTPS state and never publishes;
 * RosDiscoveryInfoWriter (ros_discovery_info_writer.hpp) does that.
 *
 * Wire format (XCDR1, the CDR body after the encapsulation header):
 *
 *     ParticipantEntitiesInfo { Gid gid; sequence<NodeEntitiesInfo> nodes; }
 *     NodeEntitiesInfo { string<=256 node_namespace; string<=256 node_name;
 *                        sequence<Gid> reader_gid_seq;
 *                        sequence<Gid> writer_gid_seq; }
 *     Gid { char data[RMW_GID_STORAGE_SIZE]; }
 *
 * Gid.msg and rmw's RMW_GID_STORAGE_SIZE change together: 24 octets on Humble,
 * 16 from Iron on. A GID is the 16-octet RTPS GUID followed by zero padding to
 * that size, the layout rmw_fastrtps_cpp and rmw_cyclonedds_cpp use and
 * guid_to_rmw_gid() writes, so a peer on the same distribution decodes it.
 * The participant's own GID uses ENTITYID_PARTICIPANT, which is how a peer
 * ties each message to the participant it discovered through SPDP.
 *
 * Storage is fixed: one slot per node the node pool can hold and one per RTPS
 * endpoint the participant can create, so the table cannot fill before the
 * pools that feed it do.
 */

#pragma once

#include "capacity.hpp"
#include "rosette/rtps/types.hpp"

#include <rmw/types.h>

#include <cstdint>

namespace rmw_rosette {

struct Node;
class CdrWriter;

/// Octets in an `rmw_dds_common/msg/Gid`, which matches rmw_gid_t storage on every distribution.
constexpr uint32_t kGidSize = RMW_GID_STORAGE_SIZE;

/// Longest node name or namespace ParticipantEntitiesInfo carries (`string<=256`).
constexpr uint32_t kNodeEntitiesMaxNameLength = 256;

/**
 * @brief The local participant's nodes and the endpoint GIDs each one owns.
 *
 * Mirrors the local-participant operations of rmw_dds_common::GraphCache
 * (add_node, associate_reader/writer, dissociate_reader/writer, remove_node),
 * restricted to fixed storage. Nodes are encoded in the order they were added.
 *
 * Not thread-safe; callers hold the context lock (types.hpp, ContextLock).
 */
class LocalNodeEntities {
public:
    /**
     * @brief Records a node, with no endpoints yet.
     *
     * @param[in] node Node to record; borrowed until remove_node(). Its name
     *   and namespace are read at every encode().
     * @return `true` when recorded; `false` when the table already holds
     *   RMW_ROSETTE_MAX_NODES nodes.
     */
    bool add_node(const Node& node) noexcept;

    /**
     * @brief Forgets a node and every endpoint associated with it.
     *
     * @param[in] node Node previously passed to add_node(); unknown nodes are ignored.
     */
    void remove_node(const Node& node) noexcept;

    /**
     * @brief Associates an RTPS endpoint with the node that owns it.
     *
     * @param[in] node      Owning node; must already be recorded.
     * @param[in] guid      RTPS GUID of the endpoint.
     * @param[in] is_reader `true` for a reader (subscription, service request,
     *   client reply); `false` for a writer.
     * @return `true` when recorded; `false` when `node` is not recorded or the
     *   endpoint table is full.
     */
    bool associate(const Node& node, const rosette::rtps::Guid& guid, bool is_reader) noexcept;

    /**
     * @brief Removes an endpoint association.
     *
     * @param[in] guid RTPS GUID previously passed to associate(); unknown GUIDs are ignored.
     */
    void dissociate(const rosette::rtps::Guid& guid) noexcept;

    /**
     * @brief Encodes the table as a ParticipantEntitiesInfo CDR body.
     *
     * Writes the participant GID, then one NodeEntitiesInfo per recorded node
     * in insertion order, each listing its readers then its writers. Names
     * longer than kNodeEntitiesMaxNameLength are cut to that bound, as the
     * message's `string<=256` fields require.
     *
     * @param[in]  participant GUID of the local participant (ENTITYID_PARTICIPANT).
     * @param[out] out         Destination for the body, after the encapsulation header.
     * @param[in]  cap         Capacity of `out` in octets.
     * @return Octets written, or 0 when the body does not fit in `cap`.
     *
     * Verification:
     * - Correct means: the bytes decode, through rmw_dds_common's own
     *   introspection type support, to a message with the same GID, nodes,
     *   and per-node GID sequences; a buffer one octet short returns 0.
     * - Covered by: rmw_rosette/test/test_local_node_entities.cpp.
     *
     * References:
     * - rmw_dds_common/msg/ParticipantEntitiesInfo.msg, NodeEntitiesInfo.msg,
     *   Gid.msg (Humble: char[24]; Jazzy: char[16])
     * - OMG DDS-XTypes 1.3 §7.4.1 (XCDR1 encoding of strings and sequences)
     */
    uint32_t encode(const rosette::rtps::Guid& participant, uint8_t* out,
                    uint32_t cap) const noexcept;

    /**
     * @brief Octets encode() writes for the table as it stands.
     *
     * The ros_discovery_info writer allocates its pool block at exactly this
     * size, so the message lands in the smallest class that holds it with
     * one encoding pass.
     *
     * @param[in] participant GUID of the local participant, as passed to encode().
     * @return The body length encode() returns given enough capacity.
     *
     * Verification:
     * - Correct means: encode() into a buffer of encoded_size() octets
     *   succeeds and returns encoded_size(); one octet fewer returns 0.
     * - Covered by: rmw_rosette/test/test_local_node_entities.cpp.
     *
     * References:
     * - OMG DDS-XTypes 1.3 §7.4.1 (XCDR1 encoding of strings and sequences)
     */
    uint32_t encoded_size(const rosette::rtps::Guid& participant) const noexcept;

private:
    /**
     * @brief Writes the ParticipantEntitiesInfo body that encode() documents.
     *
     * @param[in]     participant GUID of the local participant.
     * @param[in,out] cdr         Writer over the destination, or a measuring writer.
     */
    void write(const rosette::rtps::Guid& participant, CdrWriter& cdr) const noexcept;

    struct EndpointEntry {
        const Node* node      = nullptr;
        rosette::rtps::Guid guid{};
        bool        is_reader = false;
    };

    static constexpr uint32_t kMaxEndpoints = kRtpsWriterCapacity + kRtpsReaderCapacity;

    const Node*   nodes_[RMW_ROSETTE_MAX_NODES] = {};
    uint32_t      node_count_ = 0;
    EndpointEntry endpoints_[kMaxEndpoints] = {};
    uint32_t      endpoint_count_ = 0;
};

}  // namespace rmw_rosette
