/**
 * @file ros/discovery.hpp
 * @brief ROS 2 graph-discovery types and CDR encoder for ros_discovery_info.
 *
 * Implements the wire format for rmw_dds_common::msg::dds_::ParticipantEntitiesInfo_
 * so that ros2 node list / ros2 topic list / rqt_graph can discover the embedded
 * participant's nodes, publishers, and subscriptions.
 *
 * Reference: https://design.ros2.org/articles/Node_to_Participant_mapping.html
 *
 * Message layout (CDR Little-Endian, RepresentationHeader = {0x00,0x01,0x00,0x00}):
 *
 *   ParticipantEntitiesInfo_:
 *     Gid gid                                  — 24 bytes (fixed array)
 *     sequence<NodeEntitiesInfo_> node_entities_info_seq
 *       → uint32_t length + N × NodeEntitiesInfo_
 *
 *   NodeEntitiesInfo_:
 *     string  node_namespace
 *     string  node_name
 *     sequence<Gid> writer_gid_seq
 *     sequence<Gid> reader_gid_seq
 *
 *   Gid:
 *     uint8[24] data  — bytes 0-11: GuidPrefix, 12-15: EntityId, 16-23: zeros
 *
 * CDR strings: uint32 (strlen+1, i.e. includes null) + chars + '\0' + pad to 4.
 * All uint32 fields are aligned to 4-byte boundaries from the start of the buffer.
 *
 * Sizing:
 *   Default MILKROS_MAX_NODES=4, MILKROS_MAX_ENTITIES_PER_NODE=8.
 *   Worst-case encoded size ≈ 2016 bytes — fits in a single 2048-byte pool block.
 */

#pragma once

#include <stdint.h>
#include <string.h>
#include "rosette/rtps/types.hpp"

namespace milkros {

// ============================================================
// Compile-time limits (all #ifndef-overridable)
// ============================================================

#ifndef MILKROS_MAX_NODES
#  define MILKROS_MAX_NODES 4
#endif
#ifndef MILKROS_MAX_ENTITIES_PER_NODE
#  define MILKROS_MAX_ENTITIES_PER_NODE 8
#endif

// ============================================================
// Gid — mirrors rmw_dds_common/msg/Gid.msg (uint8[24] data)
// ============================================================

struct Gid {
    uint8_t data[24] = {};

    /// Build a Gid from an RTPS Guid.
    /// Layout: bytes 0-11 = GuidPrefix.value, 12-15 = EntityId, 16-23 = zeros.
    static Gid from_guid(const rtps::Guid& g) noexcept {
        Gid gid{};
        static_assert(sizeof(g.prefix.value) == 12, "GuidPrefix must be 12 bytes");
        memcpy(gid.data, g.prefix.value, 12);
        gid.data[12] = g.entityId.entityKey[0];
        gid.data[13] = g.entityId.entityKey[1];
        gid.data[14] = g.entityId.entityKey[2];
        gid.data[15] = g.entityId.entityKind;
        // bytes 16-23 remain zero
        return gid;
    }

    bool operator==(const Gid& o) const noexcept {
        return memcmp(data, o.data, 24) == 0;
    }
    bool operator!=(const Gid& o) const noexcept {
        return !(*this == o);
    }
};

// ============================================================
// NodeEntitiesInfo — mirrors rmw_dds_common/msg/NodeEntitiesInfo.msg
// ============================================================

struct NodeEntitiesInfo {
    char    node_namespace[64] = {};   ///< ROS node namespace (e.g. "/")
    char    node_name[32]      = {};   ///< ROS node name (e.g. "my_node")
    Gid     writer_gids[MILKROS_MAX_ENTITIES_PER_NODE] = {};
    Gid     reader_gids[MILKROS_MAX_ENTITIES_PER_NODE] = {};
    uint8_t writer_count = 0u;
    uint8_t reader_count = 0u;

    bool add_writer(const rtps::Guid& g) noexcept {
        if (writer_count >= MILKROS_MAX_ENTITIES_PER_NODE) return false;
        writer_gids[writer_count++] = Gid::from_guid(g);
        return true;
    }

    bool add_reader(const rtps::Guid& g) noexcept {
        if (reader_count >= MILKROS_MAX_ENTITIES_PER_NODE) return false;
        reader_gids[reader_count++] = Gid::from_guid(g);
        return true;
    }
};

// ============================================================
// ParticipantEntitiesInfo — mirrors rmw_dds_common/msg/ParticipantEntitiesInfo.msg
// ============================================================

struct ParticipantEntitiesInfo {
    Gid              participant_gid = {};
    NodeEntitiesInfo nodes[MILKROS_MAX_NODES] = {};
    uint8_t          node_count = 0u;

    /// Register a new node.  Returns its index (0-based) for use with
    /// NodeEntitiesInfo::add_writer/add_reader, or -1 if the pool is full.
    int add_node(const char* name, const char* ns) noexcept {
        if (node_count >= MILKROS_MAX_NODES) return -1;
        const int idx = static_cast<int>(node_count++);
        NodeEntitiesInfo& n = nodes[idx];
        n = NodeEntitiesInfo{};
        if (name) {
            strncpy(n.node_name, name, sizeof(n.node_name) - 1u);
            n.node_name[sizeof(n.node_name) - 1u] = '\0';
        }
        if (ns) {
            strncpy(n.node_namespace, ns, sizeof(n.node_namespace) - 1u);
            n.node_namespace[sizeof(n.node_namespace) - 1u] = '\0';
        }
        return idx;
    }
};

// ============================================================
// CDR Little-Endian encoder
//
// Encodes a ParticipantEntitiesInfo into the supplied buffer.
//
// Returns the number of bytes written, or 0 if cap is insufficient.
// ============================================================

inline uint32_t encode_participant_entities_info(
    const ParticipantEntitiesInfo& info,
    rtps::Octet* buf, uint32_t cap) noexcept
{
    uint32_t pos = 0u;

    // ---- helpers ----

    // Write raw bytes without alignment.
    const auto write_bytes = [&](const void* src, uint32_t n) -> bool {
        if (pos + n > cap) return false;
        memcpy(buf + pos, src, n);
        pos += n;
        return true;
    };

    // Align pos to a 4-byte boundary (from buf start), inserting zero padding.
    const auto align4 = [&]() -> bool {
        const uint32_t rem = pos % 4u;
        if (rem == 0u) return true;
        const uint32_t pad = 4u - rem;
        if (pos + pad > cap) return false;
        memset(buf + pos, 0, pad);
        pos += pad;
        return true;
    };

    // Write a uint32_t in little-endian, 4-byte aligned.
    const auto write_u32 = [&](uint32_t v) -> bool {
        if (!align4()) return false;
        if (pos + 4u > cap) return false;
        buf[pos + 0u] = static_cast<rtps::Octet>( v        & 0xFFu);
        buf[pos + 1u] = static_cast<rtps::Octet>((v >>  8u) & 0xFFu);
        buf[pos + 2u] = static_cast<rtps::Octet>((v >> 16u) & 0xFFu);
        buf[pos + 3u] = static_cast<rtps::Octet>((v >> 24u) & 0xFFu);
        pos += 4u;
        return true;
    };

    // Write a CDR string: uint32 (len+1) + chars + '\0' + pad-to-4.
    const auto write_str = [&](const char* s) -> bool {
        const uint32_t len = s ? static_cast<uint32_t>(strlen(s)) : 0u;
        if (!write_u32(len + 1u)) return false;
        if (len > 0u) {
            if (!write_bytes(s, len)) return false;
        }
        if (pos >= cap) return false;
        buf[pos++] = '\0';
        // Pad to 4-byte boundary for the next field.
        const uint32_t rem = pos % 4u;
        if (rem != 0u) {
            const uint32_t pad = 4u - rem;
            if (pos + pad > cap) return false;
            memset(buf + pos, 0, pad);
            pos += pad;
        }
        return true;
    };

    // ---- RepresentationHeader: CDR_LE = {0x00, 0x01, 0x00, 0x00} ----
    static const rtps::Octet kCdrLeHeader[4] = {0x00u, 0x01u, 0x00u, 0x00u};
    if (!write_bytes(kCdrLeHeader, 4u)) return 0u;

    // ---- participant_gid (fixed 24-byte array, no alignment needed) ----
    if (!write_bytes(info.participant_gid.data, 24u)) return 0u;

    // ---- node_entities_info_seq length ----
    if (!write_u32(info.node_count)) return 0u;

    // ---- nodes ----
    for (uint8_t i = 0u; i < info.node_count; ++i) {
        const NodeEntitiesInfo& n = info.nodes[i];

        if (!write_str(n.node_namespace)) return 0u;
        if (!write_str(n.node_name))      return 0u;

        // writer_gid_seq
        if (!write_u32(n.writer_count)) return 0u;
        for (uint8_t j = 0u; j < n.writer_count; ++j) {
            if (!write_bytes(n.writer_gids[j].data, 24u)) return 0u;
        }

        // reader_gid_seq
        if (!write_u32(n.reader_count)) return 0u;
        for (uint8_t j = 0u; j < n.reader_count; ++j) {
            if (!write_bytes(n.reader_gids[j].data, 24u)) return 0u;
        }
    }

    return pos;
}

} // namespace milkros
