/**
 * @file local_node_entities.cpp
 * @brief LocalNodeEntities: the local node table and its
 *        ParticipantEntitiesInfo encoder (local_node_entities.hpp).
 */

#include "rmw_rosette/local_node_entities.hpp"

#include "rmw_rosette/cdr_stream.hpp"
#include "rmw_rosette/types.hpp"

#include <cstring>

namespace rtps = rosette::rtps;

namespace rmw_rosette {

namespace {

/**
 * @brief Writes an RTPS GUID as an `rmw_dds_common/msg/Gid`.
 *
 * The 12-octet prefix, the 4-octet EntityId, then zero padding to kGidSize,
 * the same layout guid_to_rmw_gid() gives rmw_gid_t::data.
 *
 * @param[in,out] out  Stream positioned at the Gid.
 * @param[in]     guid GUID to write.
 */
void put_gid(CdrWriter& out, const rtps::Guid& guid) noexcept {
    uint8_t gid[kGidSize] = {};
    std::memcpy(gid, guid.prefix.value, sizeof(guid.prefix.value));
    std::memcpy(gid + sizeof(guid.prefix.value), guid.entityId.entityKey,
                sizeof(guid.entityId.entityKey));
    gid[sizeof(guid.prefix.value) + sizeof(guid.entityId.entityKey)] = guid.entityId.entityKind;
    out.put_bytes(gid, sizeof(gid));
}

/**
 * @brief Writes a bounded CDR string: length including the NUL, the
 *        characters, then the NUL.
 *
 * @param[in,out] out Stream positioned at the string.
 * @param[in]     s   NUL-terminated text; cut to kNodeEntitiesMaxNameLength characters.
 */
void put_bounded_string(CdrWriter& out, const char* s) noexcept {
    size_t len = std::strlen(s);
    if (len > kNodeEntitiesMaxNameLength) len = kNodeEntitiesMaxNameLength;
    out.put<uint32_t>(static_cast<uint32_t>(len + 1));
    out.put_bytes(s, len);
    const uint8_t nul = 0;
    out.put_bytes(&nul, 1);
}

}  // namespace

bool LocalNodeEntities::add_node(const Node& node) noexcept {
    if (node_count_ >= RMW_ROSETTE_MAX_NODES) return false;
    nodes_[node_count_++] = &node;
    return true;
}

void LocalNodeEntities::remove_node(const Node& node) noexcept {
    // Compact both tables in place so encode() keeps insertion order.
    uint32_t kept = 0;
    for (uint32_t i = 0; i < endpoint_count_; ++i) {
        if (endpoints_[i].node != &node) endpoints_[kept++] = endpoints_[i];
    }
    endpoint_count_ = kept;

    kept = 0;
    for (uint32_t i = 0; i < node_count_; ++i) {
        if (nodes_[i] != &node) nodes_[kept++] = nodes_[i];
    }
    node_count_ = kept;
}

bool LocalNodeEntities::associate(const Node& node, const rtps::Guid& guid,
                                  bool is_reader) noexcept {
    bool known = false;
    for (uint32_t i = 0; i < node_count_; ++i) known = known || nodes_[i] == &node;
    if (!known || endpoint_count_ >= kMaxEndpoints) return false;
    endpoints_[endpoint_count_++] = EndpointEntry{&node, guid, is_reader};
    return true;
}

void LocalNodeEntities::dissociate(const rtps::Guid& guid) noexcept {
    uint32_t kept = 0;
    for (uint32_t i = 0; i < endpoint_count_; ++i) {
        if (!(endpoints_[i].guid == guid)) endpoints_[kept++] = endpoints_[i];
    }
    endpoint_count_ = kept;
}

uint32_t LocalNodeEntities::encode(const rtps::Guid& participant, uint8_t* out,
                                   uint32_t cap) const noexcept {
    CdrWriter cdr(out, cap);
    write(participant, cdr);
    return cdr.ok() ? static_cast<uint32_t>(cdr.length()) : 0u;
}

uint32_t LocalNodeEntities::encoded_size(const rtps::Guid& participant) const noexcept {
    CdrWriter cdr(nullptr, SIZE_MAX);
    write(participant, cdr);
    return static_cast<uint32_t>(cdr.length());
}

void LocalNodeEntities::write(const rtps::Guid& participant, CdrWriter& cdr) const noexcept {
    put_gid(cdr, participant);
    cdr.put<uint32_t>(node_count_);
    for (uint32_t n = 0; n < node_count_; ++n) {
        const Node* node = nodes_[n];
        put_bounded_string(cdr, node->namespace_);
        put_bounded_string(cdr, node->name);
        // reader_gid_seq, then writer_gid_seq: count first, then each GID.
        for (const bool readers : {true, false}) {
            uint32_t count = 0;
            for (uint32_t i = 0; i < endpoint_count_; ++i) {
                if (endpoints_[i].node == node && endpoints_[i].is_reader == readers) ++count;
            }
            cdr.put<uint32_t>(count);
            for (uint32_t i = 0; i < endpoint_count_; ++i) {
                if (endpoints_[i].node == node && endpoints_[i].is_reader == readers) {
                    put_gid(cdr, endpoints_[i].guid);
                }
            }
        }
    }
}

}  // namespace rmw_rosette
