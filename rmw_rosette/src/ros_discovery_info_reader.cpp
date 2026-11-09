// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file ros_discovery_info_reader.cpp
 * @brief RosDiscoveryInfoReader and its ParticipantEntitiesInfo decoder.
 */

#include "rmw_rosette/ros_discovery_info_reader.hpp"

#include <cstring>
#include <new>
#include <string>
#include <utility>

#include "rmw_rosette/cdr_stream.hpp"
#include "rmw_rosette/local_node_entities.hpp"
#include "rmw_rosette/message_codec.hpp"
#include "rmw_rosette/ros_discovery_info_writer.hpp"

namespace rtps = rosette::rtps;

namespace rmw_rosette {
namespace {

/**
 * @brief Reads a CDR string whose length comes from a remote peer.
 *
 * @param[in,out] cdr Stream positioned at the string; failed on a malformed one.
 * @return The characters without the terminator; empty when the stream failed.
 */
std::string get_string(CdrReader& cdr) {
    const uint32_t length = cdr.get<uint32_t>();
    if (!cdr.ok() || length == 0u) {
        cdr.fail();
        return {};
    }

    const uint8_t* text = cdr.take_view(length);
    if (text == nullptr || text[length - 1u] != 0u
        || std::memchr(text, 0, length - 1u) != nullptr) {
        cdr.fail();
        return {};
    }

    return std::string(reinterpret_cast<const char*>(text), length - 1u);
}

/// @brief Consumes one sequence of GIDs, failing the stream when it overruns.
void skip_gid_sequence(CdrReader& cdr) {
    const uint32_t count = cdr.get<uint32_t>();
    if (!cdr.ok() || count > cdr.remaining() / kGidSize) {
        cdr.fail();
        return;
    }

    (void)cdr.take_view(count * size_t{kGidSize});
}

}  // namespace

bool decode_participant_node_names(const uint8_t* payload, size_t size,
                                   std::vector<NodeName>& nodes) {
    // Stock rmw implementations serialize this topic as little-endian plain CDR.
    if (payload == nullptr || size < kRtpsEncapsulationHeaderSize || payload[0] != 0x00u
        || payload[1] != 0x01u) {
        return false;
    }

    CdrReader cdr(payload + kRtpsEncapsulationHeaderSize, size - kRtpsEncapsulationHeaderSize);
    (void)cdr.take_view(kGidSize);  // The participant, known from the writer's GUID.

    const uint32_t        count = cdr.get<uint32_t>();
    std::vector<NodeName> decoded;
    for (uint32_t i = 0; i < count && cdr.ok(); ++i) {
        NodeName node;
        node.namespace_ = get_string(cdr);
        node.name = get_string(cdr);
        skip_gid_sequence(cdr);  // reader_gid_seq
        skip_gid_sequence(cdr);  // writer_gid_seq
        if (cdr.ok()) {
            decoded.push_back(std::move(node));
        }
    }

    if (!cdr.ok()) {
        return false;
    }

    nodes = std::move(decoded);
    return true;
}

bool RosDiscoveryInfoReader::create(rtps::RTPSParticipant& participant,
                                    GraphCache&            cache) noexcept {
    cache_ = &cache;

    rtps::ReaderConfig cfg{};
    cfg.topicName.assign(kRosDiscoveryInfoTopic);
    cfg.typeName.assign(kParticipantEntitiesInfoType);
    cfg.reliability = rtps::ReliabilityKind::RELIABLE;
    cfg.durability = rtps::DurabilityKind::TRANSIENT_LOCAL;
    cfg.minMatchedWriters = static_cast<uint8_t>(RTPS_MAX_PEERS);
    cfg.maxMatchedWriters = static_cast<uint8_t>(RTPS_MAX_PEERS);
    return participant.create_reader(cfg, &RosDiscoveryInfoReader::on_change, this) != nullptr;
}

void RosDiscoveryInfoReader::on_change(const rtps::CacheChange& change, void* self) noexcept {
    if (change.kind != rtps::ChangeKind::ALIVE || !change.payload.valid()) {
        return;
    }

    auto& reader = *static_cast<RosDiscoveryInfoReader*>(self);

    // A node that cannot be stored stays out of the listing; discovery goes on.
    try {
        std::vector<NodeName> nodes;
        if (!decode_participant_node_names(change.payload.data, change.payload.size, nodes)) {
            return;
        }

        (void)reader.cache_->set_participant_nodes(change.writerGuid.prefix, std::move(nodes));
    } catch (const std::bad_alloc&) {
        return;
    }
}

}  // namespace rmw_rosette
