// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file cyclone_request_header.cpp
 * @brief Reads, writes and packs the rmw_cyclonedds_cpp service request header.
 */

#include "rmw_rosette/compat/cyclone_request_header.hpp"

#include "rmw_rosette/message_codec.hpp"

#include <cstring>

namespace rtps = rosette::rtps;

namespace rmw_rosette {

namespace {

uint64_t load_le64(const uint8_t* in) noexcept {
    uint64_t value = 0;
    for (int i = 7; i >= 0; --i) {
        value = (value << 8) | in[i];
    }

    return value;
}

void store_le64(uint64_t value, uint8_t* out) noexcept {
    for (int i = 0; i < 8; ++i) {
        out[i] = static_cast<uint8_t>(value >> (8 * i));
    }
}

}  // namespace

bool is_cyclone_peer(const rtps::RTPSParticipant& participant,
                     const rtps::GuidPrefix&      prefix) noexcept {
    return participant.peer_vendor_id(prefix) == rtps::VENDORID_ECLIPSE_CYCLONE;
}

uint64_t request_writer_key(const rtps::Guid& request_writer) noexcept {
    RequestWriterGuid octets{};
    guid_to_request_writer(request_writer, octets);
    uint64_t hash = 0xcbf29ce484222325ull;
    for (int i = 0; i < 16; ++i) {
        hash ^= static_cast<uint8_t>(octets[i]);
        hash *= 0x100000001b3ull;
    }

    return hash;
}

bool read_cyclone_request_header(const uint8_t* payload, uint32_t size,
                                 CycloneRequestHeader& out) noexcept {
    // Only CDR_LE bodies are decoded (deserialize_message()), so only a
    // little-endian header is read.
    if (size < kRtpsEncapsulationHeaderSize + kCycloneRequestHeaderSize || payload[0] != 0x00
        || payload[1] != 0x01) {
        return false;
    }

    const uint8_t* body = payload + kRtpsEncapsulationHeaderSize;
    out.client_id = load_le64(body);
    out.sequence = static_cast<int64_t>(load_le64(body + 8));
    return true;
}

void write_cyclone_request_header(const CycloneRequestHeader& header, uint8_t* body) noexcept {
    store_le64(header.client_id, body);
    store_le64(static_cast<uint64_t>(header.sequence), body + 8);
}

void cyclone_request_to_request_id(const CycloneRequestHeader& header,
                                   const rtps::Guid&           request_writer,
                                   rmw_request_id_t&           request_id) noexcept {
    const uint64_t writer_key = request_writer_key(request_writer);
    std::memset(request_id.writer_guid, 0, sizeof(request_id.writer_guid));
    std::memcpy(request_id.writer_guid, &header.client_id, 8);
    std::memcpy(request_id.writer_guid + 8, &writer_key, 8);
    request_id.sequence_number = header.sequence;
}

bool find_cyclone_requester(const rtps::StatefulReader& request_reader,
                            const rmw_request_id_t&     request_id,
                            CycloneRequestHeader&       header) noexcept {
    uint64_t writer_key = 0;
    std::memcpy(&writer_key, request_id.writer_guid + 8, 8);
    const bool matched = request_reader.any_matched_writer([writer_key](const rtps::Guid& writer) {
        return request_writer_key(writer) == writer_key;
    });
    if (!matched) {
        return false;
    }

    std::memcpy(&header.client_id, request_id.writer_guid, 8);
    header.sequence = request_id.sequence_number;
    return true;
}

}  // namespace rmw_rosette
