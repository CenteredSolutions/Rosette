// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file rmw_rosette/compat/cyclone_request_header.hpp
 * @brief The request header rmw_cyclonedds_cpp puts inside service payloads.
 *
 * ROS 2 services have two wire dialects. rmw_fastrtps_cpp, and rosette with
 * every other peer, correlate a reply with its request through the DDS-RPC
 * related sample identity in inline QoS. rmw_cyclonedds_cpp carries no
 * inline QoS for this. It puts a 16-octet header at the front of the CDR body
 * of every request and reply instead:
 *
 * @verbatim
 *  0               4               8                              24
 *  +---------------+---------------+------------------------------+---
 *  | encapsulation |   client_id   |  sequence    |  ROS message (XCDR1,
 *  | 00 01 00 00   |  uint64, LE   |  int64, LE   |  aligned from octet 4)
 *  +---------------+---------------+------------------------------+---
 *                  |<-------- CDR body; alignment origin -------->
 * @endverbatim
 *
 * The client chooses `client_id` (Cyclone uses its request writer's instance
 * handle) and `sequence`; the server copies both into the reply, and a
 * client keeps only replies whose `client_id` is its own. The header is 16
 * octets, a multiple of the largest CDR alignment, so the message after it
 * encodes exactly as it would at the start of the body.
 *
 * Rosette picks the dialect per peer from the vendor ID the peer's
 * participant announced in SPDP: Eclipse Cyclone DDS speaks this one, every
 * other vendor the DDS-RPC one. A rosette client uses
 * request_writer_key() of its request writer as its `client_id`, and the
 * request writer's next sequence number as `sequence`, so `sequence_id`
 * means the same in both dialects.
 *
 * rcl hands a request's `rmw_request_id_t` back to rmw_send_response()
 * unchanged, so a Cyclone request's identity is packed into it the way
 * rmw_cyclonedds_cpp packs its own:
 *
 * | `writer_guid` octets | DDS-RPC request          | Cyclone request                 |
 * |----------------------|--------------------------|---------------------------------|
 * | 0-7                  | request writer GUID 0-7  | `client_id`                     |
 * | 8-15                 | request writer GUID 8-15 | request_writer_key() of writer  |
 * | `sequence_number`    | request DATA's SN        | header `sequence`               |
 *
 * References:
 * - rmw_cyclonedds_cpp 1.3.5 (Humble) and 2.2 (Jazzy): serdata.hpp
 *   (cdds_request_header_t), Serialization.cpp (serialize_top_level),
 *   rmw_node.cpp (rmw_send_request, rmw_take_response_request,
 *   rmw_send_response)
 * - OMG DDSI-RTPS 2.5 §8.3.3.1.3, §9.3.1.5 (vendor ID)
 */

#pragma once

#include "rmw_rosette/guid.hpp"

#include "rosette/rtps/participant.hpp"
#include "rosette/rtps/stateful_reader.hpp"
#include "rosette/rtps/types.hpp"

#include <rmw/types.h>

#include <cstdint>

namespace rmw_rosette {

/// Octets the header occupies at the front of the CDR body.
constexpr uint32_t kCycloneRequestHeaderSize = 16;

/// One Cyclone request header, as the diagram above lays it out.
struct CycloneRequestHeader {
    uint64_t client_id = 0;
    int64_t  sequence = 0;
};

/**
 * @brief Whether a remote participant speaks the Cyclone service dialect.
 *
 * @param[in] participant This context's participant.
 * @param[in] prefix      GUID prefix of the remote endpoint's participant.
 * @return `true` when the participant announced VENDORID_ECLIPSE_CYCLONE in SPDP.
 */
bool is_cyclone_peer(const rosette::rtps::RTPSParticipant& participant,
                     const rosette::rtps::GuidPrefix&      prefix) noexcept;

/**
 * @brief A 64-bit key naming one request writer: FNV-1a over its 16 GUID octets.
 *
 * A rosette client sends it as its `client_id`; a Cyclone request's
 * `rmw_request_id_t` carries it to name the request writer, as in the
 * table above.
 */
uint64_t request_writer_key(const rosette::rtps::Guid& request_writer) noexcept;

/**
 * @brief Reads the header from a serialized payload.
 *
 * @param[in]  payload Payload including the encapsulation header.
 * @param[in]  size    Payload length in octets.
 * @param[out] out     The header; written only on success.
 * @return `false` when the payload is not CDR_LE or is too short for the header.
 */
bool read_cyclone_request_header(const uint8_t* payload, uint32_t size,
                                 CycloneRequestHeader& out) noexcept;

/**
 * @brief Writes the header, little-endian, at the start of a CDR body.
 *
 * @param[in]  header Header to write.
 * @param[out] body   First octet of the CDR body; receives kCycloneRequestHeaderSize octets.
 */
void write_cyclone_request_header(const CycloneRequestHeader& header, uint8_t* body) noexcept;

/**
 * @brief Reports a Cyclone request's identity to rcl, packed as in the table above.
 *
 * @param[in]  header         The request's header.
 * @param[in]  request_writer GUID of the writer that sent the request.
 * @param[out] request_id     Receives the packed identity.
 */
void cyclone_request_to_request_id(const CycloneRequestHeader& header,
                                   const rosette::rtps::Guid&  request_writer,
                                   rmw_request_id_t&           request_id) noexcept;

/**
 * @brief Finds the matched Cyclone request writer an `rmw_request_id_t` names.
 *
 * rmw_send_response() calls this for an identity that names no matched
 * writer by GUID.
 *
 * @param[in]  request_reader The service's request reader.
 * @param[in]  request_id     Identity cyclone_request_to_request_id() reported.
 * @param[out] header         The header to send with the reply; written when found.
 * @return `true` when a writer matched to `request_reader` has the packed
 *   request_writer_key(); `false` when that client has left (or the identity
 *   was never a Cyclone one).
 *
 * Verification:
 * - Correct means: for an identity reported by cyclone_request_to_request_id(),
 *   returns the original header while its writer stays matched, and `false`
 *   after matched_writer_remove() of that writer.
 * - Covered by: rmw_rosette/test/test_cyclone_request_header.cpp.
 */
bool find_cyclone_requester(const rosette::rtps::StatefulReader& request_reader,
                            const rmw_request_id_t&              request_id,
                            CycloneRequestHeader&                header) noexcept;

}  // namespace rmw_rosette
