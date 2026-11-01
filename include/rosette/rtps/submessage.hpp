// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file rtps/submessage.hpp
 * @brief RTPS submessage wire-layout structs and the RTPS message header.
 *
 * All struct layouts, field names, and constant values are taken directly
 * from the DDSI-RTPS 2.5 wire protocol specification
 * (OMG formal/2022-04-01, §9.4) so that each struct can be verified
 * field-by-field against the corresponding table in the PDF.
 *
 * Design
 * ------
 *  - All structs are Plain Old Data (POD); fields are in wire order.
 *  - Endianness is handled by MessageBuilder (TX) and MessageParser (RX);
 *    these structs always carry host-endian values once parsed.
 *  - No padding surprises: every multi-byte field sits on its natural
 *    alignment; static_asserts verify the sizes that the spec mandates.
 *  - The SubmessageKind enum and SubmessageFlags helpers are defined here
 *    so parser and builder share the same constants.
 *
 * Spec references (all OMG formal/2022-04-01)
 * --------------------------------------------
 *   RTPS message header        — §9.4.1,   Table 9.1,  p. 193
 *   Submessage header          — §9.4.5.1, Table 9.6,  p. 200
 *   SubmessageKind             — §9.4.5.1, Table 9.7,  p. 200
 *   DATA                       — §9.4.5.3, Table 9.14, p. 204
 *   DATA_FRAG                  — §9.4.5.4, Table 9.16, p. 208
 *   HEARTBEAT                  — §9.4.5.5, Table 9.18, p. 211
 *   HEARTBEAT_FRAG             — §9.4.5.6, Table 9.20, p. 214
 *   ACKNACK                    — §9.4.5.7, Table 9.22, p. 216
 *   NACK_FRAG                  — §9.4.5.8, Table 9.24, p. 218
 *   GAP                        — §9.4.5.11,Table 9.29, p. 224
 *   INFO_TS                    — §9.4.5.9, Table 9.26, p. 221
 *   INFO_DST                   — §9.4.5.13,Table 9.33, p. 228
 */

#pragma once

#include "types.hpp"

namespace rosette::rtps {

// ============================================================
// §9.4.1  RTPS Message Header
//
// "The Header identifies the message as belonging to the RTPS protocol.
//  The Header contains the fields protocol, version, vendorId and
//  guidPrefix."  — §9.4.1, Table 9.1, p. 193
//
// Wire layout (28 bytes):
//   octet[4]   protocol   = {'R','T','P','S'}
//   octet[2]   version    = {major, minor}
//   octet[2]   vendorId
//   octet[12]  guidPrefix
// ============================================================

struct RTPSMessageHeader {
    Octet           protocol[4] = {'R','T','P','S'};
    ProtocolVersion version     = PROTOCOLVERSION_2_5;
    VendorId        vendorId    = VENDORID_THIS_IMPL;
    GuidPrefix      guidPrefix  = {};
};
static_assert(sizeof(RTPSMessageHeader) == 20,
    "RTPSMessageHeader must be 20 bytes — §9.4.1, Table 9.1");

inline bool rtps_header_valid(const RTPSMessageHeader& h) noexcept {
    return h.protocol[0] == 'R' && h.protocol[1] == 'T' &&
           h.protocol[2] == 'P' && h.protocol[3] == 'S';
}

// ============================================================
// §9.4.5.1  SubmessageKind
//
// "Each Submessage has a SubmessageKind."
//  — §9.4.5.1, Table 9.7, p. 200
// ============================================================

enum class SubmessageKind : Octet {
    PAD            = 0x01,
    ACKNACK        = 0x06,
    HEARTBEAT      = 0x07,
    GAP            = 0x08,
    INFO_TS        = 0x09,
    INFO_SRC       = 0x0C,
    INFO_REPLY_IP4 = 0x0D,
    INFO_DST       = 0x0E,
    INFO_REPLY     = 0x0F,
    NACK_FRAG      = 0x12,
    HEARTBEAT_FRAG = 0x13,
    DATA           = 0x15,
    DATA_FRAG      = 0x16,
};

// ============================================================
// §9.4.5.1  SubmessageHeader
//
// "Every Submessage starts with a SubmessageHeader."
//  — §9.4.5.1, Table 9.6, p. 200
//
// Wire layout (4 bytes):
//   octet      submessageId
//   octet      flags          bit 0 = Endianness (E), rest submessage-specific
//   uint16     submessageLength (bytes following this header; 0 = last)
// ============================================================

struct SubmessageHeader {
    SubmessageKind submessageId     = SubmessageKind::PAD;
    Octet          flags            = 0;
    uint16_t       submessageLength = 0;
};
static_assert(sizeof(SubmessageHeader) == 4,
    "SubmessageHeader must be 4 bytes — §9.4.5.1, Table 9.6");

/// Endianness flag — bit 0 of flags.  Set ↔ little-endian.  — §9.4.5.1
inline constexpr Octet FLAG_E = 0x01u;

/// Helper: true when the submessage is encoded little-endian.
inline bool smsg_is_little_endian(const SubmessageHeader& h) noexcept {
    return (h.flags & FLAG_E) != 0;
}

// ============================================================
// §9.4.5.3  DATA Submessage
//
// "The DATA Submessage is sent from an RTPS Writer to an RTPS Reader
//  and contains information regarding changes to a data-object."
//  — §9.4.5.3, p. 204, Table 9.14
//
// Flags (in addition to E):
//   bit 1  Q — inlineQos present
//   bit 2  D — serializedPayload present (dataFlag)
//   bit 3  K — key-hash present (keyFlag)
//
// Fixed header (after SubmessageHeader, before optional fields):
//   uint16  extraFlags        = 0 (reserved)
//   uint16  octetsToInlineQos = 16 (distance from here to inlineQos/payload)
//   EntityId readerId
//   EntityId writerId
//   SequenceNumber writerSN
//
// Followed by: [InlineQos if Q], SerializedPayload if D|K.
// ============================================================

namespace data_flags {
    inline constexpr Octet Q = 0x02u;   ///< inlineQos present.
    inline constexpr Octet D = 0x04u;   ///< data payload present.
    inline constexpr Octet K = 0x08u;   ///< key payload present.
}

/// Fixed fields of a DATA submessage (excludes SubmessageHeader).
/// — §9.4.5.3, Table 9.14, p. 204
struct DataSubmessageFixed {
    uint16_t       extraFlags        = 0;
    uint16_t       octetsToInlineQos = 16;  ///< Always 16 when no inlineQos key.
    EntityId       readerId          = {};
    EntityId       writerId          = {};
    SequenceNumber writerSN          = {};
    // InlineQos and SerializedPayload follow at variable offset.
};
static_assert(sizeof(DataSubmessageFixed) == 20,
    "DataSubmessageFixed must be 20 bytes — §9.4.5.3, Table 9.14");

// ============================================================
// §9.4.5.4  DATA_FRAG Submessage
//
// "The DATA_FRAG Submessage extends DATA to allow the serializedPayload
//  to be fragmented and sent as multiple DATA_FRAG Submessages."
//  — §9.4.5.4, p. 208, Table 9.16
//
// Flags (in addition to E):
//   bit 1  Q — inlineQos present
//   bit 2  H — keyhash present
//
// Fixed fields after SubmessageHeader:
//   uint16  extraFlags
//   uint16  octetsToInlineQos
//   EntityId readerId
//   EntityId writerId
//   SequenceNumber writerSN
//   uint32  fragmentStartingNum   (1-based)
//   uint16  fragmentsInSubmessage
//   uint16  fragmentSize          (bytes per fragment, except possibly last)
//   uint32  dataSize              (total serializedPayload bytes)
// ============================================================

namespace data_frag_flags {
    inline constexpr Octet Q = 0x02u;   ///< inlineQos present.
    inline constexpr Octet H = 0x04u;   ///< keyhash present.
}

/// Fixed fields of a DATA_FRAG submessage.
/// — §9.4.5.4, Table 9.16, p. 208
struct DataFragSubmessageFixed {
    uint16_t       extraFlags            = 0;
    uint16_t       octetsToInlineQos     = 28;  ///< Without inlineQos.
    EntityId       readerId              = {};
    EntityId       writerId              = {};
    SequenceNumber writerSN              = {};
    uint32_t       fragmentStartingNum   = 0;   ///< 1-based fragment index.
    uint16_t       fragmentsInSubmessage = 0;
    uint16_t       fragmentSize          = 0;
    uint32_t       dataSize              = 0;   ///< Total payload bytes.
};
static_assert(sizeof(DataFragSubmessageFixed) == 32,
    "DataFragSubmessageFixed must be 32 bytes — §9.4.5.4, Table 9.16");

// ============================================================
// §9.4.5.5  HEARTBEAT Submessage
//
// "A Heartbeat message is sent from an RTPS Writer to an RTPS Reader."
//  — §9.4.5.5, p. 211, Table 9.18
//
// Flags (in addition to E):
//   bit 1  F — FinalFlag: if set, reader need not respond with ACKNACK.
//   bit 2  L — LivelinessFlag: sent by participant to assert liveliness.
//
// Wire layout (fixed, 28 bytes after SubmessageHeader):
//   EntityId       readerId
//   EntityId       writerId
//   SequenceNumber firstSN        (lowest SN available in Writer cache)
//   SequenceNumber lastSN         (highest SN available)
//   int32          count          (monotonically increasing per writer/reader pair)
// ============================================================

namespace hb_flags {
    inline constexpr Octet F = 0x02u;   ///< FinalFlag.
    inline constexpr Octet L = 0x04u;   ///< LivelinessFlag.
}

/// Full (fixed-size) HEARTBEAT submessage payload.
/// — §9.4.5.5, Table 9.18, p. 211
struct HeartbeatSubmessage {
    EntityId       readerId = {};
    EntityId       writerId = {};
    SequenceNumber firstSN  = {};
    SequenceNumber lastSN   = {};
    Long           count    = 0;
};
static_assert(sizeof(HeartbeatSubmessage) == 28,
    "HeartbeatSubmessage must be 28 bytes — §9.4.5.5, Table 9.18");

// ============================================================
// §9.4.5.6  HEARTBEAT_FRAG Submessage
//
// "The HEARTBEAT_FRAG Submessage is used to indicate which fragments
//  of a DATA_FRAG are available."
//  — §9.4.5.6, p. 214, Table 9.20
//
// Wire layout (fixed, 24 bytes after SubmessageHeader):
//   EntityId       readerId
//   EntityId       writerId
//   SequenceNumber writerSN
//   uint32         lastFragmentNum   (highest fragment delivered so far)
//   int32          count
// ============================================================

struct HeartbeatFragSubmessage {
    EntityId       readerId        = {};
    EntityId       writerId        = {};
    SequenceNumber writerSN        = {};
    uint32_t       lastFragmentNum = 0;
    Long           count           = 0;
};
static_assert(sizeof(HeartbeatFragSubmessage) == 24,
    "HeartbeatFragSubmessage must be 24 bytes — §9.4.5.6, Table 9.20");

// ============================================================
// §9.4.5.7  ACKNACK Submessage
//
// "The ACKNACK Submessage provides information on the state of a
//  Reader to a matched Writer."
//  — §9.4.5.7, p. 216, Table 9.22
//
// Flags (in addition to E):
//   bit 1  F — FinalFlag: if set, writer need not respond with more data.
//
// Wire layout (variable):
//   EntityId         readerId
//   EntityId         writerId
//   SequenceNumberSet readerSNState  (variable length: base + numBits + bitmap)
//   int32            count
// ============================================================

namespace acknack_flags {
    inline constexpr Octet F = 0x02u;   ///< FinalFlag.
}

/// Fixed prefix of an ACKNACK (the variable SequenceNumberSet follows).
/// — §9.4.5.7, Table 9.22, p. 216
struct AckNackSubmessageFixed {
    EntityId readerId = {};
    EntityId writerId = {};
    // SequenceNumberSet readerSNState follows (variable).
    // Long count follows after the set.
};
static_assert(sizeof(AckNackSubmessageFixed) == 8,
    "AckNackSubmessageFixed must be 8 bytes — §9.4.5.7, Table 9.22");

// ============================================================
// §9.4.5.8  NACK_FRAG Submessage
//
// "The NACK_FRAG Submessage is used to indicate that a Reader is
//  missing certain fragments."
//  — §9.4.5.8, p. 218, Table 9.24
//
// Wire layout (variable):
//   EntityId         readerId
//   EntityId         writerId
//   SequenceNumber   writerSN
//   FragmentNumberSet fragmentNumberState
//   int32            count
// ============================================================

/// FragmentNumberSet — same bitmap structure as SequenceNumberSet but
/// for 32-bit fragment indices.  — §9.4.2.8, p. 199
struct FragmentNumberSet {
    static constexpr int kMaxBits = 256;
    static constexpr int kWords   = kMaxBits / 32;

    uint32_t bitmapBase = 0;            ///< 1-based lowest fragment number.
    uint32_t numBits    = 0;
    uint32_t bitmap[kWords] = {};

    bool set(uint32_t offset) noexcept {
        if (offset >= kMaxBits) return false;
        bitmap[offset / 32] |= (1u << (31u - (offset % 32u)));
        if (offset + 1u > numBits) numBits = offset + 1u;
        return true;
    }
    bool test(uint32_t offset) const noexcept {
        if (offset >= numBits) return false;
        return (bitmap[offset / 32] >> (31u - (offset % 32u))) & 1u;
    }
    bool empty() const noexcept { return numBits == 0; }
    void clear() noexcept {
        numBits = 0;
        memset(bitmap, 0, sizeof(bitmap));
    }
    bool add(uint32_t fragNum) noexcept {
        if (numBits == 0) bitmapBase = fragNum;
        if (fragNum < bitmapBase) return false;
        return set(fragNum - bitmapBase);
    }

    template<typename Fn>
    void for_each(Fn&& fn) const noexcept {
        for (uint32_t i = 0; i < numBits; ++i) {
            if (test(i)) fn(bitmapBase + i);
        }
    }
};

/// Fixed prefix of a NACK_FRAG (FragmentNumberSet and count follow).
/// — §9.4.5.8, Table 9.24, p. 218
struct NackFragSubmessageFixed {
    EntityId       readerId = {};
    EntityId       writerId = {};
    SequenceNumber writerSN = {};
    // FragmentNumberSet fragmentNumberState follows (variable).
    // Long count follows.
};
static_assert(sizeof(NackFragSubmessageFixed) == 16,
    "NackFragSubmessageFixed must be 16 bytes — §9.4.5.8, Table 9.24");

// ============================================================
// §9.4.5.11  GAP Submessage
//
// "The GAP Submessage indicates to the Reader that a range of
//  sequence numbers is no longer relevant."
//  — §9.4.5.11, p. 224, Table 9.29
//
// Wire layout (variable):
//   EntityId          readerId
//   EntityId          writerId
//   SequenceNumber    gapStart      (first irrelevant SN)
//   SequenceNumberSet gapList       (additional irrelevant SNs)
// ============================================================

/// Fixed prefix of a GAP submessage.
/// — §9.4.5.11, Table 9.29, p. 224
struct GapSubmessageFixed {
    EntityId       readerId = {};
    EntityId       writerId = {};
    SequenceNumber gapStart = {};
    // SequenceNumberSet gapList follows (variable).
};
static_assert(sizeof(GapSubmessageFixed) == 16,
    "GapSubmessageFixed must be 16 bytes — §9.4.5.11, Table 9.29");

// ============================================================
// §9.4.5.9  INFO_TS Submessage
//
// "The INFO_TS Submessage provides the source timestamp for
//  subsequent Submessages."
//  — §9.4.5.9, p. 221, Table 9.26
//
// Flags (in addition to E):
//   bit 1  I — InvalidateFlag: if set, timestamp field is absent.
//
// Wire layout (8 bytes when I == 0):
//   Time_t  timestamp
// ============================================================

namespace info_ts_flags {
    inline constexpr Octet I = 0x02u;   ///< InvalidateFlag.
}

struct InfoTsSubmessage {
    Timestamp timestamp = TIME_ZERO;
};
static_assert(sizeof(InfoTsSubmessage) == 8,
    "InfoTsSubmessage must be 8 bytes — §9.4.5.9, Table 9.26");

// ============================================================
// §9.4.5.13  INFO_DST Submessage
//
// "The INFO_DST Submessage changes the GuidPrefix used to interpret
//  subsequent Submessages."
//  — §9.4.5.13, p. 228, Table 9.33
//
// Wire layout (12 bytes):
//   GuidPrefix  guidPrefix
// ============================================================

struct InfoDstSubmessage {
    GuidPrefix guidPrefix = {};
};
static_assert(sizeof(InfoDstSubmessage) == 12,
    "InfoDstSubmessage must be 12 bytes — §9.4.5.13, Table 9.33");

// ============================================================
// Wire-format serialised size helpers
//
// Used by MessageBuilder to pre-calculate submessageLength values
// and by MessageParser to bounds-check before reading.
// ============================================================

/// Serialised byte size of a SequenceNumberSet with numBits bits.
/// Layout: bitmapBase(8) + numBits(4) + ceil(numBits/32)*4 words.
/// — §9.4.2.6, p. 197
inline constexpr uint16_t sn_set_serialised_size(uint32_t num_bits) noexcept {
    const uint32_t words = (num_bits + 31u) / 32u;
    return static_cast<uint16_t>(8u + 4u + words * 4u);
}

/// Serialised byte size of a FragmentNumberSet with numBits bits.
/// — §9.4.2.8, p. 199
inline constexpr uint16_t frag_set_serialised_size(uint32_t num_bits) noexcept {
    const uint32_t words = (num_bits + 31u) / 32u;
    return static_cast<uint16_t>(4u + 4u + words * 4u);
}

// ============================================================
// Submessage total size helpers (payload bytes, excluding header)
// ============================================================

/// Total payload bytes for a HEARTBEAT submessage (always fixed).
inline constexpr uint16_t heartbeat_payload_size() noexcept {
    return static_cast<uint16_t>(sizeof(HeartbeatSubmessage));
}

/// Total payload bytes for an ACKNACK submessage.
inline constexpr uint16_t acknack_payload_size(uint32_t num_bits) noexcept {
    return static_cast<uint16_t>(
        sizeof(AckNackSubmessageFixed) +
        sn_set_serialised_size(num_bits) +
        4u);  // count field
}

/// Total payload bytes for a GAP submessage.
inline constexpr uint16_t gap_payload_size(uint32_t num_bits) noexcept {
    return static_cast<uint16_t>(
        sizeof(GapSubmessageFixed) +
        sn_set_serialised_size(num_bits));
}

/// Total payload bytes for a NACK_FRAG submessage.
inline constexpr uint16_t nack_frag_payload_size(uint32_t num_bits) noexcept {
    return static_cast<uint16_t>(
        sizeof(NackFragSubmessageFixed) +
        frag_set_serialised_size(num_bits) +
        4u);  // count field
}

}  // namespace rosette::rtps
