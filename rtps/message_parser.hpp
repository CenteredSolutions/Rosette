/**
 * @file rtps/message_parser.hpp
 * @brief RTPS message deserialisation — §9.4 (OMG formal/2022-04-01).
 *
 * Parses a raw UDP datagram into a sequence of typed submessage views and
 * dispatches each to a user-supplied callback via a flat function-pointer
 * table indexed by SubmessageKind.
 *
 * Design
 * ------
 *  - Zero-copy: the parser operates on the raw receive buffer in-place.
 *    No data is copied into parsed structs beyond scalar fields; payload
 *    pointers point directly into the receive buffer.  The buffer must
 *    remain live for the duration of dispatch.
 *  - No virtual dispatch: the callback table (ParseOps) is a plain
 *    struct of function pointers.  Null slots are skipped silently.
 *  - Endian-aware: the E-flag (§9.4.5.1.1, p. 228) is tested per
 *    submessage.  On the little-endian Cortex-M33 host, only big-endian
 *    submessages from remote senders need byte-swapping; the swap helpers
 *    are inline and compile to nothing when the flag is set (LE path).
 *  - INFO_TS and INFO_DST are absorbed internally to update the
 *    ParseContext; they do not surface as user callbacks.
 *  - Bounds checking: every field read is guarded by a remaining-bytes
 *    check.  A truncated submessage yields ParseResult::MALFORMED and
 *    halts further processing of that message.
 *
 * Callback signatures
 * -------------------
 * Each callback receives a const ParseContext& (sender identity,
 * source timestamp, user_data) and a Parsed* view struct populated
 * with fully decoded, host-endian fields.
 *
 * Spec references (all OMG formal/2022-04-01)
 * -------------------------------------------
 *   §9.4.1    Message structure, p. 221
 *   §9.4.2    Header, p. 222
 *   §9.4.5.1  SubmessageHeader + E-flag, p. 226
 *   §9.4.5.3  DATA, p. 230
 *   §9.4.5.4  DATA_FRAG, p. 236
 *   §9.4.5.5  HEARTBEAT, p. 244
 *   §9.4.5.6  HEARTBEAT_FRAG, p. 247
 *   §9.4.5.7  ACKNACK, p. 249
 *   §9.4.5.8  NACK_FRAG, p. 252
 *   §9.4.5.9  INFO_TS, p. 253
 *   §9.4.5.11 GAP, p. 258
 *   §9.4.5.13 INFO_DST, p. 261
 */

#pragma once

#include "types.hpp"
#include "submessage.hpp"

namespace rtps {

// ============================================================
// ParseResult
// ============================================================

enum class ParseResult : uint8_t {
    OK,               ///< All submessages processed successfully.
    BAD_HEADER,       ///< RTPS magic bytes absent or version mismatch.
    MALFORMED,        ///< A submessage was truncated or internally invalid.
    VERSION_MISMATCH, ///< Remote major version != 2 — §9.4.2.2.
};

// ============================================================
// ParseContext
//
// Immutable information from the RTPS message header, plus mutable
// state that INFO_TS / INFO_DST submessages update as parsing
// proceeds.  Passed by const-ref to every dispatch callback so
// that callbacks can identify the sender without extra parameters.
//
// §9.4.1, p. 221:
//   "The Header provides a context for interpreting the subsequent
//    Submessages."
// §9.4.5.9  INFO_TS — sourceTimestamp applies to all following
//   submessages until the next INFO_TS or end of message.
// §9.4.5.13 INFO_DST — destGuidPrefix changes the logical destination
//   for all following submessages.
// ============================================================

struct ParseContext {
    GuidPrefix  senderGuidPrefix = {};      ///< From RTPS Header §9.4.2.4.
    VendorId    senderVendorId   = {};      ///< From RTPS Header §9.4.2.3.
    Timestamp   sourceTimestamp  = TIME_INVALID; ///< From INFO_TS, or INVALID.
    GuidPrefix  destGuidPrefix   = {};      ///< From INFO_DST; init = local prefix.
    void*       user_data        = nullptr; ///< Forwarded from parse_message().
};

// ============================================================
// Parsed submessage view types
//
// Each struct holds the fully decoded, host-endian scalar fields for
// one submessage kind.  Pointer members (serializedPayload,
// fragmentPayload) alias into the original receive buffer and are
// valid only during the callback invocation.
// ============================================================

/// Decoded DATA submessage — §9.4.5.3, p. 230.
struct ParsedData {
    EntityId        readerId;
    EntityId        writerId;
    SequenceNumber  writerSN;
    bool            dataFlag;               ///< D-flag: payload is data.
    bool            keyFlag;                ///< K-flag: payload is key.
    bool            inlineQosPresent;       ///< Q-flag: inlineQos follows.
    const Octet*    serializedPayload;      ///< Pointer into receive buffer.
    uint32_t        serializedPayloadSize;  ///< Bytes at serializedPayload.
};

/// Decoded DATA_FRAG submessage — §9.4.5.4, p. 236.
struct ParsedDataFrag {
    EntityId        readerId;
    EntityId        writerId;
    SequenceNumber  writerSN;
    uint32_t        fragmentStartingNum;    ///< 1-based first fragment index.
    uint16_t        fragmentsInSubmessage;
    uint16_t        fragmentSize;           ///< Bytes per fragment (except last).
    uint32_t        dataSize;              ///< Total serialized sample size.
    const Octet*    fragmentPayload;        ///< Pointer into receive buffer.
    uint32_t        fragmentPayloadSize;
};

/// Decoded HEARTBEAT submessage — §9.4.5.5, p. 244.
struct ParsedHeartbeat {
    EntityId        readerId;
    EntityId        writerId;
    SequenceNumber  firstSN;        ///< Lowest SN still in writer cache.
    SequenceNumber  lastSN;         ///< Highest SN ever written.
    int32_t         count;          ///< Monotonically increasing per writer.
    bool            finalFlag;      ///< F-flag — §9.4.5.5.
    bool            livelinessFlag; ///< L-flag — §9.4.5.5.
};

/// Decoded HEARTBEAT_FRAG submessage — §9.4.5.6, p. 247.
struct ParsedHeartbeatFrag {
    EntityId        readerId;
    EntityId        writerId;
    SequenceNumber  writerSN;
    uint32_t        lastFragmentNum; ///< Highest fragment number sent (1-based).
    int32_t         count;
};

/// Decoded ACKNACK submessage — §9.4.5.7, p. 249.
struct ParsedAckNack {
    EntityId          readerId;
    EntityId          writerId;
    SequenceNumberSet readerSNState; ///< Bitmap of SNs the reader is missing.
    int32_t           count;
    bool              finalFlag;     ///< F-flag — §9.4.5.7.
};

/// Decoded NACK_FRAG submessage — §9.4.5.8, p. 252.
struct ParsedNackFrag {
    EntityId          readerId;
    EntityId          writerId;
    SequenceNumber    writerSN;
    FragmentNumberSet fragmentNumberState;
    int32_t           count;
};

/// Decoded GAP submessage — §9.4.5.11, p. 258.
struct ParsedGap {
    EntityId          readerId;
    EntityId          writerId;
    SequenceNumber    gapStart; ///< First irrelevant SN.
    SequenceNumberSet gapList;  ///< Additional irrelevant SNs beyond gapStart.
};

// ============================================================
// ParseOps — vtable-free dispatch table
//
// A plain struct of function pointers, one per handled submessage
// kind.  Set a slot to nullptr to silently ignore that kind.
// The void* user_data is carried inside ParseContext so callers
// don't need separate closure state.
//
// All callbacks are noexcept — they run on the single RX dispatch
// task and must not throw or block indefinitely.
// ============================================================

struct ParseOps {
    /// DATA received — §9.4.5.3, p. 230.
    void (*on_data)(const ParseContext&,
                    const ParsedData&) noexcept                   = nullptr;

    /// DATA_FRAG received — §9.4.5.4, p. 236.
    void (*on_data_frag)(const ParseContext&,
                         const ParsedDataFrag&) noexcept          = nullptr;

    /// HEARTBEAT received — §9.4.5.5, p. 244.
    void (*on_heartbeat)(const ParseContext&,
                         const ParsedHeartbeat&) noexcept         = nullptr;

    /// HEARTBEAT_FRAG received — §9.4.5.6, p. 247.
    void (*on_heartbeat_frag)(const ParseContext&,
                              const ParsedHeartbeatFrag&) noexcept = nullptr;

    /// ACKNACK received — §9.4.5.7, p. 249.
    void (*on_acknack)(const ParseContext&,
                       const ParsedAckNack&) noexcept             = nullptr;

    /// NACK_FRAG received — §9.4.5.8, p. 252.
    void (*on_nack_frag)(const ParseContext&,
                         const ParsedNackFrag&) noexcept          = nullptr;

    /// GAP received — §9.4.5.11, p. 258.
    void (*on_gap)(const ParseContext&,
                   const ParsedGap&) noexcept                     = nullptr;

    /// Any SubmessageKind not handled above.
    /// raw_payload points into the receive buffer; raw_len is the
    /// submessageLength value from the header (may be 0).
    void (*on_unknown)(const ParseContext&,
                       const SubmessageHeader&,
                       const Octet* raw_payload,
                       uint16_t     raw_len) noexcept             = nullptr;
};

// ============================================================
// parse_message()
//
// Top-level entry point — call once per received UDP datagram.
//
// "An RTPS Message consists of a fixed-size Header followed by a
//  variable number of Submessages."  — §9.4.1, p. 221
//
// Processing steps:
//   1. Validate 20-byte RTPS Header (§9.4.2):
//      magic == "RTPS", version.major == 2.
//   2. Populate ParseContext from header.
//   3. Loop over remaining bytes:
//      a. Read 4-byte SubmessageHeader — §9.4.5.1.
//      b. Determine submessage byte extent:
//         - If submessageLength > 0: extent = submessageLength.
//         - If submessageLength == 0: extent = remaining bytes
//           (last submessage — §9.4.5.1, p. 227).
//      c. Check E-flag; byte-swap scalars on mismatch with host order.
//      d. Dispatch to the appropriate ParseOps slot.
//      e. Advance cursor by (4 + extent), aligned to 4 bytes.
//
// @param buf          Raw receive buffer (must outlive dispatch callbacks).
// @param len          Total valid bytes in buf.
// @param local_prefix This participant's GuidPrefix; used to seed
//                     ParseContext::destGuidPrefix and to accept
//                     submessages directed to ENTITYID_UNKNOWN.
// @param ops          Dispatch table.
// @param user_data    Forwarded via ParseContext::user_data.
// @returns            ParseResult indicating outcome.
//
// Thread safety: intended for single-threaded RX dispatch only.
// ============================================================
ParseResult parse_message(const Octet*      buf,
                          uint32_t          len,
                          const GuidPrefix& local_prefix,
                          const ParseOps&   ops,
                          void*             user_data = nullptr) noexcept;

} // namespace rtps
