/**
 * @file rtps/message_parser.cpp
 * @brief RTPS message parser implementation — §9.4 (OMG formal/2022-04-01).
 *
 * Implements parse_message() declared in message_parser.hpp.
 * All spec references use OMG formal/2022-04-01.
 */

#include "message_parser.hpp"
#include <string.h>     // memcpy

namespace rtps {

// ============================================================
// Internal: byte-swap helpers
//
// On the LE Cortex-M33 host, received submessages that have the E-flag
// clear are big-endian and require swapping.  The swap calls compile
// away entirely when (host_le == wire_le) — i.e. on every message from
// another LE sender, which is the overwhelming common case.
// ============================================================

static inline uint16_t bswap16(uint16_t v) noexcept {
    return static_cast<uint16_t>((v >> 8u) | (v << 8u));
}

static inline uint32_t bswap32(uint32_t v) noexcept {
    return ((v & 0x000000FFu) << 24u) |
           ((v & 0x0000FF00u) <<  8u) |
           ((v & 0x00FF0000u) >>  8u) |
           ((v & 0xFF000000u) >> 24u);
}

static inline int32_t bswap32s(int32_t v) noexcept {
    return static_cast<int32_t>(bswap32(static_cast<uint32_t>(v)));
}

// ============================================================
// Internal: Reader — bounds-checked cursor over a byte buffer
//
// All read operations return false on underflow; the parser propagates
// this as ParseResult::MALFORMED.
// ============================================================

struct Reader {
    const Octet* buf;
    uint32_t     pos;
    uint32_t     end;   ///< One past the last valid byte.

    Reader(const Octet* b, uint32_t start, uint32_t stop) noexcept
        : buf(b), pos(start), end(stop) {}

    uint32_t remaining() const noexcept {
        return (pos < end) ? (end - pos) : 0u;
    }

    bool can_read(uint32_t n) const noexcept {
        return remaining() >= n;
    }

    /// Read T by value.  T must be a trivially-copyable scalar or struct
    /// with no alignment assumptions (we use memcpy to avoid UB on unaligned
    /// wire data).
    template<typename T>
    bool read(T& out) noexcept {
        if (!can_read(sizeof(T))) return false;
        memcpy(&out, buf + pos, sizeof(T));
        pos += static_cast<uint32_t>(sizeof(T));
        return true;
    }

    /// Advance past n bytes without reading.
    bool skip(uint32_t n) noexcept {
        if (!can_read(n)) return false;
        pos += n;
        return true;
    }

    /// Return a pointer to the current position without advancing.
    const Octet* peek() const noexcept { return buf + pos; }

    /// Restrict the readable window to [pos, pos+extent).
    /// Returns false if extent > remaining().
    bool limit(uint32_t extent) noexcept {
        if (extent > remaining()) return false;
        end = pos + extent;
        return true;
    }
};

// ============================================================
// Internal: per-field endian fixup helpers
//
// Called after reading raw scalars when the submessage E-flag == 0
// (big-endian sender).  On an LE host every multi-byte field needs
// reversing; single-byte fields and byte arrays are already correct.
// ============================================================

static void fixup_sequence_number(SequenceNumber& sn, bool need_swap) noexcept {
    if (!need_swap) return;
    sn.high = static_cast<int32_t>(bswap32(static_cast<uint32_t>(sn.high)));
    sn.low  = bswap32(sn.low);
}

static void fixup_u16(uint16_t& v, bool need_swap) noexcept {
    if (need_swap) v = bswap16(v);
}

static void fixup_u32(uint32_t& v, bool need_swap) noexcept {
    if (need_swap) v = bswap32(v);
}

static void fixup_i32(int32_t& v, bool need_swap) noexcept {
    if (need_swap) v = bswap32s(v);
}

// ============================================================
// Internal: read_entity_id()
//
// EntityId is 4 octets with no multi-byte sub-fields; endianness
// does not affect it.  — §9.4.2.1, p. 194
// ============================================================

static bool read_entity_id(Reader& r, EntityId& out) noexcept {
    return r.read(out);
}

// ============================================================
// Internal: read_sequence_number()
//
// Wire layout (8 bytes, §9.4.2.6):
//   int32   high
//   uint32  low
// ============================================================

static bool read_sequence_number(Reader& r, SequenceNumber& out,
                                 bool need_swap) noexcept {
    if (!r.read(out.high) || !r.read(out.low)) return false;
    fixup_sequence_number(out, need_swap);
    return true;
}

// ============================================================
// Internal: read_sn_set()
//
// Wire layout (§9.4.2.6, p. 197):
//   int32   bitmapBase.high
//   uint32  bitmapBase.low
//   uint32  numBits            (<= 256)
//   uint32  bitmap[ceil(numBits/32)]
// ============================================================

static bool read_sn_set(Reader& r, SequenceNumberSet& out,
                        bool need_swap) noexcept {
    out.clear();
    if (!read_sequence_number(r, out.bitmapBase, need_swap)) return false;

    if (!r.read(out.numBits)) return false;
    fixup_u32(out.numBits, need_swap);

    if (out.numBits > static_cast<uint32_t>(SequenceNumberSet::kMaxBits))
        return false;

    const uint32_t words = (out.numBits + 31u) / 32u;
    for (uint32_t w = 0; w < words; ++w) {
        if (!r.read(out.bitmap[w])) return false;
        fixup_u32(out.bitmap[w], need_swap);
    }
    return true;
}

// ============================================================
// Internal: read_frag_number_set()
//
// Wire layout (§9.4.2.8, p. 199):
//   uint32  bitmapBase   (1-based fragment number)
//   uint32  numBits
//   uint32  bitmap[ceil(numBits/32)]
// ============================================================

static bool read_frag_number_set(Reader& r, FragmentNumberSet& out,
                                 bool need_swap) noexcept {
    out.clear();
    if (!r.read(out.bitmapBase)) return false;
    fixup_u32(out.bitmapBase, need_swap);

    if (!r.read(out.numBits)) return false;
    fixup_u32(out.numBits, need_swap);

    if (out.numBits > static_cast<uint32_t>(FragmentNumberSet::kMaxBits))
        return false;

    const uint32_t words = (out.numBits + 31u) / 32u;
    for (uint32_t w = 0; w < words; ++w) {
        if (!r.read(out.bitmap[w])) return false;
        fixup_u32(out.bitmap[w], need_swap);
    }
    return true;
}

// ============================================================
// Internal: skip_inline_qos()
//
// inlineQos is a ParameterList ending with PID_SENTINEL (0x0001).
// We do not consume inline QoS on this embedded target; skip it.
//
// §9.6.2, p. 285: each parameter is { uint16 pid, uint16 length,
//   octet[length] value }.  PID_SENTINEL has length 0.
// ============================================================

static bool skip_inline_qos(Reader& r, bool need_swap) noexcept {
    static constexpr uint16_t PID_SENTINEL = 0x0001u;
    while (true) {
        uint16_t pid, length;
        if (!r.read(pid) || !r.read(length)) return false;
        fixup_u16(pid,    need_swap);
        fixup_u16(length, need_swap);
        if (pid == PID_SENTINEL) return true;
        if (!r.skip(length))    return false;
    }
}

// ============================================================
// Per-kind parse functions
//
// Each returns false on malformed input.  The ParseContext is passed
// by reference so that INFO_TS / INFO_DST can mutate it directly.
// ============================================================

// ----------------------------------------------------------
// parse_info_ts()
//
// Updates ParseContext::sourceTimestamp for all following
// submessages in this message.
//
// §9.4.5.9, p. 253:
//   "The INFO_TS Submessage provides source timestamp information."
//
// Flags:
//   bit 1  I (InvalidateFlag): if set, no Timestamp field is present;
//   the sourceTimestamp is set to TIME_INVALID.
//
// Wire layout when I == 0 (8 bytes):
//   Time_t  timestamp  { seconds(4), fraction(4) }
// ----------------------------------------------------------
static bool parse_info_ts(Reader& r, Octet flags, bool need_swap,
                          ParseContext& ctx) noexcept {
    static constexpr Octet FLAG_I = 0x02u;  ///< InvalidateFlag — §9.4.5.9.
    if (flags & FLAG_I) {
        ctx.sourceTimestamp = TIME_INVALID;
        return true;
    }
    if (!r.read(ctx.sourceTimestamp.seconds) ||
        !r.read(ctx.sourceTimestamp.fraction)) return false;
    fixup_i32(ctx.sourceTimestamp.seconds,  need_swap);
    fixup_u32(ctx.sourceTimestamp.fraction, need_swap);
    return true;
}

// ----------------------------------------------------------
// parse_info_dst()
//
// Updates ParseContext::destGuidPrefix for all following
// submessages in this message.
//
// §9.4.5.13, p. 261:
//   "The INFO_DST Submessage changes the GuidPrefix used to
//    interpret Receiver.destGuidPrefix."
//
// Wire layout (12 bytes):
//   GuidPrefix  guidPrefix
// ----------------------------------------------------------
static bool parse_info_dst(Reader& r, ParseContext& ctx) noexcept {
    GuidPrefix gp{};
    if (!r.read(gp)) return false;
    ctx.destGuidPrefix = gp;
    return true;
}

// ----------------------------------------------------------
// parse_data()
//
// §9.4.5.3, p. 230.
//
// Fixed fields after SubmessageHeader:
//   uint16  extraFlags
//   uint16  octetsToInlineQos
//   EntityId readerId
//   EntityId writerId
//   SequenceNumber writerSN
// Followed by optional inlineQos, then serializedPayload.
//
// octetsToInlineQos gives the byte distance from the end of the
// octetsToInlineQos field to the start of inlineQos / payload.
// For well-formed messages without extensions this is always 16.
// We use it to skip any future interoperability fields.
// ----------------------------------------------------------
static bool parse_data(Reader& r, Octet flags, bool need_swap,
                       const ParseContext& ctx, const ParseOps& ops) noexcept {
    uint16_t extraFlags, octetsToInlineQos;
    if (!r.read(extraFlags) || !r.read(octetsToInlineQos)) return false;
    fixup_u16(octetsToInlineQos, need_swap);

    ParsedData out{};
    out.dataFlag         = (flags & data_flags::D) != 0;
    out.keyFlag          = (flags & data_flags::K) != 0;
    out.inlineQosPresent = (flags & data_flags::Q) != 0;

    if (!read_entity_id(r, out.readerId)) return false;
    if (!read_entity_id(r, out.writerId)) return false;
    if (!read_sequence_number(r, out.writerSN, need_swap)) return false;

    // Skip any bytes between writerSN and inlineQos/payload.
    // octetsToInlineQos counts from after itself (2 bytes) to payload:
    // standard distance for { readerId(4) writerId(4) writerSN(8) } = 16.
    // Subtract the 16 bytes we already read; skip any remainder.
    const uint16_t already_read = 16u;
    if (octetsToInlineQos > already_read) {
        if (!r.skip(static_cast<uint32_t>(octetsToInlineQos - already_read)))
            return false;
    }

    if (out.inlineQosPresent) {
        if (!skip_inline_qos(r, need_swap)) return false;
    }

    out.serializedPayload     = r.peek();
    out.serializedPayloadSize = r.remaining();

    if (ops.on_data) ops.on_data(ctx, out);
    return true;
}

// ----------------------------------------------------------
// parse_data_frag()
//
// §9.4.5.4, p. 236.
//
// Fixed fields:
//   uint16  extraFlags
//   uint16  octetsToInlineQos
//   EntityId readerId
//   EntityId writerId
//   SequenceNumber writerSN
//   uint32  fragmentStartingNum
//   uint16  fragmentsInSubmessage
//   uint16  fragmentSize
//   uint32  dataSize
// Followed by optional inlineQos, then fragment payload.
// ----------------------------------------------------------
static bool parse_data_frag(Reader& r, Octet flags, bool need_swap,
                             const ParseContext& ctx, const ParseOps& ops) noexcept {
    uint16_t extraFlags, octetsToInlineQos;
    if (!r.read(extraFlags) || !r.read(octetsToInlineQos)) return false;
    fixup_u16(octetsToInlineQos, need_swap);

    ParsedDataFrag out{};
    if (!read_entity_id(r, out.readerId)) return false;
    if (!read_entity_id(r, out.writerId)) return false;
    if (!read_sequence_number(r, out.writerSN, need_swap)) return false;
    if (!r.read(out.fragmentStartingNum))    return false;
    if (!r.read(out.fragmentsInSubmessage))  return false;
    if (!r.read(out.fragmentSize))           return false;
    if (!r.read(out.dataSize))               return false;
    fixup_u32(out.fragmentStartingNum,   need_swap);
    fixup_u16(out.fragmentsInSubmessage, need_swap);
    fixup_u16(out.fragmentSize,          need_swap);
    fixup_u32(out.dataSize,              need_swap);

    // Skip extension bytes as for DATA.
    const uint16_t already_read = 28u; // readerId+writerId+SN+fragStart+fragsIn+fragSize+dataSize
    if (octetsToInlineQos > already_read) {
        if (!r.skip(static_cast<uint32_t>(octetsToInlineQos - already_read)))
            return false;
    }

    if (flags & data_frag_flags::Q) {
        if (!skip_inline_qos(r, need_swap)) return false;
    }

    out.fragmentPayload     = r.peek();
    out.fragmentPayloadSize = r.remaining();

    if (ops.on_data_frag) ops.on_data_frag(ctx, out);
    return true;
}

// ----------------------------------------------------------
// parse_heartbeat()
//
// §9.4.5.5, p. 244.
//
// Wire layout (28 bytes after SubmessageHeader):
//   EntityId       readerId
//   EntityId       writerId
//   SequenceNumber firstSN
//   SequenceNumber lastSN
//   int32          count
// ----------------------------------------------------------
static bool parse_heartbeat(Reader& r, Octet flags, bool need_swap,
                             const ParseContext& ctx, const ParseOps& ops) noexcept {
    ParsedHeartbeat out{};
    out.finalFlag      = (flags & hb_flags::F) != 0;
    out.livelinessFlag = (flags & hb_flags::L) != 0;

    if (!read_entity_id(r, out.readerId))              return false;
    if (!read_entity_id(r, out.writerId))              return false;
    if (!read_sequence_number(r, out.firstSN, need_swap)) return false;
    if (!read_sequence_number(r, out.lastSN,  need_swap)) return false;
    if (!r.read(out.count))                            return false;
    fixup_i32(out.count, need_swap);

    if (ops.on_heartbeat) ops.on_heartbeat(ctx, out);
    return true;
}

// ----------------------------------------------------------
// parse_heartbeat_frag()
//
// §9.4.5.6, p. 247.
//
// Wire layout (24 bytes after SubmessageHeader):
//   EntityId       readerId
//   EntityId       writerId
//   SequenceNumber writerSN
//   uint32         lastFragmentNum
//   int32          count
// ----------------------------------------------------------
static bool parse_heartbeat_frag(Reader& r, bool need_swap,
                                  const ParseContext& ctx,
                                  const ParseOps& ops) noexcept {
    ParsedHeartbeatFrag out{};
    if (!read_entity_id(r, out.readerId))              return false;
    if (!read_entity_id(r, out.writerId))              return false;
    if (!read_sequence_number(r, out.writerSN, need_swap)) return false;
    if (!r.read(out.lastFragmentNum))                  return false;
    if (!r.read(out.count))                            return false;
    fixup_u32(out.lastFragmentNum, need_swap);
    fixup_i32(out.count,           need_swap);

    if (ops.on_heartbeat_frag) ops.on_heartbeat_frag(ctx, out);
    return true;
}

// ----------------------------------------------------------
// parse_acknack()
//
// §9.4.5.7, p. 249.
//
// Wire layout (variable):
//   EntityId          readerId
//   EntityId          writerId
//   SequenceNumberSet readerSNState
//   int32             count
// ----------------------------------------------------------
static bool parse_acknack(Reader& r, Octet flags, bool need_swap,
                           const ParseContext& ctx, const ParseOps& ops) noexcept {
    ParsedAckNack out{};
    out.finalFlag = (flags & acknack_flags::F) != 0;

    if (!read_entity_id(r, out.readerId))              return false;
    if (!read_entity_id(r, out.writerId))              return false;
    if (!read_sn_set(r, out.readerSNState, need_swap)) return false;
    if (!r.read(out.count))                            return false;
    fixup_i32(out.count, need_swap);

    if (ops.on_acknack) ops.on_acknack(ctx, out);
    return true;
}

// ----------------------------------------------------------
// parse_nack_frag()
//
// §9.4.5.8, p. 252.
//
// Wire layout (variable):
//   EntityId          readerId
//   EntityId          writerId
//   SequenceNumber    writerSN
//   FragmentNumberSet fragmentNumberState
//   int32             count
// ----------------------------------------------------------
static bool parse_nack_frag(Reader& r, bool need_swap,
                             const ParseContext& ctx,
                             const ParseOps& ops) noexcept {
    ParsedNackFrag out{};
    if (!read_entity_id(r, out.readerId))                    return false;
    if (!read_entity_id(r, out.writerId))                    return false;
    if (!read_sequence_number(r, out.writerSN, need_swap))   return false;
    if (!read_frag_number_set(r, out.fragmentNumberState, need_swap)) return false;
    if (!r.read(out.count))                                  return false;
    fixup_i32(out.count, need_swap);

    if (ops.on_nack_frag) ops.on_nack_frag(ctx, out);
    return true;
}

// ----------------------------------------------------------
// parse_gap()
//
// §9.4.5.11, p. 258.
//
// Wire layout (variable):
//   EntityId          readerId
//   EntityId          writerId
//   SequenceNumber    gapStart
//   SequenceNumberSet gapList
//
// "The set of sequence numbers that are irrelevant consists of:
//   { gapStart, gapStart+1, ..., gapList.base-1 }
//   ∪ { gapList.base + i : gapList.bitmap[i] == 1 }"
//  — §9.4.5.11, p. 258
// ----------------------------------------------------------
static bool parse_gap(Reader& r, bool need_swap,
                      const ParseContext& ctx, const ParseOps& ops) noexcept {
    ParsedGap out{};
    if (!read_entity_id(r, out.readerId))               return false;
    if (!read_entity_id(r, out.writerId))               return false;
    if (!read_sequence_number(r, out.gapStart, need_swap)) return false;
    if (!read_sn_set(r, out.gapList, need_swap))        return false;

    if (ops.on_gap) ops.on_gap(ctx, out);
    return true;
}

// ============================================================
// parse_message() — public entry point
//
// §9.4.1, p. 221:
//   "An RTPS Message consists of a fixed-size Header followed by
//    a variable number of Submessages."
//
// Submessage iteration rule — §9.4.5.1, p. 227:
//   "The submessageLength field contains the number of octets from
//    the start of the contents of the Submessage to the end of the
//    Submessage.  A submessageLength of 0 is used for the last
//    Submessage in the Message, and indicates that the Submessage
//    extends to the end of the Message."
// ============================================================

ParseResult parse_message(const Octet*      buf,
                          uint32_t          len,
                          const GuidPrefix& local_prefix,
                          const ParseOps&   ops,
                          void*             user_data) noexcept
{
    // --------------------------------------------------------
    // 1. Validate RTPS message header — §9.4.2, p. 222.
    // --------------------------------------------------------
    if (len < sizeof(RTPSMessageHeader)) return ParseResult::BAD_HEADER;

    RTPSMessageHeader msg_hdr{};
    memcpy(&msg_hdr, buf, sizeof(msg_hdr));

    // §9.4.2.1: magic must be {'R','T','P','S'}.
    if (!rtps_header_valid(msg_hdr)) return ParseResult::BAD_HEADER;

    // §9.4.2.2: this implementation supports version 2.x only.
    if (msg_hdr.version.major != 2) return ParseResult::VERSION_MISMATCH;

    // --------------------------------------------------------
    // 2. Populate ParseContext.
    // --------------------------------------------------------
    ParseContext ctx{};
    ctx.senderGuidPrefix = msg_hdr.guidPrefix;
    ctx.senderVendorId   = msg_hdr.vendorId;
    ctx.sourceTimestamp  = TIME_INVALID;     // Set by INFO_TS if present.
    ctx.destGuidPrefix   = local_prefix;    // Updated by INFO_DST.
    ctx.user_data        = user_data;

    // --------------------------------------------------------
    // 3. Iterate submessages.
    // --------------------------------------------------------
    uint32_t cursor = sizeof(RTPSMessageHeader);

    while (cursor < len) {
        // Read 4-byte SubmessageHeader — §9.4.5.1, p. 226.
        if (cursor + sizeof(SubmessageHeader) > len)
            return ParseResult::MALFORMED;

        SubmessageHeader smh{};
        memcpy(&smh, buf + cursor, sizeof(smh));
        cursor += sizeof(SubmessageHeader);

        // Endianness of this submessage — §9.4.5.1.1, p. 228.
        // E-flag == 1 → little-endian; 0 → big-endian.
        const bool wire_le   = (smh.flags & FLAG_E) != 0;
        const bool need_swap = !wire_le;   // Host is LE (Cortex-M33).

        // submessageLength is always encoded in the wire byte order of
        // *this* submessage — swap it now before using as an extent.
        uint16_t smlen = smh.submessageLength;
        if (need_swap) smlen = bswap16(smlen);

        // Determine submessage byte extent — §9.4.5.1, p. 227.
        uint32_t extent;
        if (smlen == 0u) {
            // Last submessage: extends to end of message.
            extent = (cursor <= len) ? (len - cursor) : 0u;
        } else {
            extent = smlen;
        }

        if (cursor + extent > len) return ParseResult::MALFORMED;

        // Build a Reader window limited to this submessage's payload.
        Reader r(buf, cursor, cursor + extent);

        bool ok = true;

        switch (smh.submessageId) {

            // ------------------------------------------------
            // INFO_TS — absorbed; updates ctx.sourceTimestamp.
            // §9.4.5.9, p. 253.
            // ------------------------------------------------
            case SubmessageKind::INFO_TS:
                ok = parse_info_ts(r, smh.flags, need_swap, ctx);
                break;

            // ------------------------------------------------
            // INFO_DST — absorbed; updates ctx.destGuidPrefix.
            // §9.4.5.13, p. 261.
            // ------------------------------------------------
            case SubmessageKind::INFO_DST:
                ok = parse_info_dst(r, ctx);
                break;

            // ------------------------------------------------
            // DATA — §9.4.5.3, p. 230.
            // ------------------------------------------------
            case SubmessageKind::DATA:
                ok = parse_data(r, smh.flags, need_swap, ctx, ops);
                break;

            // ------------------------------------------------
            // DATA_FRAG — §9.4.5.4, p. 236.
            // ------------------------------------------------
            case SubmessageKind::DATA_FRAG:
                ok = parse_data_frag(r, smh.flags, need_swap, ctx, ops);
                break;

            // ------------------------------------------------
            // HEARTBEAT — §9.4.5.5, p. 244.
            // ------------------------------------------------
            case SubmessageKind::HEARTBEAT:
                ok = parse_heartbeat(r, smh.flags, need_swap, ctx, ops);
                break;

            // ------------------------------------------------
            // HEARTBEAT_FRAG — §9.4.5.6, p. 247.
            // ------------------------------------------------
            case SubmessageKind::HEARTBEAT_FRAG:
                ok = parse_heartbeat_frag(r, need_swap, ctx, ops);
                break;

            // ------------------------------------------------
            // ACKNACK — §9.4.5.7, p. 249.
            // ------------------------------------------------
            case SubmessageKind::ACKNACK:
                ok = parse_acknack(r, smh.flags, need_swap, ctx, ops);
                break;

            // ------------------------------------------------
            // NACK_FRAG — §9.4.5.8, p. 252.
            // ------------------------------------------------
            case SubmessageKind::NACK_FRAG:
                ok = parse_nack_frag(r, need_swap, ctx, ops);
                break;

            // ------------------------------------------------
            // GAP — §9.4.5.11, p. 258.
            // ------------------------------------------------
            case SubmessageKind::GAP:
                ok = parse_gap(r, need_swap, ctx, ops);
                break;

            // ------------------------------------------------
            // PAD — §9.4.5.2: no-op, skip payload.
            // ------------------------------------------------
            case SubmessageKind::PAD:
                break;

            // ------------------------------------------------
            // INFO_SRC, INFO_REPLY_IP4, INFO_REPLY — silently skip.
            // Not needed by this embedded implementation.
            // ------------------------------------------------
            case SubmessageKind::INFO_SRC:
            case SubmessageKind::INFO_REPLY_IP4:
            case SubmessageKind::INFO_REPLY:
                break;

            // ------------------------------------------------
            // Unknown kind: dispatch to on_unknown callback if set.
            // §9.4.5.1, p. 227:
            //   "An RTPS receiver that receives an unknown
            //    SubmessageKind shall skip it."
            // ------------------------------------------------
            default:
                if (ops.on_unknown) {
                    ops.on_unknown(ctx, smh, buf + cursor,
                                  static_cast<uint16_t>(extent));
                }
                break;
        }

        if (!ok) return ParseResult::MALFORMED;

        // --------------------------------------------------------
        // Advance cursor.
        //
        // §9.4.5.1, p. 227:
        //   "Submessages shall be aligned on a 32-bit boundary."
        // Advance by extent then round up to the next 4-byte boundary.
        // --------------------------------------------------------
        cursor += extent;
        // Align to 4 bytes (submessage boundary padding).
        cursor = (cursor + 3u) & ~3u;
    }

    return ParseResult::OK;
}

} // namespace rtps
