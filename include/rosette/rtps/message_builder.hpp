/**
 * @file rtps/message_builder.hpp
 * @brief RTPS message serialisation into pool-backed buffers.
 *
 * MessageBuilder writes a complete RTPS message (header + one or more
 * submessages) into a single BufRef obtained from the global pool.
 * Callers call begin(), append submessages, then call finish() to get a
 * ready-to-send BufRef.
 *
 * Design
 * ------
 *  - All writes are little-endian (E flag always set), the native byte
 *    order of the RP2350 (Cortex-M33).  No runtime byte-swap branches.
 *  - The builder writes directly into BufBlock::data via a cursor; no
 *    intermediate copies.
 *  - Bounds are checked on every write; append_* functions return false
 *    on overflow so the caller can abort() and signal an error.
 *  - Variable-length fields (SequenceNumberSet, FragmentNumberSet,
 *    SerializedPayload) are handled by dedicated write helpers.
 *
 * Spec references (all OMG formal/2022-04-01)
 * --------------------------------------------
 *   Message structure       — §9.4,      p. 192
 *   Header layout           — §9.4.1,    p. 193
 *   Submessage framing      — §9.4.5.1,  p. 200  (E bit = little-endian)
 *   Serialised SN set       — §9.4.2.6,  p. 197
 *   Serialised frag set     — §9.4.2.8,  p. 199
 *   EncapsulationHeader     — §10.2,     p. 244  (CDR_LE = 0x0001)
 */

#pragma once

#include "submessage.hpp"
#include "buf_pool.hpp"
#include <string.h>

namespace rtps {

class MessageBuilder {
public:

    MessageBuilder() noexcept = default;

    // ----------------------------------------------------------
    // begin()
    //
    // Allocates a pool block and writes the RTPS message header.
    // Must be called before any append_* call.
    //
    // "The Header identifies the message as belonging to the RTPS
    //  protocol."  — §9.4.1, Table 9.1, p. 193
    //
    // @param guid_prefix  GuidPrefix of the sending participant.
    // @param pool_size    Size of the buffer to allocate from the pool.
    // @returns false if the pool is exhausted.
    // ----------------------------------------------------------
    /// Default capacity for begin(): control messages (HEARTBEAT, GAP,
    /// ACKNACK, NACK_FRAG, INFO_DST) stay under ~100 octets even with a full
    /// 256-bit set, so they never borrow a large sample block. Data paths
    /// pass the exact message size.
    static constexpr uint32_t kControlMessageCapacity = 256u;
    static_assert(RTPS_POOL_CLASS0_SIZE >= 128u, "control messages need 128-octet blocks");

    bool begin(const GuidPrefix& guid_prefix,
               uint32_t pool_size = kControlMessageCapacity) noexcept {
        buf_ = buf_alloc(pool_size);
        if (!buf_.valid()) return false;
        cursor_ = 0;
        RTPSMessageHeader hdr{};
        hdr.guidPrefix = guid_prefix;
        write_bytes(&hdr, sizeof(hdr));
        return true;
    }

    // ----------------------------------------------------------
    // begin_with_payload()
    //
    // Adopts a caller-provided payload buffer that was allocated
    // with enough headroom for an RTPS message header plus a DATA
    // submessage header. The payload bytes must already be written
    // at buf.data and buf.size must reflect their length.
    // ----------------------------------------------------------
    bool begin_with_payload(BufRef&& payload) noexcept {
        if (buf_.valid()) abort();
        if (!payload.valid() ||
            payload.headroom_available() < kRtpsDataHeadroom) {
            payload.reset();
            return false;
        }

        buf_ = static_cast<BufRef&&>(payload);
        cursor_ = static_cast<uint16_t>(buf_.size);
        return true;
    }

    // ----------------------------------------------------------
    // abort()
    //
    // Release the buffer without sending.  Call on any append failure.
    // ----------------------------------------------------------
    void abort() noexcept {
        if (buf_.valid()) buf_.release();
        cursor_ = 0;
    }

    // ----------------------------------------------------------
    // finish()
    //
    // Returns the completed BufRef (caller takes ownership).
    // The builder is reset; begin() may be called again.
    //
    // @returns Invalid BufRef if begin() was not called.
    // ----------------------------------------------------------
    BufRef finish() noexcept {
        if (!buf_.valid()) return {};
        buf_.size = cursor_;
        BufRef result = static_cast<BufRef&&>(buf_);
        buf_ = {};
        cursor_ = 0;
        return result;
    }

    uint16_t bytes_written() const noexcept { return cursor_; }

    // ----------------------------------------------------------
    // prepend_data_message()
    //
    // Finalises an adopted headroom-backed payload buffer by
    // prepending the DATA submessage framing and RTPS header
    // in place. This avoids allocating a second message buffer
    // and copying the serialized payload into it.
    // ----------------------------------------------------------
    bool prepend_data_message(const GuidPrefix&    guid_prefix,
                              const EntityId&      reader_id,
                              const EntityId&      writer_id,
                              const SequenceNumber& writer_sn,
                              bool                 key_flag = false,
                              const InlineQos&     inline_qos = {}) noexcept
    {
        if (!buf_.valid()) return false;
        if (!inline_qos.empty()) return false;
        if (buf_.headroom_available() < kRtpsDataHeadroom) return false;

        auto* sm = reinterpret_cast<DataSubmessageFixed*>(
            buf_.prepend(sizeof(DataSubmessageFixed)));
        if (!sm) return false;
        sm->extraFlags        = 0;
        sm->octetsToInlineQos = 16u;
        sm->readerId          = reader_id;
        sm->writerId          = writer_id;
        sm->writerSN          = writer_sn;

        auto* smh = reinterpret_cast<SubmessageHeader*>(
            buf_.prepend(sizeof(SubmessageHeader)));
        if (!smh) return false;
        smh->submessageId = SubmessageKind::DATA;
        smh->flags        = FLAG_E | (key_flag ? data_flags::K : data_flags::D);
        smh->submessageLength = static_cast<uint16_t>(
            buf_.size - sizeof(SubmessageHeader));

        auto* mh = reinterpret_cast<RTPSMessageHeader*>(
            buf_.prepend(sizeof(RTPSMessageHeader)));
        if (!mh) return false;
        *mh = RTPSMessageHeader{};
        mh->guidPrefix = guid_prefix;

        cursor_ = static_cast<uint16_t>(buf_.size);
        return true;
    }

    // ----------------------------------------------------------
    // append_heartbeat()
    //
    // Serialises a HEARTBEAT submessage.
    //
    // "A Heartbeat message communicates the range of sequence numbers
    //  available in the Writer's HistoryCache."
    //  — §9.4.5.5, Table 9.18, p. 211
    //
    // Flags:
    //   F (bit 1) — FinalFlag: reader need not respond if up-to-date.
    //   L (bit 2) — LivelinessFlag: liveliness assertion heartbeat.
    // ----------------------------------------------------------
    bool append_heartbeat(const EntityId&       reader_id,
                          const EntityId&       writer_id,
                          const SequenceNumber& first_sn,
                          const SequenceNumber& last_sn,
                          Long                  count,
                          bool                  final_flag,
                          bool                  liveliness = false) noexcept
    {
        Octet flags = FLAG_E;
        if (final_flag) flags |= hb_flags::F;
        if (liveliness) flags |= hb_flags::L;

        SubmessageHeader smhdr{};
        smhdr.submessageId     = SubmessageKind::HEARTBEAT;
        smhdr.flags            = flags;
        smhdr.submessageLength = heartbeat_payload_size();

        HeartbeatSubmessage body{};
        body.readerId = reader_id;
        body.writerId = writer_id;
        body.firstSN  = first_sn;
        body.lastSN   = last_sn;
        body.count    = count;

        return write_submessage(&smhdr, &body, sizeof(body));
    }

    // ----------------------------------------------------------
    // append_acknack()
    //
    // Serialises an ACKNACK submessage.
    //
    // "The ACKNACK Submessage provides information on the state of a
    //  Reader to a matched Writer."
    //  — §9.4.5.7, Table 9.22, p. 216
    //
    // Flags:
    //   F (bit 1) — FinalFlag: writer need not reply if nothing missing.
    // ----------------------------------------------------------
    bool append_acknack(const EntityId&          reader_id,
                        const EntityId&          writer_id,
                        const SequenceNumberSet& reader_sn_state,
                        Long                     count,
                        bool                     final_flag) noexcept
    {
        const uint16_t sn_bytes = sn_set_serialised_size(reader_sn_state.numBits);
        const uint16_t body_len = static_cast<uint16_t>(
            sizeof(AckNackSubmessageFixed) + sn_bytes + 4u);

        Octet flags = FLAG_E;
        if (final_flag) flags |= acknack_flags::F;

        SubmessageHeader smhdr{};
        smhdr.submessageId     = SubmessageKind::ACKNACK;
        smhdr.flags            = flags;
        smhdr.submessageLength = body_len;

        if (!write_submessage_header(&smhdr)) return false;

        AckNackSubmessageFixed fixed{};
        fixed.readerId = reader_id;
        fixed.writerId = writer_id;
        if (!write_bytes(&fixed, sizeof(fixed))) return false;
        if (!write_sn_set(reader_sn_state))      return false;
        return write_long(count);
    }

    // ----------------------------------------------------------
    // append_gap()
    //
    // Serialises a GAP submessage.
    //
    // "The GAP Submessage indicates to the Reader that a range of
    //  sequence numbers is no longer relevant."
    //  — §9.4.5.11, Table 9.29, p. 224
    // ----------------------------------------------------------
    bool append_gap(const EntityId&          reader_id,
                    const EntityId&          writer_id,
                    const SequenceNumber&    gap_start,
                    const SequenceNumberSet& gap_list) noexcept
    {
        const uint16_t sn_bytes = sn_set_serialised_size(gap_list.numBits);
        const uint16_t body_len = static_cast<uint16_t>(
            sizeof(GapSubmessageFixed) + sn_bytes);

        SubmessageHeader smhdr{};
        smhdr.submessageId     = SubmessageKind::GAP;
        smhdr.flags            = FLAG_E;
        smhdr.submessageLength = body_len;

        if (!write_submessage_header(&smhdr)) return false;

        GapSubmessageFixed fixed{};
        fixed.readerId = reader_id;
        fixed.writerId = writer_id;
        fixed.gapStart = gap_start;
        if (!write_bytes(&fixed, sizeof(fixed))) return false;
        return write_sn_set(gap_list);
    }

    // ----------------------------------------------------------
    // append_data()
    //
    // Serialises a DATA submessage carrying a pool-backed payload.
    //
    // "The DATA Submessage contains information regarding changes to a
    //  data-object."  — §9.4.5.3, Table 9.14, p. 204
    //
    // The payload is preceded by a 4-byte CDR EncapsulationHeader
    // (representation identifier CDR_LE=0x0001, options=0x0000).
    //  — §10.2, p. 244
    //
    // Flags:
    //   D (bit 2) — dataFlag: serializedPayload contains data.
    //   K (bit 3) — keyFlag:  serializedPayload contains key hash.
    // ----------------------------------------------------------
    bool append_data(const EntityId&       reader_id,
                     const EntityId&       writer_id,
                     const SequenceNumber& writer_sn,
                     const BufRef&         payload,
                     bool                  key_flag = false,
                     const InlineQos&      inline_qos = {}) noexcept
    {
        const uint16_t payload_bytes = payload.valid() ? payload.size : 0u;
        const uint16_t encap_size    = 4u;
        const uint16_t inline_qos_bytes = inline_qos_serialized_size(inline_qos);
        const uint16_t body_len = static_cast<uint16_t>(
            sizeof(DataSubmessageFixed) + inline_qos_bytes + encap_size + payload_bytes);

        Octet flags = FLAG_E;
        flags |= (key_flag ? data_flags::K : data_flags::D);
        if (!inline_qos.empty()) flags |= data_flags::Q;

        SubmessageHeader smhdr{};
        smhdr.submessageId     = SubmessageKind::DATA;
        smhdr.flags            = flags;
        smhdr.submessageLength = body_len;

        if (!write_submessage_header(&smhdr)) return false;

        DataSubmessageFixed fixed{};
        fixed.octetsToInlineQos = 16u;
        fixed.readerId          = reader_id;
        fixed.writerId          = writer_id;
        fixed.writerSN          = writer_sn;
        if (!write_bytes(&fixed, sizeof(fixed))) return false;
        if (!inline_qos.empty()) {
            if (!write_inline_qos(inline_qos)) return false;
        }

        // EncapsulationHeader — §10.2, p. 244. The representation
        // identifier is an octet pair, not a host-order integer:
        // CDR_LE = {0x00, 0x01}, followed by two zero option octets.
        static constexpr Octet kCdrLeEncapsulation[4] = {0x00u, 0x01u, 0x00u, 0x00u};
        if (!write_bytes(kCdrLeEncapsulation, sizeof(kCdrLeEncapsulation))) return false;

        if (payload_bytes > 0) {
            if (!write_bytes(payload.data, payload_bytes)) return false;
        }
        return true;
    }

    // ----------------------------------------------------------
    // append_data_frag()
    //
    // Serialises a DATA_FRAG submessage for one fragment of a large
    // serialised payload.
    //
    // "The DATA_FRAG Submessage extends DATA to allow the
    //  serializedPayload to be fragmented."
    //  — §9.4.5.4, Table 9.16, p. 208
    // ----------------------------------------------------------
    bool append_data_frag(const EntityId&       reader_id,
                          const EntityId&       writer_id,
                          const SequenceNumber& writer_sn,
                          uint32_t              fragment_start,
                          uint16_t              fragments_in_msg,
                          uint16_t              fragment_size,
                          uint32_t              data_size,
                          const Octet*          frag_data,
                          uint16_t              frag_data_len) noexcept
    {
        const uint16_t body_len = static_cast<uint16_t>(
            sizeof(DataFragSubmessageFixed) + frag_data_len);

        SubmessageHeader smhdr{};
        smhdr.submessageId     = SubmessageKind::DATA_FRAG;
        smhdr.flags            = FLAG_E;
        smhdr.submessageLength = body_len;

        if (!write_submessage_header(&smhdr)) return false;

        DataFragSubmessageFixed fixed{};
        fixed.readerId              = reader_id;
        fixed.writerId              = writer_id;
        fixed.writerSN              = writer_sn;
        fixed.fragmentStartingNum   = fragment_start;
        fixed.fragmentsInSubmessage = fragments_in_msg;
        fixed.fragmentSize          = fragment_size;
        fixed.dataSize              = data_size;
        if (!write_bytes(&fixed, sizeof(fixed))) return false;
        return write_bytes(frag_data, frag_data_len);
    }

    // ----------------------------------------------------------
    // append_data_frag_parts()
    //
    // Serialises one DATA_FRAG submessage carrying a single fragment, with
    // optional inline QoS, whose bytes are the concatenation of `prefix`
    // and `body`. A writer fragmenting a sample whose stored bytes lack the
    // CDR encapsulation header passes that header as the prefix of
    // fragment 1, so the reassembled serializedPayload is complete.
    //
    // §9.4.5.4, Table 9.16, p. 208: fragmentStartingNum is 1-based;
    // dataSize is the whole serializedPayload, header included.
    //
    // Verification: TC-SWR-8.4.14-001 (build → parse → reassemble).
    // ----------------------------------------------------------
    bool append_data_frag_parts(const EntityId&       reader_id,
                                const EntityId&       writer_id,
                                const SequenceNumber& writer_sn,
                                uint32_t              fragment_num,
                                uint16_t              fragment_size,
                                uint32_t              data_size,
                                const Octet*          prefix,
                                uint16_t              prefix_len,
                                const Octet*          body,
                                uint16_t              body_len,
                                const InlineQos&      inline_qos = {}) noexcept
    {
        const uint16_t iq = inline_qos_serialized_size(inline_qos);
        const uint32_t body_total = sizeof(DataFragSubmessageFixed) + iq + prefix_len + body_len;
        if (body_total > 0xFFFFu) return false;

        SubmessageHeader smhdr{};
        smhdr.submessageId     = SubmessageKind::DATA_FRAG;
        smhdr.flags            = static_cast<Octet>(FLAG_E | (iq ? data_frag_flags::Q : 0u));
        smhdr.submessageLength = static_cast<uint16_t>(body_total);
        if (!write_submessage_header(&smhdr)) return false;

        DataFragSubmessageFixed fixed{};
        fixed.readerId              = reader_id;
        fixed.writerId              = writer_id;
        fixed.writerSN              = writer_sn;
        fixed.fragmentStartingNum   = fragment_num;
        fixed.fragmentsInSubmessage = 1u;
        fixed.fragmentSize          = fragment_size;
        fixed.dataSize              = data_size;
        if (!write_bytes(&fixed, sizeof(fixed))) return false;
        if (iq && !write_inline_qos(inline_qos)) return false;
        if (prefix_len && !write_bytes(prefix, prefix_len)) return false;
        return body_len == 0u || write_bytes(body, body_len);
    }

    // ----------------------------------------------------------
    // append_heartbeat_frag()
    //
    // "The HEARTBEAT_FRAG Submessage is used to indicate which
    //  fragments of a DATA_FRAG are available."
    //  — §9.4.5.6, Table 9.20, p. 214
    // ----------------------------------------------------------
    bool append_heartbeat_frag(const EntityId&       reader_id,
                               const EntityId&       writer_id,
                               const SequenceNumber& writer_sn,
                               uint32_t              last_fragment_num,
                               Long                  count) noexcept
    {
        SubmessageHeader smhdr{};
        smhdr.submessageId     = SubmessageKind::HEARTBEAT_FRAG;
        smhdr.flags            = FLAG_E;
        smhdr.submessageLength = static_cast<uint16_t>(sizeof(HeartbeatFragSubmessage));

        HeartbeatFragSubmessage body{};
        body.readerId        = reader_id;
        body.writerId        = writer_id;
        body.writerSN        = writer_sn;
        body.lastFragmentNum = last_fragment_num;
        body.count           = count;

        return write_submessage(&smhdr, &body, sizeof(body));
    }

    // ----------------------------------------------------------
    // append_nack_frag()
    //
    // "The NACK_FRAG Submessage is used to indicate that a Reader is
    //  missing certain fragments."
    //  — §9.4.5.8, Table 9.24, p. 218
    // ----------------------------------------------------------
    bool append_nack_frag(const EntityId&          reader_id,
                          const EntityId&          writer_id,
                          const SequenceNumber&    writer_sn,
                          const FragmentNumberSet& missing_frags,
                          Long                     count) noexcept
    {
        const uint16_t frag_bytes = frag_set_serialised_size(missing_frags.numBits);
        const uint16_t body_len   = static_cast<uint16_t>(
            sizeof(NackFragSubmessageFixed) + frag_bytes + 4u);

        SubmessageHeader smhdr{};
        smhdr.submessageId     = SubmessageKind::NACK_FRAG;
        smhdr.flags            = FLAG_E;
        smhdr.submessageLength = body_len;

        if (!write_submessage_header(&smhdr)) return false;

        NackFragSubmessageFixed fixed{};
        fixed.readerId = reader_id;
        fixed.writerId = writer_id;
        fixed.writerSN = writer_sn;
        if (!write_bytes(&fixed, sizeof(fixed))) return false;
        if (!write_frag_set(missing_frags))      return false;
        return write_long(count);
    }

    // ----------------------------------------------------------
    // append_info_ts()
    //
    // "The INFO_TS Submessage provides the source timestamp for
    //  subsequent Submessages."  — §9.4.5.9, Table 9.26, p. 221
    // ----------------------------------------------------------
    bool append_info_ts(const Timestamp& ts) noexcept {
        SubmessageHeader smhdr{};
        smhdr.submessageId     = SubmessageKind::INFO_TS;
        smhdr.flags            = FLAG_E;
        smhdr.submessageLength = static_cast<uint16_t>(sizeof(InfoTsSubmessage));

        InfoTsSubmessage body{};
        body.timestamp = ts;
        return write_submessage(&smhdr, &body, sizeof(body));
    }

    // ----------------------------------------------------------
    // append_info_dst()
    //
    // "The INFO_DST Submessage changes the GuidPrefix used to interpret
    //  subsequent Submessages."  — §9.4.5.13, Table 9.33, p. 228
    // ----------------------------------------------------------
    bool append_info_dst(const GuidPrefix& prefix) noexcept {
        SubmessageHeader smhdr{};
        smhdr.submessageId     = SubmessageKind::INFO_DST;
        smhdr.flags            = FLAG_E;
        smhdr.submessageLength = static_cast<uint16_t>(sizeof(InfoDstSubmessage));

        InfoDstSubmessage body{};
        body.guidPrefix = prefix;
        return write_submessage(&smhdr, &body, sizeof(body));
    }

private:

    BufRef   buf_    = {};
    uint16_t cursor_ = 0;

    bool write_bytes(const void* src, uint16_t len) noexcept {
        if (!buf_.valid()) return false;
        // Use the actual runtime capacity of the allocated block, not a
        // compile-time constant — blocks are now variable-sized.
        if (static_cast<uint32_t>(cursor_) + len > buf_.capacity()) return false;
        memcpy(buf_.data + cursor_, src, len);
        cursor_ += len;
        return true;
    }
    bool write_uint16(uint16_t v) noexcept { return write_bytes(&v, 2u); }
    bool write_uint32(uint32_t v) noexcept { return write_bytes(&v, 4u); }
    bool write_long(Long v)       noexcept { return write_bytes(&v, 4u); }
    bool write_byte(Octet v)      noexcept { return write_bytes(&v, 1u); }
    bool write_inline_qos(const InlineQos& inline_qos) noexcept {
        Octet encoded[kMaxInlineQosSize];
        const uint16_t n = serialize_inline_qos(encoded, inline_qos);
        return write_bytes(encoded, n);
    }

    bool write_submessage_header(const SubmessageHeader* h) noexcept {
        return write_bytes(h, sizeof(SubmessageHeader));
    }
    bool write_submessage(const SubmessageHeader* h,
                          const void* body, uint16_t body_len) noexcept {
        return write_submessage_header(h) && write_bytes(body, body_len);
    }

    // §9.4.2.6 — bitmapBase(8) + numBits(4) + words*4
    bool write_sn_set(const SequenceNumberSet& s) noexcept {
        if (!write_bytes(&s.bitmapBase, sizeof(s.bitmapBase))) return false;
        if (!write_uint32(s.numBits)) return false;
        const uint32_t words = (s.numBits + 31u) / 32u;
        return write_bytes(s.bitmap, static_cast<uint16_t>(words * 4u));
    }

    // §9.4.2.8 — bitmapBase(4) + numBits(4) + words*4
    bool write_frag_set(const FragmentNumberSet& s) noexcept {
        if (!write_uint32(s.bitmapBase)) return false;
        if (!write_uint32(s.numBits))    return false;
        const uint32_t words = (s.numBits + 31u) / 32u;
        return write_bytes(s.bitmap, static_cast<uint16_t>(words * 4u));
    }
};

} // namespace rtps
