/**
 * @file rtps/fragment_assembler.hpp
 * @brief Fragment reassembly for DATA_FRAG — §8.4.14 (OMG formal/2022-04-01).
 *
 * The spec defines fragment assembly behaviour in §8.4.14, p. 156:
 *
 *   "The FragmentedChange is a partially received CacheChange that is
 *    being assembled from DATA_FRAG Submessages.  Once all fragments
 *    have been received the FragmentedChange becomes a complete
 *    CacheChange."
 *
 * Per §8.4.14, Table 8.72, the FragmentedChange attributes are:
 *   writerGuid          — GUID of the sending writer.
 *   sequenceNumber      — SN of the sample being assembled.
 *   fragmentSize        — size in bytes of each fragment (last may differ).
 *   dataSize            — total serialized payload size.
 *   receivedFragments   — FragmentNumberSet of received fragment numbers.
 *
 * Fragment numbers are 1-based (§9.4.5.4, p. 208).
 *
 * NACK_FRAG generation (§8.4.12, p. 153):
 *   The Reader sends a NACK_FRAG when it receives a HEARTBEAT_FRAG
 *   indicating fragments are available that it has not yet received.
 *   The NACK_FRAG contains a FragmentNumberSet of the missing fragments.
 *   — §8.4.12, p. 153; §8.3.7.4, p. 90.
 *
 * Design
 * ------
 *  - One FragmentAssembler<MaxSamples> per StatefulReader.
 *  - Each slot assembles one in-flight fragmented sample.
 *  - KEEP_LAST(1) per writer: a new SN from the same writer evicts
 *    any partial assembly for an older SN (§8.3.3.1, p. 68).
 *  - The received-fragment set uses a full 256-bit bitmap (kWords = 8
 *    words of 32 bits each) matching the SequenceNumberSet wire format
 *    of FragmentNumberSet (§9.4.2.8, p. 199).  This supports up to 256
 *    fragments per sample — i.e. samples up to 256 × fragmentSize bytes.
 *  - Pool-backed assembly buffer; one BufRef per active slot.
 *
 * Spec references (all OMG formal/2022-04-01)
 * -------------------------------------------
 *   FragmentedChange           — §8.4.14,  Table 8.72, p. 156
 *   Fragment assembly          — §8.4.14,  p. 156
 *   NACK_FRAG generation       — §8.4.12,  p. 153
 *   HEARTBEAT_FRAG processing  — §8.3.7.4, p. 90
 *   FragmentNumberSet wire     — §9.4.2.8, p. 199
 *   DATA_FRAG fragmentStartingNum (1-based) — §9.4.5.4, p. 208
 */

#pragma once

#include "log.hpp"
#include "types.hpp"
#include "buf_pool.hpp"
#include "submessage.hpp"   // FragmentNumberSet
#include "history_cache.hpp"

namespace rtps {

// ============================================================
// FragmentedChange
//
// §8.4.14, Table 8.72, p. 156 — one partially-received sample.
//
// receivedFragments tracks which 1-based fragment numbers have
// arrived.  The bitmap encoding mirrors FragmentNumberSet exactly
// so that missing_fragments() can produce a ready-to-send
// FragmentNumberSet with no further transformation.
//
// Bitmap encoding (§9.4.2.8, p. 199):
//   Bit 0 of word 0 (MSB) = fragment (bitmapBase + 0).
//   Bit 1 of word 0       = fragment (bitmapBase + 1).
//   etc.
//
// Here bitmapBase is always 1 (fragments are always numbered from 1),
// so offset = fragNum - 1.
// ============================================================

struct FragmentedChange {
    // ── §8.4.14, Table 8.72 attributes ─────────────────────
    Guid           writerGuid    = GUID_UNKNOWN;
    SequenceNumber sequenceNumber= SEQUENCENUMBER_UNKNOWN;
    uint16_t       fragmentSize  = 0;   ///< Bytes per fragment (last may differ).
    uint32_t       dataSize      = 0;   ///< Total serialized payload size.

    // ── Implementation fields ────────────────────────────────
    BufRef   buffer;        ///< Pool block into which fragments are written.
    uint32_t totalFragments = 0;  ///< ceil(dataSize / fragmentSize).
    /// Inline QoS carried by the sample's fragments (key hash, status info,
    /// related sample identity). A writer may send it with any or every
    /// fragment; the first fragment that carries it is kept, so a completed
    /// service reply keeps the identity that routes it to its client.
    InlineQos inlineQos{};

    /// Received-fragment bitmap.  Bit position = (fragNum - 1).
    /// Word 0 bit 31 (MSB) = fragment 1.
    /// — matches FragmentNumberSet wire encoding §9.4.2.8, p. 199.
    static constexpr uint8_t kWords = FragmentNumberSet::kWords;
    uint32_t receivedBitmap[kWords] = {};

    bool is_active() const noexcept {
        return sequenceNumber != SEQUENCENUMBER_UNKNOWN;
    }

    void clear() noexcept {
        writerGuid     = GUID_UNKNOWN;
        sequenceNumber = SEQUENCENUMBER_UNKNOWN;
        inlineQos      = {};
        fragmentSize   = 0;
        dataSize       = 0;
        totalFragments = 0;
        memset(receivedBitmap, 0, sizeof(receivedBitmap));
        buffer.reset();
    }

    // ----------------------------------------------------------
    // mark_received()
    //
    // Record that fragment fragNum (1-based) has been received.
    // Returns false if fragNum is out of range.
    //
    // "The receivedFragments set is updated to include the
    //  fragmentStartingNum."  — §8.4.14, p. 156.
    // ----------------------------------------------------------
    bool mark_received(uint32_t frag_num) noexcept {
        if (frag_num < 1 || frag_num > totalFragments) return false;
        const uint32_t offset = frag_num - 1u;
        const uint32_t word   = offset / 32u;
        const uint32_t bit    = 31u - (offset % 32u);   // MSB-first encoding.
        if (word >= kWords) return false;
        receivedBitmap[word] |= (1u << bit);
        return true;
    }

    // ----------------------------------------------------------
    // is_received()
    //
    // Returns true if fragment fragNum has been received.
    // ----------------------------------------------------------
    bool is_received(uint32_t frag_num) const noexcept {
        if (frag_num < 1 || frag_num > totalFragments) return false;
        const uint32_t offset = frag_num - 1u;
        const uint32_t word   = offset / 32u;
        const uint32_t bit    = 31u - (offset % 32u);
        if (word >= kWords) return false;
        return (receivedBitmap[word] >> bit) & 1u;
    }

    // ----------------------------------------------------------
    // is_complete()
    //
    // "A FragmentedChange is complete when all fragments have been
    //  received."  — §8.4.14, p. 156.
    // ----------------------------------------------------------
    bool is_complete() const noexcept {
        if (totalFragments == 0 ||
            totalFragments > static_cast<uint32_t>(kWords * 32))
            return false;

        // Full words that must be all-ones.
        const uint32_t full_words = totalFragments / 32u;
        for (uint32_t w = 0; w < full_words; ++w) {
            if (receivedBitmap[w] != 0xFFFF'FFFFu) return false;
        }
        // Partial final word.
        const uint32_t rem = totalFragments % 32u;
        if (rem > 0) {
            // The top `rem` bits of receivedBitmap[full_words] must be set.
            const uint32_t mask = ~((1u << (32u - rem)) - 1u);
            if ((receivedBitmap[full_words] & mask) != mask) return false;
        }
        return true;
    }

    // ----------------------------------------------------------
    // missing_fragments()
    //
    // Builds the FragmentNumberSet of fragments that have NOT been
    // received, up to last_known_frag_num (from HEARTBEAT_FRAG).
    //
    // "The NACK_FRAG contains the set of fragment numbers that the
    //  Reader does not have."  — §8.4.12, p. 153.
    //
    // @param last_known  lastFragmentNum from HEARTBEAT_FRAG — the
    //                    writer's highest fragment number sent.
    //                    Only fragments up to this value are NACKable.
    // @param out         Populated with missing fragment numbers.
    // ----------------------------------------------------------
    void missing_fragments(uint32_t last_known, FragmentNumberSet& out) const noexcept {
        out.clear();
        const uint32_t up_to = (last_known < totalFragments)
                               ? last_known : totalFragments;
        for (uint32_t f = 1; f <= up_to; ++f) {
            if (!is_received(f)) out.add(f);
        }
    }

    // ----------------------------------------------------------
    // write_fragment()
    //
    // Copies one fragment's bytes into the assembly buffer at the
    // correct offset.
    //
    // §9.4.5.4, p. 208:
    //   "fragmentStartingNum is the 1-based index of the first fragment
    //    contained in this DATA_FRAG.  The byte offset of that fragment
    //    in the serialized payload is (fragmentStartingNum - 1) * fragmentSize."
    //
    // @param frag_num    1-based fragment number being written.
    // @param src         Pointer to the fragment bytes (from DATA_FRAG).
    // @param src_len     Byte count for this fragment.
    // @returns false if src_len or offset would exceed the buffer.
    // ----------------------------------------------------------
    bool write_fragment(uint32_t    frag_num,
                        const Octet* src,
                        uint32_t     src_len) noexcept {
        if (!buffer.valid() || frag_num < 1 || frag_num > totalFragments)
            return false;

        const uint32_t byte_offset = (frag_num - 1u) *
                                     static_cast<uint32_t>(fragmentSize);
        if (byte_offset + src_len > buffer.capacity()) return false;
        if (src_len > 0 && src) {
            memcpy(buffer.data + byte_offset, src, src_len);
        }
        return true;
    }

    // ----------------------------------------------------------
    // take_as_cache_change()
    //
    // Converts the completed assembly into a CacheChange ready for
    // insertion into the HistoryCache.  Transfers buffer ownership.
    // After this call the FragmentedChange is cleared.
    //
    // "Once all fragments have been received the FragmentedChange
    //  becomes a CacheChange."  — §8.4.14, p. 157.
    // ----------------------------------------------------------
    CacheChange take_as_cache_change(const Timestamp& source_ts) noexcept {
        CacheChange ch{};
        ch.kind            = ChangeKind::ALIVE;
        ch.writerGuid      = writerGuid;
        ch.sequenceNumber  = sequenceNumber;
        ch.sourceTimestamp = source_ts;
        ch.inlineQos       = inlineQos;
        ch.payload.size    = dataSize;
        // Move the buffer into the CacheChange.
        ch.payload = static_cast<BufRef&&>(buffer);
        ch.payload.size = dataSize;
        // Clear this slot (buffer is now owned by ch).
        sequenceNumber = SEQUENCENUMBER_UNKNOWN;
        writerGuid     = GUID_UNKNOWN;
        fragmentSize   = 0;
        dataSize       = 0;
        totalFragments = 0;
        inlineQos      = {};
        memset(receivedBitmap, 0, sizeof(receivedBitmap));
        return ch;
    }
};

// ============================================================
// FragmentAssembler<MaxSamples>
//
// §8.4.14, p. 156 — manages a pool of FragmentedChange slots.
//
// MaxSamples: number of simultaneously in-flight fragmented samples
// this reader can assemble.  One slot per matched writer is typical;
// default is RTPS_MAX_MATCHED_WRITERS.
//
// KEEP_LAST(1) policy (§8.3.3.1, p. 68):
//   When a new SN arrives from a writer that already has an active
//   slot, the old slot is cleared and reused.  This ensures a
//   slow writer cannot block the assembler table indefinitely.
// ============================================================

#ifndef RTPS_MAX_ASSEMBLY_SLOTS
#  define RTPS_MAX_ASSEMBLY_SLOTS RTPS_MAX_MATCHED_WRITERS
#endif

template<uint8_t MaxSamples = RTPS_MAX_ASSEMBLY_SLOTS>
class FragmentAssembler {
    static_assert(MaxSamples >= 1, "FragmentAssembler needs at least one slot");

public:

    FragmentAssembler() noexcept = default;

    // ----------------------------------------------------------
    // on_data_frag()
    //
    // Process one DATA_FRAG submessage.
    //
    // §8.4.14, p. 156 — algorithm:
    //   1. Find or create a FragmentedChange slot for
    //      (writerGuid, writerSN).
    //   2. For each fragment number in
    //      [fragmentStartingNum .. fragmentStartingNum +
    //       fragmentsInSubmessage - 1]:
    //        a. write_fragment() into the assembly buffer.
    //        b. mark_received().
    //   3. If is_complete(): return the assembled change via out_change.
    //
    // @param writer_guid         GUID of the sending writer.
    // @param writer_sn           SequenceNumber of the sample.
    // @param frag_start          1-based first fragment in this submessage.
    // @param frags_in_submsg     Number of fragments in this submessage.
    // @param frag_size           Size of each fragment in bytes.
    // @param data_size           Total serialized payload size.
    // @param payload             Pointer to the fragment bytes.
    // @param payload_len         Byte count in payload.
    // @param source_ts           Source timestamp (from INFO_TS or invalid).
    // @param out_change          Set to the completed CacheChange on return
    //                            of true; untouched otherwise.
    // @param inline_qos          This fragment's inline QoS, or null when it
    //                            carried none; kept for the completed change.
    // @returns true when the sample is complete and out_change is valid.
    // ----------------------------------------------------------
    bool on_data_frag(const Guid&           writer_guid,
                      const SequenceNumber& writer_sn,
                      uint32_t              frag_start,
                      uint16_t              frags_in_submsg,
                      uint16_t              frag_size,
                      uint32_t              data_size,
                      const Octet*          payload,
                      uint32_t              payload_len,
                      const Timestamp&      source_ts,
                      CacheChange&          out_change,
                      const InlineQos*      inline_qos = nullptr) noexcept
    {
        if (frag_start == 0 || frag_size == 0 || data_size == 0) return false;

        const uint32_t total_frags =
            (data_size + static_cast<uint32_t>(frag_size) - 1u) /
            static_cast<uint32_t>(frag_size);

        if (total_frags > static_cast<uint32_t>(FragmentedChange::kWords * 32))
            return false;   // Beyond 256-fragment limit.

        // 1. Find or allocate slot.
        FragmentedChange* slot = find_slot(writer_guid, writer_sn);
        if (!slot) {
            slot = alloc_slot(writer_guid, writer_sn, frag_size,
                              data_size, total_frags);
            if (!slot) return false;
        }

        // 2. Write fragments.
        for (uint16_t i = 0; i < frags_in_submsg; ++i) {
            const uint32_t frag_num = frag_start + static_cast<uint32_t>(i);
            if (frag_num > total_frags) break;

            // Byte length of this particular fragment.
            // The last fragment may be smaller than frag_size.
            const uint32_t frag_byte_offset =
                (frag_num - 1u) * static_cast<uint32_t>(frag_size);
            const uint32_t frag_byte_len =
                (frag_num == total_frags)
                    ? (data_size - frag_byte_offset)
                    : static_cast<uint32_t>(frag_size);

            // Source offset within this DATA_FRAG payload.
            const uint32_t src_off = i * static_cast<uint32_t>(frag_size);
            if (src_off + frag_byte_len > payload_len) break;

            // Only a fragment actually stored counts toward completion.
            if (!slot->write_fragment(frag_num, payload + src_off, frag_byte_len)) break;
            if (inline_qos && !inline_qos->empty() && slot->inlineQos.empty()) {
                slot->inlineQos = *inline_qos;
            }
            slot->mark_received(frag_num);
        }

        // 3. Check for completion.
        if (!slot->is_complete()) return false;

        out_change = slot->take_as_cache_change(source_ts);
        return true;
    }

    // ----------------------------------------------------------
    // missing_fragments()
    //
    // Returns the FragmentNumberSet of fragments missing for the
    // sample identified by (writerGuid, writerSN), up to
    // last_known_frag_num.
    //
    // Called from StatefulReader::on_heartbeat_frag() to build a
    // NACK_FRAG.  — §8.4.12, p. 153.
    //
    // @param writer_guid     Writer sending the fragmented sample.
    // @param writer_sn       SN of the fragmented sample.
    // @param last_known      lastFragmentNum from HEARTBEAT_FRAG.
    // @param out             Populated with missing fragment numbers.
    // @returns true if a slot exists (i.e. there is something to NACK).
    // ----------------------------------------------------------
    bool missing_fragments(const Guid&           writer_guid,
                           const SequenceNumber& writer_sn,
                           uint32_t              last_known,
                           FragmentNumberSet&    out) const noexcept
    {
        const FragmentedChange* slot = find_slot(writer_guid, writer_sn);
        if (!slot) return false;
        slot->missing_fragments(last_known, out);
        return true;
    }

    // ----------------------------------------------------------
    // discard()
    //
    // Remove the assembly slot for (writerGuid, writerSN) and
    // release the pool buffer.  Called when a GAP covers the SN
    // (the writer will never send the remaining fragments).
    // ----------------------------------------------------------
    void discard(const Guid&           writer_guid,
                 const SequenceNumber& writer_sn) noexcept
    {
        FragmentedChange* slot = find_slot(writer_guid, writer_sn);
        if (slot) slot->clear();
    }

    /**
     * @brief Drops every assembly from `writer_guid` whose SN is in [first, end).
     *
     * A GAP's contiguous run may span thousands of sequence numbers, so the
     * slots are scanned instead of the range.
     *
     * @param[in] writer_guid Writer the GAP came from.
     * @param[in] first       First irrelevant sequence number.
     * @param[in] end         One past the last irrelevant sequence number.
     */
    void discard_range(const Guid&           writer_guid,
                       const SequenceNumber& first,
                       const SequenceNumber& end) noexcept
    {
        for (uint8_t i = 0; i < MaxSamples; ++i) {
            if (slots_[i].is_active() && slots_[i].writerGuid == writer_guid &&
                !(slots_[i].sequenceNumber < first) && slots_[i].sequenceNumber < end)
                slots_[i].clear();
        }
    }

    // ----------------------------------------------------------
    // discard_writer()
    //
    // Remove all assembly slots for a given writer.
    // Called from StatefulReader::matched_writer_remove().
    // ----------------------------------------------------------
    void discard_writer(const Guid& writer_guid) noexcept {
        for (uint8_t i = 0; i < MaxSamples; ++i) {
            if (slots_[i].is_active() &&
                slots_[i].writerGuid == writer_guid)
                slots_[i].clear();
        }
    }

    /// Diagnostic: number of active assembly slots.
    uint8_t active_count() const noexcept {
        uint8_t n = 0;
        for (uint8_t i = 0; i < MaxSamples; ++i) {
            if (slots_[i].is_active()) ++n;
        }
        return n;
    }

private:

    FragmentedChange slots_[MaxSamples];

    FragmentedChange* find_slot(const Guid&           wg,
                                const SequenceNumber& sn) noexcept {
        for (uint8_t i = 0; i < MaxSamples; ++i) {
            if (slots_[i].is_active() &&
                slots_[i].writerGuid    == wg &&
                slots_[i].sequenceNumber == sn)
                return &slots_[i];
        }
        return nullptr;
    }

    const FragmentedChange* find_slot(const Guid&           wg,
                                      const SequenceNumber& sn) const noexcept {
        for (uint8_t i = 0; i < MaxSamples; ++i) {
            if (slots_[i].is_active() &&
                slots_[i].writerGuid    == wg &&
                slots_[i].sequenceNumber == sn)
                return &slots_[i];
        }
        return nullptr;
    }

    // ----------------------------------------------------------
    // alloc_slot()
    //
    // Find a free slot, or apply KEEP_LAST(1) eviction:
    //   - If the same writer has an older in-progress slot, evict it.
    //   - Otherwise evict the globally oldest active slot.
    //
    // §8.3.3.1, p. 68 KEEP_LAST(1) eviction policy.
    // ----------------------------------------------------------
    FragmentedChange* alloc_slot(const Guid&           wg,
                                 const SequenceNumber& sn,
                                 uint16_t              frag_size,
                                 uint32_t              data_size,
                                 uint32_t              total_frags) noexcept
    {
        // Prefer an empty slot.
        FragmentedChange* candidate = nullptr;
        for (uint8_t i = 0; i < MaxSamples; ++i) {
            if (!slots_[i].is_active()) { candidate = &slots_[i]; break; }
        }

        // KEEP_LAST(1): evict same writer's older SN.
        if (!candidate) {
            for (uint8_t i = 0; i < MaxSamples; ++i) {
                if (slots_[i].writerGuid == wg) {
                    slots_[i].clear();
                    candidate = &slots_[i];
                    break;
                }
            }
        }

        // Last resort: evict globally oldest slot.
        if (!candidate) {
            FragmentedChange* oldest = nullptr;
            for (uint8_t i = 0; i < MaxSamples; ++i) {
                if (!oldest ||
                    slots_[i].sequenceNumber < oldest->sequenceNumber)
                    oldest = &slots_[i];
            }
            if (oldest) { oldest->clear(); candidate = oldest; }
        }

        if (!candidate) return nullptr;

        // Allocate a pool buffer that holds the whole sample. A sample larger
        // than the largest pool class cannot be reassembled and is refused
        // here rather than delivered short.
        BufRef buf = buf_alloc(data_size);
        if (!buf.valid() || buf.capacity() < data_size) {
            RTPS_LOG_W("frag", "no pool block for a %lu-octet fragmented sample",
                       (unsigned long)data_size);
            return nullptr;
        }

        candidate->writerGuid     = wg;
        candidate->sequenceNumber = sn;
        candidate->fragmentSize   = frag_size;
        candidate->dataSize       = data_size;
        candidate->totalFragments = total_frags;
        candidate->inlineQos      = {};
        candidate->buffer         = static_cast<BufRef&&>(buf);
        memset(candidate->receivedBitmap, 0, sizeof(candidate->receivedBitmap));
        return candidate;
    }
};

/// Default alias — one slot per matched writer.
using DefaultFragmentAssembler = FragmentAssembler<RTPS_MAX_ASSEMBLY_SLOTS>;

} // namespace rtps
