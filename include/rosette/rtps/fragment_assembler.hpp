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
 *  - One FragmentAssembler per StatefulReader, taking its slots from a
 *    FragmentAssemblyPool. A participant's user readers share one pool of
 *    RTPS_ASSEMBLY_SLOTS slots, each reader holding at most
 *    RTPS_ASSEMBLY_SLOTS_PER_READER; the SEDP builtin readers and
 *    standalone readers (BoundedStatefulReader) have private pools.
 *  - Each slot assembles one in-flight fragmented sample and is tagged with
 *    the reader that holds it. A reader holds one partial sample per writer
 *    (KEEP_LAST(1), §8.3.3.1, p. 68): a newer sample from a writer replaces
 *    its older one, and a delayed older one is refused. Only when a sample
 *    comes from a writer the reader holds no slot for and the reader is at
 *    its limit does the reader replace its oldest slot. It replaces only its
 *    own samples. A reader holding no slot when the pool is full
 *    is refused, and a reliable writer repairs the sample (§8.4.12).
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

namespace rosette::rtps {

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
// Assembly slot sizing
// ============================================================

/// Fragmented samples in reassembly at once across a participant's user
/// readers. A slot assembles into one buffer-pool block that holds the whole
/// sample, which is larger than a datagram's payload, so the default is the
/// number of blocks of the datagram class and the largest class: a slot
/// beyond those could never hold a buffer.
#ifndef RTPS_ASSEMBLY_SLOTS
#  define RTPS_ASSEMBLY_SLOTS (RTPS_POOL_CLASS3_COUNT + RTPS_POOL_CLASS4_COUNT)
#endif

/// Fragmented samples one reader may assemble at once. The default leaves
/// one slot of the participant's pool to the other readers, so a burst on
/// one topic never takes the slot another topic's sample needs.
#ifndef RTPS_ASSEMBLY_SLOTS_PER_READER
#  define RTPS_ASSEMBLY_SLOTS_PER_READER (RTPS_ASSEMBLY_SLOTS > 1u ? RTPS_ASSEMBLY_SLOTS - 1u : 1u)
#endif

static_assert(RTPS_ASSEMBLY_SLOTS >= 1u && RTPS_ASSEMBLY_SLOTS <= 65535u,
              "RTPS_ASSEMBLY_SLOTS must be 1 to 65,535");
static_assert(RTPS_ASSEMBLY_SLOTS_PER_READER >= 1u
                  && RTPS_ASSEMBLY_SLOTS_PER_READER <= RTPS_ASSEMBLY_SLOTS,
              "RTPS_ASSEMBLY_SLOTS_PER_READER must be 1 to RTPS_ASSEMBLY_SLOTS");

class FragmentAssembler;

/// @brief One slot of a FragmentAssemblyPool: a sample in reassembly and the reader it belongs to.
struct AssemblySlot {
    FragmentedChange change;
    /// Assembler that holds the slot; meaningful only while `change` is active.
    const FragmentAssembler* owner = nullptr;
    /// Pool allocation count when the slot was taken; orders an owner's
    /// samples by age, so the oldest is replaced first. It is 64 bits wide so
    /// it never wraps and a plain comparison orders any two slots, however
    /// long the older partial sample has been held.
    uint64_t stamp = 0;
};

/**
 * @brief A fixed set of assembly slots that several readers' assemblers share.
 *
 * RTPSParticipant holds one, of RTPS_ASSEMBLY_SLOTS slots, for all of its
 * user readers; BoundedStatefulReader and BoundedFragmentAssembler each hold
 * a private one. Each slot is
 * tagged with the assembler that took it, and an assembler only ever finds,
 * replaces, or discards its own slots, so one reader's traffic never evicts
 * another reader's partial sample (FragmentAssembler::on_data_frag()).
 *
 * The pool holds no slots itself: FragmentAssemblyPoolStorage owns them.
 */
class FragmentAssemblyPool {
public:
    FragmentAssemblyPool(const FragmentAssemblyPool&) = delete;
    FragmentAssemblyPool& operator=(const FragmentAssemblyPool&) = delete;

    /// @brief Slots in the pool.
    uint16_t capacity() const noexcept { return capacity_; }

    /// @brief Slots holding a sample in reassembly, whatever reader holds them.
    uint16_t active_count() const noexcept;

protected:
    /// @brief Creates a pool over `capacity` slots owned by the derived class.
    FragmentAssemblyPool(AssemblySlot* slots, uint16_t capacity) noexcept
        : slots_(slots), capacity_(capacity) {}

    ~FragmentAssemblyPool() noexcept = default;

private:
    friend class FragmentAssembler;

    AssemblySlot* slots_;
    uint16_t capacity_;
    uint64_t next_stamp_ = 0;
};

namespace detail {
/// @brief The slots of a FragmentAssemblyPoolStorage, as a base class so they
/// are constructed before the pool that points at them.
template <uint16_t N>
struct AssemblyPoolSlots {
    AssemblySlot slots[N];
};
}  // namespace detail

/**
 * @brief A FragmentAssemblyPool that owns its N slots.
 *
 * @tparam N Slots in the pool, 1 to 65,535.
 */
template <uint16_t N>
class FragmentAssemblyPoolStorage : private detail::AssemblyPoolSlots<N>,
                                    public FragmentAssemblyPool {
    static_assert(N >= 1u, "an assembly pool holds at least one slot");

public:
    FragmentAssemblyPoolStorage() noexcept : FragmentAssemblyPool(this->slots, N) {}
};

// ============================================================
// FragmentAssembler
// ============================================================

/**
 * @brief One reader's fragment reassembly, in slots taken from a shared pool (§8.4.14).
 *
 * Every StatefulReader holds one. It takes a slot for each fragmented sample
 * it starts, writes each DATA_FRAG's fragments into the slot's buffer, and
 * hands the sample back once every fragment has arrived; the reader then
 * accepts it as it would a DATA. Its missing fragments answer HEARTBEAT_FRAG
 * and the ACKNACK with a NACK_FRAG (§8.4.12), and a GAP or an unmatch
 * discards a partial sample.
 *
 * A reader holds at most `limit` slots. A new sample takes a free slot while
 * the reader is below its limit. Otherwise it replaces the reader's partial
 * sample from the same writer (KEEP_LAST(1) per writer, §8.3.3.1), or else
 * the reader's oldest partial sample. A reader that holds no slot when the
 * pool is full is refused: the sample's fragments are dropped, and a
 * reliable writer repairs them once a slot frees, as it would any lost
 * fragment.
 *
 * The pool tags each slot with the assembler's address, so an assembler is
 * neither copied nor moved; its destructor returns its slots and their
 * buffers to the pool.
 *
 * Verification:
 * - Correct means: a reader never finds, replaces, or discards another
 *   reader's slot; it never holds more than its limit; a refused sample
 *   completes after repair.
 * - Covered by: TC-FAS-8.4.14-001…008, TC-FAS-8.3.3.1-001,
 *   TC-FAS-8.4.14-009…010 (pool sharing), TC-SRR-8.4.12-004 (repair after
 *   refusal).
 *
 * References:
 * - OMG DDSI-RTPS 2.5 §8.4.14 (fragment reassembly), §8.4.12 (NACK_FRAG),
 *   §8.3.3.1 (KEEP_LAST), §9.4.5.4 (DATA_FRAG)
 *
 * Example:
 * @code{.cpp}
 * rosette::rtps::FragmentAssemblyPoolStorage<6> pool;
 * rosette::rtps::FragmentAssembler motion(pool, 5);
 * rosette::rtps::FragmentAssembler map(pool, 5);
 * rosette::rtps::CacheChange done{};
 * if (map.on_data_frag(writer, sn, 1u, 1u, 1024u, 2000u, bytes, 1024u, ts, done)) {
 *     // `done` holds the whole sample.
 * }
 * @endcode
 * Full program: tests/spec/src/test_fragment_8_4_14.cpp (TC-FAS-8.4.14-009)
 */
class FragmentAssembler {
public:
    /**
     * @brief Creates an assembler that holds no slots yet.
     *
     * @param[in] pool  Slots shared with other assemblers; outlives this one.
     * @param[in] limit Most slots this assembler holds at once, 1 to the
     *   pool's capacity.
     */
    FragmentAssembler(FragmentAssemblyPool& pool, uint16_t limit) noexcept
        : pool_(pool), limit_(limit) {}

    /// @brief Returns every slot this assembler holds, and its buffer, to the pool.
    ~FragmentAssembler() noexcept;

    FragmentAssembler(const FragmentAssembler&) = delete;
    FragmentAssembler& operator=(const FragmentAssembler&) = delete;

    /**
     * @brief Writes one DATA_FRAG's fragments into its sample's slot, and returns the sample once
     * complete.
     *
     * Finds the slot of (writer_guid, writer_sn), or takes one by the rule in
     * the class comment and allocates a buffer-pool block that holds the
     * whole sample. Each fragment is copied to offset (n − 1) × frag_size
     * and marked received; the first inline QoS any fragment carries is
     * kept. When every fragment is marked, the slot's buffer moves into
     * `out_change` and the slot frees.
     *
     * @param[in]  writer_guid      GUID of the sending writer.
     * @param[in]  writer_sn        Sequence number of the sample.
     * @param[in]  frag_start       1-based number of the first fragment in the submessage.
     * @param[in]  frags_in_submsg  Fragments in the submessage.
     * @param[in]  frag_size        Octets per fragment; the last may be shorter.
     * @param[in]  data_size        Octets in the whole serialized sample.
     * @param[in]  payload          The submessage's fragment octets.
     * @param[in]  payload_len      Octets at `payload`.
     * @param[in]  source_ts        Source timestamp (INFO_TS), or invalid.
     * @param[out] out_change       The completed sample, written only when this returns true.
     * @param[in]  inline_qos       This fragment's inline QoS, or null when it carried none.
     * @return true when the sample is complete and `out_change` holds it;
     *   false while fragments are outstanding, and when the fields are
     *   invalid, the sample has more than 256 fragments, `frag_size` or
     *   `data_size` differs from the sample already in assembly, the sample
     *   is older than the same writer's in assembly, no slot is available,
     *   or no buffer-pool block holds the sample.
     *
     * Verification:
     * - Correct means: a sample completes exactly once, with its fragments
     *   at their offsets; a refused or invalid fragment takes no slot.
     * - Covered by: TC-FAS-8.4.14-001…010, TC-FAS-8.3.3.1-001.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.4.14, §9.4.5.4
     */
    bool on_data_frag(const Guid& writer_guid, const SequenceNumber& writer_sn, uint32_t frag_start,
                      uint16_t frags_in_submsg, uint16_t frag_size, uint32_t data_size,
                      const Octet* payload, uint32_t payload_len, const Timestamp& source_ts,
                      CacheChange& out_change, const InlineQos* inline_qos = nullptr) noexcept;

    /**
     * @brief Lists the fragments of a partial sample that have not arrived, for a NACK_FRAG.
     *
     * @param[in]  writer_guid Writer of the sample.
     * @param[in]  writer_sn   Sequence number of the sample.
     * @param[in]  last_known  Highest fragment number the writer has sent
     *   (HEARTBEAT_FRAG lastFragmentNum); later fragments are not listed.
     * @param[out] out         The missing fragment numbers.
     * @return true when this assembler holds the sample; false otherwise,
     *   and `out` is untouched.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.4.12, §8.3.7.4
     */
    bool missing_fragments(const Guid& writer_guid, const SequenceNumber& writer_sn,
                           uint32_t last_known, FragmentNumberSet& out) const noexcept;

    /// @brief Drops the partial sample (writer_guid, writer_sn), returning its slot and buffer.
    void discard(const Guid& writer_guid, const SequenceNumber& writer_sn) noexcept;

    /**
     * @brief Drops every partial sample from `writer_guid` whose SN is in [first, end).
     *
     * A GAP's contiguous run may span thousands of sequence numbers, so the
     * slots are scanned instead of the range.
     *
     * @param[in] writer_guid Writer the GAP came from.
     * @param[in] first       First irrelevant sequence number.
     * @param[in] end         One past the last irrelevant sequence number.
     */
    void discard_range(const Guid& writer_guid, const SequenceNumber& first,
                       const SequenceNumber& end) noexcept;

    /// @brief Drops every partial sample from `writer_guid`; called when the writer unmatches.
    void discard_writer(const Guid& writer_guid) noexcept;

    /// @brief Slots this assembler holds.
    uint16_t active_count() const noexcept;

private:
    /// @brief This assembler's slot for (wg, sn), or null.
    AssemblySlot* find_slot(const Guid& wg, const SequenceNumber& sn) const noexcept;

    /**
     * @brief Chooses the slot a new sample (`wg`, `sn`) goes into, without emptying it.
     *
     * Applies the rule in the class comment: this assembler's slot for the
     * same writer if its sample is older than `sn`, else a free slot below
     * the limit, else this assembler's oldest slot. A fragment of a sample no
     * newer than the one in assembly from the same writer is refused, so a
     * delayed fragment never evicts a newer sample or starts beside it.
     *
     * @param[in] wg Writer of the new sample.
     * @param[in] sn Sequence number of the new sample.
     * @return A free slot, or the active slot the caller may replace; null
     *   when the pool is full and this assembler holds no slot to replace,
     *   or the sample is older than the writer's one in assembly.
     *
     * Verification:
     * - Correct means: after any sequence of calls this assembler holds at most one slot per
     *   writer, and it is the one with the newest sample seen for that writer.
     * - Covered by: TC-FAS-8.3.3.1-001, TC-FAS-8.4.14-012, TC-FAS-8.4.14-014.
     */
    AssemblySlot* select_slot(const Guid& wg, const SequenceNumber& sn) noexcept;

    /**
     * @brief Takes a slot for a new sample and gives it a buffer that holds the whole sample.
     *
     * The buffer is obtained before any replaced sample is dropped, so a
     * refusal leaves the existing assembly intact.
     *
     * @return The slot, ready for fragments; null when no slot is available
     *   or no buffer-pool block holds `data_size` octets.
     */
    FragmentedChange* alloc_slot(const Guid& wg, const SequenceNumber& sn, uint16_t frag_size,
                                 uint32_t data_size, uint32_t total_frags) noexcept;

    FragmentAssemblyPool& pool_;
    uint16_t limit_;
};

namespace detail {
/// @brief The private pool of a BoundedFragmentAssembler, as a base class so
/// it is constructed before the assembler that points into it.
template <uint16_t N>
struct PrivateAssemblyPool {
    FragmentAssemblyPoolStorage<N> assembly_pool;
};
}  // namespace detail

/**
 * @brief A FragmentAssembler with a private pool of N slots, all its own.
 *
 * For reassembly outside a reader, such as the fragment spec tests and the
 * fuzz harness, where one assembler has every slot to itself.
 *
 * @tparam N Slots, 1 to 65,535.
 */
template <uint16_t N>
class BoundedFragmentAssembler final : private detail::PrivateAssemblyPool<N>,
                                       public FragmentAssembler {
public:
    BoundedFragmentAssembler() noexcept : FragmentAssembler(this->assembly_pool, N) {}
};

}  // namespace rosette::rtps
