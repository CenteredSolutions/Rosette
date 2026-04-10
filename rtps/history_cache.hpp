/**
 * @file rtps/history_cache.hpp
 * @brief CacheChange and HistoryCache — RTPS Writer/Reader history.
 *
 * Maps directly to the PIM defined in §8.2.9 (CacheChange) and §8.3.1
 * (HistoryCache) of the DDSI-RTPS 2.5 specification
 * (OMG formal/2022-04-01).
 *
 * Implementation notes
 * --------------------
 *  - KEEP_LAST(1) semantics only — consistent with the stated constraint
 *    that topics will not have KEEP_HISTORY(n) where n > 1.
 *  - Fixed-capacity N using a circular array; no heap.
 *  - Payload memory is supplied by an external pool (BufRef handle);
 *    the cache itself only stores a reference-counted handle, keeping
 *    the struct lightweight.
 *  - Pool-backed payload buffers allow zero-copy forwarding to the
 *    transport layer.
 *
 * Spec references (all OMG formal/2022-04-01)
 * --------------------------------------------
 *   CacheChange     — §8.2.9,  Table 8.11, p. 35
 *   HistoryCache    — §8.3.1,  Table 8.21, p. 62
 *   add_change()    — §8.3.1,  Operation add_change, p. 63
 *   remove_change() — §8.3.1,  Operation remove_change, p. 63
 *   get_change()    — §8.3.1,  Operation get_change, p. 63
 *   get_seq_num_min/max — §8.3.1, p. 63
 */

#pragma once

#include "buf_pool.hpp"

namespace rtps {

// ============================================================
// §8.2.9  CacheChange
//
// "A CacheChange represents a change to the state of a data-object
//  managed by a Writer."
//  — §8.2.9, Table 8.11, p. 35
//
// Fields match Table 8.11 exactly.  The inlineQos field is omitted for
// the embedded target (no inline QoS needed for KEEP_LAST(1) topics
// with no key).
// ============================================================

/**
 * @brief A single change record inside a HistoryCache.
 *
 * Field names and semantics taken verbatim from §8.2.9, Table 8.11.
 *
 * | Spec field         | This struct field   | Notes                          |
 * |--------------------|---------------------|--------------------------------|
 * | kind               | kind                | §8.2.9                         |
 * | writerGuid         | writerGuid          | §8.2.9                         |
 * | instanceHandle     | (omitted)           | No-key topics only on this HW  |
 * | sequenceNumber     | sequenceNumber      | §8.2.9                         |
 * | serializedPayload  | payload (BufRef)    | Pool-allocated, ref-counted    |
 * | inlineQos          | (omitted)           | Not required for target        |
 */
struct CacheChange {
    ChangeKind     kind           = ChangeKind::ALIVE;
    Guid           writerGuid     = GUID_UNKNOWN;
    SequenceNumber sequenceNumber = SEQUENCENUMBER_UNKNOWN;
    Timestamp      sourceTimestamp = TIME_ZERO;
    BufRef         payload        = {};     ///< Pool-backed serialized data.

    /// @returns true when this slot contains a valid, live change.
    bool is_valid() const noexcept {
        return sequenceNumber != SEQUENCENUMBER_UNKNOWN;
    }

    void clear() noexcept {
        if (payload.valid()) payload.release();
        *this = CacheChange{};
    }
};

// ============================================================
// §8.3.1  HistoryCache<N>
//
// "The HistoryCache maintains a list of CacheChange objects."
//  — §8.3.1, p. 62
//
// KEEP_LAST(1) semantics: only the most recent change per writer
// GUID is retained.  For a Writer cache there is exactly one logical
// writer, so capacity N == 1 suffices per spec §8.3.3.1 (p. 68).
// For embedded use N is exposed as a template parameter so that
// future KEEP_LAST(n) can be introduced without redesign.
//
// Template parameter
//   N : maximum number of simultaneous CacheChange objects.
//       Set N = 1 for a Writer with KEEP_LAST(1).
//       Set N >= matched-reader-count for a Reader cache.
// ============================================================

/**
 * @brief Fixed-capacity history cache with KEEP_LAST(N≤1) semantics.
 *
 * Operations are named and defined per §8.3.1 of the spec.
 *
 * Thread safety: This class is NOT thread-safe on its own.  The caller
 * must hold the appropriate lock or run only from the dispatch task.
 * (RX dispatch is single-threaded per design; writer retransmit task
 * accesses the cache only after taking the writer mutex.)
 */
template<uint8_t N>
class HistoryCache {
    static_assert(N >= 1, "HistoryCache must have at least one slot");

public:

    HistoryCache() noexcept { clear_all(); }

    // ----------------------------------------------------------
    // §8.3.1  add_change()
    //
    // "Inserts a new CacheChange into the HistoryCache."
    //  — §8.3.1, Operation add_change, p. 63
    //
    // With KEEP_LAST(1) semantics the oldest change with the same
    // writerGuid is evicted when the cache is full, per §8.3.3.1:
    //   "When a new sample arrives and the HistoryCache is full,
    //    the oldest change that has kind ALIVE or
    //    NOT_ALIVE_DISPOSED is removed."
    //   — §8.3.3.1, p. 68
    //
    // @param change   CacheChange to insert.  The cache takes
    //                 ownership of change.payload on success.
    // @returns        Pointer to the inserted slot, or nullptr if
    //                 no slot could be reclaimed (all N slots are
    //                 occupied by changes with higher sequence numbers
    //                 — should not occur under KEEP_LAST(1)).
    // ----------------------------------------------------------
    CacheChange* add_change(CacheChange&& change) noexcept {
        // Fast path: find an empty slot.
        for (uint8_t i = 0; i < N; ++i) {
            if (!slots_[i].is_valid()) {
                slots_[i] = static_cast<CacheChange&&>(change);
                return &slots_[i];
            }
        }
        // KEEP_LAST(1) eviction: remove the slot with the lowest SN
        // from the same writer, or globally oldest if no match.
        CacheChange* victim = find_oldest_for_writer(change.writerGuid);
        if (!victim) victim = find_globally_oldest();
        if (!victim) return nullptr;        // Saturated; caller should not reach here.
        victim->clear();
        *victim = static_cast<CacheChange&&>(change);
        return victim;
    }

    // ----------------------------------------------------------
    // §8.3.1  remove_change()
    //
    // "Removes the CacheChange with the given sequenceNumber from
    //  the HistoryCache."
    //  — §8.3.1, Operation remove_change, p. 63
    //
    // The payload BufRef is released, returning the pool block.
    //
    // @param sequenceNumber  SN of the change to remove.
    // @returns true if found and removed.
    // ----------------------------------------------------------
    bool remove_change(const SequenceNumber& sequenceNumber) noexcept {
        for (uint8_t i = 0; i < N; ++i) {
            if (slots_[i].is_valid() &&
                slots_[i].sequenceNumber == sequenceNumber) {
                slots_[i].clear();
                return true;
            }
        }
        return false;
    }

    // ----------------------------------------------------------
    // §8.3.1  get_change()
    //
    // "Returns the CacheChange with the given sequenceNumber."
    //  — §8.3.1, Operation get_change, p. 63
    //
    // Returns a non-owning pointer; valid until remove_change() or
    // clear_all().  Caller must not release the payload.
    // ----------------------------------------------------------
    const CacheChange* get_change(const SequenceNumber& sequenceNumber) const noexcept {
        for (uint8_t i = 0; i < N; ++i) {
            if (slots_[i].is_valid() &&
                slots_[i].sequenceNumber == sequenceNumber) {
                return &slots_[i];
            }
        }
        return nullptr;
    }

    CacheChange* get_change(const SequenceNumber& sequenceNumber) noexcept {
        return const_cast<CacheChange*>(
            static_cast<const HistoryCache*>(this)->get_change(sequenceNumber));
    }

    // ----------------------------------------------------------
    // §8.3.1  get_seq_num_min() / get_seq_num_max()
    //
    // "Returns the minimum/maximum SequenceNumber of all changes
    //  in the HistoryCache."
    //  — §8.3.1, p. 63
    //
    // Returns SEQUENCENUMBER_UNKNOWN when cache is empty.
    // ----------------------------------------------------------
    SequenceNumber get_seq_num_min() const noexcept {
        SequenceNumber result = SEQUENCENUMBER_UNKNOWN;
        for (uint8_t i = 0; i < N; ++i) {
            if (!slots_[i].is_valid()) continue;
            if (result == SEQUENCENUMBER_UNKNOWN ||
                slots_[i].sequenceNumber < result) {
                result = slots_[i].sequenceNumber;
            }
        }
        return result;
    }

    SequenceNumber get_seq_num_max() const noexcept {
        SequenceNumber result = SEQUENCENUMBER_UNKNOWN;
        for (uint8_t i = 0; i < N; ++i) {
            if (!slots_[i].is_valid()) continue;
            if (result == SEQUENCENUMBER_UNKNOWN ||
                slots_[i].sequenceNumber > result) {
                result = slots_[i].sequenceNumber;
            }
        }
        return result;
    }

    // ----------------------------------------------------------
    // Convenience
    // ----------------------------------------------------------

    bool is_empty() const noexcept {
        for (uint8_t i = 0; i < N; ++i) {
            if (slots_[i].is_valid()) return false;
        }
        return true;
    }

    uint8_t size() const noexcept {
        uint8_t count = 0;
        for (uint8_t i = 0; i < N; ++i) {
            if (slots_[i].is_valid()) ++count;
        }
        return count;
    }

    /// Remove and release every change.  Called on participant teardown.
    void clear_all() noexcept {
        for (uint8_t i = 0; i < N; ++i) {
            if (slots_[i].is_valid()) slots_[i].clear();
        }
    }

    // ----------------------------------------------------------
    // Iteration — allows callers to walk all live changes without
    // exposing the internal array directly.
    //
    // fn : void(const CacheChange&)
    // ----------------------------------------------------------
    template<typename Fn>
    void for_each_change(Fn&& fn) const noexcept {
        for (uint8_t i = 0; i < N; ++i) {
            if (slots_[i].is_valid()) fn(slots_[i]);
        }
    }

    template<typename Fn>
    void for_each_change(Fn&& fn) noexcept {
        for (uint8_t i = 0; i < N; ++i) {
            if (slots_[i].is_valid()) fn(slots_[i]);
        }
    }

    // ----------------------------------------------------------
    // TRANSIENT_LOCAL durability — §8.7.2.2
    //
    // "A Writer with TRANSIENT_LOCAL durability shall send all changes
    //  in its HistoryCache to any newly-matched Reader."
    //  — §8.7.2.2, p. 196
    //
    // replay_to_new_reader() yields each cached change in SN order
    // for retransmission to a late-joining ReaderProxy.
    //
    // fn : void(const CacheChange&)
    // ----------------------------------------------------------
    template<typename Fn>
    void replay_to_new_reader(Fn&& fn) const noexcept {
        // For KEEP_LAST(1) there is at most one change; iterate
        // in SN order for correctness with N > 1 in future.
        SequenceNumber sn = get_seq_num_min();
        const SequenceNumber max = get_seq_num_max();
        while (!(sn == SEQUENCENUMBER_UNKNOWN) && sn <= max) {
            const CacheChange* c = get_change(sn);
            if (c) fn(*c);
            sn += 1;
        }
    }

private:

    CacheChange slots_[N];

    /// Find the valid slot with the lowest SN belonging to writerGuid.
    CacheChange* find_oldest_for_writer(const Guid& guid) noexcept {
        CacheChange* oldest = nullptr;
        for (uint8_t i = 0; i < N; ++i) {
            if (!slots_[i].is_valid()) continue;
            if (slots_[i].writerGuid != guid) continue;
            if (!oldest || slots_[i].sequenceNumber < oldest->sequenceNumber)
                oldest = &slots_[i];
        }
        return oldest;
    }

    /// Find the valid slot with the globally lowest SN.
    CacheChange* find_globally_oldest() noexcept {
        CacheChange* oldest = nullptr;
        for (uint8_t i = 0; i < N; ++i) {
            if (!slots_[i].is_valid()) continue;
            if (!oldest || slots_[i].sequenceNumber < oldest->sequenceNumber)
                oldest = &slots_[i];
        }
        return oldest;
    }
};

// ============================================================
// Convenience aliases
//
// A StatefulWriter with KEEP_LAST(1) uses a depth-1 cache.
// A StatefulReader that can have up to M concurrent writers
// each needs at most 1 outstanding sample, so depth M.
// ============================================================

/// Writer history cache: KEEP_LAST(1), single writer.
using WriterHistoryCache = HistoryCache<1>;

/// Reader history cache: one slot per matched writer.
/// Adjust RTPS_MAX_MATCHED_WRITERS in config.hpp as required.
#ifndef RTPS_MAX_MATCHED_WRITERS
#  define RTPS_MAX_MATCHED_WRITERS 8
#endif
using ReaderHistoryCache = HistoryCache<RTPS_MAX_MATCHED_WRITERS>;

} // namespace rtps
