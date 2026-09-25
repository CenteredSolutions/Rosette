/**
 * @file rtps/history_cache.hpp
 * @brief CacheChange and HistoryCache — the RTPS writer history.
 *
 * Maps to the PIM of the DDSI-RTPS 2.5 specification (OMG formal/2022-04-01):
 * CacheChange (§8.2.9) and HistoryCache (§8.3.1).
 *
 * Design
 * ------
 *  - HistoryCache is one compiled class (src/rtps/history_cache.cpp) that
 *    operates on slot storage it is given: a pointer to CacheChange slots and
 *    their count. HistoryCacheStorage<N> owns an array of N slots and hands it
 *    to the HistoryCache base, so a cache of any depth is laid out exactly
 *    while every depth shares one copy of the code.
 *  - Fixed capacity, no heap. Payload memory comes from the buffer pool
 *    (BufRef handle); a slot holds only the reference-counted handle, so
 *    payloads are forwarded to the transport without copying.
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
// Fields follow Table 8.11. instanceHandle is omitted: ROS 2 topics are
// keyless, so every change belongs to the one instance of its writer.
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
 * | inlineQos          | inlineQos           | Minimal DATA InlineQoS support |
 */
struct CacheChange {
    ChangeKind     kind           = ChangeKind::ALIVE;
    Guid           writerGuid     = GUID_UNKNOWN;
    SequenceNumber sequenceNumber = SEQUENCENUMBER_UNKNOWN;
    Timestamp      sourceTimestamp = TIME_ZERO;
    InlineQos      inlineQos      = {};
    BufRef         payload        = {};     ///< Pool-backed serialized data.
    /// True when `payload` is a complete serializedPayload (encapsulation
    /// header included, as SEDP's PL_CDR parameter lists are); false when
    /// it is the CDR body and the sender prepends the CDR_LE header.
    bool           preframed      = false;

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
// §8.3.1  HistoryCache
//
// "The HistoryCache maintains a list of CacheChange objects."
//  — §8.3.1, p. 62
// ============================================================

/**
 * @brief Fixed-capacity history of CacheChanges with KEEP_LAST eviction.
 *
 * A StatefulWriter keeps every change it has written here until each matched
 * reliable reader acknowledges it or KEEP_LAST(depth) evicts it; the writer's
 * HEARTBEAT range (firstSN, lastSN) is get_seq_num_min() / get_seq_num_max(),
 * and repairs are served from get_change(). Operations are named and defined
 * per §8.3.1.
 *
 * The cache does not own its slots: it operates on the `capacity` slots the
 * derived HistoryCacheStorage<N> passes to the constructor, so there is one
 * compiled copy of this class for every depth. It is neither copyable nor
 * movable, because the slot pointer refers into the object that owns it.
 *
 * Thread safety: not thread-safe on its own. The caller holds the
 * participant lock (ParticipantDriver::mutex()) for every call.
 */
class HistoryCache {
public:
    HistoryCache(const HistoryCache&) = delete;
    HistoryCache& operator=(const HistoryCache&) = delete;

    /**
     * @brief Inserts a change, evicting the oldest one when the cache is full.
     *
     * Implements §8.3.1 add_change(). When every slot is occupied, the change
     * with the lowest sequence number from the same writer is removed and its
     * payload released (KEEP_LAST, §8.3.3.1); when no slot holds a change from
     * that writer, the globally oldest change is removed instead.
     *
     * @param[in] change Change to insert; the cache takes ownership of
     *   change.payload.
     * @return The slot now holding the change. Never null: a full cache always
     *   has a change to evict.
     *
     * Verification:
     * - Correct means: after capacity() insertions from one writer, the next
     *   insertion removes exactly the lowest sequence number and keeps the rest.
     * - Covered by: TC-HC-8.3.1-001, TC-HC-8.3.3.1-001, TC-HC-8.3.3.1-002.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.3.1 (add_change), §8.3.3.1 (KEEP_LAST)
     */
    CacheChange* add_change(CacheChange&& change) noexcept;

    /**
     * @brief Removes the change with `sequenceNumber` and releases its payload.
     *
     * @param[in] sequenceNumber Sequence number of the change to remove.
     * @return `true` when a change was removed; `false` when none had that
     *   sequence number.
     *
     * Verification:
     * - Correct means: a removed change is no longer returned by get_change()
     *   and its pool block is returned; an absent number changes nothing.
     * - Covered by: TC-HC-8.3.1-002.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.3.1 (remove_change)
     */
    bool remove_change(const SequenceNumber& sequenceNumber) noexcept;

    /**
     * @brief Finds the change with `sequenceNumber`.
     *
     * @param[in] sequenceNumber Sequence number to look up.
     * @return Non-owning pointer, valid until the change is removed or
     *   evicted; null when no change has that number. The caller must not
     *   release its payload.
     *
     * Verification:
     * - Covered by: TC-HC-8.3.1-001.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.3.1 (get_change)
     */
    const CacheChange* get_change(const SequenceNumber& sequenceNumber) const noexcept;

    /// @brief Mutable form of get_change(const SequenceNumber&) const.
    CacheChange* get_change(const SequenceNumber& sequenceNumber) noexcept {
        return const_cast<CacheChange*>(
            static_cast<const HistoryCache*>(this)->get_change(sequenceNumber));
    }

    /**
     * @brief Lowest sequence number in the cache.
     *
     * @return The minimum, or SEQUENCENUMBER_UNKNOWN when the cache is empty.
     *
     * Verification:
     * - Covered by: TC-HC-8.3.1-001, TC-HC-8.3.1-004.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.3.1 (get_seq_num_min)
     */
    SequenceNumber get_seq_num_min() const noexcept;

    /**
     * @brief Highest sequence number in the cache.
     *
     * @return The maximum, or SEQUENCENUMBER_UNKNOWN when the cache is empty.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.3.1 (get_seq_num_max)
     */
    SequenceNumber get_seq_num_max() const noexcept;

    /// @brief True when no slot holds a change.
    bool is_empty() const noexcept { return size() == 0u; }

    /// @brief Number of changes held.
    uint8_t size() const noexcept;

    /// @brief Number of slots: the most changes the cache holds at once.
    uint8_t capacity() const noexcept { return capacity_; }

    /**
     * @brief Removes every change and releases each payload.
     *
     * Verification:
     * - Covered by: TC-HC-8.3.1-004.
     */
    void clear_all() noexcept;

    /**
     * @brief Calls `fn(const CacheChange&)` for each change held, in slot order.
     *
     * Verification:
     * - Covered by: TC-HC-8.3.1-003.
     */
    template<typename Fn>
    void for_each_change(Fn&& fn) const noexcept {
        for (uint8_t i = 0; i < capacity_; ++i) {
            if (slots_[i].is_valid()) fn(static_cast<const CacheChange&>(slots_[i]));
        }
    }

    /// @brief Calls `fn(CacheChange&)` for each change held, in slot order.
    template<typename Fn>
    void for_each_change(Fn&& fn) noexcept {
        for (uint8_t i = 0; i < capacity_; ++i) {
            if (slots_[i].is_valid()) fn(slots_[i]);
        }
    }

    /**
     * @brief Calls `fn(const CacheChange&)` for each change in ascending
     *   sequence-number order.
     *
     * A TRANSIENT_LOCAL writer uses this to send its history to a newly
     * matched reader (§8.7.2.2). Sequence numbers absent from the cache are
     * skipped.
     *
     * Verification:
     * - Correct means: every held change is visited once, lowest number first.
     * - Covered by: TC-HC-8.2.2-001.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.7.2.2 (TRANSIENT_LOCAL)
     */
    template<typename Fn>
    void replay_to_new_reader(Fn&& fn) const noexcept {
        SequenceNumber sn = get_seq_num_min();
        const SequenceNumber max = get_seq_num_max();
        while (!(sn == SEQUENCENUMBER_UNKNOWN) && sn <= max) {
            const CacheChange* c = get_change(sn);
            if (c) fn(*c);
            sn += 1;
        }
    }

protected:
    /**
     * @brief Binds the cache to slot storage owned by the derived object.
     *
     * The slots must already be constructed (HistoryCacheStorage<N> holds
     * them in a base class initialized before this one) and outlive the
     * cache. The constructor does not touch them.
     *
     * @param[in] slots    First of `capacity` default-constructed (empty) slots.
     * @param[in] capacity Number of slots, at least 1.
     */
    HistoryCache(CacheChange* slots, uint8_t capacity) noexcept
        : slots_(slots), capacity_(capacity) {}

    ~HistoryCache() = default;

private:
    /// @brief The held change with the lowest sequence number from `guid`, or null.
    CacheChange* find_oldest_for_writer(const Guid& guid) noexcept;

    /// @brief The held change with the lowest sequence number, or null.
    CacheChange* find_globally_oldest() noexcept;

    CacheChange* slots_;
    uint8_t capacity_;
};

namespace detail {
/// @brief The slot array of a HistoryCacheStorage, as a base class so it is
/// constructed before the HistoryCache base that points into it.
template<uint8_t N>
struct HistoryCacheSlots {
    CacheChange slots[N];
};
}  // namespace detail

/**
 * @brief A HistoryCache with storage for N changes.
 *
 * Holds only the slot array; every operation is HistoryCache's. Use it
 * wherever a cache is owned (a writer's history, a test); pass it as
 * `HistoryCache&` everywhere else.
 *
 * @tparam N Number of slots (the writer's KEEP_LAST depth), at least 1.
 */
template<uint8_t N>
class HistoryCacheStorage : private detail::HistoryCacheSlots<N>, public HistoryCache {
    static_assert(N >= 1, "HistoryCache must have at least one slot");

public:
    HistoryCacheStorage() noexcept : HistoryCache(this->slots, N) {}
};

} // namespace rtps
