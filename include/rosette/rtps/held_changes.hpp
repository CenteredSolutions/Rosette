// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file rtps/held_changes.hpp
 * @brief HeldChangePool and HeldChanges: changes reliable readers hold behind a gap (§8.4.10.4).
 *
 * A reliable StatefulReader makes each writer's changes available in the
 * order written. A change that arrives while an earlier one from the same
 * writer is still MISSING cannot be delivered yet, and dropping it would
 * cost a repair round trip per change after the gap. The reader keeps it,
 * payload and all, until the gap below it closes (the missing change
 * arrives, or a GAP or HEARTBEAT resolves it), then delivers the held run in
 * order.
 *
 * Design
 * ------
 *  - The slots live in a HeldChangePool, which a participant's user readers
 *    share (RTPS_HELD_CHANGES slots); each reader holds at most its limit
 *    (RTPS_HELD_CHANGES_PER_READER) through its HeldChanges view. A
 *    BoundedStatefulReader, such as an SEDP builtin reader, has a private
 *    pool of RTPS_BOUNDED_READER_HELD_CHANGES slots. This is the layout of
 *    the fragment assembly pool (fragment_assembler.hpp): readers that are
 *    quiet cost nothing, and a reader that is losing changes may use more
 *    than an even share.
 *  - A reader holds only the `limit` changes after the next one it expects
 *    from a writer (StatefulReader::accept_change()): those are exactly the
 *    changes its slots can always cover, so no held change is ever given up
 *    to admit a lower one. A change above that window, or one that finds no
 *    free slot, is dropped and requested again. The ACKNACK still requests
 *    every MISSING change: a writer resends them in order, and each closes
 *    the gap in turn without a slot, so a long gap is repaired in one round
 *    trip.
 *  - Held changes never take the last block an ACKNACK can be built in
 *    (StatefulReader::copy_payload()): a reader that could not send its
 *    ACKNACK could not request the change that releases what it holds, and
 *    would stall with its writer still sending HEARTBEATs.
 *  - The change that closes a gap needs no slot: it is delivered at once,
 *    with the held run behind it. It does need a buffer block, and every held
 *    change pins one; when the buffer pool is exhausted, the closing change
 *    takes the block of a held change in the same pool, its own reader's
 *    first, otherwise another reader's (evict_for_block()). The evicted
 *    change is MISSING again and is requested again. Without this, a writer
 *    that falls silent with changes held would pin blocks another reader's
 *    gap needs, until its lease expires.
 *
 * tests/formal/RosetteReaderDelivery.tla checks these rules (AdmitRule
 * "window", BlockRule "evict_any"): changes are delivered in order, nothing
 * is acknowledged before it is delivered, the pool and each reader's limit
 * are never exceeded, and every reader whose writer keeps going finishes,
 * even when another reader's writer stops with changes held.
 *
 * Every reader of one pool runs under its participant's receive pump, as for
 * the fragment assembly pool, so the pool takes no lock.
 */

#pragma once

#include <cstdint>

#include "history_cache.hpp"
#include "types.hpp"

/// Hold slots a participant's user readers share. Each pins a pool block
/// while in use.
#ifndef RTPS_HELD_CHANGES
#  define RTPS_HELD_CHANGES 8u
#endif

/// Most hold slots one user reader occupies, and so how far past its next
/// expected change from a writer it accepts a change (its window).
#ifndef RTPS_HELD_CHANGES_PER_READER
#  define RTPS_HELD_CHANGES_PER_READER 4u
#endif

/// Hold slots in the private pool of each BoundedStatefulReader, such as the
/// two SEDP builtin readers.
#ifndef RTPS_BOUNDED_READER_HELD_CHANGES
#  define RTPS_BOUNDED_READER_HELD_CHANGES 2u
#endif

namespace rosette::rtps {

class StatefulReader;

/// @brief One held change and the reader holding it; `owner` is null when free.
struct HeldSlot {
    CacheChange     change{};
    StatefulReader* owner = nullptr;
};

/// @brief A held change given up to free its buffer block, and whose it was.
struct HeldEviction {
    StatefulReader* owner = nullptr;
    Guid            writer = GUID_UNKNOWN;
    SequenceNumber  sn = SEQUENCENUMBER_UNKNOWN;
};

/**
 * @brief Hold slots shared by the readers that take them through HeldChanges.
 *
 * The pool only owns storage and keeps its used slots dense; HeldChanges
 * decides what to hold, release, and evict. Derive from it through
 * HeldChangePoolStorage, which supplies the slots.
 */
class HeldChangePool {
public:
    HeldChangePool(const HeldChangePool&) = delete;
    HeldChangePool& operator=(const HeldChangePool&) = delete;

    /// @brief Slots in the pool.
    uint8_t capacity() const noexcept { return capacity_; }

    /// @brief Slots holding a change, whatever reader holds them.
    uint8_t count() const noexcept { return count_; }

protected:
    /// @brief Creates a pool over `capacity` slots owned by the derived class.
    HeldChangePool(HeldSlot* slots, uint8_t capacity) noexcept
        : slots_(slots), capacity_(capacity) {}

    ~HeldChangePool() noexcept = default;

private:
    friend class HeldChanges;

    /// Releases slot `index`'s payload and moves the last used slot into it.
    void remove_at(uint8_t index) noexcept;

    HeldSlot* slots_;
    uint8_t   capacity_;
    uint8_t   count_ = 0;
};

namespace detail {
/// @brief The slots of a HeldChangePoolStorage, as a base class so they are
/// constructed before the pool that points at them.
template <uint8_t N>
struct HeldPoolSlots {
    HeldSlot slots[N];
};
}  // namespace detail

/// @brief A HeldChangePool with `N` slots of its own.
template <uint8_t N>
class HeldChangePoolStorage : private detail::HeldPoolSlots<N>, public HeldChangePool {
    static_assert(N >= 1u, "a hold pool holds at least one change");

public:
    HeldChangePoolStorage() noexcept : HeldChangePool(this->slots, N) {}
};

/**
 * @brief One reader's changes held in a HeldChangePool until they can be delivered in order.
 *
 * The view knows nothing of gaps: StatefulReader decides what to hold, what
 * to release (every held change of a writer below that writer's first
 * unresolved sequence number), and when to evict. Changes are found by
 * scanning the pool, which is small.
 *
 * Verification:
 * - Correct means: a held change is returned exactly once, by take_below()
 *   in ascending sequence order per writer, by evict_for_block(), or
 *   released by discard_writer()/clear(); a reader never holds more than its
 *   limit and the pool never more than its capacity.
 * - Covered by: TC-SRR-8.4.10.4-010 through TC-SRR-8.4.10.4-019.
 *
 * References:
 * - OMG DDSI-RTPS 2.5 §8.4.10.4 (WriterProxy; changes made available in order)
 * - OMG DDS 1.4 §2.2.3.17 (DESTINATION_ORDER; one writer's samples keep their order)
 */
class HeldChanges {
public:
    /**
     * @brief Creates a reader's view of `pool`.
     *
     * @param[in] pool  The slots, possibly shared with other readers; outlives the view.
     * @param[in] owner The reader whose changes these are; tags its slots.
     * @param[in] limit Most slots this reader occupies, 1 to the pool's capacity.
     */
    HeldChanges(HeldChangePool& pool, StatefulReader* owner, uint8_t limit) noexcept
        : pool_(pool), owner_(owner), limit_(limit) {}

    HeldChanges(const HeldChanges&) = delete;
    HeldChanges& operator=(const HeldChanges&) = delete;

    /// @brief Releases every change this reader holds.
    ~HeldChanges() noexcept { clear(); }

    /// @brief Most slots this reader occupies; also the width of its window.
    uint8_t limit() const noexcept { return limit_; }

    /// @brief Changes this reader holds now, over all its writers.
    uint8_t count() const noexcept;

    /**
     * @brief Keeps `change` until it can be delivered.
     *
     * @param[in,out] change A complete change from a matched writer; on
     *   success its payload moves into the pool and `change` is left empty.
     * @return `true` when held; `false` when the reader is at its limit or
     *   the pool is full, leaving `change` untouched.
     */
    bool hold(CacheChange& change) noexcept;

    /**
     * @brief Takes the lowest held change of `writer` below `limit`.
     *
     * StatefulReader calls this repeatedly with the writer's first
     * unresolved sequence number to deliver the run a closed gap released.
     *
     * @param[in]  writer The writer whose change to take.
     * @param[in]  limit  Only changes with a sequence number below it qualify.
     * @param[out] out    Receives the change (payload moved) when found.
     * @return `true` when a change was taken into `out`.
     */
    bool take_below(const Guid& writer, const SequenceNumber& limit, CacheChange& out) noexcept;

    /**
     * @brief Gives up a held change in this pool so its buffer block can close a gap.
     *
     * Called only for the change that closes `writer`'s gap, when the buffer
     * pool has no block for it. The victim is this reader's highest held
     * change of `writer` when there is one, and otherwise the highest held
     * change of whichever reader and writer hold the pool's last used slot.
     * Taking the highest keeps the lower changes, which are delivered first.
     * The caller withdraws the victim from its writer's received set
     * (StatefulReader::withdraw_evicted()) so it is requested again.
     *
     * @param[in]  writer  The writer whose gap the block is for.
     * @param[out] evicted The change given up, when one was.
     * @return `true` when a change was evicted and its block released.
     *
     * Verification:
     * - Correct means: a reader whose writer keeps going is never left
     *   without a block for its next change while any change in its pool is
     *   held (RosetteReaderDelivery.tla, BlockRule "evict_any",
     *   EveryChangeDone with SilentWriter).
     * - Covered by: TC-SRR-8.4.10.4-016, TC-SRR-8.4.10.4-019.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.4.10.4
     */
    bool evict_for_block(const Guid& writer, HeldEviction& evicted) noexcept;

    /// @brief Releases every change held for `writer` (its match ended).
    void discard_writer(const Guid& writer) noexcept;

    /// @brief Releases every change this reader holds.
    void clear() noexcept;

private:
    HeldChangePool& pool_;
    StatefulReader* owner_;
    uint8_t         limit_;
};

}  // namespace rosette::rtps
