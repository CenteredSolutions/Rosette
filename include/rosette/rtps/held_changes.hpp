// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file rtps/held_changes.hpp
 * @brief HeldChanges: a reliable reader's changes waiting behind a gap (§8.4.10.4).
 *
 * A reliable StatefulReader makes each writer's changes available in the
 * order written. A change that arrives while an earlier one from the same
 * writer is still MISSING cannot be delivered yet, and dropping it would
 * cost a repair round trip per change after the gap. The reader keeps it
 * here, payload and all, until the gap below it closes (the missing change
 * arrives, or a GAP or HEARTBEAT resolves it), then delivers the held run in
 * order.
 *
 * Each held change pins one pool block, and the pool is shared by every
 * reader of the participant, so a reader holds at most
 * RTPS_READER_HELD_CHANGES. A full store never blocks progress: the change
 * that closes a writer's gap is delivered without a slot, and the reader
 * frees a slot or a block by evicting its highest held change, which
 * becomes MISSING again and is requested again (tests/formal/
 * RosetteReaderDelivery.tla, FullRule "evict_highest").
 */

#pragma once

#include <cstdint>

#include "history_cache.hpp"
#include "types.hpp"

/// Changes one reliable reader may hold behind gaps, over all its writers.
/// Each pins a pool block until delivered; 0 is not allowed, since a reader
/// that holds nothing drops every change that arrives after a loss.
#ifndef RTPS_READER_HELD_CHANGES
#  define RTPS_READER_HELD_CHANGES 4u
#endif

namespace rosette::rtps {

/**
 * @brief A fixed set of changes a reliable reader holds until they can be delivered in order.
 *
 * The store knows nothing of gaps: StatefulReader decides what to hold,
 * what to release (every held change of a writer below that writer's first
 * unresolved sequence number), and what to evict. Changes are found by
 * writer and sequence number; the store is small, so every lookup scans it.
 *
 * Verification:
 * - Correct means: a held change is returned exactly once, by take_below()
 *   in ascending sequence order per writer, by evict_highest_above(), or
 *   released by discard_writer()/clear(); nothing is held past capacity.
 * - Covered by: TC-SRR-8.4.10.4-010 through TC-SRR-8.4.10.4-016.
 *
 * References:
 * - OMG DDSI-RTPS 2.5 §8.4.10.4 (WriterProxy; changes made available in order)
 * - OMG DDS 1.4 §2.2.3.17 (DESTINATION_ORDER; one writer's samples keep their order)
 */
class HeldChanges {
public:
    static constexpr uint8_t kCapacity = RTPS_READER_HELD_CHANGES;
    static_assert(kCapacity >= 1u, "RTPS_READER_HELD_CHANGES must be at least 1");

    HeldChanges() noexcept = default;
    HeldChanges(const HeldChanges&) = delete;
    HeldChanges& operator=(const HeldChanges&) = delete;

    /// @brief Releases every held change's payload.
    ~HeldChanges() noexcept { clear(); }

    /// @brief Changes held now, over all writers.
    uint8_t count() const noexcept { return count_; }

    /// @brief True when no slot is free.
    bool full() const noexcept { return count_ == kCapacity; }

    /**
     * @brief Keeps `change` until it can be delivered.
     *
     * @param[in,out] change A complete change from a matched writer; on
     *   success its payload moves into the store and `change` is left empty.
     * @return `true` when held; `false` when the store is full, leaving
     *   `change` untouched.
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
     * @brief Frees the slot of `writer`'s highest held change above `sn`.
     *
     * Admits a lower change of the same writer when the store or the buffer
     * pool is full. The caller withdraws the returned sequence number from
     * the writer's received set so it is requested again.
     *
     * @param[in]  writer The writer whose change to evict.
     * @param[in]  sn     Only a change above it is evicted.
     * @param[out] evicted Sequence number of the evicted change, when one was.
     * @return `true` when a change was evicted and its payload released.
     */
    bool evict_highest_above(const Guid& writer, const SequenceNumber& sn,
                             SequenceNumber& evicted) noexcept;

    /// @brief Releases every change held for `writer` (its match ended).
    void discard_writer(const Guid& writer) noexcept;

    /// @brief Releases every held change.
    void clear() noexcept;

private:
    /// Moves the last slot into `index`, keeping slots [0, count_) dense.
    void remove_at(uint8_t index) noexcept;

    CacheChange slots_[kCapacity];
    uint8_t     count_ = 0;
};

}  // namespace rosette::rtps
