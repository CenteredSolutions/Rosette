// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file rtps/sn_window.hpp
 * @brief Which sequence numbers from one writer a reader has resolved.
 *
 * A reader tracks, per matched writer, every change it has received or
 * given up on (DDSI-RTPS 2.5 §8.4.10.4, `changes_from_writer`). The spec
 * models that as an unbounded set of ChangeFromWriter records; a
 * SequenceNumberWindow stores the same information in constant space:
 *
 *   - every sequence number below base() is resolved;
 *   - a 64-bit bitmap records which of base() .. base()+63 are resolved;
 *   - nothing at or beyond base()+64 is recorded.
 *
 * "Resolved" means the reader no longer expects the change: it was received
 * (RECEIVED), declared irrelevant by a GAP, or dropped by the writer
 * (LOST). Readers never act differently on RECEIVED and LOST, so one state
 * covers both. base() itself is always unresolved: the window advances over
 * every contiguous resolved prefix as soon as it forms.
 *
 * WriterProxy uses the window as its changes_from_writer store and adds the
 * writer's announced range (which changes are MISSING rather than UNKNOWN)
 * on top. SEDP's builtin readers use it directly.
 */

#pragma once

#include <cstdint>

#include "types.hpp"

namespace rosette::rtps {

class SequenceNumberWindow {
public:
    /// Sequence numbers recorded past base(), inclusive of base().
    static constexpr uint32_t kCapacity = 64u;

    /// @brief Lowest unresolved sequence number; every one below it is resolved.
    SequenceNumber base() const noexcept { return base_; }

    /// @brief True when `sn` lies in [base(), base() + kCapacity), where it can be recorded.
    bool covers(const SequenceNumber& sn) const noexcept {
        return !(sn < base_) && (sn - base_) < static_cast<int64_t>(kCapacity);
    }

    /**
     * @brief True when `sn` is known resolved.
     *
     * @param[in] sn Any sequence number.
     * @return `true` below base() or for a recorded number inside the window;
     *   `false` for an unresolved number and for anything beyond the window,
     *   which is never recorded.
     */
    bool is_resolved(const SequenceNumber& sn) const noexcept;

    /**
     * @brief Records `sn` as resolved (received, irrelevant, or lost).
     *
     * @param[in] sn Sequence number of a received change or one named by GAP.
     * @return `true` when `sn` was unresolved and inside the window and is now
     *   recorded; `false` when it was already resolved or lies beyond the
     *   window (check covers() to tell the two apart).
     *
     * Verification:
     * - Correct means: each sequence number inside the window is reported
     *   newly resolved at most once; base() advances over every contiguous
     *   resolved prefix, so base() is never resolved.
     * - Covered by: TC-SEDP-8.5.4-010, TC-SRR-8.4.10-001.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.4.10.4 (received_change_set, irrelevant_change_set)
     */
    bool resolve(const SequenceNumber& sn) noexcept;

    /**
     * @brief Returns a recorded `sn` above base() to unresolved.
     *
     * A reliable reader records a change on arrival and may then fail to
     * keep it (WriterProxy::received_change_withdraw()); unresolved, it is
     * requested again. base() never moves: a number at or below it, or
     * outside the window, is left as it is.
     *
     * @param[in] sn Sequence number of a change recorded but not kept.
     * @return `true` when `sn` was resolved inside the window above base()
     *   and is now unresolved.
     *
     * Verification:
     * - Correct means: only `sn` changes state, base() is unchanged, and a
     *   later resolve(sn) records it again.
     * - Covered by: TC-SRR-8.4.10.4-016, TC-SRR-8.4.10.4-017, TC-SRR-8.4.10.4-019.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.4.10.4 (changes_from_writer)
     */
    bool unresolve(const SequenceNumber& sn) noexcept;

    /**
     * @brief Resolves every sequence number below `first_unresolved`.
     *
     * Applies a HEARTBEAT's firstSN (the writer no longer holds anything
     * older), a best-effort reader's acceptance of a newer change, or a GAP
     * run that starts at or below base(). Recorded numbers at or above
     * `first_unresolved` keep their state.
     *
     * @param[in] first_unresolved New lower bound; no effect at or below base().
     *
     * Verification:
     * - Correct means: afterwards base() >= first_unresolved and the resolved
     *   state of every number >= first_unresolved is unchanged.
     * - Covered by: TC-SEDP-8.5.4-010, TC-SEDP-8.5.4-012, TC-SRR-8.4.12-001.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.4.10.4 (lost_changes_update)
     */
    void resolve_below(const SequenceNumber& first_unresolved) noexcept;

    /**
     * @brief Resolves every sequence number in [first, end).
     *
     * Applies a GAP's contiguous run (§8.3.7.4.5). A run reaching down to
     * base() moves base() to `end` at once, so a run of any length costs
     * constant time; otherwise the part inside the window is recorded.
     *
     * @param[in] first The GAP's gapStart.
     * @param[in] end   One past the run's last sequence number.
     *
     * Verification:
     * - Correct means: afterwards no number in [first, end) inside the window
     *   is unresolved, and no number outside [first, end) changed state.
     * - Covered by: TC-SEDP-8.5.4-012, TC-SRR-8.4.9-003, TC-SRR-8.4.9-004.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.3.7.4.5, §8.4.10.4 (irrelevant_change_set)
     */
    void resolve_range(const SequenceNumber& first, const SequenceNumber& end) noexcept;

    /**
     * @brief Lists the unresolved sequence numbers up to `last`.
     *
     * Builds the readerSNState of an ACKNACK: bitmapBase = base(), which also
     * acknowledges every change below it, and one bit per unresolved number
     * inside the window up to `last`.
     *
     * @param[in]  last      Highest sequence number the writer is known to
     *   hold; nothing above it is listed.
     * @param[in]  max_count Most numbers to list, lowest first (1 on a link
     *   whose repairs must each fit one datagram).
     * @param[out] out       Receives the set; empty (a pure acknowledgement of
     *   everything below base()) when nothing up to `last` is unresolved.
     *
     * Verification:
     * - Correct means: bitmapBase never exceeds an unresolved number, and the
     *   lowest unresolved numbers are listed first.
     * - Covered by: TC-SEDP-8.5.4-010, TC-SRR-8.4.7-002, TC-SRR-8.4.7-003.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.3.5.5 (SequenceNumberSet), §8.4.10.4 (missing_changes)
     */
    void unresolved(const SequenceNumber& last, uint32_t max_count,
                    SequenceNumberSet& out) const noexcept;

    /// @brief Forgets everything: base() returns to 1 and nothing is resolved.
    void clear() noexcept {
        base_ = SequenceNumber::from_int64(1);
        resolved_ = 0u;
    }

private:
    /// Moves base_ past the contiguous resolved prefix of the bitmap.
    void advance_base() noexcept;

    SequenceNumber base_ = SequenceNumber::from_int64(1);
    uint64_t       resolved_ = 0u;  ///< Bit i: base_ + i is resolved; bit 0 is always clear.
};

}  // namespace rosette::rtps
