/**
 * @file rtps/sn_receive_window.hpp
 * @brief Which sequence numbers from one reliable writer a reader still lacks.
 *
 * A reliable reader must be able to say, at any HEARTBEAT, exactly which
 * changes it is missing (DDSI-RTPS 2.5 §8.4.12.1): changes may arrive out of
 * order, some are lost for good on a lossy or size-limited link, and a writer
 * may declare some irrelevant (GAP) or no longer available (HEARTBEAT
 * firstSN). SnReceiveWindow records that state in constant space: every
 * sequence number below base() is resolved, and a 64-bit bitmap records which
 * of the next 64 are. It is used by the SEDP builtin readers, whose writers
 * (one per remote participant) announce endpoints in bursts that a
 * datagram-limited link loses in part.
 *
 * A change beyond the 64-number window is accepted but not recorded, so it
 * is requested again later; duplicate delivery of discovery data is
 * harmless, and the window reaches it once the earlier holes close.
 */

#pragma once

#include <cstdint>

#include "types.hpp"

namespace rtps {

class SnReceiveWindow {
public:
    static constexpr uint32_t kWindow = 64u;

    /// @brief Lowest sequence number not yet resolved; all below it are.
    SequenceNumber base() const noexcept { return base_; }

    /**
     * @brief Records that `sn` arrived (or is irrelevant) and says if it is new.
     *
     * @param[in] sn Sequence number of a received DATA, or one named by GAP.
     * @return `true` when `sn` was not resolved before (process the change),
     *   `false` for a duplicate.
     *
     * Verification:
     * - Correct means: each sequence number is reported new at most once
     *   while it lies inside the window; base() advances over every
     *   contiguous resolved prefix.
     * - Covered by: TC-SEDP-8.5.4-010.
     */
    bool resolve(const SequenceNumber& sn) noexcept {
        if (sn < base_) return false;
        const int64_t offset = sn - base_;
        if (offset >= static_cast<int64_t>(kWindow)) return true;
        const uint64_t bit = uint64_t{1} << offset;
        if ((resolved_ & bit) != 0u) return false;
        resolved_ |= bit;
        compact();
        return true;
    }

    /**
     * @brief Resolves every sequence number below `first_available`.
     *
     * A HEARTBEAT's firstSN says the writer no longer holds anything older
     * (§8.3.7.5), so waiting for those changes would stall forever.
     *
     * @param[in] first_available The HEARTBEAT's firstSN.
     */
    void resolve_below(const SequenceNumber& first_available) noexcept {
        while (base_ < first_available) {
            if (first_available - base_ >= static_cast<int64_t>(kWindow)) {
                base_ = first_available;
                resolved_ = 0u;
                return;
            }
            resolved_ |= 1u;
            compact();
        }
    }

    /// @brief True when `sn` is at or above base() and has not been resolved.
    bool is_missing(const SequenceNumber& sn) const noexcept {
        if (sn < base_) return false;
        const int64_t offset = sn - base_;
        return offset >= static_cast<int64_t>(kWindow) ||
               (resolved_ & (uint64_t{1} << offset)) == 0u;
    }

    /**
     * @brief Builds an ACKNACK readerSNState for a writer whose lastSN is known.
     *
     * @param[in]  last_sn    Highest sequence number the writer announced.
     * @param[in]  max_nacks  Most sequence numbers to request (1 on a link
     *   whose repairs must each fit one datagram; up to 256 otherwise).
     * @param[out] out        bitmapBase = base(); one bit per unresolved
     *   sequence number up to last_sn, at most `max_nacks` of them. Empty
     *   (a pure acknowledgement of everything below base()) when none is
     *   missing.
     *
     * Verification:
     * - Correct means: bitmapBase never exceeds an unresolved sequence
     *   number, and the lowest unresolved one is always requested first.
     * - Covered by: TC-SEDP-8.5.4-010.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.3.5.5 (SequenceNumberSet), §8.4.12.1
     */
    void missing(const SequenceNumber& last_sn, uint32_t max_nacks,
                 SequenceNumberSet& out) const noexcept {
        out.clear();
        out.bitmapBase = base_;
        uint32_t added = 0u;
        for (SequenceNumber sn = base_; sn <= last_sn && added < max_nacks; sn += 1) {
            if ((sn - base_) >= 256) break;
            if (is_missing(sn)) {
                out.set(static_cast<uint32_t>(sn - base_));
                ++added;
            }
        }
    }

private:
    void compact() noexcept {
        while ((resolved_ & 1u) != 0u) {
            resolved_ >>= 1u;
            base_ += 1;
        }
    }

    SequenceNumber base_ = SequenceNumber::from_int64(1);
    uint64_t       resolved_ = 0u;  ///< Bit i: base_ + i is resolved.
};

}  // namespace rtps
