/**
 * @file rtps/writer_proxy.hpp
 * @brief WriterProxy — per-writer state maintained by a StatefulReader.
 *
 * A WriterProxy represents a remote RTPS Writer that has been matched to
 * a local StatefulReader.  The reader maintains one WriterProxy per
 * matched writer and uses it to:
 *   - Track the reception status of each expected sequence number.
 *   - Drive the HEARTBEAT→ACKNACK response logic.
 *   - Detect writer liveliness failure.
 *
 * Spec mapping
 * ------------
 * Implements the WriterProxy entity defined in §8.4.7.1, Table 8.61,
 * p. 134 of OMG formal/2022-04-01.
 *
 * Attributes (§8.4.7.1, Table 8.61, p. 134):
 *   remoteWriterGuid          — GUID of the matched remote writer.
 *   remoteGroupEntityId       — group entity (unused for non-group).
 *   unicastLocatorList        — locators used to reach this writer.
 *   multicastLocatorList      — multicast locators.
 *
 * Additional implementation attributes (§8.4.7.1, p. 134-135):
 *   changes_from_writer[]     — per-SN reception status.
 *   highestSNReceived         — highest SN ever received from this writer.
 *   highestSNAcknowledged     — highest SN covered by our last ACKNACK.
 *   lastHBCount               — count from the last processed HEARTBEAT
 *                               (de-duplicates retransmitted HBs).
 *   acknackCount              — our own monotone ACKNACK counter.
 *   isAlive                   — liveliness state; cleared on lease expiry.
 *   heartbeatResponseDeadlineMs — when to send the pending ACKNACK.
 *
 * heartbeatResponseDelay (§8.4.7.1, p. 135)
 * ------------------------------------------
 * "After receiving a HEARTBEAT the Reader waits heartbeatResponseDelay
 *  before sending an ACKNACK."  The delay is enforced in
 *  StatefulReader::tick(); WriterProxy stores the deadline timestamp.
 *
 * KEEP_LAST(1) constraint
 * -----------------------
 * With KEEP_LAST(1) the reader only needs to track one live sample
 * per writer at a time.  The ChangeFromWriter array is templated on
 * N = RTPS_WRITER_PROXY_WINDOW_DEPTH (default 8); a depth of 1 would
 * suffice for KEEP_LAST(1) but a small window gives better duplicate
 * detection and correct MISSING/LOST accounting across sparse GAPs.
 *
 * Spec references (all OMG formal/2022-04-01)
 * -------------------------------------------
 *   WriterProxy attributes      — §8.4.7.1, Table 8.61, p. 134
 *   ChangeFromWriter            — §8.4.7.1, p. 134
 *   ChangeFromWriterStatus      — §8.4.7.1, p. 134
 *   heartbeatResponseDelay      — §8.4.7.1, p. 135
 *   missing_changes()           — §8.4.11,  p. 150
 *   missing_changes_update()    — §8.4.11,  p. 150
 *   lost_changes_update()       — §8.4.11,  p. 150
 *   received_change_set()       — §8.4.11,  p. 150
 *   HEARTBEAT processing        — §8.4.9,   p. 145
 *   HEARTBEAT final-flag rule   — §8.3.7.5, p. 91
 *   GAP processing              — §8.4.9,   p. 145
 */

#pragma once

#include "types.hpp"
#include <string.h>

namespace rtps {

// ============================================================
// ChangeFromWriter
//
// "For each expected sample the WriterProxy maintains a
//  ChangeFromWriter object."
//  — §8.4.7.1, Table 8.61, p. 134
// ============================================================

struct ChangeFromWriter {
    SequenceNumber        sequenceNumber = SEQUENCENUMBER_UNKNOWN;
    ChangeFromWriterStatus status        = ChangeFromWriterStatus::UNKNOWN;

    bool is_valid() const noexcept {
        return sequenceNumber != SEQUENCENUMBER_UNKNOWN;
    }

    void clear() noexcept {
        sequenceNumber = SEQUENCENUMBER_UNKNOWN;
        status         = ChangeFromWriterStatus::UNKNOWN;
    }
};

// ============================================================
// WriterProxy
//
// §8.4.7.1, Table 8.61, p. 134.
//
// Template parameter N: depth of the ChangeFromWriter tracking
// window.  Must be >= 1; a depth of 8 handles typical GAP spans.
// ============================================================

#ifndef RTPS_WRITER_PROXY_WINDOW_DEPTH
#  define RTPS_WRITER_PROXY_WINDOW_DEPTH 8
#endif

template<uint8_t N = RTPS_WRITER_PROXY_WINDOW_DEPTH>
class WriterProxy {
    static_assert(N >= 1, "WriterProxy window depth must be at least 1");

public:

    // ----------------------------------------------------------
    // §8.4.7.1, Table 8.61, p. 134 — spec attributes
    // ----------------------------------------------------------

    Guid remoteWriterGuid    = GUID_UNKNOWN;  ///< §8.4.7.1
    Guid remoteGroupEntityId = GUID_UNKNOWN;  ///< §8.4.7.1 (unused)

#ifndef RTPS_MAX_WRITER_PROXY_LOCATORS
#  define RTPS_MAX_WRITER_PROXY_LOCATORS 2
#endif
    Locator unicastLocatorList[RTPS_MAX_WRITER_PROXY_LOCATORS]   = {};
    uint8_t unicastLocatorCount                                   = 0;
    Locator multicastLocatorList[RTPS_MAX_WRITER_PROXY_LOCATORS] = {};
    uint8_t multicastLocatorCount                                 = 0;

    // ----------------------------------------------------------
    // Implementation tracking attributes
    // ----------------------------------------------------------

    /// Highest SN ever received from this writer (DATA or DATA_FRAG).
    SequenceNumber highestSNReceived = SEQUENCENUMBER_ZERO;

    /// Highest SN covered by the most recent ACKNACK we sent.
    /// Used to avoid resending an identical ACKNACK bitmap.
    SequenceNumber highestSNAcknowledged = SEQUENCENUMBER_ZERO;

    /// Count from the last HEARTBEAT we processed.
    /// Duplicate HBs (same count) are ignored — §8.3.7.5, p. 91.
    int32_t lastHBCount = 0;

    /// Our own ACKNACK count, incremented before each send.
    /// — §8.4.7.3, p. 140.
    int32_t acknackCount = 0;

    /// Liveliness state.  Cleared when the writer lease expires.
    /// — §8.4.7.1, p. 134.
    bool isAlive = false;

    /// True while we owe a pending ACKNACK response to a HEARTBEAT.
    bool acknackPending = false;

    /// Deadline by which we must send the pending ACKNACK (ms).
    /// Set when a HEARTBEAT is processed; cleared when ACKNACK is sent.
    /// — §8.4.7.1 heartbeatResponseDelay, p. 135.
    uint32_t heartbeatResponseDeadlineMs = 0;

    // ----------------------------------------------------------
    // Construction / reset
    // ----------------------------------------------------------

    WriterProxy() noexcept { clear(); }

    /// Initialise proxy for a newly matched writer.
    ///
    /// "When a new writer is matched, the Reader creates a WriterProxy."
    ///  — §8.4.7.4, p. 141.
    void init(const Guid& writer_guid, const Locator* unicast_locs,
              uint8_t unicast_count) noexcept {
        clear();
        remoteWriterGuid  = writer_guid;
        isAlive           = true;
        uint8_t n = unicast_count < RTPS_MAX_WRITER_PROXY_LOCATORS
                    ? unicast_count
                    : RTPS_MAX_WRITER_PROXY_LOCATORS;
        for (uint8_t i = 0; i < n; ++i)
            unicastLocatorList[i] = unicast_locs[i];
        unicastLocatorCount = n;
    }

    void clear() noexcept {
        remoteWriterGuid       = GUID_UNKNOWN;
        remoteGroupEntityId    = GUID_UNKNOWN;
        unicastLocatorCount    = 0;
        multicastLocatorCount  = 0;
        highestSNReceived      = SEQUENCENUMBER_ZERO;
        highestSNAcknowledged  = SEQUENCENUMBER_ZERO;
        lastHBCount            = 0;
        acknackCount           = 0;
        isAlive                = false;
        acknackPending         = false;
        heartbeatResponseDeadlineMs = 0;
        for (uint8_t i = 0; i < N; ++i) changes_[i].clear();
    }

    bool is_matched() const noexcept {
        return remoteWriterGuid != GUID_UNKNOWN;
    }

    // ----------------------------------------------------------
    // received_change_set()
    //
    // Records a successfully received DATA/DATA_FRAG.
    //
    // "received_change_set(a_change) — Modifies the status of the
    //  ChangeFromWriter with sequenceNumber equal to a_change to
    //  RECEIVED."
    //  — §8.4.11, p. 150
    //
    // If the SN is new (not yet in the window) an entry is inserted.
    // Any SNs between highestSNReceived and this SN that have no
    // entry are recorded as MISSING by missing_changes_update().
    //
    // @param sn  SequenceNumber of the received sample.
    // ----------------------------------------------------------
    void received_change_set(const SequenceNumber& sn) noexcept {
        if (sn > highestSNReceived) {
            // Mark all intermediate SNs as MISSING before recording receipt.
            missing_changes_update(sn - 1);
            highestSNReceived = sn;
        }
        ChangeFromWriter* slot = find_or_insert(sn);
        if (slot) slot->status = ChangeFromWriterStatus::RECEIVED;
    }

    // ----------------------------------------------------------
    // missing_changes_update()
    //
    // Ensures that all SNs in [highestSNReceived+1 .. last_available_sn]
    // that have no existing entry are tracked as MISSING.
    //
    // "missing_changes_update(last_available_seq_num) — Sets the status
    //  of any ChangeFromWriter with SN <= last_available_seq_num that
    //  is not RECEIVED or LOST to MISSING."
    //  — §8.4.11, p. 150
    //
    // Called from on_heartbeat() with lastSN, and from
    // received_change_set() as a side-effect of advancing the frontier.
    //
    // @param last_available_sn  The writer's lastSN from a HEARTBEAT,
    //                           or (received_sn - 1) from received_change_set().
    // ----------------------------------------------------------
    void missing_changes_update(const SequenceNumber& last_available_sn) noexcept {
        // Walk from one above highestSNReceived to last_available_sn.
        SequenceNumber sn = highestSNReceived + 1;
        while (sn <= last_available_sn) {
            // Only insert MISSING if we don't already know about this SN.
            ChangeFromWriter* slot = find_slot(sn);
            if (!slot) {
                slot = insert_new(sn);
            }
            if (slot && slot->status == ChangeFromWriterStatus::UNKNOWN) {
                slot->status = ChangeFromWriterStatus::MISSING;
            }
            sn += 1;
        }
    }

    // ----------------------------------------------------------
    // lost_changes_update()
    //
    // Marks all MISSING SNs < first_available_sn as LOST.
    //
    // "lost_changes_update(first_available_seq_num) — Sets the status
    //  of any ChangeFromWriter with SN < first_available_seq_num that
    //  is MISSING to LOST."
    //  — §8.4.11, p. 150
    //
    // Called from on_heartbeat() with firstSN: the writer is declaring
    // it no longer holds those samples.
    //
    // @param first_available_sn  The writer's firstSN from a HEARTBEAT.
    // ----------------------------------------------------------
    void lost_changes_update(const SequenceNumber& first_available_sn) noexcept {
        for (uint8_t i = 0; i < N; ++i) {
            if (!changes_[i].is_valid()) continue;
            if (changes_[i].sequenceNumber < first_available_sn &&
                changes_[i].status == ChangeFromWriterStatus::MISSING) {
                changes_[i].status = ChangeFromWriterStatus::LOST;
            }
        }
        // Also advance highestSNReceived past lost SNs so the MISSING
        // range doesn't grow backwards.
        if (first_available_sn > highestSNReceived + 1) {
            highestSNReceived = first_available_sn - 1;
        }
    }

    // ----------------------------------------------------------
    // on_gap()
    //
    // Processes a GAP submessage.
    //
    // "The GAP Submessage indicates that a range of sequence numbers
    //  is no longer relevant.  The Reader shall set the status of
    //  those SNs to LOST."
    //  — §9.4.5.11, p. 258; §8.4.9, p. 145
    //
    // "Irrelevant" SNs are those the writer will never send:
    //   [gap_start .. gap_list.bitmapBase - 1]
    //   ∪ { gap_list.bitmapBase + i : gap_list.bitmap[i] == 1 }
    //
    // @param gap_start  First irrelevant SN.
    // @param gap_list   Bitmap of additional irrelevant SNs.
    // ----------------------------------------------------------
    void on_gap(const SequenceNumber& gap_start,
                const SequenceNumberSet& gap_list) noexcept {
        // Mark contiguous range [gap_start .. bitmapBase - 1] as LOST.
        SequenceNumber range_end = gap_list.empty()
            ? gap_start
            : (gap_list.bitmapBase - 1);

        for (SequenceNumber sn = gap_start; sn <= range_end; sn += 1) {
            mark_lost(sn);
        }

        // Mark individual SNs flagged in the bitmap.
        gap_list.for_each([&](const SequenceNumber& sn) noexcept {
            mark_lost(sn);
        });

        // Advance highestSNReceived to cover the gap so missing_changes_update
        // does not later re-insert these as MISSING.
        if (!gap_list.empty()) {
            SequenceNumber gap_end = gap_list.bitmapBase +
                                    static_cast<int64_t>(gap_list.numBits - 1);
            if (gap_end > highestSNReceived) {
                highestSNReceived = gap_end;
            }
        } else if (range_end > highestSNReceived) {
            highestSNReceived = range_end;
        }
    }

    // ----------------------------------------------------------
    // on_heartbeat()
    //
    // Processes a HEARTBEAT and determines whether an ACKNACK must
    // be sent, per the final-flag rules.
    //
    // §8.3.7.5, p. 91 — HEARTBEAT final-flag semantics:
    //   F == 0: Reader MUST send an ACKNACK in response.
    //   F == 1 and reader is up-to-date: reader MAY suppress ACKNACK.
    //   F == 1 and reader has missing changes: reader MUST still NACK.
    //
    // §8.4.9, p. 145: also calls lost_changes_update(firstSN) and
    //   missing_changes_update(lastSN) to update proxy state.
    //
    // Duplicate detection: if hb_count <= lastHBCount the HEARTBEAT is
    // stale and silently ignored.  — §8.4.9, p. 145.
    //
    // @param first_sn        firstSN from the HEARTBEAT.
    // @param last_sn         lastSN from the HEARTBEAT.
    // @param hb_count        count from the HEARTBEAT.
    // @param final_flag      true when the F-flag is set.
    // @param now_ms          Current rosette::platform::monotonic_ms().
    // @param response_delay_ms  heartbeatResponseDelay in milliseconds.
    //
    // @returns  true  if an ACKNACK should be sent (after the deadline).
    //           false if the HEARTBEAT was duplicate or suppressed.
    // ----------------------------------------------------------
    bool on_heartbeat(const SequenceNumber& first_sn,
                      const SequenceNumber& last_sn,
                      int32_t              hb_count,
                      bool                 final_flag,
                      uint32_t             now_ms,
                      uint32_t             response_delay_ms) noexcept
    {
        // Duplicate / reordered heartbeat — §8.4.9, p. 145.
        if (hb_count <= lastHBCount) return false;
        lastHBCount = hb_count;

        // Update missing / lost windows.
        lost_changes_update(first_sn);
        missing_changes_update(last_sn);

        // Decide whether to schedule an ACKNACK.
        //
        // §8.3.7.5, p. 91:
        //   "If the FinalFlag is not set, the Reader must send an ACKNACK."
        //   "If the FinalFlag is set and the Reader has missing changes,
        //    the Reader must send an ACKNACK."
        //   "If the FinalFlag is set and the Reader has no missing changes,
        //    the Reader need not send an ACKNACK."
        const bool has_missing = has_missing_changes();
        const bool must_reply  = !final_flag || has_missing;

        if (must_reply) {
            acknackPending = true;
            heartbeatResponseDeadlineMs = now_ms + response_delay_ms;
        }

        return must_reply;
    }

    // ----------------------------------------------------------
    // missing_changes()
    //
    // "missing_changes() — Returns the set of SNs that are MISSING."
    //  — §8.4.11, p. 150
    //
    // Builds a SequenceNumberSet covering all MISSING SNs in the
    // tracking window.  Used by StatefulReader to populate the
    // ACKNACK readerSNState bitmap.
    //
    // The ACKNACK bitmapBase is set to highestSNAcknowledged + 1
    // (the first SN the reader is not sure it has), with bits set
    // for each MISSING SN above that base.  — §8.4.7.3, p. 140.
    //
    // @param out  Output SequenceNumberSet to populate.
    // ----------------------------------------------------------
    void missing_changes(SequenceNumberSet& out) const noexcept {
        out.clear();
        // bitmapBase = one past the highest cumulatively acknowledged SN.
        out.bitmapBase = highestSNAcknowledged + 1;
        for (uint8_t i = 0; i < N; ++i) {
            if (!changes_[i].is_valid()) continue;
            if (changes_[i].status != ChangeFromWriterStatus::MISSING) continue;
            out.add(changes_[i].sequenceNumber);
        }
    }

    // ----------------------------------------------------------
    // has_missing_changes()
    //
    // Returns true if any ChangeFromWriter has status MISSING.
    // — §8.4.11, p. 150.
    // ----------------------------------------------------------
    bool has_missing_changes() const noexcept {
        for (uint8_t i = 0; i < N; ++i) {
            if (changes_[i].is_valid() &&
                changes_[i].status == ChangeFromWriterStatus::MISSING)
                return true;
        }
        return false;
    }

    // ----------------------------------------------------------
    // acknack_response_due()
    //
    // Returns true when the heartbeatResponseDelay has elapsed and
    // a pending ACKNACK should be sent.
    //
    // — §8.4.7.1 heartbeatResponseDelay, p. 135.
    //
    // @param now_ms  Current rosette::platform::monotonic_ms().
    // ----------------------------------------------------------
    bool acknack_response_due(uint32_t now_ms) const noexcept {
        if (!acknackPending) return false;
        return (now_ms >= heartbeatResponseDeadlineMs) ||
               // Wrap-safe: also fire if the ms counter has wrapped
               // and the difference is > half the uint32 space.
               ((heartbeatResponseDeadlineMs - now_ms) > 0x80000000u);
    }

    /// Called after the ACKNACK is sent; clears the pending flag and
    /// advances highestSNAcknowledged to the current frontier.
    void on_acknack_sent(const SequenceNumber& acked_up_to) noexcept {
        ++acknackCount;
        acknackPending = false;
        heartbeatResponseDeadlineMs = 0;
        if (acked_up_to > highestSNAcknowledged) {
            highestSNAcknowledged = acked_up_to;
        }
    }

    // ----------------------------------------------------------
    // Iteration
    // ----------------------------------------------------------
    template<typename Fn>
    void for_each_change(Fn&& fn) const noexcept {
        for (uint8_t i = 0; i < N; ++i) {
            if (changes_[i].is_valid()) fn(changes_[i]);
        }
    }

    // ----------------------------------------------------------
    // Best locator for sending ACKNACK to the remote writer.
    // ----------------------------------------------------------
    const Locator& best_locator() const noexcept {
        for (uint8_t i = 0; i < unicastLocatorCount; ++i) {
            const Long kind = unicastLocatorList[i].kind;
            if (kind == locator_kind::UDPv4 ||
                kind == locator_kind::UDPv6) {
                return unicastLocatorList[i];
            }
        }
        if (unicastLocatorCount > 0) return unicastLocatorList[0];
        return LOCATOR_INVALID;
    }

    // ----------------------------------------------------------
    // Evict changes that are no longer useful to track:
    // ACKNOWLEDGED, LOST, or RECEIVED with SN <= retention_floor.
    //
    // Called periodically by StatefulReader::tick() to prevent the
    // window from filling with stale entries.
    //
    // @param retention_floor  SNs <= this may be evicted if RECEIVED.
    // ----------------------------------------------------------
    void evict_stale(const SequenceNumber& retention_floor) noexcept {
        for (uint8_t i = 0; i < N; ++i) {
            if (!changes_[i].is_valid()) continue;
            auto s = changes_[i].status;
            if (s == ChangeFromWriterStatus::LOST) {
                changes_[i].clear();
                continue;
            }
            if (s == ChangeFromWriterStatus::RECEIVED &&
                changes_[i].sequenceNumber <= retention_floor) {
                changes_[i].clear();
            }
        }
    }

private:

    ChangeFromWriter changes_[N];

    /// Find the slot for a given SN, or nullptr.
    ChangeFromWriter* find_slot(const SequenceNumber& sn) noexcept {
        for (uint8_t i = 0; i < N; ++i) {
            if (changes_[i].sequenceNumber == sn) return &changes_[i];
        }
        return nullptr;
    }

    /// Find an existing slot for sn, or allocate a new empty one.
    ChangeFromWriter* find_or_insert(const SequenceNumber& sn) noexcept {
        ChangeFromWriter* slot = find_slot(sn);
        if (slot) return slot;
        return insert_new(sn);
    }

    /// Allocate a new slot.  Evicts the oldest entry if the window is full.
    ChangeFromWriter* insert_new(const SequenceNumber& sn) noexcept {
        // First try an empty slot.
        for (uint8_t i = 0; i < N; ++i) {
            if (!changes_[i].is_valid()) {
                changes_[i].sequenceNumber = sn;
                changes_[i].status         = ChangeFromWriterStatus::UNKNOWN;
                return &changes_[i];
            }
        }
        // Window full: evict the entry with the lowest SN.
        ChangeFromWriter* oldest = nullptr;
        for (uint8_t i = 0; i < N; ++i) {
            if (!oldest || changes_[i].sequenceNumber < oldest->sequenceNumber)
                oldest = &changes_[i];
        }
        if (oldest) {
            oldest->sequenceNumber = sn;
            oldest->status         = ChangeFromWriterStatus::UNKNOWN;
        }
        return oldest;
    }

    /// Set an SN's status to LOST; insert an entry if it doesn't exist.
    void mark_lost(const SequenceNumber& sn) noexcept {
        ChangeFromWriter* slot = find_or_insert(sn);
        if (slot && slot->status != ChangeFromWriterStatus::RECEIVED) {
            slot->status = ChangeFromWriterStatus::LOST;
        }
    }
};

/// Default alias used by StatefulReader.
using DefaultWriterProxy = WriterProxy<RTPS_WRITER_PROXY_WINDOW_DEPTH>;

} // namespace rtps
