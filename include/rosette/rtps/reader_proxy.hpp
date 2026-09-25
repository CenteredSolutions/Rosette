/**
 * @file rtps/reader_proxy.hpp
 * @brief ReaderProxy — per-reader state maintained by a StatefulWriter.
 *
 * A ReaderProxy represents a remote RTPS Reader that has been matched to
 * a local StatefulWriter.  The writer maintains one ReaderProxy per
 * matched reader and uses it to track which CacheChanges have been sent,
 * acknowledged, or NACKed, and to implement reliable delivery state
 * transitions.
 *
 * Spec mapping
 * ------------
 * This class implements the ReaderProxy entity defined in §8.4.1.1,
 * Table 8.49, p. 103 of OMG formal/2022-04-01.
 *
 * Attributes (§8.4.1.1, Table 8.49, p. 103):
 *   remoteReaderGuid          — GUID of the matched remote reader.
 *   remoteGroupEntityId       — group entity (not used for non-group topics).
 *   unicastLocatorList        — locators used to address this reader.
 *   multicastLocatorList      — multicast locators (optional).
 *   expectsInlineQos          — whether this reader requires inline QoS.
 *   isActive                  — true while reader is reachable / alive.
 *
 * Additional implementation attributes (§8.4.1.1, p. 103-104):
 *   changes_for_reader[]      — tracks per-SN delivery status.
 *   highestAckedSN            — highest SN cumulatively acknowledged.
 *   lastAcknackCount          — count field from last processed ACKNACK
 *                               (de-duplicates retransmitted ACKNACKs).
 *   nackPending               — true when a NACK has been received but
 *                               nackResponseDelay has not yet elapsed.
 *   nackPendingSinceMs        — timestamp when the NACK was recorded,
 *                               used by StatefulWriter::tick() to fire
 *                               the deferred retransmit.
 *
 * KEEP_LAST(1) constraint
 * -----------------------
 * The target platform uses KEEP_LAST(1), so the writer history cache
 * holds at most one CacheChange at a time.  ReaderProxy therefore tracks
 * at most one ChangeForReader entry.  The ChangeForReader array is
 * templated on N = 1 for this target; N > 1 is reserved for future use.
 *
 * nackResponseDelay (§8.4.1.1, p. 104)
 * ---------------------------------------
 * "After receiving a NACK, the Writer waits nackResponseDelay before
 *  retransmitting."  The delay is enforced in StatefulWriter::tick();
 *  ReaderProxy records nackPendingSinceMs at the moment on_acknack()
 *  marks a change as REQUESTED.
 *
 * Spec references (all OMG formal/2022-04-01)
 * -------------------------------------------
 *   ReaderProxy attributes    — §8.4.1.1, Table 8.49, p. 103
 *   ChangeForReader           — §8.4.1.1, Table 8.49, p. 103
 *   ChangeForReaderStatus     — §8.4.1.1, p. 103
 *   nackResponseDelay         — §8.4.1.1, p. 104
 *   unsent_changes()          — §8.4.6,   p. 128
 *   unacked_changes()         — §8.4.6,   p. 128
 *   requested_changes()       — §8.4.6,   p. 128
 *   acked_changes_set()       — §8.4.6,   p. 128
 *   next_requested_change()   — §8.4.6,   p. 128
 *   next_unsent_change()      — §8.4.6,   p. 128
 */

#pragma once

#include "types.hpp"
#include <string.h>

namespace rtps {

// ============================================================
// ChangeForReader
//
// "For each change in the writer's HistoryCache the ReaderProxy
//  maintains a ChangeForReader object."
//  — §8.4.1.1, Table 8.49, p. 103
//
// Tracks the delivery status of a single CacheChange with respect
// to one remote reader.
// ============================================================

struct ChangeForReader {
    SequenceNumber       sequenceNumber = SEQUENCENUMBER_UNKNOWN;
    ChangeForReaderStatus status        = ChangeForReaderStatus::UNSENT;

    bool is_valid() const noexcept {
        return sequenceNumber != SEQUENCENUMBER_UNKNOWN;
    }

    void clear() noexcept {
        sequenceNumber = SEQUENCENUMBER_UNKNOWN;
        status         = ChangeForReaderStatus::UNSENT;
    }
};

// ============================================================
// ReaderProxy
//
// §8.4.1.1, Table 8.49, p. 103.
//
// Template parameter N is the maximum number of simultaneously
// tracked ChangeForReader entries.  For KEEP_LAST(1) set N = 1.
// ============================================================

#ifndef RTPS_READER_PROXY_CHANGE_DEPTH
#  define RTPS_READER_PROXY_CHANGE_DEPTH 1
#endif

template<uint8_t N = RTPS_READER_PROXY_CHANGE_DEPTH>
class ReaderProxy {
    static_assert(N >= 1, "ReaderProxy must track at least one change");

public:

    // ----------------------------------------------------------
    // §8.4.1.1, Table 8.49, p. 103 — spec attributes
    // ----------------------------------------------------------

    Guid    remoteReaderGuid   = GUID_UNKNOWN;  ///< §8.4.1.1
    Guid    remoteGroupEntityId = GUID_UNKNOWN; ///< §8.4.1.1 (unused for non-group)
    bool    expectsInlineQos   = false;         ///< §8.4.1.1
    bool    isActive           = false;         ///< §8.4.1.1

    /// Unicast locators used to address this reader — §8.4.1.1.
#ifndef RTPS_MAX_READER_PROXY_LOCATORS
#  define RTPS_MAX_READER_PROXY_LOCATORS 2
#endif
    Locator unicastLocatorList[RTPS_MAX_READER_PROXY_LOCATORS]   = {};
    uint8_t unicastLocatorCount                                   = 0;
    Locator multicastLocatorList[RTPS_MAX_READER_PROXY_LOCATORS] = {};
    uint8_t multicastLocatorCount                                 = 0;

    // ----------------------------------------------------------
    // Implementation tracking attributes
    // ----------------------------------------------------------

    /// Highest SN cumulatively acknowledged by this reader.
    /// Initialised to SEQUENCENUMBER_ZERO before first acknowledgement.
    /// — §8.4.1.1, p. 103; updated in acked_changes_set().
    SequenceNumber highestAckedSN = SEQUENCENUMBER_ZERO;

    /// Count field from the last ACKNACK processed for this proxy.
    /// Used to detect and drop duplicate / reordered ACKNACKs.
    /// — §8.4.7.3, p. 140 (by analogy: count is monotone).
    int32_t lastAcknackCount = 0;

    /// True when a NACK has been received and nackResponseDelay is pending.
    /// Set by on_acknack(); cleared by StatefulWriter::tick() after firing.
    /// — §8.4.1.1 nackResponseDelay, p. 104.
    bool nackPending = false;

    /// Millisecond timestamp at which the pending NACK was first recorded.
    /// Used by tick() to compute elapsed time vs. nackResponseDelay.
    uint32_t nackPendingSinceMs = 0;

    // ----------------------------------------------------------
    // Construction / reset
    // ----------------------------------------------------------

    ReaderProxy() noexcept { clear(); }

    /// Initialise proxy for a newly matched reader.
    ///
    /// "When a new reader is matched, the Writer creates a ReaderProxy
    ///  and associates with it a ChangeForReader for every change in the
    ///  HistoryCache that has not yet been acknowledged by that reader."
    ///  — §8.4.1.4, p. 107
    void init(const Guid& reader_guid, const Locator* unicast_locs,
              uint8_t unicast_count, bool expects_inline_qos = false) noexcept {
        clear();
        remoteReaderGuid  = reader_guid;
        expectsInlineQos  = expects_inline_qos;
        isActive          = true;
        uint8_t n = unicast_count < RTPS_MAX_READER_PROXY_LOCATORS
                    ? unicast_count
                    : RTPS_MAX_READER_PROXY_LOCATORS;
        for (uint8_t i = 0; i < n; ++i)
            unicastLocatorList[i] = unicast_locs[i];
        unicastLocatorCount = n;
    }

    void clear() noexcept {
        remoteReaderGuid    = GUID_UNKNOWN;
        remoteGroupEntityId = GUID_UNKNOWN;
        expectsInlineQos    = false;
        isActive            = false;
        unicastLocatorCount    = 0;
        multicastLocatorCount  = 0;
        highestAckedSN      = SEQUENCENUMBER_ZERO;
        lastAcknackCount    = 0;
        nackPending         = false;
        nackPendingSinceMs  = 0;
        for (uint8_t i = 0; i < N; ++i) changes_[i].clear();
    }

    bool is_matched() const noexcept {
        return remoteReaderGuid != GUID_UNKNOWN;
    }

    // ----------------------------------------------------------
    // add_change()
    //
    // Records a new CacheChange in the proxy's tracking table with
    // status UNSENT.  Called by StatefulWriter::add_change() after
    // inserting into the HistoryCache.
    //
    // "When add_change is called, the change is associated with a
    //  ChangeForReader object with status UNSENT."
    //  — §8.4.1.4, p. 107; §8.4.6, p. 128.
    //
    // @param sn  SequenceNumber of the new CacheChange.
    // @returns   true if a slot was available.
    // ----------------------------------------------------------
    bool add_change(const SequenceNumber& sn) noexcept {
        // For KEEP_LAST(1) evict the oldest slot if full.
        ChangeForReader* slot = find_slot(sn);
        if (!slot) slot = evict_oldest();
        if (!slot) return false;
        slot->sequenceNumber = sn;
        slot->status         = ChangeForReaderStatus::UNSENT;
        return true;
    }

    /**
     * @brief True when this proxy tracks a ChangeForReader for `sn`.
     *
     * The writer adds a ChangeForReader for each change written after the
     * reader matched (and, for TRANSIENT_LOCAL, each cached change replayed
     * at matching). A NACKed change it does not track is irrelevant to this
     * reader, and the writer answers it with GAP.
     *
     * @param[in] sn Sequence number the reader requested.
     * @return `true` when the change is tracked in any status.
     */
    bool tracks(const SequenceNumber& sn) const noexcept {
        for (uint8_t i = 0; i < N; ++i) {
            if (changes_[i].is_valid() && changes_[i].sequenceNumber == sn) return true;
        }
        return false;
    }

    // ----------------------------------------------------------
    // remove_change()
    //
    // Removes a ChangeForReader entry once the change has been
    // evicted from the HistoryCache (KEEP_LAST eviction or explicit
    // remove_change on the writer).
    //
    // — §8.4.1.4 remove_change, p. 108.
    //
    // @param sn  SequenceNumber to remove.
    // ----------------------------------------------------------
    void remove_change(const SequenceNumber& sn) noexcept {
        for (uint8_t i = 0; i < N; ++i) {
            if (changes_[i].sequenceNumber == sn) {
                changes_[i].clear();
                return;
            }
        }
    }

    // ----------------------------------------------------------
    // acked_changes_set()
    //
    // Marks all changes with SN <= committed_sn as ACKNOWLEDGED,
    // and advances highestAckedSN.
    //
    // "acked_changes_set(committed_seq_num) — Transition all changes
    //  with sequenceNumber <= committed_seq_num to ACKNOWLEDGED."
    //  — §8.4.6, p. 128
    //
    // Called from StatefulWriter::on_acknack() after validating the
    // ACKNACK count and extracting the cumulative ACK point.
    //
    // @param committed_sn  All SNs <= this are now acknowledged.
    // ----------------------------------------------------------
    void acked_changes_set(const SequenceNumber& committed_sn) noexcept {
        if (committed_sn > highestAckedSN) {
            highestAckedSN = committed_sn;
        }
        for (uint8_t i = 0; i < N; ++i) {
            if (!changes_[i].is_valid()) continue;
            if (changes_[i].sequenceNumber <= committed_sn) {
                changes_[i].status = ChangeForReaderStatus::ACKNOWLEDGED;
            }
        }
    }

    // ----------------------------------------------------------
    // requested_changes_set()
    //
    // Marks a set of specific SNs as REQUESTED (NACKed by the reader).
    // Records nackPendingSinceMs for the nackResponseDelay timer.
    //
    // "requested_changes_set(req_sn_set) — For each change in the
    //  req_sn_set, set the status of the ChangeForReader to REQUESTED."
    //  — §8.4.6, p. 128
    //
    // Called from StatefulWriter::on_acknack() for each SN in the
    // ACKNACK's readerSNState bitmap.
    //
    // @param sn_set          Bitmap of SNs being NACKed.
    // @param now_ms          Current rosette::platform::monotonic_ms() value.
    // ----------------------------------------------------------
    void requested_changes_set(const SequenceNumberSet& sn_set,
                               uint32_t now_ms) noexcept {
        bool any_requested = false;
        sn_set.for_each([&](const SequenceNumber& sn) noexcept {
            for (uint8_t i = 0; i < N; ++i) {
                if (changes_[i].sequenceNumber == sn &&
                    changes_[i].status != ChangeForReaderStatus::ACKNOWLEDGED) {
                    changes_[i].status = ChangeForReaderStatus::REQUESTED;
                    any_requested = true;
                }
            }
        });
        if (any_requested && !nackPending) {
            nackPending        = true;
            nackPendingSinceMs = now_ms;
        }
    }

    // ----------------------------------------------------------
    // mark_underway()
    //
    // Transitions a REQUESTED change to UNDERWAY when the writer
    // begins retransmitting it.
    //
    // "The status of the ChangeForReader is set to UNDERWAY when
    //  the Writer begins sending the change."
    //  — §8.4.1.1, p. 103 (ChangeForReaderStatus description)
    //
    // @param sn  SequenceNumber being retransmitted.
    // ----------------------------------------------------------
    void mark_underway(const SequenceNumber& sn) noexcept {
        for (uint8_t i = 0; i < N; ++i) {
            if (changes_[i].sequenceNumber == sn &&
                changes_[i].status == ChangeForReaderStatus::REQUESTED) {
                changes_[i].status = ChangeForReaderStatus::UNDERWAY;
            }
        }
    }

    // ----------------------------------------------------------
    // mark_sent()
    //
    // Transitions an UNSENT change to UNACKNOWLEDGED after the
    // writer dispatches it to the transport for this reader.
    //
    // "The status transitions from UNSENT to UNACKNOWLEDGED when
    //  the change is sent."
    //  — §8.4.1.1, p. 103
    //
    // @param sn  SequenceNumber that was just sent.
    // ----------------------------------------------------------
    void mark_sent(const SequenceNumber& sn) noexcept {
        for (uint8_t i = 0; i < N; ++i) {
            if (changes_[i].sequenceNumber == sn &&
                changes_[i].status == ChangeForReaderStatus::UNSENT) {
                changes_[i].status = ChangeForReaderStatus::UNACKNOWLEDGED;
            }
        }
    }

    // ----------------------------------------------------------
    // §8.4.6  unsent_changes()
    //
    // Returns whether any ChangeForReader has status UNSENT.
    //
    // "unsent_changes() — Returns the set of changes for the reader
    //  that have not been sent yet."
    //  — §8.4.6, p. 128
    // ----------------------------------------------------------
    bool has_unsent_changes() const noexcept {
        for (uint8_t i = 0; i < N; ++i) {
            if (changes_[i].is_valid() &&
                changes_[i].status == ChangeForReaderStatus::UNSENT)
                return true;
        }
        return false;
    }

    // ----------------------------------------------------------
    // §8.4.6  next_unsent_change()
    //
    // "next_unsent_change() — Returns the next unsent CacheChange
    //  for the reader."  — §8.4.6, p. 128
    //
    // Returns the UNSENT ChangeForReader with the lowest SN, or
    // nullptr if none.
    // ----------------------------------------------------------
    const ChangeForReader* next_unsent_change() const noexcept {
        const ChangeForReader* best = nullptr;
        for (uint8_t i = 0; i < N; ++i) {
            if (!changes_[i].is_valid()) continue;
            if (changes_[i].status != ChangeForReaderStatus::UNSENT) continue;
            if (!best || changes_[i].sequenceNumber < best->sequenceNumber)
                best = &changes_[i];
        }
        return best;
    }

    // ----------------------------------------------------------
    // §8.4.6  unacked_changes()
    //
    // "unacked_changes() — Returns the set of changes for the reader
    //  that have not been acknowledged."  — §8.4.6, p. 128
    //
    // Returns true if any change is UNACKNOWLEDGED, REQUESTED, or
    // UNDERWAY.
    // ----------------------------------------------------------
    bool has_unacked_changes() const noexcept {
        for (uint8_t i = 0; i < N; ++i) {
            if (!changes_[i].is_valid()) continue;
            auto s = changes_[i].status;
            if (s == ChangeForReaderStatus::UNACKNOWLEDGED ||
                s == ChangeForReaderStatus::REQUESTED      ||
                s == ChangeForReaderStatus::UNDERWAY)
                return true;
        }
        return false;
    }

    /**
     * @brief True while some tracked change is not yet acknowledged by this reader.
     *
     * The reliable StatefulWriter keeps a ReaderProxy "announcing" (sending
     * it the periodic HEARTBEAT) exactly while unacked_changes() is
     * non-empty, and "idle" otherwise. unacked_changes() is every change not
     * ACKNOWLEDGED, so a change still UNSENT counts too.
     *
     * @return `true` when any tracked change has a status other than ACKNOWLEDGED.
     *
     * Verification:
     * - Correct means: a proxy whose changes are all ACKNOWLEDGED (or that
     *   tracks none) is idle; any UNSENT, UNACKNOWLEDGED, REQUESTED or
     *   UNDERWAY change makes it announcing.
     * - Covered by: TC-SWR-8.4.9-004, TC-SWR-8.2.2-002.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.4.9.2 (transitions T5-T7), §8.4.7.5 (unacked_changes)
     */
    bool is_announcing() const noexcept {
        for (uint8_t i = 0; i < N; ++i) {
            if (changes_[i].is_valid() &&
                changes_[i].status != ChangeForReaderStatus::ACKNOWLEDGED)
                return true;
        }
        return false;
    }

    // ----------------------------------------------------------
    // §8.4.6  requested_changes()
    //
    // Returns true if any change has status REQUESTED.
    // ----------------------------------------------------------
    bool has_requested_changes() const noexcept {
        for (uint8_t i = 0; i < N; ++i) {
            if (changes_[i].is_valid() &&
                changes_[i].status == ChangeForReaderStatus::REQUESTED)
                return true;
        }
        return false;
    }

    // ----------------------------------------------------------
    // §8.4.6  next_requested_change()
    //
    // "next_requested_change() — Returns the next change that has
    //  been requested by the Reader."  — §8.4.6, p. 128
    //
    // Returns the REQUESTED ChangeForReader with the lowest SN, or
    // nullptr if none.
    // ----------------------------------------------------------
    const ChangeForReader* next_requested_change() const noexcept {
        const ChangeForReader* best = nullptr;
        for (uint8_t i = 0; i < N; ++i) {
            if (!changes_[i].is_valid()) continue;
            if (changes_[i].status != ChangeForReaderStatus::REQUESTED) continue;
            if (!best || changes_[i].sequenceNumber < best->sequenceNumber)
                best = &changes_[i];
        }
        return best;
    }

    // ----------------------------------------------------------
    // Iteration over all tracked changes.
    // fn : void(const ChangeForReader&)
    // ----------------------------------------------------------
    template<typename Fn>
    void for_each_change(Fn&& fn) const noexcept {
        for (uint8_t i = 0; i < N; ++i) {
            if (changes_[i].is_valid()) fn(changes_[i]);
        }
    }

    template<typename Fn>
    void for_each_change(Fn&& fn) noexcept {
        for (uint8_t i = 0; i < N; ++i) {
            if (changes_[i].is_valid()) fn(changes_[i]);
        }
    }

    // ----------------------------------------------------------
    // nack_delay_elapsed()
    //
    // Returns true when a pending NACK's nackResponseDelay has elapsed.
    // Called from StatefulWriter::tick() every tick period.
    //
    // §8.4.1.1, p. 104:
    //   "nackResponseDelay: the duration of time that the Writer will
    //    wait for an ACKNACK response before retransmitting."
    //
    // @param now_ms         Current rosette::platform::monotonic_ms().
    // @param delay_ms       Configured nackResponseDelay in milliseconds.
    // ----------------------------------------------------------
    bool nack_delay_elapsed(uint32_t now_ms, uint32_t delay_ms) const noexcept {
        if (!nackPending) return false;
        return (now_ms - nackPendingSinceMs) >= delay_ms;
    }

    /// Clear the nackPending flag after retransmission is dispatched.
    void clear_nack_pending() noexcept {
        nackPending        = false;
        nackPendingSinceMs = 0;
    }

    // ----------------------------------------------------------
    // Best locator for addressing this reader.
    // Prefer a UDP locator; some peers advertise other transports
    // (for example SHM) ahead of UDP in SEDP.
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

private:

    ChangeForReader changes_[N];

    /// Find an existing slot for sn, or an empty slot.
    ChangeForReader* find_slot(const SequenceNumber& sn) noexcept {
        for (uint8_t i = 0; i < N; ++i) {
            if (!changes_[i].is_valid() ||
                changes_[i].sequenceNumber == sn)
                return &changes_[i];
        }
        return nullptr;
    }

    /// Evict the slot with the lowest SN — KEEP_LAST(1) eviction.
    ChangeForReader* evict_oldest() noexcept {
        ChangeForReader* oldest = nullptr;
        for (uint8_t i = 0; i < N; ++i) {
            if (!oldest ||
                changes_[i].sequenceNumber < oldest->sequenceNumber)
                oldest = &changes_[i];
        }
        if (oldest) oldest->clear();
        return oldest;
    }
};

/// Default alias used by StatefulWriter.
using DefaultReaderProxy = ReaderProxy<RTPS_READER_PROXY_CHANGE_DEPTH>;

} // namespace rtps
