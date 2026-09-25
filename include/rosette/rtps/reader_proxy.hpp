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
 *   unicastLocatorList        — locators used to address this reader.
 *   expectsInlineQos          — whether this reader requires inline QoS.
 *   isActive                  — true while reader is reachable / alive.
 *
 * remoteGroupEntityId and multicastLocatorList are not stored: Rosette has
 * no group (Publisher/Subscriber) entities, and a writer sends to each
 * matched reader at its unicast locators only.
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
 * Change tracking storage
 * ------------------------
 * A ReaderProxy tracks one ChangeForReader per change in its writer's
 * HistoryCache, so it needs as many slots as the writer's KEEP_LAST depth.
 * It does not own them: the StatefulWriter holds one row of `depth` slots per
 * proxy and binds each proxy to its row (bind_changes()), so ReaderProxy is
 * one compiled class (src/rtps/reader_proxy.cpp) for every writer depth.
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
// ============================================================

#ifndef RTPS_MAX_READER_PROXY_LOCATORS
#  define RTPS_MAX_READER_PROXY_LOCATORS 2
#endif

/**
 * @brief A StatefulWriter's record of one matched remote reader (§8.4.1.1).
 *
 * The writer keeps one ReaderProxy per matched reader and moves each change's
 * ChangeForReader through UNSENT → UNACKNOWLEDGED → (REQUESTED → UNDERWAY →)
 * ACKNOWLEDGED as it sends DATA and receives ACKNACKs (§8.4.9.2).
 *
 * Change slots come from the writer (bind_changes()); a proxy that is not
 * bound tracks nothing. Proxies are array elements of their writer and are
 * never copied: the row pointer refers into the writer that owns it.
 *
 * Thread safety: not thread-safe; the caller holds the participant lock.
 */
class ReaderProxy {
public:
    // ----------------------------------------------------------
    // §8.4.1.1, Table 8.49, p. 103 — spec attributes
    // ----------------------------------------------------------

    Guid    remoteReaderGuid   = GUID_UNKNOWN;  ///< §8.4.1.1
    bool    expectsInlineQos   = false;         ///< §8.4.1.1
    bool    isActive           = false;         ///< §8.4.1.1

    /// Unicast locators used to address this reader — §8.4.1.1.
    Locator unicastLocatorList[RTPS_MAX_READER_PROXY_LOCATORS]   = {};
    uint8_t unicastLocatorCount                                   = 0;

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
    /// Set by requested_changes_set(); cleared by StatefulWriter::tick()
    /// after firing. — §8.4.1.1 nackResponseDelay, p. 104.
    bool nackPending = false;

    /// Millisecond timestamp at which the pending NACK was first recorded.
    /// Used by tick() to compute elapsed time vs. nackResponseDelay.
    uint32_t nackPendingSinceMs = 0;

    ReaderProxy() noexcept = default;
    ReaderProxy(const ReaderProxy&) = delete;
    ReaderProxy& operator=(const ReaderProxy&) = delete;

    /**
     * @brief Gives this proxy its ChangeForReader slots.
     *
     * The owning StatefulWriter calls this once, before the proxy is used,
     * with the row of its per-proxy change table that belongs to this proxy.
     *
     * @param[in] row   First of `depth` slots; owned by the writer and valid
     *   for the proxy's lifetime.
     * @param[in] depth Number of slots: the writer's HistoryCache capacity.
     */
    void bind_changes(ChangeForReader* row, uint8_t depth) noexcept {
        changes_ = row;
        depth_ = depth;
        clear();
    }

    /**
     * @brief Starts tracking a newly matched reader.
     *
     * Resets every attribute and change slot, then records the reader's GUID,
     * inline-QoS expectation, and up to RTPS_MAX_READER_PROXY_LOCATORS unicast
     * locators. The writer then adds a ChangeForReader for each change the
     * reader is to receive (§8.4.1.4, "matched_reader_add").
     *
     * @param[in] reader_guid        The remote reader's GUID.
     * @param[in] unicast_locs       Its unicast locators; `unicast_count` entries.
     * @param[in] unicast_count      Number of locators; extra ones are dropped.
     * @param[in] expects_inline_qos Its expectsInlineQos (§8.4.1.1).
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.4.1.4 (matched_reader_add), §8.4.1.1
     */
    void init(const Guid& reader_guid, const Locator* unicast_locs,
              uint8_t unicast_count, bool expects_inline_qos = false) noexcept;

    /// @brief Unmatches the proxy: resets every attribute and change slot.
    void clear() noexcept;

    /// @brief True while the proxy represents a matched reader.
    bool is_matched() const noexcept { return remoteReaderGuid != GUID_UNKNOWN; }

    /**
     * @brief Tracks a newly written change with status UNSENT.
     *
     * Reuses the slot already holding `sn`, else an empty slot, else evicts
     * the lowest sequence number (the writer's KEEP_LAST eviction removed it
     * from the HistoryCache too).
     *
     * @param[in] sn Sequence number of the new change.
     * @return `true` when the change is tracked; `false` only for an unbound proxy.
     *
     * Verification:
     * - Correct means: after add_change(sn), next_unsent_change() is the
     *   lowest UNSENT number and tracks(sn) holds.
     * - Covered by: TC-SWR-8.2.2-001, TC-SWR-8.4.1.4-001.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.4.1.4 (new change is UNSENT), §8.4.6
     */
    bool add_change(const SequenceNumber& sn) noexcept;

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
     *
     * Verification:
     * - Covered by: TC-SWR-8.4.9-001 (irrelevant changes are GAPped).
     */
    bool tracks(const SequenceNumber& sn) const noexcept;

    /**
     * @brief Stops tracking `sn` after the writer removed it from its history.
     *
     * @param[in] sn Sequence number removed; untracked numbers are ignored.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.4.1.4 (remove_change)
     */
    void remove_change(const SequenceNumber& sn) noexcept;

    /**
     * @brief Marks every tracked change up to `committed_sn` ACKNOWLEDGED.
     *
     * Also raises highestAckedSN to `committed_sn` (never lowers it).
     *
     * @param[in] committed_sn The reader has every change with a number at or
     *   below this (the ACKNACK readerSNState base minus one).
     *
     * Verification:
     * - Correct means: no tracked change at or below `committed_sn` remains
     *   in any other status.
     * - Covered by: TC-SWR-8.4.2.2-001.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.4.6 (acked_changes_set)
     */
    void acked_changes_set(const SequenceNumber& committed_sn) noexcept;

    /**
     * @brief Marks the NACKed changes REQUESTED and starts nackResponseDelay.
     *
     * Changes already ACKNOWLEDGED keep that status. When at least one change
     * became REQUESTED and no NACK is pending yet, records `now_ms` so tick()
     * retransmits after nackResponseDelay.
     *
     * @param[in] sn_set The ACKNACK's readerSNState: numbers the reader lacks.
     * @param[in] now_ms Current rosette::platform::monotonic_ms().
     *
     * Verification:
     * - Covered by: TC-SWR-8.4.2.2-001, TC-SWR-8.4.3-001.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.4.6 (requested_changes_set), §8.4.1.1 (nackResponseDelay)
     */
    void requested_changes_set(const SequenceNumberSet& sn_set, uint32_t now_ms) noexcept;

    /// @brief REQUESTED → UNDERWAY for `sn` as its retransmission starts (§8.4.1.1).
    void mark_underway(const SequenceNumber& sn) noexcept;

    /// @brief UNSENT → UNACKNOWLEDGED for `sn` once it is sent (§8.4.1.1).
    void mark_sent(const SequenceNumber& sn) noexcept;

    /// @brief True when some tracked change is UNSENT (§8.4.6 unsent_changes()).
    bool has_unsent_changes() const noexcept;

    /**
     * @brief The UNSENT change with the lowest sequence number, or null.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.4.6 (next_unsent_change)
     */
    const ChangeForReader* next_unsent_change() const noexcept;

    /// @brief True when some tracked change is UNACKNOWLEDGED, REQUESTED, or
    /// UNDERWAY (§8.4.6 unacked_changes()).
    bool has_unacked_changes() const noexcept;

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
    bool is_announcing() const noexcept;

    /// @brief True when some tracked change is REQUESTED (§8.4.6 requested_changes()).
    bool has_requested_changes() const noexcept;

    /**
     * @brief The REQUESTED change with the lowest sequence number, or null.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.4.6 (next_requested_change)
     */
    const ChangeForReader* next_requested_change() const noexcept;

    /// @brief Calls `fn(const ChangeForReader&)` for each tracked change.
    template<typename Fn>
    void for_each_change(Fn&& fn) const noexcept {
        for (uint8_t i = 0; i < depth_; ++i) {
            if (changes_[i].is_valid()) fn(static_cast<const ChangeForReader&>(changes_[i]));
        }
    }

    /// @brief Calls `fn(ChangeForReader&)` for each tracked change.
    template<typename Fn>
    void for_each_change(Fn&& fn) noexcept {
        for (uint8_t i = 0; i < depth_; ++i) {
            if (changes_[i].is_valid()) fn(changes_[i]);
        }
    }

    /**
     * @brief True when a pending NACK has waited at least `delay_ms`.
     *
     * @param[in] now_ms   Current rosette::platform::monotonic_ms().
     * @param[in] delay_ms The writer's nackResponseDelay in milliseconds.
     * @return `false` when no NACK is pending.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.4.1.1 (nackResponseDelay)
     */
    bool nack_delay_elapsed(uint32_t now_ms, uint32_t delay_ms) const noexcept {
        if (!nackPending) return false;
        return (now_ms - nackPendingSinceMs) >= delay_ms;
    }

    /// @brief Clears the pending NACK once its retransmission is dispatched.
    void clear_nack_pending() noexcept {
        nackPending        = false;
        nackPendingSinceMs = 0;
    }

    /**
     * @brief The locator to send this reader's traffic to.
     *
     * Prefers a UDP locator, because some peers advertise other transports
     * (for example shared memory) ahead of UDP in SEDP.
     *
     * @return The first UDPv4/UDPv6 locator, else the first locator, else
     *   LOCATOR_INVALID.
     */
    const Locator& best_locator() const noexcept;

private:
    /// @brief The slot tracking `sn`, else an empty slot, else null.
    ChangeForReader* find_slot(const SequenceNumber& sn) noexcept;

    /// @brief Clears and returns the slot with the lowest sequence number.
    ChangeForReader* evict_oldest() noexcept;

    ChangeForReader* changes_ = nullptr;  ///< Row of depth_ slots owned by the writer.
    uint8_t depth_ = 0;
};

} // namespace rtps
