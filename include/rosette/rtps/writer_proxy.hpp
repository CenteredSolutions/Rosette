// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file rtps/writer_proxy.hpp
 * @brief WriterProxy: a StatefulReader's state for one matched remote writer.
 *
 * A StatefulReader keeps one WriterProxy per matched writer (DDSI-RTPS 2.5
 * §8.4.10.4) and uses it to decide which arriving changes to deliver, which
 * changes to request in an ACKNACK, and when a HEARTBEAT must be answered.
 *
 * changes_from_writer
 * -------------------
 * The spec gives every change from the writer a ChangeFromWriterStatus:
 *
 *   RECEIVED / LOST  resolved: received, declared irrelevant by GAP, or no
 *                    longer held by the writer (HEARTBEAT firstSN)
 *   MISSING          unresolved, and the writer is known to hold it
 *   UNKNOWN          unresolved, and the writer has not announced it yet
 *
 * The proxy stores the resolved set in a SequenceNumberWindow and the
 * boundary between MISSING and UNKNOWN as the highest sequence number the
 * writer is known to hold (HEARTBEAT lastSN, or any change or GAP it sent).
 * Nothing about a MISSING change is ever discarded to make room, so a hole
 * is requested until DATA or GAP closes it, however many later changes
 * arrive meanwhile. A change more than SequenceNumberWindow::kCapacity past
 * the lowest hole cannot be recorded; a reliable reader does not deliver it
 * on arrival and requests it once the hole closes, so every change is
 * delivered exactly once.
 *
 * Spec mapping (OMG DDSI-RTPS 2.5, formal/2022-04-01)
 * ---------------------------------------------------
 *   WriterProxy attributes             §8.4.10.4
 *   available_changes_max()            §8.4.10.4
 *   irrelevant_change_set()            §8.4.10.4
 *   lost_changes_update()              §8.4.10.4
 *   missing_changes()                  §8.4.10.4
 *   missing_changes_update()           §8.4.10.4
 *   received_change_set()              §8.4.10.4
 *   heartbeatResponseDelay             §8.4.10.1
 *   HEARTBEAT final-flag rule          §8.3.7.5
 *   reliable reader behavior           §8.4.12.2
 *
 * Locator reference
 * -----------------
 * ACKNACKs and NACK_FRAGs go to one unicast locator of the writer, and the
 * proxy keeps a pointer to it in the record it was matched from (the SPDP
 * peer record or an EndpointMatchTable locator list) rather than a copy, exactly
 * as ReaderProxy does (see "Locator reference" in rtps/reader_proxy.hpp).
 * When that record takes a new address, the proxy follows it.
 */

#pragma once

#include "sn_window.hpp"
#include "types.hpp"

/// Quiet time, in milliseconds, after the last change from a writer before
/// a reader requests the changes it is missing from that writer without
/// waiting for a HEARTBEAT; the same wait as a lost fragment's
/// (RTPS_FRAGMENT_REPAIR_DELAY_MS).
#ifndef RTPS_GAP_REPAIR_DELAY_MS
#  define RTPS_GAP_REPAIR_DELAY_MS 50u
#endif

/// Times the gap-repair wait doubles after a request goes unanswered.
#ifndef RTPS_GAP_REPAIR_MAX_DOUBLINGS
#  define RTPS_GAP_REPAIR_MAX_DOUBLINGS 3u
#endif

/// Unprompted requests for a writer's missing changes while no new change
/// arrives from it; 0 leaves every request to the writer's HEARTBEATs.
#ifndef RTPS_GAP_REPAIR_ROUNDS
#  define RTPS_GAP_REPAIR_ROUNDS 4u
#endif

static_assert(RTPS_GAP_REPAIR_MAX_DOUBLINGS <= 8u, "RTPS_GAP_REPAIR_MAX_DOUBLINGS must be 0 to 8");
static_assert(RTPS_GAP_REPAIR_DELAY_MS >= 1u
                  && RTPS_GAP_REPAIR_DELAY_MS <= (0x7fffffffu >> RTPS_GAP_REPAIR_MAX_DOUBLINGS),
              "RTPS_GAP_REPAIR_DELAY_MS, doubled RTPS_GAP_REPAIR_MAX_DOUBLINGS times, "
              "must fit a wrap-safe millisecond deadline");
static_assert(RTPS_GAP_REPAIR_ROUNDS <= 255u, "RTPS_GAP_REPAIR_ROUNDS must be 0 to 255");

namespace rosette::rtps {

class WriterProxy {
public:

    // ----------------------------------------------------------
    // Spec attributes (§8.4.10.4)
    // ----------------------------------------------------------

    // remoteGroupEntityId and multicastLocatorList are not stored: Rosette
    // has no group entities, and ACKNACKs go to the writer's unicast
    // locators only.

    Guid remoteWriterGuid = GUID_UNKNOWN;

    /// The entry of the writer's unicastLocatorList ACKNACKs go to, owned by
    /// the record the proxy was matched from; null when matched with none.
    const Locator* unicastLocator = nullptr;

    // ----------------------------------------------------------
    // Reader-side protocol state
    // ----------------------------------------------------------

    /// Count of the last HEARTBEAT processed; older counts are ignored (§8.3.7.5).
    int32_t lastHBCount = 0;

    /// Count of the last ACKNACK or NACK_FRAG sent to this writer (§8.3.7.1).
    int32_t acknackCount = 0;

    /// Liveliness of the remote writer.
    bool isAlive = false;

    /// True while an ACKNACK answering a HEARTBEAT is owed.
    bool acknackPending = false;

    /// monotonic_ms() at which the owed ACKNACK is due (heartbeatResponseDelay).
    uint32_t heartbeatResponseDeadlineMs = 0;

    WriterProxy() noexcept = default;

    /**
     * @brief Starts tracking a newly matched writer from a clean state.
     *
     * The proxy refers to the first UDP entry of `unicast_locs` (else its
     * first entry; preferred_unicast_locator()) and reads it at every send.
     *
     * @param[in] writer_guid   GUID of the remote writer.
     * @param[in] unicast_locs  Locators ACKNACKs are sent to. Not copied: the
     *   array must stay valid, and is read in place, until the proxy is
     *   cleared. May be null when `unicast_count` is 0.
     * @param[in] unicast_count Number of entries in `unicast_locs`.
     *
     * Verification:
     * - Correct means: the proxy is matched with no changes resolved, and
     *   best_locator() tracks the referenced entry's current value.
     * - Covered by: TC-SRR-8.4.7.4-001, TC-SRR-8.4.10-004.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.4.10.4, §8.4.7.4 (matched_writer_add)
     */
    void init(const Guid& writer_guid, const Locator* unicast_locs,
              uint8_t unicast_count) noexcept;

    /// @brief Returns the proxy to the unmatched state.
    void clear() noexcept { *this = WriterProxy{}; }

    /// @brief True while the proxy represents a matched writer.
    bool is_matched() const noexcept { return remoteWriterGuid != GUID_UNKNOWN; }

    /**
     * @brief Highest sequence number below which every change is resolved.
     *
     * @return base of the resolved window minus one; an ACKNACK's bitmapBase
     *   is this plus one.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.4.10.4 (available_changes_max)
     */
    SequenceNumber available_changes_max() const noexcept { return changes_.base() - 1; }

    /**
     * @brief True when a reliable reader should deliver the change `sn`.
     *
     * @param[in] sn Sequence number of an arriving DATA or DATA_FRAG.
     * @return `true` when `sn` is unresolved and can be recorded; `false`
     *   for a duplicate, for a change a GAP or HEARTBEAT resolved, and for a
     *   change too far past the lowest hole to record (it is requested again
     *   once the hole closes).
     *
     * Verification:
     * - Correct means: every change is delivered exactly once, including
     *   repairs that fill a hole below later changes.
     * - Covered by: TC-SRR-8.4.8-001, TC-SRR-8.4.8-002, TC-SRR-8.4.10-002.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.4.12.2
     */
    bool is_new_change(const SequenceNumber& sn) const noexcept {
        return changes_.covers(sn) && !changes_.is_resolved(sn);
    }

    /**
     * @brief Records that the change `sn` arrived.
     *
     * The change is resolved, and every change below it is known to exist:
     * those still unresolved become MISSING.
     *
     * @param[in] sn Sequence number of a delivered or accounted-for change.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.4.10.4 (received_change_set)
     */
    void received_change_set(const SequenceNumber& sn) noexcept;

    /**
     * @brief Returns a received change the reader evicted to MISSING.
     *
     * A reader that holds changes behind a gap (HeldChanges) evicts its
     * highest held change when it runs out of room; that change was
     * received but never delivered, so it is requested again. Never applied
     * to a change at or below available_changes_max(): those are delivered.
     *
     * @param[in] sn Sequence number of the evicted change.
     *
     * Verification:
     * - Correct means: `sn` is MISSING again and the next ACKNACK names it,
     *   and available_changes_max() is unchanged.
     * - Covered by: TC-SRR-8.4.10.4-014.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.4.10.4 (changes_from_writer: RECEIVED to MISSING)
     */
    void received_change_withdraw(const SequenceNumber& sn) noexcept {
        (void)changes_.unresolve(sn);
    }

    /**
     * @brief Records the changes a GAP declares irrelevant.
     *
     * A writer sends GAP for changes it will never deliver to this reader:
     * changes written before a VOLATILE reader matched, or evicted from its
     * history. Each is resolved, so the reader stops requesting it and never
     * delivers it. The contiguous run is applied in constant time however
     * long it is; bitmap entries (at most 256) one by one.
     *
     * @param[in] gap_start First sequence number of the contiguous run.
     * @param[in] gap_list  bitmapBase ends the run; each set bit names one
     *   more irrelevant change.
     *
     * Verification:
     * - Correct means: every change named by the GAP is resolved, no other
     *   change changes state, and later DATA for a named change is not
     *   delivered.
     * - Covered by: TC-SRR-8.4.9-003, TC-SRR-8.4.9-004, TC-SRR-8.4.7-003.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.3.7.4.5, §8.4.10.4 (irrelevant_change_set)
     */
    void irrelevant_change_set(const SequenceNumber&    gap_start,
                               const SequenceNumberSet& gap_list) noexcept;

    /**
     * @brief Resolves every change below `first_available_sn` as LOST.
     *
     * Applies a HEARTBEAT's firstSN, below which the writer holds nothing,
     * and a best-effort reader's acceptance of a newer change.
     *
     * @param[in] first_available_sn Lowest sequence number still expected.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.4.10.4 (lost_changes_update), §8.4.12.1
     */
    void lost_changes_update(const SequenceNumber& first_available_sn) noexcept {
        changes_.resolve_below(first_available_sn);
    }

    /**
     * @brief Records that the writer holds changes up to `last_available_sn`.
     *
     * Unresolved changes up to it become MISSING rather than UNKNOWN.
     *
     * @param[in] last_available_sn A HEARTBEAT's lastSN, or the sequence
     *   number of a change the writer sent.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.4.10.4 (missing_changes_update)
     */
    void missing_changes_update(const SequenceNumber& last_available_sn) noexcept {
        if (last_available_ < last_available_sn) last_available_ = last_available_sn;
    }

    /**
     * @brief Builds the readerSNState of an ACKNACK to this writer.
     *
     * @param[out] out       bitmapBase = available_changes_max() + 1 (which
     *   acknowledges every change below it) with one bit per MISSING change,
     *   lowest first; empty when nothing is MISSING.
     * @param[in]  max_count Most changes to request (1 on a link whose
     *   repairs must each fit one datagram).
     *
     * Verification:
     * - Correct means: every set bit is MISSING, bitmapBase is never above a
     *   MISSING change, and the lowest MISSING changes are requested first.
     * - Covered by: TC-SRR-8.4.9-001, TC-SRR-8.4.7-002, TC-SRR-8.4.7-003,
     *   TC-SRR-8.4.10-001.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.4.10.4 (missing_changes), §8.3.7.1 (ACKNACK)
     */
    void missing_changes(SequenceNumberSet& out, uint32_t max_count) const noexcept {
        changes_.unresolved(last_available_, max_count, out);
    }

    /// @brief True when some change is MISSING.
    bool has_missing_changes() const noexcept { return !(last_available_ < changes_.base()); }

    /// @brief True when `sn` is MISSING, so its arrival is a repair.
    bool is_missing(const SequenceNumber& sn) const noexcept {
        return !(last_available_ < sn) && !changes_.is_resolved(sn);
    }

    /**
     * @brief Applies a HEARTBEAT and decides whether it must be answered.
     *
     * A HEARTBEAT whose count is not newer than lastHBCount is a duplicate
     * and is ignored. Otherwise firstSN resolves everything the writer no
     * longer holds, lastSN makes the rest MISSING, and an ACKNACK is owed
     * when the F-flag is clear or something is MISSING (§8.3.7.5). An owed
     * ACKNACK is due `response_delay_ms` after the HEARTBEAT that first owed
     * it; later HEARTBEATs before it is sent do not push that deadline back,
     * so a steady stream of them is still answered once per delay.
     *
     * @param[in] first_sn          HEARTBEAT firstSN.
     * @param[in] last_sn           HEARTBEAT lastSN.
     * @param[in] hb_count          HEARTBEAT count.
     * @param[in] final_flag        HEARTBEAT F-flag.
     * @param[in] now_ms            rosette::platform::monotonic_ms().
     * @param[in] response_delay_ms heartbeatResponseDelay in milliseconds.
     * @return `true` when an ACKNACK is now owed; `false` for a duplicate or
     *   a final HEARTBEAT to an up-to-date reader.
     *
     * Verification:
     * - Correct means: duplicates change nothing; F=0 always owes an
     *   ACKNACK; F=1 owes one only while something is MISSING; an owed
     *   ACKNACK keeps the deadline of the HEARTBEAT that first owed it.
     * - Covered by: TC-SRR-8.4.9-001, TC-SRR-8.4.7-001, TC-SRR-8.4.7.4-007.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.3.7.5, §8.4.12.2
     */
    bool on_heartbeat(const SequenceNumber& first_sn,
                      const SequenceNumber& last_sn,
                      int32_t               hb_count,
                      bool                  final_flag,
                      uint32_t              now_ms,
                      uint32_t              response_delay_ms) noexcept;

    /**
     * @brief True when the owed ACKNACK's heartbeatResponseDelay has elapsed.
     *
     * @param[in] now_ms rosette::platform::monotonic_ms(); compared wrap-safely.
     */
    bool acknack_response_due(uint32_t now_ms) const noexcept {
        return acknackPending && (now_ms - heartbeatResponseDeadlineMs) < 0x80000000u;
    }

    /// @brief Records that an ACKNACK was sent: advances its count and clears the owed response.
    void on_acknack_sent() noexcept {
        ++acknackCount;
        acknackPending = false;
        heartbeatResponseDeadlineMs = 0;
    }

    /**
     * @brief Starts or renews the gap-repair timer on a new change or a HEARTBEAT.
     *
     * A writer resends a lost change only when a reader names it, and a
     * reader learns of a loss when a later change arrives. Without a request
     * of its own the reader names it only in answer to the writer's next
     * HEARTBEAT, up to 3 s later with Fast DDS defaults. So the first new
     * change that leaves a change MISSING sets a deadline
     * RTPS_GAP_REPAIR_DELAY_MS (doubled once per unanswered request,
     * gapRepairDoublings) ahead, and when it passes the reader's timer pass
     * sends an ACKNACK naming the missing changes. The wait covers ordinary
     * reordering in the network. Later changes restore the
     * RTPS_GAP_REPAIR_ROUNDS rounds, since the writer is evidently still
     * sending, but do not push a running deadline back, so a writer that
     * streams faster than the wait is still asked in time. With nothing
     * MISSING the timer is idle and the next gap starts at the first wait.
     * A HEARTBEAT that shows changes MISSING starts or renews the timer the
     * same way, which covers the tail of a burst, where no later change
     * reveals the loss: the HEARTBEAT's own answer spends the first round,
     * and a lost resend is requested again after the doubled wait rather
     * than at the writer's next HEARTBEAT.
     * This is the repair a partial sample gets from
     * RTPS_FRAGMENT_REPAIR_DELAY_MS, for whole changes.
     *
     * @param[in] now_ms Monotonic time, in milliseconds, of the arriving change.
     *
     * Verification:
     * - Correct means: with a change MISSING the rounds are
     *   RTPS_GAP_REPAIR_ROUNDS and the deadline is `now_ms` plus the doubled
     *   wait unless a deadline was already running, which is kept; with
     *   none the rounds and doublings are 0.
     * - Covered by: TC-SRR-8.4.12-020, TC-SRR-8.4.12-021, TC-SRR-8.4.12-024.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.4.12.2 (a reader may send an ACKNACK without a
     *   HEARTBEAT), §8.4.15.4 (writers answer an ACKNACK that NACKs changes)
     */
    void schedule_gap_repair(uint32_t now_ms) noexcept;

    /**
     * @brief True when the writer has gone quiet with a change still MISSING and rounds left.
     *
     * @param[in] now_ms Monotonic time, in milliseconds; compared wrap-safely.
     */
    bool gap_repair_due(uint32_t now_ms) const noexcept {
        return ms_until_gap_repair(now_ms) == 0u;
    }

    /**
     * @brief Milliseconds until gap_repair_due() holds; UINT32_MAX when it never will without a new
     * change.
     *
     * @param[in] now_ms Monotonic time, in milliseconds.
     */
    uint32_t ms_until_gap_repair(uint32_t now_ms) const noexcept;

    /**
     * @brief Records that an ACKNACK requesting changes went to the writer.
     *
     * Any such request, from the timer or in answer to a HEARTBEAT, spends a
     * round and doubles the wait (up to RTPS_GAP_REPAIR_MAX_DOUBLINGS), so
     * the timer does not ask again while the writer's resend is in flight.
     *
     * @param[in] now_ms Monotonic time, in milliseconds, at which it was sent.
     *
     * Verification:
     * - Correct means: with rounds left, one fewer round, doublings up by one
     *   to the cap, and the next deadline `now_ms` plus the doubled wait.
     * - Covered by: TC-SRR-8.4.12-021, TC-SRR-8.4.12-022.
     */
    void gap_repair_sent(uint32_t now_ms) noexcept;

    /// @brief Moves a due request that could not be sent one current wait later, spending no
    /// round.
    void gap_repair_postpone(uint32_t now_ms) noexcept {
        gapRepairDueMs = now_ms + gap_repair_wait();
    }

    /// @brief Locator for ACKNACKs: the referenced locator's current value, or
    /// LOCATOR_INVALID when there is none (referenced_locator()).
    const Locator& best_locator() const noexcept { return referenced_locator(unicastLocator); }

private:
    /// The current gap-repair wait: RTPS_GAP_REPAIR_DELAY_MS doubled gapRepairDoublings times.
    uint32_t gap_repair_wait() const noexcept {
        return static_cast<uint32_t>(RTPS_GAP_REPAIR_DELAY_MS) << gapRepairDoublings;
    }

    uint32_t gapRepairDueMs = 0;      ///< When the next unprompted request is due.
    uint8_t  gapRepairsLeft = 0;      ///< Unprompted requests left until a new change arrives.
    uint8_t  gapRepairDoublings = 0;  ///< Unanswered requests since the gap opened, capped.

    SequenceNumberWindow changes_;          ///< Resolved changes (changes_from_writer).
    SequenceNumber       last_available_ = SEQUENCENUMBER_ZERO;  ///< Highest SN the writer holds.
};

}  // namespace rosette::rtps
