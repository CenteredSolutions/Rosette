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

    /// The writer has been heard since matching: a HEARTBEAT, DATA, GAP or
    /// fragment submessage arrived from it. Until then the reader keeps announcing itself with
    /// a preemptive ACKNACK (StatefulReader::tick()), since a writer that
    /// believes this reader up to date has no reason to send anything.
    bool heard = false;

    /// The writer's vendor reads a preemptive ACKNACK with bitmapBase 0
    /// (eProsima, RTI) rather than 1 (StatefulReader::send_preemptive_acknack()).
    bool zeroBaseAcknack = false;

    /// Preemptive ACKNACKs still to send while the writer is unheard, the
    /// next included (RTPS_PREEMPTIVE_ACKNACK_ATTEMPTS, or the builtin
    /// readers' RTPS_BUILTIN_PREEMPTIVE_ACKNACK_ATTEMPTS, at match).
    uint8_t preemptiveAcknacksLeft = 0;

    /// True while an ACKNACK is owed: one answering a HEARTBEAT, or, before
    /// the writer is heard, the next preemptive ACKNACK.
    bool acknackPending = false;

    /// monotonic_ms() at which the owed ACKNACK is due (heartbeatResponseDelay,
    /// or the preemptive ACKNACK's retry period).
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
     * so a steady stream of them is still answered once per delay. The
     * first HEARTBEAT heard from the writer replaces a pending preemptive
     * ACKNACK: it is answered after `response_delay_ms`, or not at all when
     * it owes nothing.
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
     *   ACKNACK keeps the deadline of the HEARTBEAT that first owed it; the
     *   first HEARTBEAT heard marks the writer heard and cancels the retry.
     * - Covered by: TC-SRR-8.4.9-001, TC-SRR-8.4.7-001, TC-SRR-8.4.7.4-007,
     *   TC-SRR-8.4.7.4-008.
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
     * @brief Records a preemptive ACKNACK sent, and owes the next one while the writer is
     * unheard and attempts are left.
     *
     * @param[in] now_ms   rosette::platform::monotonic_ms() when it was sent.
     * @param[in] retry_ms RTPS_PREEMPTIVE_ACKNACK_RETRY_MS: when the next one is due.
     */
    void on_preemptive_acknack_sent(uint32_t now_ms, uint32_t retry_ms) noexcept {
        on_acknack_sent();
        if (preemptiveAcknacksLeft > 0u) {
            --preemptiveAcknacksLeft;
        }

        if (!heard && preemptiveAcknacksLeft > 0u) {
            acknackPending = true;
            heartbeatResponseDeadlineMs = now_ms + retry_ms;
        }
    }

    /// @brief Marks the writer heard, cancelling a pending preemptive ACKNACK; idempotent.
    void on_heard() noexcept;

    /// @brief Locator for ACKNACKs: the referenced locator's current value, or
    /// LOCATOR_INVALID when there is none (referenced_locator()).
    const Locator& best_locator() const noexcept { return referenced_locator(unicastLocator); }

private:
    SequenceNumberWindow changes_;          ///< Resolved changes (changes_from_writer).
    SequenceNumber       last_available_ = SEQUENCENUMBER_ZERO;  ///< Highest SN the writer holds.
};

}  // namespace rosette::rtps
