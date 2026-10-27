/**
 * @file rtps/reader_proxy.hpp
 * @brief ReaderProxy — per-reader state maintained by a StatefulWriter.
 *
 * A ReaderProxy represents a remote RTPS Reader that has been matched to
 * a local StatefulWriter.  The writer maintains one ReaderProxy per
 * matched reader and uses it to decide which changes to send, which to
 * repair, which to GAP, and whether the reader still owes an
 * acknowledgement.
 *
 * Spec mapping
 * ------------
 * This class implements the ReaderProxy entity defined in §8.4.1.1,
 * Table 8.49, p. 103 of OMG formal/2022-04-01.
 *
 * Attributes (§8.4.1.1, Table 8.49, p. 103):
 *   remoteReaderGuid          — GUID of the matched remote reader.
 *   unicastLocatorList        — the locator this reader is addressed at
 *                               (see "Locator reference" below).
 *   expectsInlineQos          — whether this reader requires inline QoS.
 *   isActive                  — true while reader is reachable / alive.
 *
 * remoteGroupEntityId and multicastLocatorList are not stored: Rosette has
 * no group (Publisher/Subscriber) entities, and a writer sends to each
 * matched reader at its unicast locators only.
 *
 * Reader reliability
 * ------------------
 * The proxy also records whether its reader is RELIABLE. A writer's behavior
 * toward each matched reader depends on the reliabilityLevel of both
 * (§8.4.4): a RELIABLE writer may match a BEST_EFFORT reader, and toward it
 * follows the best-effort behavior (§8.4.9.1): it pushes each change once,
 * sends no HEARTBEAT, and expects no ACKNACK. StatefulWriter reads it from
 * isReliable.
 *
 * Locator reference
 * -----------------
 * A writer sends each reader's traffic to one unicast locator, so the proxy
 * keeps one: a pointer to the chosen entry of the list it was matched with,
 * not a copy. That list belongs to a record that outlives the match and is
 * updated in place: the SPDP peer record for the remote participant's
 * locators, or an EndpointMatchTable locator list for locators an endpoint
 * announced itself. A peer that keeps its GUID and moves to a new address
 * re-announces SPDP, the peer record takes the new address, and the proxy
 * sends there from then on, with no rematch and no reliability state lost.
 * The proxy costs the size of a pointer however many locators the reader
 * announced.
 *
 * changes_for_reader as sequence-number bounds
 * --------------------------------------------
 * The spec gives every change in the writer's HistoryCache a
 * ChangeForReaderStatus per reader (§8.4.1.1). A writer sends changes in
 * sequence order and readers acknowledge cumulatively, so those statuses
 * are fully determined by a few sequence numbers plus the writer's history,
 * and the proxy stores only those:
 *
 *   irrelevant      below firstRelevantSN: written before a VOLATILE reader
 *                   matched (§8.7.2.2.1). Such a change, or one no longer
 *                   in the history, is answered with GAP when NACKed.
 *   ACKNOWLEDGED    at or below highestAckedSN.
 *   UNSENT          above highestSentSN and still in the history.
 *   REQUESTED       named in the requested window (below).
 *   UNACKNOWLEDGED  every other relevant change up to highestSentSN;
 *                   UNDERWAY is folded in here, because a change whose
 *                   repair has been sent is treated exactly like one sent
 *                   the first time.
 *
 * The requested window is a base sequence number and a 64-bit bitmap
 * holding the lowest REQUESTED changes. A NACK for a change beyond the
 * window is not recorded; the directed HEARTBEAT that follows each repair
 * makes the reader NACK it again (§8.3.7.5), so every NACKed change is
 * repaired, in rounds of at most 64.
 *
 * The cost of a proxy is therefore independent of the writer's history
 * depth. That matters most for SEDP's builtin writers, which keep one
 * proxy per remote participant and whose history holds one announcement
 * per local endpoint.
 *
 * nackResponseDelay (§8.4.1.1, p. 104)
 * ---------------------------------------
 * "After receiving a NACK, the Writer waits nackResponseDelay before
 *  retransmitting."  The delay is enforced in StatefulWriter::tick();
 *  ReaderProxy records nackPendingSinceMs at the moment
 *  requested_changes_set() first records a request.
 *
 * Spec references (all OMG formal/2022-04-01)
 * -------------------------------------------
 *   ReaderProxy attributes    — §8.4.1.1, Table 8.49, p. 103
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

#include "history_cache.hpp"
#include "types.hpp"

namespace rosette::rtps {

/**
 * @brief A StatefulWriter's record of one matched remote reader (§8.4.1.1).
 *
 * The writer keeps one ReaderProxy per matched reader. The proxy records
 * how far the reader has been sent and has acknowledged, where its relevant
 * changes start, and which changes it has NACKed (see the file comment);
 * operations that depend on which changes exist take the writer's history.
 *
 * Thread safety: not thread-safe; the caller holds the participant lock.
 */
class ReaderProxy {
public:
    /// Most REQUESTED changes recorded at once (the requested window's width).
    static constexpr uint32_t kRequestedWindow = 64u;

    // ----------------------------------------------------------
    // §8.4.1.1, Table 8.49, p. 103 — spec attributes
    // ----------------------------------------------------------

    Guid    remoteReaderGuid   = GUID_UNKNOWN;  ///< §8.4.1.1
    bool    expectsInlineQos   = false;         ///< §8.4.1.1
    bool    isActive           = false;         ///< §8.4.1.1
    /// The reader announced RELIABLE; see "Reader reliability" above. Kept
    /// beside the other flags, where it costs no padding.
    bool    isReliable = true;
    /// A TRANSIENT_LOCAL writer is replaying its history to this reader from
    /// tick(), a budget at a time (StatefulWriter::drain_replays()), so the
    /// write paths leave its UNSENT changes to the replay. Fills the padding
    /// after the other flags.
    bool    replaying = false;

    /// The entry of this reader's unicastLocatorList it is addressed at
    /// (§8.4.1.1), owned by the record the proxy was matched from; null
    /// when it was matched with none. See "Locator reference" above.
    const Locator* unicastLocator = nullptr;

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

    /// Count field from the last NACK_FRAG processed for this proxy; a
    /// NACK_FRAG whose count is not above it is a duplicate. — §8.3.7.5.
    int32_t lastNackFragCount = 0;

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
     * @brief Starts tracking a newly matched reader.
     *
     * Resets every attribute, then records the reader's GUID, inline-QoS
     * expectation, reliability, the locator it is addressed at, and the first
     * change relevant to it. Every relevant change still in the writer's history is
     * then UNSENT (§8.4.1.4, "matched_reader_add").
     *
     * The proxy refers to the first UDP entry of `unicast_locs` (else its
     * first entry) and reads it at every send; see "Locator reference" in
     * the file comment.
     *
     * @param[in] reader_guid        The remote reader's GUID.
     * @param[in] unicast_locs       Its unicast locators; `unicast_count`
     *   entries. Not copied: the array must stay valid, and is read in place,
     *   until the proxy is cleared. May be null when `unicast_count` is 0.
     * @param[in] unicast_count      Number of entries in `unicast_locs`.
     * @param[in] expects_inline_qos Its expectsInlineQos (§8.4.1.1).
     * @param[in] first_relevant_sn  Lowest sequence number the reader is to
     *   receive: 1 for a TRANSIENT_LOCAL writer, which replays its history;
     *   the writer's next sequence number for a VOLATILE one.
     * @param[in] reliable           True when the reader announced RELIABLE,
     *   false for BEST_EFFORT (§8.4.4).
     *
     * Verification:
     * - Correct means: after init, next_unsent_change() walks exactly the
     *   history's changes at or above `first_relevant_sn`, lowest first; and
     *   best_locator() returns the referenced entry's current value, so a
     *   change to it in place redirects the reader's traffic.
     * - Covered by: TC-SWR-8.4.1.4-002, TC-SWR-8.4.1.4-003, TC-SWR-8.4.9-001,
     *   TC-SWR-8.4.1.1-001.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.4.1.4 (matched_reader_add), §8.4.1.1, §8.7.2.2
     */
    void init(const Guid& reader_guid, const Locator* unicast_locs, uint8_t unicast_count,
              bool expects_inline_qos, const SequenceNumber& first_relevant_sn,
              bool reliable) noexcept;

    /**
     * @brief Starts tracking the same reader again as if it had just matched.
     *
     * A remote participant that lost this participant's SPDP lease forgets
     * its endpoints and matches them again on the next announcement, and its
     * readers start over while this proxy still records their old progress.
     * The writer calls this when such a reader shows it has started over
     * (StatefulWriter::on_acknack()). The reader keeps its GUID, locators,
     * inline-QoS expectation and reliability; its acknowledgement, sent and requested
     * changes, and ACKNACK and NACK_FRAG counts reset as init() resets them.
     *
     * @param[in] first_relevant_sn Lowest sequence number the reader is to
     *   receive, chosen as for init().
     *
     * Verification:
     * - Correct means: after restart the proxy answers every query exactly
     *   as a proxy init() gave the same reader and `first_relevant_sn`.
     * - Covered by: TC-SWR-8.4.1.4-004, TC-SWR-8.4.1.4-005, TC-SWR-8.4.1.4-006.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.4.1.4 (matched_reader_add), §8.5.3.2 (lease expiry)
     */
    void restart(const SequenceNumber& first_relevant_sn) noexcept;

    /// @brief Unmatches the proxy: resets every attribute.
    void clear() noexcept;

    /// @brief True while the proxy represents a matched reader.
    bool is_matched() const noexcept { return remoteReaderGuid != GUID_UNKNOWN; }

    /**
     * @brief True when the change `sn` is one this reader is to receive.
     *
     * A change written before a VOLATILE reader matched is irrelevant to
     * it: the writer never sends it and answers a NACK for it with GAP.
     * Whether the change is still in the history is the writer's to check.
     *
     * @param[in] sn Any sequence number the writer has assigned.
     * @return `true` when `sn` is at or above the first relevant change.
     *
     * Verification:
     * - Covered by: TC-SWR-8.4.9-001 (irrelevant changes are GAPped).
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.4.9.2.12, §8.7.2.2.1 (VOLATILE durability)
     */
    bool is_relevant(const SequenceNumber& sn) const noexcept { return !(sn < firstRelevantSN_); }

    /**
     * @brief True when the writer owes this reader nothing more for the change `sn`.
     *
     * A VOLATILE writer asks every matched proxy this before it drops a
     * change from its history. The reader is done with a change it was
     * never to receive (irrelevant), and otherwise with one it has
     * acknowledged (RELIABLE) or been sent (BEST_EFFORT, which repairs
     * nothing).
     *
     * @param[in] sn       Any sequence number the writer has assigned.
     * @param[in] reliable True for a RELIABLE writer, false for BEST_EFFORT.
     * @return `true` when no send or repair of `sn` to this reader can follow.
     *
     * Verification:
     * - Correct means: a RELIABLE proxy is done only with changes at or
     *   below highestAckedSN or below firstRelevantSN; a BEST_EFFORT proxy
     *   also with changes at or below highestSentSN.
     * - Covered by: TC-SWR-8.4.7-003, TC-SWR-8.4.7-005.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.4.7.1 (is_acked_by_all), §8.4.8.1 (best-effort),
     *   §8.7.2.2.1 (VOLATILE durability)
     */
    bool is_done_with(const SequenceNumber& sn, bool reliable) const noexcept {
        return !is_relevant(sn) || !(highestAckedSN < sn) || (!reliable && !(highestSentSN_ < sn));
    }

    /**
     * @brief Marks every change up to `committed_sn` ACKNOWLEDGED.
     *
     * Raises highestAckedSN to `committed_sn` (never lowers it) and drops
     * any request at or below it.
     *
     * @param[in] committed_sn The reader has every change with a number at or
     *   below this (the ACKNACK readerSNState base minus one).
     *
     * Verification:
     * - Correct means: no change at or below `committed_sn` remains
     *   REQUESTED, and is_announcing() is false once `committed_sn` reaches
     *   the history's newest change.
     * - Covered by: TC-SWR-8.4.2.2-001, TC-SWR-8.4.3-001.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.4.6 (acked_changes_set)
     */
    void acked_changes_set(const SequenceNumber& committed_sn) noexcept;

    /**
     * @brief Marks the given changes REQUESTED and starts nackResponseDelay.
     *
     * The caller passes only changes that are relevant to this reader and
     * still in the history (StatefulWriter::on_acknack() GAPs the rest).
     * Changes at or below highestAckedSN are ignored, and so is any change
     * beyond the requested window (kRequestedWindow past the lowest
     * request): the reader NACKs it again after the next repair round. When
     * a change is newly recorded and no NACK is pending yet, records
     * `now_ms` so tick() retransmits after nackResponseDelay.
     *
     * @param[in] sn_set Sequence numbers to repair, from an ACKNACK's
     *   readerSNState or a NACK_FRAG.
     * @param[in] now_ms Current rosette::platform::monotonic_ms().
     *
     * Verification:
     * - Correct means: next_requested_change() then returns the lowest
     *   recorded request; a request below the window's base rebases the
     *   window on it, keeping the lowest requests.
     * - Covered by: TC-SWR-8.4.2.2-001, TC-SWR-8.4.3-001, TC-SWR-8.4.4-001,
     *   TC-SWR-8.4.6-001.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.4.6 (requested_changes_set), §8.4.1.1 (nackResponseDelay)
     */
    void requested_changes_set(const SequenceNumberSet& sn_set, uint32_t now_ms) noexcept;

    /**
     * @brief Takes `sn` out of the requested set as its repair is sent.
     *
     * REQUESTED → UNDERWAY (§8.4.1.1). An UNDERWAY change is tracked as
     * UNACKNOWLEDGED: the reader acknowledges it or NACKs it again.
     *
     * @param[in] sn A change returned by next_requested_change().
     */
    void mark_underway(const SequenceNumber& sn) noexcept;

    /**
     * @brief UNSENT → UNACKNOWLEDGED for `sn` and every change below it.
     *
     * Changes are sent in sequence order, so the caller passes the change
     * next_unsent_change() returned.
     *
     * @param[in] sn Sequence number of the change being sent.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.4.1.1, §8.4.9.2 (reliable StatefulWriter)
     */
    void mark_sent(const SequenceNumber& sn) noexcept {
        if (highestSentSN_ < sn) highestSentSN_ = sn;
    }

    /**
     * @brief The UNSENT change with the lowest sequence number.
     *
     * @param[in] history The writer's history.
     * @return Its sequence number, or SEQUENCENUMBER_UNKNOWN when no change
     *   in `history` is UNSENT.
     *
     * Verification:
     * - Correct means: a change added after the reader matched is UNSENT
     *   until mark_sent(); changes are returned in sequence order.
     * - Covered by: TC-SWR-8.2.2-001, TC-SWR-8.4.1.4-001.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.4.6 (next_unsent_change)
     */
    SequenceNumber next_unsent_change(const HistoryCache& history) const noexcept;

    /// @brief True when some change in `history` is UNSENT (§8.4.6 unsent_changes()).
    bool has_unsent_changes(const HistoryCache& history) const noexcept {
        return next_unsent_change(history) != SEQUENCENUMBER_UNKNOWN;
    }

    /// @brief True when the change `sn` has not been sent to this reader yet:
    ///   it is UNSENT, so a request for it is answered by the send already due.
    bool is_unsent(const SequenceNumber& sn) const noexcept { return highestSentSN_ < sn; }

    /**
     * @brief True while some relevant change is not yet acknowledged by this reader.
     *
     * The reliable StatefulWriter keeps a ReaderProxy "announcing" (sending
     * it the periodic HEARTBEAT) exactly while unacked_changes() is
     * non-empty, and "idle" otherwise. unacked_changes() is every change not
     * ACKNOWLEDGED, so a change still UNSENT counts too.
     *
     * @param[in] history The writer's history.
     * @return `true` when the history's newest change is relevant to this
     *   reader and above highestAckedSN.
     *
     * Verification:
     * - Correct means: a proxy that has acknowledged the history's newest
     *   change, or to which no change in it is relevant, is idle; otherwise
     *   it is announcing.
     * - Covered by: TC-SWR-8.4.9-004, TC-SWR-8.2.2-002.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.4.9.2 (transitions T5-T7), §8.4.7.5 (unacked_changes)
     */
    bool is_announcing(const HistoryCache& history) const noexcept;

    /// @brief True when some change is REQUESTED (§8.4.6 requested_changes()).
    bool has_requested_changes() const noexcept { return requestedBits_ != 0u; }

    /**
     * @brief The REQUESTED change with the lowest sequence number.
     *
     * @return Its sequence number, or SEQUENCENUMBER_UNKNOWN when none is.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.4.6 (next_requested_change)
     */
    SequenceNumber next_requested_change() const noexcept;

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
     * init() chose the entry: a UDP locator when the reader announced one,
     * because some peers advertise other transports (for example shared
     * memory) ahead of UDP. Its value is read now, so it reflects the owning
     * record's latest update.
     *
     * @return The referenced locator; LOCATOR_INVALID when the proxy has
     *   none, or the owning record has since emptied that entry (its kind is
     *   INVALID or RESERVED). Callers skip the send on LOCATOR_INVALID.
     *
     * Verification:
     * - Correct means: after the referenced entry changes in place, the
     *   next HEARTBEAT and DATA go to its new value.
     * - Covered by: TC-SWR-8.4.1.1-001.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.4.1.1 (unicastLocatorList)
     */
    const Locator& best_locator() const noexcept { return referenced_locator(unicastLocator); }

private:
    /// @brief Records one request in the requested window (see requested_changes_set()).
    bool request(const SequenceNumber& sn) noexcept;

    /// Lowest change relevant to this reader (see is_relevant()).
    SequenceNumber firstRelevantSN_ = SequenceNumber::from_int64(1);
    /// Highest change sent to this reader; every relevant change above it is UNSENT.
    SequenceNumber highestSentSN_ = SEQUENCENUMBER_ZERO;
    /// Sequence number of bit 0 of requestedBits_.
    SequenceNumber requestedBase_ = SEQUENCENUMBER_ZERO;
    /// Bit i set: change requestedBase_ + i is REQUESTED.
    uint64_t requestedBits_ = 0u;
};

}  // namespace rosette::rtps
