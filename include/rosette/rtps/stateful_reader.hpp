// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file rtps/stateful_reader.hpp
 * @brief StatefulReader — reliable RTPS reader with per-writer proxy tracking.
 *
 * Implements the StatefulReader behaviour model defined in §8.4.7–§8.4.9
 * of the DDSI-RTPS 2.5 specification (OMG formal/2022-04-01).
 *
 * Delivery
 * --------
 * Each matched writer has a WriterProxy (writer_proxy.hpp) recording which of
 * its changes are resolved. A RELIABLE reader delivers every change exactly
 * once, repairs included, and requests MISSING changes in ACKNACKs until DATA
 * or GAP resolves them (§8.4.12.2). A BEST_EFFORT reader delivers only
 * changes newer than every change it has seen from that writer (§8.4.12.1).
 * A change that withdraws an instance (inline QoS StatusInfo DISPOSED or
 * UNREGISTERED) is delivered with its NOT_ALIVE ChangeKind even when it
 * carries no payload, so consumers such as SEDP learn of the withdrawal.
 * Every accepted change goes to the OnChangeAvailable callback, whose owner
 * plays the role of the reader's HistoryCache (§8.2.2); the reader itself
 * retains no delivered change.
 *
 * Proxy storage
 * -------------
 * StatefulReader binds its WriterProxies from a WriterProxyPool
 * (proxy_pool.hpp) when writers match and returns them when they unmatch,
 * keeping the bound ones on a list, so one compiled reader serves readers of
 * every capacity and every pass visits only matched writers. A participant's
 * user readers are SharedPoolStatefulReader, all binding from the
 * participant's pool, each with a floor of RTPS_MIN_MATCHES_PER_ENDPOINT and
 * a cap of RTPS_MAX_MATCHED_WRITERS. The SEDP builtin readers, which match
 * one announcer per active peer and must never wait for a proxy, are
 * BoundedStatefulReader<RTPS_MAX_PEERS>, each with a private pool. Pass
 * either around as StatefulReader.
 *
 * Fragment reassembly
 * -------------------
 * DATA_FRAG submessages go to the reader's FragmentAssembler (§8.4.14),
 * whose slots come from a FragmentAssemblyPool: a participant's user readers
 * share one, and a BoundedStatefulReader has a private one with a slot per
 * writer it matches, up to RTPS_ASSEMBLY_SLOTS_PER_READER. A completed sample is accepted
 * exactly as a DATA would be. A missing
 * fragment is requested with a NACK_FRAG built from the assembler's missing
 * fragments, in answer to a HEARTBEAT_FRAG or, for a sample the writer's
 * HEARTBEAT shows as missing, alongside the ACKNACK (send_acknack()). A
 * reliable writer's partial sample that receives no fragment for
 * RTPS_FRAGMENT_REPAIR_DELAY_MS is requested unprompted, with the wait
 * doubling after each request (send_due_fragment_repairs()), so a lost last
 * fragment costs one quiet period rather than one HEARTBEAT period.
 * Un-matching a writer discards its partial samples.
 *
 * Spec references (all OMG formal/2022-04-01)
 * -------------------------------------------
 *   StatefulReader               — §8.4.7,   p. 131
 *   WriterProxy                  — §8.4.7.1, p. 134
 *   heartbeatResponseDelay       — §8.4.7.1, p. 135
 *   send_acknack()               — §8.4.7.3, p. 140
 *   matched_writer_add/remove    — §8.4.7.4, p. 141
 *   on_data / on_data_frag       — §8.4.8,   p. 142
 *   on_heartbeat / on_gap        — §8.4.9,   p. 145
 *   FragmentedChange / assembly  — §8.4.14,  p. 156
 *   NACK_FRAG generation         — §8.4.12,  p. 153
 *   HEARTBEAT_FRAG processing    — §8.3.7.4, p. 90
 *   HEARTBEAT FinalFlag rule     — §8.3.7.5, p. 91
 *   missing_changes()            — §8.4.11,  p. 150
 */

#pragma once

#include "fragment_assembler.hpp"
#include "history_cache.hpp"
#include "message_builder.hpp"
#include "message_parser.hpp"
#include "proxy_pool.hpp"
#include "types.hpp"
#include "writer_proxy.hpp"

namespace rosette::rtps {

struct ITransport;

/// The pool a reader takes its writer proxies from.
using WriterProxyPool = TypedProxyPool<WriterProxy>;

// ============================================================
// StatefulReader
// ============================================================

class StatefulReader {
public:

    /// Partly received changes requested by NACK_FRAG in one ACKNACK message
    /// (send_acknack()); two fit a control-message block
    /// (MessageBuilder::kControlMessageCapacity) with the ACKNACK beside them.
    static constexpr uint8_t kMaxNackFragsPerAcknack = 2u;

    /// Consumer callback — fired when a complete CacheChange arrives. The
    /// change is valid only for the duration of the call; a consumer that
    /// needs the payload later retains its buffer (BufRef::retain()), as
    /// rmw_rosette's ChangeRing does. The consumer is the reader's history:
    /// the reader itself keeps no delivered changes.
    using OnChangeAvailable = void (*)(const CacheChange& change,
                                      void* user_data) noexcept;

    StatefulReader(const StatefulReader&)            = delete;
    StatefulReader& operator=(const StatefulReader&) = delete;

    /**
     * @brief Adds a WriterProxy for a newly matched writer and, when RELIABLE, asks it for a
     * HEARTBEAT.
     *
     * Discovery (SEDP, or the participant matching a local reader to a known
     * remote writer) calls this once per match. A RELIABLE reader drops DATA
     * from writers it has not matched, so DATA a writer sent before this side
     * matched it is lost here, and the writer has no reason to resend it until
     * its next periodic HEARTBEAT (3 s by default in Fast DDS). The first
     * request a stock ROS 2 client sends right after creating the client is
     * exactly such DATA.
     *
     * A RELIABLE reader therefore sends a preemptive ACKNACK at once
     * (send_preemptive_acknack()): nothing acknowledged, nothing requested,
     * F-flag clear. The writer answers with a HEARTBEAT, the HEARTBEAT shows
     * the dropped changes as MISSING, and the ordinary ACKNACK requests them,
     * so the loss costs a few round trips instead of a HEARTBEAT period. A
     * BEST_EFFORT reader sends nothing.
     *
     * @param[in] writer_guid      GUID of the remote writer.
     * @param[in] unicast_locators Locators ACKNACKs are sent to; may be null
     *   when `locator_count` is 0, in which case no ACKNACK can be sent.
     * @param[in] locator_count    Number of entries in `unicast_locators`.
     * @param[in] writer_vendor    Vendor of the writer's participant (SPDP
     *   PID_VENDOR_ID); selects the preemptive ACKNACK's form. Unknown for a
     *   local writer.
     * @return `true` when a proxy was added; `false` when the writer is
     *   already matched or the pool refuses a proxy (the reader is at its
     *   cap, or above its floor with every free proxy promised to other
     *   floors; ProxyPool::bind()). RTPSParticipant keeps a refused match
     *   waiting and calls this again once may_add_writer() holds.
     *
     * Verification:
     * - Correct means: one proxy per matched writer; a RELIABLE reader sends
     *   exactly one non-final, empty ACKNACK on a new match, in the form the
     *   writer's vendor recognizes, with a count above every ACKNACK this
     *   reader sent before (1 on its first match), so a writer that
     *   remembers this reader from an earlier match accepts it; a
     *   BEST_EFFORT reader and a rejected duplicate send none.
     * - Covered by: TC-SRR-8.4.7.4-001, TC-SRR-8.4.7.4-002,
     *   TC-SRR-8.4.7.4-003, TC-SRR-8.4.7.4-004, TC-SRR-8.4.7.4-006,
     *   TC-SRR-8.4.10-004.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.4.7.4 (matched_writer_add), §8.3.7.1 (ACKNACK
     *   F-flag), §8.4.10.4 (WriterProxy)
     */
    bool matched_writer_add(const Guid& writer_guid, const Locator* unicast_locators,
                            uint8_t locator_count,
                            const VendorId& writer_vendor = VENDORID_UNKNOWN) noexcept;

    // ----------------------------------------------------------
    // §8.4.7.4  matched_writer_remove()
    //
    // "Removes the WriterProxy from the set of matched writers."
    //  — §8.4.7.4, p. 141
    //
    // Returns the writer's proxy to the proxy pool, and discards any
    // in-progress fragment assembly for this writer by calling
    // assembler_.discard_writer(), returning buffer-pool blocks.
    // ----------------------------------------------------------
    void matched_writer_remove(const Guid& writer_guid) noexcept;

    /// §8.4.7.4 — returns nullptr if not found.
    WriterProxy* matched_writer_lookup(const Guid& writer_guid) noexcept;

    /**
     * @brief Processes a DATA submessage addressed to this reader.
     *
     * The change is delivered (to the OnChangeAvailable callback, or the
     * reader history without one) when its writer's proxy says it is new:
     * see is_new_change(). A RELIABLE reader drops DATA from writers it has
     * not matched. The change's kind comes from the inline QoS StatusInfo
     * (see change_kind_of()). A DATA with neither payload nor key is
     * delivered, without payload, only when its StatusInfo withdraws the
     * instance; otherwise it is accounted for but not delivered. When the
     * DATA repairs a MISSING change on a narrow link, the next MISSING change
     * is requested at once.
     *
     * @param[in] ctx Parse context; senderGuidPrefix and sourceTimestamp.
     * @param[in] msg The DATA; its payload is copied into a pool buffer.
     *
     * Verification:
     * - Correct means: each change is delivered at most once; a reliable
     *   reader delivers every change the writer does not GAP; a disposal
     *   arrives as NOT_ALIVE with the writer's key hash.
     * - Covered by: TC-SRR-8.4.8-001, TC-SRR-8.4.8-002, TC-SRR-8.4.8-003,
     *   TC-SRR-8.4.10-001, TC-SRR-8.4.10-002, TC-SRR-8.4.10-003,
     *   TC-SRR-8.4.12-001.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.3.7.2 (DATA), §8.4.12.1, §8.4.12.2,
     *   §9.6.3.3 (StatusInfo_t)
     */
    void on_data(const ParseContext& ctx,
                 const ParsedData&   msg) noexcept;

    // ----------------------------------------------------------
    // §8.4.8  on_data_frag()
    //
    // "Process an incoming DATA_FRAG submessage."  — §8.4.8, p. 142
    //
    // Delegates to assembler_.on_data_frag().  When the assembler
    // reports completion (returns true), the assembled CacheChange is
    // delivered identically to a full DATA reception.
    //
    // Fragment numbers are 1-based — §9.4.5.4, p. 208.
    // Supports up to 256 fragments per sample — §9.4.2.8, p. 199.
    // KEEP_LAST(1) per writer: a newer SN evicts any partial assembly
    // for an older SN — §8.3.3.1, p. 68.
    // ----------------------------------------------------------
    void on_data_frag(const ParseContext&   ctx,
                      const ParsedDataFrag& msg) noexcept;

    // ----------------------------------------------------------
    // §8.4.9  on_heartbeat()
    //
    // "Process an incoming HEARTBEAT submessage."  — §8.4.9, p. 145
    //
    // Delegates to proxy.on_heartbeat() which applies:
    //   lost_changes_update(firstSN)   — §8.4.11, p. 150
    //   missing_changes_update(lastSN) — §8.4.11, p. 150
    //   FinalFlag rule                 — §8.3.7.5, p. 91
    //
    // If heartbeatResponseDelay == 0, the ACKNACK is sent immediately.
    // Otherwise tick() fires it at the deadline.
    // ----------------------------------------------------------
    void on_heartbeat(const ParseContext&    ctx,
                      const ParsedHeartbeat& msg) noexcept;

    // ----------------------------------------------------------
    // §8.4.9 / §8.3.7.4  on_heartbeat_frag()
    //
    // "Process an incoming HEARTBEAT_FRAG submessage."
    //  — §8.4.9, p. 145; §8.3.7.4, p. 90
    //
    // Calls assembler_.missing_fragments() to find which fragments are
    // absent below lastFragmentNum, then sends a NACK_FRAG if any are
    // missing — §8.4.12, p. 153.
    // ----------------------------------------------------------
    void on_heartbeat_frag(const ParseContext&       ctx,
                           const ParsedHeartbeatFrag& msg) noexcept;

    // ----------------------------------------------------------
    // §8.4.9  on_gap()
    //
    // "Process an incoming GAP submessage."  — §8.4.9, p. 145
    //
    // Delegates to proxy.irrelevant_change_set().  Also calls
    // assembler_.discard(writer_guid, sn) for each irrelevant SN that
    // has a partial assembly in progress — those fragments will never
    // complete.
    // ----------------------------------------------------------
    void on_gap(const ParseContext& ctx,
                const ParsedGap&   msg) noexcept;

    /**
     * @brief Sends an ACKNACK to the matched writer.
     *
     * readerSNState normally lists every MISSING change. When the transport's
     * datagram limit is below the IPv4 UDP maximum (a link without IP
     * fragmentation, see narrow_repairs()), it lists only the lowest MISSING
     * change: a writer answers an ACKNACK by packing all requested changes
     * into as few datagrams as its own message size allows (Fast DDS: 65500
     * octets), and such a datagram would be lost on this link again on every
     * retry. One change per request keeps each repair a single DATA, and
     * request_next_repair() asks for the next one as soon as it arrives. A
     * pure ACK (empty set, F-flag set) is sent when nothing is missing.
     *
     * A MISSING change of which some fragments have arrived is requested
     * with a NACK_FRAG naming only the missing fragments, in the same
     * message, and is left out of readerSNState (bitmapBase unchanged), as
     * Fast DDS's reader does. A NACK of the whole change would have the
     * writer resend every fragment back to back; on a link whose receive
     * buffer holds fewer than all of them, the same fragments are lost on
     * every retry, while each NACK_FRAG round keeps what arrived and asks
     * only for the rest. Up to kMaxNackFragsPerAcknack such changes go in
     * one message; further ones are NACKed whole in readerSNState.
     *
     * @param[in,out] proxy      Writer to acknowledge; its count is not
     *   advanced here (the caller calls on_acknack_sent()).
     * @param[in]     final_flag F-flag: the writer need not answer with a
     *   HEARTBEAT.
     *
     * Verification:
     * - Correct means: bitmapBase is at most the lowest MISSING change, every
     *   set bit is MISSING and has no fragment received, on a narrow link at
     *   most one change is requested, and a partly received change is
     *   requested by a NACK_FRAG listing exactly its missing fragments.
     * - Covered by: TC-SRR-8.4.7-002, TC-SRR-8.4.7-003, TC-SRR-8.4.12-001,
     *   TC-SRR-8.4.12-002, tests/ros2/run_rmw_frag_loopback.sh (stock peer).
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.4.7.3, §8.4.12 (reader behavior), §8.3.7.1
     *   (ACKNACK), §8.3.7.5 (NACK_FRAG)
     * - Fast DDS 2.6 StatefulReader::send_acknack (NACK_FRAG for incomplete changes)
     */
    void send_acknack(WriterProxy& proxy,
                      bool         final_flag = false) noexcept;

    // ----------------------------------------------------------
    // tick()
    //
    // Periodic maintenance: sends the ACKNACKs whose heartbeatResponseDelay
    // has elapsed, and the NACK_FRAGs of partial samples whose fragments have
    // stopped arriving. Call at least as often as ms_until_next_timer() asks.
    // ----------------------------------------------------------
    void tick(uint32_t now_ms) noexcept;

    // ----------------------------------------------------------
    // Accessors
    // ----------------------------------------------------------

    const Guid&              guid()        const noexcept { return guid_; }
    ReliabilityKind          reliability() const noexcept { return reliability_; }

    uint32_t heartbeat_response_delay()            const noexcept { return hb_response_ms_; }
    void     heartbeat_response_delay(uint32_t ms)       noexcept { hb_response_ms_ = ms; }

    void set_on_change_available(OnChangeAvailable cb, void* ud) noexcept {
        on_change_avail_ = cb;
        user_data_       = ud;
    }

    /// @brief Number of matched writers (§8.4.7.1 matched_writers).
    uint8_t matched_writer_count() const noexcept { return writers_.count; }

    /// @brief Writers this reader is always granted proxies for (its floor).
    uint8_t min_matched_writers() const noexcept { return writers_.floor; }

    // ----------------------------------------------------------
    // ms_until_next_timer()
    //
    // How long tick() can be deferred: the time left on the earliest
    // pending heartbeatResponseDelay (§8.4.7.1) or fragment repair deadline,
    // or UINT32_MAX when neither is pending. The participant driver sleeps no
    // longer than this.
    //
    // Verification: TC-SRR-8.4.7-001, TC-SRR-8.4.12-005.
    // ----------------------------------------------------------
    uint32_t ms_until_next_timer(uint32_t now_ms) const noexcept;

    /// @brief Most writers this reader can match at once (its cap).
    uint8_t max_matched_writers() const noexcept { return writers_.cap; }

    /// @brief Whether matched_writer_add() of a new writer would now get a
    /// proxy: below the cap, and below the floor or with an unpromised one free.
    bool may_add_writer() const noexcept { return pool_.may_bind(writers_); }

protected:
    /**
     * @brief Creates a reader that binds its WriterProxies from `pool`.
     *
     * Only BoundedStatefulReader and SharedPoolStatefulReader construct a
     * StatefulReader; code that uses a reader holds a StatefulReader& or *.
     *
     * @param[in] guid            This reader's GUID (§8.2.4).
     * @param[in] reliability     RELIABLE or BEST_EFFORT (§8.3.3).
     * @param[in] transport       Transport for ACKNACK and NACK_FRAG; must
     *   outlive the reader.
     * @param[in] pool            The pool the reader binds writer proxies
     *   from. Its owner has reserved `min_writers` proxies in it for this
     *   reader (ProxyPool::reserve()); it outlives the reader and is not
     *   accessed during construction.
     * @param[in] min_writers     The reader's floor: matches always granted.
     * @param[in] max_writers     The reader's cap, at least `min_writers` and 1.
     * @param[in] assembly_pool   Slots the reader reassembles fragmented
     *   samples in, possibly shared with other readers; outlives the reader.
     * @param[in] assembly_limit  Most slots of `assembly_pool` the reader
     *   holds at once, 1 to its capacity.
     * @param[in] on_change_avail Callback fired on each new complete change;
     *   null discards delivered changes.
     * @param[in] user_data       Forwarded to on_change_avail.
     * @param[in] hb_response_ms  heartbeatResponseDelay in milliseconds
     *   (§8.4.7.1); 0 answers a HEARTBEAT immediately.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.4.7.1 (StatefulReader attributes), §8.4.14
     */
    StatefulReader(const Guid& guid, ReliabilityKind reliability, ITransport& transport,
                   WriterProxyPool& pool, uint8_t min_writers, uint8_t max_writers,
                   FragmentAssemblyPool& assembly_pool, uint16_t assembly_limit,
                   OnChangeAvailable on_change_avail, void* user_data,
                   uint32_t hb_response_ms) noexcept;

    /// @brief Returns every bound proxy to the pool.
    ~StatefulReader() noexcept;

private:

    WriterProxy* find_proxy(const Guid& guid) noexcept;
    WriterProxy* find_proxy_by_sender(const GuidPrefix& prefix,
                                      const EntityId&   writer_eid) noexcept;

    /// @brief The proxies of the matched writers, in match order.
    WriterProxyPool::Range<WriterProxy> matched() noexcept { return pool_.bound(writers_); }

    WriterProxyPool::Range<const WriterProxy> matched() const noexcept {
        return static_cast<const WriterProxyPool&>(pool_).bound(writers_);
    }

    /**
     * @brief True when repairs must be requested one change at a time.
     *
     * @return repairs_must_fit_datagram() for this reader's transport.
     */
    bool narrow_repairs() const noexcept;

    /**
     * @brief Requests the next MISSING change right after a repair lands.
     *
     * With narrow repairs a burst of N lost changes needs N request/repair
     * round trips; sending the next ACKNACK when a repair (or a GAP for a
     * requested change) arrives, rather than waiting for the writer's next
     * periodic HEARTBEAT, makes the burst cost N round trips instead of N
     * heartbeat periods. Does nothing on unrestricted links or when nothing
     * is missing.
     *
     * @param[in,out] proxy Writer whose change was just repaired.
     *
     * Verification:
     * - Correct means: on a narrow link, receiving a repaired change while
     *   others are MISSING sends exactly one ACKNACK naming the lowest one.
     * - Covered by: TC-SRR-8.4.7-003.
     */
    void request_next_repair(WriterProxy& proxy) noexcept;

    /**
     * @brief Sends one NACK_FRAG naming a partial sample's missing fragments.
     *
     * Answers a HEARTBEAT_FRAG and a quiet-period repair deadline alike. The
     * message carries INFO_DST for the writer's participant, as the ACKNACK
     * does, and advances the proxy's count, which ACKNACK and NACK_FRAG share
     * so each is above the last the writer processed.
     *
     * @param[in,out] proxy   Writer of the sample; its count is advanced.
     * @param[in]     sn      Sequence number of the sample.
     * @param[in]     missing Fragments to request; not empty.
     *
     * Verification:
     * - Correct means: one message, sent to the writer's unicast locator,
     *   holding INFO_DST and a NACK_FRAG for `sn` with exactly `missing`.
     * - Covered by: TC-SRR-8.4.12-005.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.3.7.5 (NACK_FRAG), §9.4.5.8, §8.4.12
     */
    void send_nack_frag(WriterProxy& proxy, const SequenceNumber& sn,
                        const FragmentNumberSet& missing) noexcept;

    /**
     * @brief Requests the missing fragments of every partial sample whose fragments stopped
     * arriving.
     *
     * A writer sends a sample's fragments back to back, and repairs a
     * missing one only when a reader names it. A gap in the middle of a
     * sample shows once a later fragment arrives, but a lost last fragment
     * shows only when the writer's next HEARTBEAT or HEARTBEAT_FRAG reports
     * the sample, which Fast DDS sends every 3 s by default. So each partial
     * sample of a reliable writer carries a deadline that each fragment it
     * did not already hold pushes ahead (on_data_frag()); a duplicate or
     * rejected fragment does not, so repeated traffic cannot postpone the
     * request. Once the deadline passes, the sample's
     * missing fragments are requested. The first wait is
     * RTPS_FRAGMENT_REPAIR_DELAY_MS and each request doubles it, up to
     * RTPS_FRAGMENT_REPAIR_MAX_DOUBLINGS times: 50, 100, 200 and 400 ms
     * with the defaults, for up to RTPS_FRAGMENT_REPAIR_ROUNDS rounds while
     * no new fragment arrives. A new fragment restores the rounds but not
     * the wait, so a writer pacing its fragments further apart than the
     * first wait is asked early only until the wait outgrows its pacing,
     * and a new sample starts again at the first wait. A sample the
     * reader has since received whole, or that a GAP or HEARTBEAT declared
     * irrelevant, is discarded instead.
     *
     * @param[in] now_ms Monotonic time, in milliseconds.
     *
     * Verification:
     * - Correct means: nothing is sent while fragments keep arriving within
     *   the delay; duplicate or rejected fragments neither delay a request
     *   nor restore spent rounds; after a quiet delay exactly one NACK_FRAG per partial
     *   sample names its missing fragments, including the last; unanswered
     *   requests follow the doubling waits and stop after
     *   RTPS_FRAGMENT_REPAIR_ROUNDS; a fragment arriving after a request
     *   restarts the current wait, not the first; a best-effort reader
     *   never sends one; a sample received whole, or declared irrelevant
     *   by a HEARTBEAT or GAP, is requested no more and its slot is freed.
     * - Covered by: TC-SRR-8.4.12-005…010.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.4.12 (reader may send NACK_FRAG for missing
     *   fragments), §8.3.7.5 (NACK_FRAG), §8.4.14
     * - Fast DDS 2.6 StatefulWriter::process_nack_frag (repairs the named
     *   fragments of any change still in its history)
     */
    void send_due_fragment_repairs(uint32_t now_ms) noexcept;

    /**
     * @brief Sends the preemptive ACKNACK of a new match, in the form the writer's vendor
     * recognizes.
     *
     * The ACKNACK acknowledges nothing, requests nothing and has the F-flag
     * clear. DDSI-RTPS requires a writer to answer any non-final ACKNACK with
     * a HEARTBEAT, and Cyclone DDS does, for the spec-valid bitmapBase 1.
     * Fast DDS answers only its own form, bitmapBase 0 (which the spec's
     * SequenceNumberSet does not allow); for bitmapBase 1 it restarts its
     * periodic HEARTBEAT timer, which delays the HEARTBEAT a full period.
     * Cyclone in turn rejects bitmapBase 0 as malformed unless the sender is
     * eProsima or RTI. So the form follows the writer's vendor: bitmapBase 0
     * for eProsima and RTI (which sends the same form), bitmapBase 1 for
     * everyone else.
     *
     * @param[in,out] proxy         The writer just matched; its count is
     *   advanced (on_acknack_sent()).
     * @param[in]     writer_vendor Vendor of the writer's participant.
     *
     * Verification:
     * - Correct means: one ACKNACK to the writer's unicast locator, empty
     *   set, F-flag clear, count 1, bitmapBase 0 exactly for eProsima and RTI.
     * - Covered by: TC-SRR-8.4.7.4-002, TC-SRR-8.4.7.4-004.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.3.7.1 (ACKNACK), §8.3.5.5 (SequenceNumberSet)
     * - Fast DDS 2.6 StatefulWriter::process_acknack, WriterProxy::perform_initial_ack_nack
     * - Cyclone DDS 0.10 q_receive.c validate_AckNack, handle_AckNack
     */
    void send_preemptive_acknack(WriterProxy& proxy, const VendorId& writer_vendor) noexcept;

    /**
     * @brief True when the change `sn` from `proxy`'s writer should be delivered.
     *
     * RELIABLE: the change is unresolved and within the proxy's window
     * (WriterProxy::is_new_change()). BEST_EFFORT: the change is newer than
     * every change seen from that writer.
     *
     * @param[in] proxy Matched writer the change came from.
     * @param[in] sn    Sequence number of the arriving change.
     * @return `true` to deliver, `false` to drop.
     *
     * Verification:
     * - Correct means: no change is delivered twice; a best-effort reader
     *   never delivers a change older than one it delivered.
     * - Covered by: TC-SRR-8.4.10-002, TC-SRR-8.4.12-001.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.4.12.1, §8.4.12.2
     */
    bool is_new_change(const WriterProxy& proxy, const SequenceNumber& sn) const noexcept;

    /**
     * @brief Records an accepted change in `proxy`.
     *
     * A BEST_EFFORT reader also resolves every older change as LOST, so the
     * next change it accepts must be newer still.
     *
     * @param[in,out] proxy Matched writer the change came from.
     * @param[in]     sn    Sequence number of the accepted change.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.4.12.1 (best-effort), §8.4.10.4 (received_change_set)
     */
    void record_change(WriterProxy& proxy, const SequenceNumber& sn) noexcept;

    /**
     * @brief Resolves a change that will never be delivered, so it is never requested again.
     *
     * Two kinds of change end here. A DATA with neither payload nor key
     * that withdraws nothing has nothing to deliver. A sample larger than
     * the largest pool block (kPoolMaxBlockSize) can never be stored: its
     * writer would resend it on every request, and a reliable reader that
     * kept waiting for it could record no change more than
     * SequenceNumberWindow::kCapacity past it, so every later change on the
     * topic would stall. Such a sample is resolved as LOST, which the next
     * ACKNACK acknowledges. A transient pool shortage is not handled here:
     * that change stays MISSING and is repaired later.
     *
     * @param[in,out] proxy Matched writer the change came from.
     * @param[in]     sn    Sequence number of the change.
     *
     * Verification:
     * - Correct means: the change is resolved and never delivered; on a
     *   narrow link, resolving a MISSING change requests the next one.
     * - Covered by: TC-SRR-8.4.8-004, TC-SRR-8.4.10-003.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.4.10.4 (received_change_set), §8.4.12.2
     * - OMG DDS 1.4 §2.2.4.1 (SAMPLE_LOST status)
     */
    void account_undelivered(WriterProxy& proxy, const SequenceNumber& sn) noexcept;

    /**
     * @brief Hands a complete, accepted change to the consumer callback.
     *
     * The reader keeps no copy: holding delivered changes would pin one
     * pool block per sample until evicted. Without a callback the change
     * is dropped, and its payload returns to the pool when the caller's
     * CacheChange goes out of scope.
     *
     * @param[in] change Change accepted by on_data() or on_data_frag().
     */
    void deliver(const CacheChange& change) noexcept;

    // ── Data members ─────────────────────────────────────────

    Guid              guid_;
    ReliabilityKind   reliability_;
    ITransport&       transport_;
    OnChangeAvailable on_change_avail_ = nullptr;
    void*             user_data_       = nullptr;

    WriterProxyPool& pool_;
    ProxyList writers_;  ///< Bound proxies, floor and cap.

    /// Fragment reassembler — §8.4.14, p. 156. Its slots come from the
    /// assembly pool the reader was constructed with.
    FragmentAssembler assembler_;

    uint32_t       hb_response_ms_;
    MessageBuilder builder_;

    /// Highest ACKNACK count of any proxy this reader has removed. A writer
    /// keeps its record of a reader across the reader's unmatch and rematch
    /// (a participant that parks and reactivates a peer does exactly that)
    /// and ignores an ACKNACK whose count is not above the last it processed
    /// (§8.3.7.1), so a new proxy counts on from here.
    int32_t acknack_count_floor_ = 0;
};

namespace detail {
/// @brief The private pools of a BoundedStatefulReader, writer proxies and
/// assembly slots, as a base class so they are constructed before the
/// StatefulReader base that points into them.
template <uint8_t kMaxWriters>
struct BoundedReaderSlots {
    /// One partial sample per writer the reader matches (KEEP_LAST(1) per
    /// writer), and no more than a participant's user reader may hold.
    static constexpr uint16_t kAssemblySlots =
        kMaxWriters < RTPS_ASSEMBLY_SLOTS_PER_READER ? kMaxWriters : RTPS_ASSEMBLY_SLOTS_PER_READER;

    ProxyPoolStorage<WriterProxy, kMaxWriters> proxies;
    FragmentAssemblyPoolStorage<kAssemblySlots> assembly;

    /// Promises the whole private pool to its one reader, whose floor is kMaxWriters.
    BoundedReaderSlots() noexcept { (void)proxies.reserve(kMaxWriters); }
};
}  // namespace detail

/**
 * @brief A StatefulReader with a private pool of `kMaxWriters` WriterProxies.
 *
 * Its capacity is its own and always available: floor and cap are both
 * kMaxWriters, and it reassembles fragmented samples in a private pool of
 * one slot per writer, up to RTPS_ASSEMBLY_SLOTS_PER_READER, as many as a
 * participant's user reader may hold. The SEDP builtin readers are of this
 * kind. The protocol
 * logic lives once in StatefulReader (stateful_reader.cpp); this template
 * only adds the pool, so readers of different capacities share all code.
 *
 * @tparam kMaxWriters Most writers matched at once (1-255).
 */
template <uint8_t kMaxWriters>
class BoundedStatefulReader final : private detail::BoundedReaderSlots<kMaxWriters>,
                                    public StatefulReader {
    static_assert(kMaxWriters >= 1u, "a reader must be able to match a writer");

public:
    /**
     * @brief Creates a reader with no matched writers.
     *
     * The parameters are those of StatefulReader's constructor, less the
     * proxy pool this class provides.
     */
    BoundedStatefulReader(const Guid& guid, ReliabilityKind reliability, ITransport& transport,
                          OnChangeAvailable on_change_avail = nullptr, void* user_data = nullptr,
                          uint32_t hb_response_ms = RTPS_HEARTBEAT_RESPONSE_DELAY_MS) noexcept
        : StatefulReader(guid, reliability, transport, this->proxies, kMaxWriters, kMaxWriters,
                         this->assembly, detail::BoundedReaderSlots<kMaxWriters>::kAssemblySlots,
                         on_change_avail, user_data, hb_response_ms) {}
};

/// A standalone reader for user topics: up to RTPS_MAX_MATCHED_WRITERS matched writers.
using DefaultStatefulReader = BoundedStatefulReader<RTPS_MAX_MATCHED_WRITERS>;

/**
 * @brief A StatefulReader whose WriterProxies and assembly slots come from
 *   pools its owner shares among readers.
 *
 * RTPSParticipant creates its user readers as these, all binding from the
 * participant's WriterProxy pool and reassembling in its assembly pool, so
 * per-match storage scales with the matches the participant has at once
 * rather than with readers × cap.
 *
 * Example:
 * @code{.cpp}
 * rosette::rtps::ProxyPoolStorage<rosette::rtps::WriterProxy, 16> pool;
 * rosette::rtps::FragmentAssemblyPoolStorage<6> assembly;
 * (void)pool.reserve(1);                 // this reader's floor
 * rosette::rtps::SharedPoolStatefulReader reader(guid, rosette::rtps::ReliabilityKind::RELIABLE,
 * transport, pool, 1, 8, assembly, 5, on_change, &app); if (reader.matched_writer_add(writer_guid,
 * &locator, 1u, rosette::rtps::VENDORID_UNKNOWN)) {
 *     // DATA from that writer now reaches on_change.
 * }
 * @endcode
 * Full program: tests/spec/src/test_stateful_reader_8_4_7_4.cpp (TC-SRR-8.4.7.4-006)
 */
class SharedPoolStatefulReader final : public StatefulReader {
public:
    /**
     * @brief Creates a reader with no matched writers.
     *
     * @param[in] guid            This reader's GUID (§8.2.4).
     * @param[in] reliability     RELIABLE or BEST_EFFORT (§8.3.3).
     * @param[in] transport       Transport for ACKNACK and NACK_FRAG; outlives the reader.
     * @param[in] pool            Pool shared with other readers; the caller
     *   has reserved `min_writers` proxies in it for this reader, and it
     *   outlives the reader.
     * @param[in] min_writers     Floor: matches always granted.
     * @param[in] max_writers     Cap: most matches at once, at least `min_writers` and 1.
     * @param[in] assembly_pool   Assembly slots shared with other readers; outlives the reader.
     * @param[in] assembly_limit  Most of those slots this reader holds at once.
     * @param[in] on_change_avail Callback fired on each new complete change; null discards.
     * @param[in] user_data       Forwarded to on_change_avail.
     * @param[in] hb_response_ms  heartbeatResponseDelay in milliseconds (§8.4.7.1).
     */
    SharedPoolStatefulReader(const Guid& guid, ReliabilityKind reliability, ITransport& transport,
                             WriterProxyPool& pool, uint8_t min_writers, uint8_t max_writers,
                             FragmentAssemblyPool& assembly_pool, uint16_t assembly_limit,
                             OnChangeAvailable on_change_avail = nullptr, void* user_data = nullptr,
                             uint32_t hb_response_ms = RTPS_HEARTBEAT_RESPONSE_DELAY_MS) noexcept
        : StatefulReader(guid, reliability, transport, pool, min_writers, max_writers,
                         assembly_pool, assembly_limit, on_change_avail, user_data,
                         hb_response_ms) {}
};

}  // namespace rosette::rtps
