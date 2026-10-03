/**
 * @file rtps/stateful_writer.hpp
 * @brief StatefulWriter — the reliable RTPS writer of §8.4.1–§8.4.9.
 *        (OMG formal/2022-04-01)
 *
 * History depth and matched readers
 * ---------------------------------
 * A writer keeps KEEP_LAST(depth) changes (§8.3.3) and one ReaderProxy per
 * matched reader (§8.4.1.1). A proxy records a reader's progress as
 * sequence-number bounds over the history (reader_proxy.hpp), so its size
 * does not depend on the depth: a writer costs depth × CacheChange plus
 * readers × ReaderProxy. All of that storage is fixed-size and chosen at
 * build time.
 *
 * One compiled writer, storage by template
 * ----------------------------------------
 * StatefulWriter is a plain class: every operation lives once in
 * src/rtps/stateful_writer.cpp and works on storage it is handed (a
 * HistoryCache and an array of ReaderProxy).
 * StatefulWriterStorage<Depth, MaxReaders> owns exactly that storage and
 * derives from StatefulWriter, so writers of different depths differ only in
 * their data layout, never in their code:
 *
 *   StatefulWriterStorage<>     — KEEP_LAST(1), RTPS_MAX_MATCHED_READERS readers
 *   StatefulWriterStorage<4>    — KEEP_LAST(4)
 *   StatefulWriterStorage<8,4>  — KEEP_LAST(8), 4 readers
 *
 * Everything outside the object that owns a writer uses StatefulWriter& or
 * StatefulWriter*: RTPSParticipant keeps its writers of every depth in one
 * table of StatefulWriter*, and rmw_rosette publishes through the same type.
 * The only virtual function is the destructor, so a participant can destroy
 * a writer through that pointer and the storage's payload references are
 * released.
 *
 * Spec references (all OMG formal/2022-04-01)
 * -------------------------------------------
 *   KEEP_LAST(N) history         — §8.3.3,   p. 68
 *   StatefulWriter               — §8.4.1,   p. 101
 *   ReaderProxy                  — §8.4.1.1, p. 103
 *   new_change / add_change      — §8.4.1.4, p. 107
 *   matched_reader_add / remove  — §8.4.1.4, p. 108
 *   send_heartbeat               — §8.4.2.2, p. 113
 *   on_acknack                   — §8.4.3,   p. 118
 *   on_nack_frag                 — §8.4.4,   p. 122
 *   nackResponseDelay            — §8.4.1.1, p. 104
 *   heartbeatPeriod              — §8.4.1.1, p. 104
 *   piggybacked HEARTBEAT        — §8.4.15.4
 *   TRANSIENT_LOCAL replay       — §8.7.2.2, p. 196
 *
 * Heartbeats under load
 * ---------------------
 * A reliable reader requests a lost change only in answer to a HEARTBEAT,
 * and it tracks a bounded window of sequence numbers past its oldest hole.
 * A writer that heartbeats only once per heartbeatPeriod therefore parks a
 * reader behind one lost datagram for up to that period, discarding every
 * change beyond its window, and by the time it asks, KEEP_LAST has evicted
 * the change. So a reliable writer also piggybacks a HEARTBEAT on the DATA
 * of every depth/2-th write (§8.4.15.4), the write paths calling
 * send_unsent_changes(heartbeat_due()): a lost change is requested while
 * the history still holds it. The HEARTBEAT travels in the same datagram as
 * the DATA, in kRtpsHeartbeatTailroom reserved after the payload, so it
 * costs 32 octets rather than a datagram; only when the DATA already fills
 * the datagram limit does it go in a datagram of its own.
 *
 * Releasing finished changes
 * --------------------------
 * KEEP_LAST(depth) bounds a history; it does not keep it full. A VOLATILE
 * writer drops a change as soon as every matched reader has acknowledged it
 * (RELIABLE) or been sent it (BEST_EFFORT), since a reader matched later
 * starts after it (§8.7.2.2.1). A TRANSIENT_LOCAL writer keeps depth changes
 * for readers that match later, except a service reply writer
 * (release_replies_on_delivery()): it is TRANSIENT_LOCAL so a reply reaches
 * a client whose reply reader matches late, and it drops each reply once
 * that client has it. Payload blocks then stay in the pool only while some
 * reader may still ask for them, which matters on a board whose pool holds a
 * few large blocks: a reply writer holding its last depth replies exhausts
 * it. A NACK for a dropped change is answered with GAP, as for an evicted one.
 */

#pragma once

#include "types.hpp"
#include "history_cache.hpp"
#include "reader_proxy.hpp"
#include "message_builder.hpp"
#include "message_parser.hpp"
#include "transport.hpp"

namespace rtps {

// ============================================================
// Configuration defaults
// ============================================================

#ifndef RTPS_MAX_MATCHED_READERS
#  define RTPS_MAX_MATCHED_READERS 8
#endif

// ============================================================
// StatefulWriter
// ============================================================

/**
 * @brief Reliable (or best-effort) RTPS writer with per-reader state (§8.4.9).
 *
 * The writer assigns sequence numbers, keeps changes in its HistoryCache,
 * sends each change to every matched reader, and — when RELIABLE — repairs
 * what readers NACK (after nackResponseDelay), GAPs what they may not have,
 * and HEARTBEATs readers that still owe it an acknowledgment.
 *
 * Construct one through StatefulWriterStorage<Depth, MaxReaders>, which owns
 * the storage this class operates on. Not copyable or movable.
 *
 * Thread safety: not thread-safe; the caller holds the participant lock
 * (ParticipantDriver::mutex()) for every call.
 */
class StatefulWriter {
public:
    StatefulWriter(const StatefulWriter&)            = delete;
    StatefulWriter& operator=(const StatefulWriter&) = delete;

    /// @brief Destroys the writer and its storage, releasing every payload
    /// reference its history holds. Virtual, so a writer is destroyed
    /// correctly through StatefulWriter*.
    virtual ~StatefulWriter() noexcept = default;

    /**
     * @brief Stages a new change and allocates a pool buffer for its payload.
     *
     * The caller writes the serialized payload into `out_payload`, sets the
     * staged change's payload size, and passes it to add_change(). Staging
     * again releases a previously staged, unadded payload.
     *
     * @param[in]  kind         ChangeKind of the change (ALIVE, NOT_ALIVE_*).
     * @param[out] out_payload  The allocated buffer (an extra reference to
     *   the staged change's payload) on success.
     * @param[in]  payload_size Octets the caller will write; the pool may
     *   return a larger block.
     * @return The staged change, or null when the pool has no block that size.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.4.1.4 (new_change)
     */
    CacheChange* new_change(ChangeKind kind, BufRef& out_payload,
                            size_t payload_size = kPoolMaxBlockSize) noexcept;

    /**
     * @brief Assigns the next sequence number and adds the change to the history.
     *
     * When the history is full, KEEP_LAST evicts its oldest change; a reader
     * that later NACKs it is sent a GAP. The added change is UNSENT on every
     * matched proxy, since each proxy's highest sent change is below it;
     * add_change() itself sends nothing. A RELIABLE writer also counts the
     * change toward its next piggybacked HEARTBEAT (see heartbeat_due()).
     *
     * @param[in] change Change from new_change() (or built by the caller);
     *   the history takes ownership of its payload.
     * @return `true` on success.
     *
     * Verification:
     * - Correct means: sequence numbers increase by one per change, and the
     *   new change is the next UNSENT change of every matched proxy that has
     *   been sent everything before it.
     * - Covered by: TC-SWR-8.4.1.4-001, TC-SWR-8.2.2-001, TC-SWR-8.2.2-002.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.4.1.4 (add_change), §8.3.3 (KEEP_LAST)
     */
    bool add_change(CacheChange&& change) noexcept;

    /**
     * @brief Removes a change from the history.
     *
     * No proxy sends it afterwards, and a reader that NACKs it is sent a GAP.
     *
     * @param[in] sn Sequence number to remove; absent numbers are ignored.
     *
     * Verification:
     * - Covered by: TC-SWR-8.2.2-001.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.4.1.4 (remove_change)
     */
    void remove_change(const SequenceNumber& sn) noexcept;

    /**
     * @brief Writes a change by copy and sends it to every matched reader now.
     *
     * Copies `payload_data` (the CDR body; the CDR_LE encapsulation header is
     * added on the wire) into a pool buffer, adds the change, and sends every
     * UNSENT change. Returns without waiting for acknowledgments: repairs run
     * from on_acknack() and tick().
     *
     * @param[in]  kind         ChangeKind of the change.
     * @param[in]  payload_data CDR body; not retained.
     * @param[in]  payload_size Its length in octets, at least 1.
     * @param[in]  inline_qos   Inline QoS sent with the change (§9.6.4).
     * @param[out] out_sn       Receives the assigned sequence number when non-null.
     * @return `false` when the payload is empty, the pool is exhausted, or
     *   the history rejects the change; `true` otherwise, whether or not the
     *   transport accepted every datagram.
     *
     * Verification:
     * - Covered by: TC-SWR-8.4.1.4-002, TC-SWR-8.4.9-005.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.4.1.4, §8.4.9.2
     */
    bool write_async(ChangeKind       kind,
                     const Octet*     payload_data,
                     uint32_t         payload_size,
                     const InlineQos& inline_qos = InlineQos{},
                     SequenceNumber*  out_sn     = nullptr) noexcept;

    /**
     * @brief Writes a change whose payload the caller already serialized
     *   into a pool buffer, without copying it.
     *
     * Samples above the transport's datagram limit go out as DATA_FRAG.
     *
     * @param[in]  kind       ChangeKind of the change.
     * @param[in]  payload    Pool buffer holding the CDR body (size set);
     *   ownership passes to the writer.
     * @param[in]  inline_qos Inline QoS sent with the change.
     * @param[out] out_sn     Receives the assigned sequence number when non-null.
     * @return `false` when `payload` is empty or the history rejects it.
     *
     * Verification:
     * - Covered by: TC-SWR-8.4.14-001.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.4.1.4, §8.4.14.1
     */
    bool write_async_owned(ChangeKind       kind,
                           BufRef&&         payload,
                           const InlineQos& inline_qos = InlineQos{},
                           SequenceNumber*  out_sn     = nullptr) noexcept;

    /**
     * @brief Writes a change whose payload is a complete serializedPayload.
     *
     * For payloads that carry their own encapsulation header (SEDP's PL_CDR
     * parameter lists). The copy is made with headroom for the RTPS and DATA
     * headers and the largest inline QoS, so the DATA is framed in place.
     *
     * @param[in]  kind               ChangeKind of the change.
     * @param[in]  serialized_payload Encapsulation header plus data; not retained.
     * @param[in]  payload_size       Its length in octets, at least 1.
     * @param[out] out_sn             Receives the assigned sequence number when non-null.
     * @param[in]  inline_qos         Inline QoS sent with the change.
     * @return `false` when the payload is empty, the pool is exhausted, or
     *   the history rejects the change.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §9.4.5.3 (DATA), §10 (serializedPayload)
     */
    bool write_preframed_async(ChangeKind       kind,
                               const Octet*     serialized_payload,
                               uint32_t         payload_size,
                               SequenceNumber*  out_sn     = nullptr,
                               const InlineQos& inline_qos = InlineQos{}) noexcept;

    /**
     * @brief Builds the serializedPayload of a change the history holds without one.
     *
     * Called each time such a change is sent (first send, repair, or replay
     * to a reader that matches later), with the stored change: its kind,
     * sequence number, and inline QoS identify what to build.
     *
     * @param[in] user   The pointer given to set_payload_builder().
     * @param[in] change The stored change; its payload is empty.
     * @return A complete serializedPayload (encapsulation header included)
     *   in a pool block the writer releases after the send. Allocating it
     *   with buf_alloc_with_headroom(size + kRtpsHeartbeatTailroom,
     *   kRtpsDataHeadroom + kMaxInlineQosSize) lets the DATA be framed in
     *   place. An invalid BufRef means the payload cannot be built now; the
     *   send is then a lost datagram.
     */
    using PayloadBuilder = BufRef (*)(void* user, const CacheChange& change) noexcept;

    /**
     * @brief Installs the builder that supplies payloads for write_on_demand_async() changes.
     *
     * A writer whose changes are a pure function of state its owner already
     * keeps (SEDP announcements, rebuilt from the participant's endpoint
     * table) stores no payload block per change; the builder recreates the
     * bytes for each send instead, trading a serialization per send for the
     * pool block the history would otherwise hold for the change's lifetime.
     *
     * @param[in] builder Builder, or null to remove it.
     * @param[in] user    Passed back to `builder`; must outlive the writer.
     *
     * Verification:
     * - Correct means: a change stored without a payload goes on the wire
     *   exactly as if its built payload had been stored.
     * - Covered by: TC-SEDP-8.5.4-015, TC-SEDP-8.5.4-016.
     */
    void set_payload_builder(PayloadBuilder builder, void* user) noexcept {
        payloadBuilder_ = builder;
        payloadBuilderUser_ = user;
    }

    /**
     * @brief Writes a change whose payload the PayloadBuilder builds each time it is sent.
     *
     * Adds a change with no payload, then sends every UNSENT change, as
     * write_preframed_async() does. The builder must be installed first, and
     * the change's `inline_qos` (for SEDP, its key hash) must identify what
     * it builds. The history slot costs its CacheChange alone; no pool block
     * stays allocated for the change.
     *
     * @param[in]  kind       ChangeKind of the change.
     * @param[in]  inline_qos Inline QoS sent with the change; the builder
     *   reads it to know which payload to build.
     * @param[out] out_sn     Receives the assigned sequence number when non-null.
     * @return `false` when no builder is installed or the history rejects
     *   the change; `true` otherwise, whether or not every send succeeded.
     *
     * Verification:
     * - Correct means: the change takes a sequence number like any other,
     *   and every reader receives the payload the builder returns for it.
     * - Covered by: TC-SEDP-8.5.4-015.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.4.1.4 (add_change), §9.4.5.3 (DATA)
     */
    bool write_on_demand_async(ChangeKind kind, const InlineQos& inline_qos,
                               SequenceNumber* out_sn = nullptr) noexcept;

    /**
     * @brief Matches a remote reader.
     *
     * A TRANSIENT_LOCAL writer then sends the new reader every cached change
     * at once, in sequence order (§8.7.2.2). A VOLATILE writer makes only
     * changes written after this relevant to the new reader.
     *
     * @param[in] reader_guid        The remote reader's GUID.
     * @param[in] unicast_locators   Its unicast locators.
     * @param[in] locator_count      Number of locators.
     * @param[in] expects_inline_qos Its expectsInlineQos.
     * @return `false` when the reader is already matched or every proxy slot
     *   is in use.
     *
     * Verification:
     * - Covered by: TC-SWR-8.4.1.4-002, TC-SWR-8.4.1.4-003.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.4.1.4 (matched_reader_add), §8.7.2.2
     */
    bool matched_reader_add(const Guid&    reader_guid,
                            const Locator* unicast_locators,
                            uint8_t        locator_count,
                            bool           expects_inline_qos = false) noexcept;

    /// @brief Unmatches a remote reader; unknown GUIDs are ignored (§8.4.1.4).
    void matched_reader_remove(const Guid& reader_guid) noexcept;

    /// @brief The proxy of a matched reader, or null (§8.4.1.4 matched_reader_lookup).
    ReaderProxy* matched_reader_lookup(const Guid& reader_guid) noexcept {
        return find_proxy(reader_guid);
    }

    /**
     * @brief Sends a HEARTBEAT advertising the history's sequence-number range.
     *
     * firstSN and lastSN are the history's minimum and maximum; an empty
     * history advertises firstSN = lastSN + 1 ("nothing available").
     *
     * @param[in] final_flag  FinalFlag: the reader need not answer when up to date.
     * @param[in] directed_to When known, send only to this reader, with its
     *   readerId; otherwise send to every matched reader with readerId UNKNOWN.
     *
     * Verification:
     * - Covered by: TC-SWR-8.2.2-002, TC-SWR-8.4.3-002.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.4.2.2, §8.3.7.5 (HEARTBEAT)
     */
    void send_heartbeat(bool final_flag = false, const Guid directed_to = GUID_UNKNOWN) noexcept;

    /**
     * @brief Processes an ACKNACK from a matched reader.
     *
     * Ignores unknown readers and non-increasing counts. Marks everything
     * below readerSNState.bitmapBase acknowledged, marks the NACKed numbers
     * that are relevant to the reader and still in the history REQUESTED
     * (repaired by tick() after nackResponseDelay), GAPs the other NACKed
     * numbers the writer has assigned, and answers a preemptive ACKNACK (no
     * NACK bits, base below the next number, from a reader that has
     * acknowledged nothing yet) with a directed HEARTBEAT. A plain ACK from
     * a reader that has acknowledged before gets no answer, even while the
     * writer has written past it: the piggybacked HEARTBEATs keep such a
     * reader current, and answering would pair every HEARTBEAT with another
     * for as long as the writer streams.
     *
     * A reader that acknowledges less than it did before has started over:
     * its participant lost this participant's SPDP lease, forgot this
     * writer, and matched it again (§8.5.3.2), while this side kept the
     * proxy. The proxy restarts as a new match (rematch_reader()) before the
     * ACKNACK is applied, so the reader receives what a newly matched reader
     * would, and the rematch listener (set_rematch_listener()) runs once the
     * ACKNACK is applied. An ACKNACK that acknowledges nothing (base 0 or 1)
     * restarts a reader that had acknowledged changes even when its count is
     * not above the last one, since a re-matched reader may restart its
     * count; a delayed ACKNACK that acknowledges something stays a duplicate.
     *
     * @param[in] ctx Parse context; supplies the sender's GUID prefix.
     * @param[in] msg The decoded ACKNACK.
     *
     * Verification:
     * - Correct means: no ACKNACK from a reader that has acknowledged before
     *   elicits a HEARTBEAT unless it NACKs, so a streaming writer sends about
     *   one datagram per sample to such a reader; and a reader that matched
     *   this writer again receives what a newly matched reader would.
     * - Covered by: TC-SWR-8.4.3-001, TC-SWR-8.4.3-002, TC-SWR-8.4.3-003,
     *   TC-SWR-8.4.9-001, TC-SWR-8.2.2-003, TC-SWR-8.4.1.4-004,
     *   TC-SWR-8.4.1.4-005, TC-SWR-8.4.1.4-006.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.4.3, §8.4.9.2.8, §8.4.9.2.12, §8.4.15.4,
     *   §8.4.1.4, §8.5.3.2
     * - Fast DDS 2.6 (Humble) src/cpp/rtps/writer/ReaderProxy.cpp,
     *   `acked_changes_set`: non-volatile writers rewind a reader that
     *   acknowledges 0 after losing its lease.
     */
    void on_acknack(const ParseContext& ctx, const ParsedAckNack& msg) noexcept;

    /**
     * @brief Callback for a matched reader that started over (set_rematch_listener()).
     *
     * @param[in] context The pointer given to set_rematch_listener().
     */
    using RematchListener = void (*)(void* context) noexcept;

    /**
     * @brief Installs the callback on_acknack() makes when a matched reader started over.
     *
     * A reader starts over when its participant lost this participant's SPDP
     * lease and matched this writer again (on_acknack()). The writer then
     * replays its history to it, but a stock reader that saw those changes
     * before the lease loss treats them as duplicates: Fast DDS keeps the
     * last sequence number it delivered from a writer it dropped by lease
     * expiry and discards anything at or below it. An owner whose newest
     * change that reader must take again writes a new change from the
     * callback. The callback runs once the ACKNACK is fully applied, under
     * the caller's participant lock, and may write to this writer.
     *
     * @param[in] listener Callback, or null to remove it.
     * @param[in] context  Passed back to `listener`; must outlive the writer.
     *
     * Verification:
     * - Correct means: the callback runs once per ACKNACK that restarts a
     *   reader and never for any other ACKNACK, including a newly matched
     *   reader's first one.
     * - Covered by: TC-SWR-8.4.1.4-004, TC-SWR-8.4.1.4-007.
     *
     * References:
     * - Fast DDS 2.6 (Humble) src/cpp/rtps/reader/RTPSReader.cpp,
     *   `may_remove_history_record`; src/cpp/rtps/builtin/discovery/endpoint/EDP.cpp,
     *   `unpairWriterProxy` (removed_by_lease)
     */
    void set_rematch_listener(RematchListener listener, void* context) noexcept {
        rematchListener_.fn = listener;
        rematchListener_.context = context;
    }

    /**
     * @brief Processes a NACK_FRAG: the named fragments are resent after nackResponseDelay.
     *
     * A reader that has part of a fragmented change asks for the rest with
     * a NACK_FRAG. The change is requested as for a NACK, so the repair goes
     * out from tick() once nackResponseDelay has passed, and a change no
     * longer held is answered with GAP; the repair carries only the named
     * fragments (record_fragment_repair()). Resending every fragment would
     * put the fragments the reader lacks back behind the ones it holds, and
     * on a link whose receive buffer takes fewer than all of them the same
     * ones would be lost on every retry. A NACK_FRAG whose count is not above
     * the last one processed for the reader is a duplicate and is ignored.
     *
     * @param[in] ctx Parse context; supplies the sender's GUID prefix.
     * @param[in] msg The decoded NACK_FRAG.
     *
     * Verification:
     * - Correct means: after nackResponseDelay the reader receives exactly the
     *   fragments it named (the whole change when it is not fragmented, or
     *   when the request cannot be recorded), a HEARTBEAT follows, and a
     *   repeated count sends nothing.
     * - Covered by: TC-SWR-8.4.4-001, TC-SWR-8.4.4-002, TC-SWR-8.4.4-003.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.4.4, §8.3.7.7 (NACK_FRAG)
     */
    void on_nack_frag(const ParseContext& ctx, const ParsedNackFrag& msg) noexcept;

    /**
     * @brief Runs the writer's timed behavior.
     *
     * First, for each reader whose nackResponseDelay has elapsed, resends
     * its REQUESTED changes (GAPping any the history evicted) and follows
     * them with a directed final HEARTBEAT. Then, once per heartbeatPeriod,
     * sends a HEARTBEAT to each reader that still has unacknowledged changes
     * and restarts the count toward the next piggybacked HEARTBEAT.
     *
     * @param[in] now_ms Current rosette::platform::monotonic_ms().
     *
     * Verification:
     * - Covered by: TC-SWR-8.4.2.2-001, TC-SWR-8.4.9-004, TC-SWR-8.4.9-002,
     *   TC-SWR-8.2.2-003, TC-SWR-8.4.4-001.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.4.1.1 (nackResponseDelay, heartbeatPeriod), §8.4.9.2
     */
    void tick(uint32_t now_ms) noexcept;

    const Guid&     guid()        const noexcept { return guid_; }
    ReliabilityKind reliability() const noexcept { return reliability_; }
    DurabilityKind  durability()  const noexcept { return durability_; }
    const HistoryCache& cache()   const noexcept { return writer_cache_; }

    /// nack_response_delay() — §8.4.1.1, p. 104.
    uint32_t nack_response_delay()            const noexcept { return nack_delay_ms_; }
    void     nack_response_delay(uint32_t ms)       noexcept { nack_delay_ms_ = ms; }

    /// heartbeat_period() — §8.4.1.1, p. 104.
    uint32_t heartbeat_period()               const noexcept { return hb_period_ms_; }
    void     heartbeat_period(uint32_t ms)          noexcept { hb_period_ms_ = ms; }

    /**
     * @brief Makes this a service reply writer that drops each reply once its requester has it.
     *
     * A ROS 2 service answers every request on one reply writer, and each
     * reply names its request in PID_RELATED_SAMPLE_IDENTITY. The requesting
     * client's reply reader lives in the same participant as the request
     * writer named there, so it shares that GUID prefix. With this on, a
     * reply leaves the history as soon as every matched reader with that
     * prefix is done with it (ReaderProxy::is_done_with), whatever the
     * writer's durability and whatever other clients' readers have seen:
     * those readers discard replies to other clients' requests. A reply
     * whose requester has no matched reader yet stays, so a TRANSIENT_LOCAL
     * reply writer still delivers it when that reader matches late, until
     * KEEP_LAST evicts it. A change without a related sample identity is
     * left to the writer's durability rule.
     *
     * @param[in] on True for a service reply writer.
     *
     * Verification:
     * - Correct means: a reply is released once, and only once, the
     *   requester's matched readers are done with it; a reply whose
     *   requester has not matched stays and reaches that reader when it
     *   matches.
     * - Covered by: TC-SWR-8.4.7-007 (with its per-reply HEARTBEAT), TC-SWR-8.4.7-008.
     *
     * References:
     * - OMG DDS-RPC 1.0 §7.8.2 (related sample identity)
     * - rmw_rosette/src/service_endpoints.cpp (reply writer configuration)
     */
    void release_replies_on_delivery(bool on) noexcept { releaseRepliesOnDelivery_ = on; }

    /// @brief True for a service reply writer (see release_replies_on_delivery(bool)).
    bool release_replies_on_delivery() const noexcept { return releaseRepliesOnDelivery_; }

    /**
     * @brief True when every matched reader has acknowledged the newest change.
     *
     * Acknowledgments are cumulative, so this means every change in the
     * history is acknowledged. An empty history, and any BEST_EFFORT writer,
     * report true.
     *
     * Verification:
     * - Covered by: TC-SWR-8.4.7-001.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.4.7.1
     */
    bool acknowledged_by_all() const noexcept;

    /**
     * @brief Milliseconds tick() can be deferred at `now_ms`.
     *
     * The sooner of the next periodic HEARTBEAT (only while some matched
     * reader has unacknowledged changes) and the end of any pending
     * nackResponseDelay; UINT32_MAX when neither applies, so an idle writer
     * never wakes the CPU.
     *
     * @param[in] now_ms Current rosette::platform::monotonic_ms().
     * @return Delay in milliseconds; 0 when overdue.
     *
     * Verification:
     * - Covered by: TC-SWR-8.4.7-002, TC-SWR-8.4.9-004.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.4.7.1, §8.4.9.2 (T5-T7)
     */
    uint32_t ms_until_next_timer(uint32_t now_ms) const noexcept;

    /// @brief Number of matched readers (§8.4.7.1 matched_readers).
    uint8_t matched_reader_count() const noexcept;

protected:
    /**
     * @brief Binds the writer to storage owned by the derived object.
     *
     * The storage must be constructed before this constructor runs
     * (StatefulWriterStorage holds it in a base class initialized first) and
     * outlive the writer.
     *
     * @param[in] guid          This writer's GUID (§8.2.4).
     * @param[in] reliability   RELIABLE or BEST_EFFORT (§8.3.3).
     * @param[in] durability    VOLATILE or TRANSIENT_LOCAL (§8.3.3).
     * @param[in] transport     Transport the writer sends on; outlives the writer.
     * @param[in] history       The writer's history; its capacity is the KEEP_LAST depth.
     * @param[in] proxies       `max_readers` proxies.
     * @param[in] max_readers   Number of proxies, at least 1.
     * @param[in] nack_delay_ms nackResponseDelay (§8.4.1.1).
     * @param[in] hb_period_ms  heartbeatPeriod (§8.4.1.1).
     */
    StatefulWriter(const Guid& guid, ReliabilityKind reliability, DurabilityKind durability,
                   ITransport& transport, HistoryCache& history, ReaderProxy* proxies,
                   uint8_t max_readers, uint32_t nack_delay_ms, uint32_t hb_period_ms) noexcept;

private:
    /**
     * @brief Sends `proxy` a GAP declaring the sequence numbers in `irrelevant` irrelevant.
     *
     * @param[in] proxy      Reader to address (its best unicast locator).
     * @param[in] irrelevant Sequence numbers the writer will not send it.
     *
     * Verification:
     * - Correct means: the GAP decodes (run plus bitmap) to exactly
     *   `irrelevant`, addressed to the proxy's readerId.
     * - Covered by: TC-SWR-8.4.9-001.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.3.7.4, §8.4.9.2.12
     */
    void send_gap(const ReaderProxy& proxy, const SequenceNumberSet& irrelevant) noexcept;

    /**
     * @brief Sorts NACKed changes into repairs for `proxy` and one GAP.
     *
     * A NACKed change that is relevant to the reader and still in the
     * history becomes REQUESTED, repaired by tick() after nackResponseDelay.
     * Every other NACKed change the writer has assigned is irrelevant to the
     * reader and is named in one GAP sent now: a change written before a
     * VOLATILE reader matched (its HEARTBEAT advertises the whole history,
     * so a reader that matched mid-stream NACKs those), and a change KEEP_LAST
     * evicted or remove_change() dropped. Without that GAP the reader
     * requests the change until HEARTBEAT firstSN passes it and then reports
     * it as a lost sample (Fast DDS raises SAMPLE_LOST, which rclpy prints as
     * "A message was lost"). Sequence numbers not yet written are ignored.
     *
     * @param[in,out] proxy     Reader that sent the ACKNACK or NACK_FRAG.
     * @param[in]     requested The sequence numbers it lacks.
     * @param[in]     now_ms    Current rosette::platform::monotonic_ms().
     *
     * Verification:
     * - Correct means: every requested SN below the next SN to be written is
     *   either REQUESTED (relevant and held) or named in the GAP, never both.
     * - Covered by: TC-SWR-8.4.9-001, TC-SWR-8.4.9-002.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.4.9.2.8 (requested changes), §8.4.9.2.12
     *   (irrelevant changes are sent as GAP), §8.7.2.2.1 (VOLATILE durability)
     */
    void answer_nack(ReaderProxy& proxy, const SequenceNumberSet& requested,
                     uint32_t now_ms) noexcept;

    /**
     * @brief Lowest sequence number a reader matched now is to receive.
     *
     * @return 1 for a TRANSIENT_LOCAL writer, which replays its history to
     *   the reader (§8.7.2.2); the next sequence number for a VOLATILE one,
     *   whose earlier changes are irrelevant to it (§8.7.2.2.1).
     */
    SequenceNumber first_relevant_for_new_reader() const noexcept;

    /**
     * @brief Restarts a matched reader's proxy as a new match.
     *
     * on_acknack() calls this for a reader that has started over after its
     * participant lost this participant's SPDP lease and matched this writer
     * again. The proxy then starts where matched_reader_add() would start
     * it, and any fragment repair pending for it is dropped; on_acknack()
     * then replays a TRANSIENT_LOCAL writer's history to it.
     *
     * @param[in,out] proxy The reader's proxy; it stays matched.
     *
     * Verification:
     * - Correct means: the reader then receives exactly what a newly matched
     *   reader receives: the history for TRANSIENT_LOCAL, GAPs for earlier
     *   changes it NACKs for VOLATILE.
     * - Covered by: TC-SWR-8.4.1.4-004, TC-SWR-8.4.1.4-006.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.4.1.4 (matched_reader_add), §8.7.2.2
     */
    void rematch_reader(ReaderProxy& proxy) noexcept;

    /// @brief The matched proxy for `g`, or null.
    ReaderProxy* find_proxy(const Guid& g) noexcept;

    /// @brief An unmatched proxy slot, or null when all are in use.
    ReaderProxy* find_free_slot() noexcept;

    /**
     * @brief Sends one change to one matched reader, as DATA or DATA_FRAGs,
     *   optionally followed by a HEARTBEAT in the same datagram.
     *
     * The proxy's sent and requested state is the caller's to update: a
     * send the transport refuses, or a reader with no locator, is treated
     * as a datagram lost on the wire, which reliable repair recovers.
     *
     * A change whose payload has headroom is framed in place
     * (send_data_in_place()); any other is copied into a message built for
     * it. With `heartbeat`, a HEARTBEAT (final flag clear, addressed to the
     * reader) follows the DATA, or the last DATA_FRAG, in the same RTPS
     * message whenever that message still fits the datagram limit and, when
     * framed in place, the payload block has kRtpsHeartbeatTailroom to spare;
     * otherwise it is sent in a datagram of its own right after.
     *
     * @param[in] ch        Change from the writer history.
     * @param[in] proxy     Reader to send to, at its best unicast locator.
     * @param[in] heartbeat Also send a HEARTBEAT advertising the history's
     *   current range; heartbeatCount_ must already be advanced for it.
     * @param[in] only_fragments For a fragmented change, the fragments to
     *   send (bit k-1 for fragment k, see send_fragments_to()); null for all.
     *   Ignored for a change sent as one DATA.
     * @return `true` when every DATA or DATA_FRAG datagram was handed to the
     *   transport.
     *
     * Verification:
     * - Correct means: every datagram is within the limit; with `heartbeat`,
     *   exactly one HEARTBEAT reaches the reader, inside the DATA's datagram
     *   when it fits, and parses as a valid submessage after the DATA.
     * - Covered by: TC-SWR-8.4.15-001, TC-SWR-8.4.15-003, TC-SWR-8.4.15-004.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.4.9.2 (reliable StatefulWriter), §8.4.14.1,
     *   §8.4.15.4 (piggybacked HEARTBEAT), §8.3.3 (a message carries any
     *   number of submessages)
     */
    bool send_data_to(const CacheChange& ch, ReaderProxy& proxy, bool heartbeat = false,
                      const uint64_t* only_fragments = nullptr) noexcept;

    /**
     * @brief Sends a change stored without a payload, building the payload for this send.
     *
     * Asks the PayloadBuilder for the bytes, sends a copy of the change
     * carrying them through send_data_to(), and releases the block when the
     * send returns (ITransport::send_unicast() does not keep it). When the
     * builder returns nothing the change is treated as a lost datagram: a
     * requested HEARTBEAT still goes out, so a reliable reader learns the
     * range and NACKs the change again.
     *
     * @param[in] ch             Stored change with an empty payload.
     * @param[in] proxy          Reader to send to.
     * @param[in] heartbeat      As for send_data_to().
     * @param[in] only_fragments As for send_data_to().
     * @return `true` when every datagram was handed to the transport.
     *
     * Verification:
     * - Correct means: the reader receives the same DATA it would for a
     *   stored payload; a failed build leaves the change for repair.
     * - Covered by: TC-SEDP-8.5.4-015, TC-SEDP-8.5.4-016.
     */
    bool send_built_data_to(const CacheChange& ch, ReaderProxy& proxy, bool heartbeat,
                            const uint64_t* only_fragments) noexcept;

    /**
     * @brief Frames a change as DATA inside its own payload block and sends it.
     *
     * The zero-copy path of §9.4.5.3: the RTPS header, DATA submessage
     * header, and inline QoS are prepended into the headroom the payload was
     * allocated with, and with `heartbeat` a HEARTBEAT submessage is appended
     * into its tailroom, so the datagram is sent without copying the
     * payload. The caller has checked that the headroom (and, with
     * `heartbeat`, the tailroom) is there.
     *
     * @param[in] ch        Change whose payload has kRtpsDataHeadroom plus
     *   its inline QoS size of headroom.
     * @param[in] proxy     Reader to address the DATA to.
     * @param[in] dest      The reader's locator.
     * @param[in] heartbeat Append a HEARTBEAT; the payload has
     *   kRtpsHeartbeatTailroom past its size.
     * @return `true` when the transport accepted the datagram.
     *
     * Verification:
     * - Correct means: the datagram parses as header, DATA, and (with
     *   `heartbeat`) HEARTBEAT, and the change's stored payload bytes and
     *   size are unchanged afterwards.
     * - Covered by: TC-SWR-8.4.15-003.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §9.4.5.3 (DATA), §9.4.5.5 (HEARTBEAT), §8.4.15.4
     */
    bool send_data_in_place(const CacheChange& ch, ReaderProxy& proxy, const Locator& dest,
                            bool heartbeat) noexcept;

    /**
     * @brief Sends a change too large for one datagram as DATA_FRAG submessages.
     *
     * Splits the serializedPayload (the stored payload, preceded by the
     * CDR_LE header when the change is not preframed) into fragments whose
     * size is a multiple of 4 octets, so each RTPS message, one DATA_FRAG
     * per datagram, fits the transport's datagram limit. Fragment 1 carries
     * the encapsulation header; inline QoS rides in every fragment, so a
     * reader holding any fragment knows the sample's related identity. A
     * NACK resends every fragment; a NACK_FRAG resends the fragments it names
     * (`only_fragments`).
     *
     * With `heartbeat`, the HEARTBEAT rides after the last fragment when that
     * datagram has room for it under the limit, and otherwise follows in a
     * datagram of its own.
     *
     * @param[in] ch        Change from the writer history.
     * @param[in] proxy     Reader to send to.
     * @param[in] dest      The reader's locator.
     * @param[in] heartbeat Also send a HEARTBEAT, as for send_data_to().
     * @param[in] only_fragments Bit k-1 set for each fragment k to send
     *   (fragments above 64 are never selected); null sends every fragment.
     * @return `true` when every fragment was handed to the transport.
     *
     * Verification:
     * - Correct means: every datagram is within the limit, reassembly
     *   restores the header plus payload, and with `heartbeat` exactly one
     *   HEARTBEAT reaches the reader.
     * - Covered by: TC-SWR-8.4.14-001, TC-SWR-8.4.15-004;
     *   tests/ros2/run_rmw_frag_loopback.sh.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.4.14.1, §9.4.5.4 (DATA_FRAG), §8.4.15.4
     */
    bool send_fragments_to(const CacheChange& ch, ReaderProxy& proxy, const Locator& dest,
                           bool heartbeat, const uint64_t* only_fragments = nullptr) noexcept;

    /**
     * @brief Records which fragments of change `sn` a NACK_FRAG from `proxy` asked for.
     *
     * The writer holds one pending fragment repair: a board rarely has more
     * than one reader repairing a fragmented change at once, and the record
     * costs a pointer, a sequence number and a 64-bit mask rather than a
     * FragmentNumberSet per reader proxy. A second NACK_FRAG from the same
     * reader for the same change adds its fragments; one from another
     * reader, or for another change, while the record is taken, leaves its
     * change to be resent whole, as is one naming a fragment above 64. tick()
     * consumes the record when it repairs that reader.
     *
     * @param[in] proxy     Reader that sent the NACK_FRAG.
     * @param[in] sn        The fragmented change it names.
     * @param[in] fragments The fragment numbers it asks for.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.3.7.5 (NACK_FRAG), §8.4.14.1
     */
    void record_fragment_repair(const ReaderProxy& proxy, const SequenceNumber& sn,
                                const FragmentNumberSet& fragments) noexcept;

    /// A NACK_FRAG waiting for nackResponseDelay (record_fragment_repair()).
    struct FragmentRepair {
        const ReaderProxy* proxy     = nullptr;  ///< Requesting reader; null when free.
        SequenceNumber     sn        = {};
        uint64_t           fragments = 0u;       ///< Bit k-1 for fragment k.
    };

    /**
     * @brief Sends every UNSENT change to every matched reader.
     *
     * Each change is marked sent before it goes to the transport, as the
     * repair path marks a change underway before resending it. A send that
     * fails (pool exhausted, transport refused, reader without a locator)
     * is thereby a lost datagram: a reliable reader requests it again, a
     * best-effort reader loses it, and the loop always terminates.
     *
     * With `with_heartbeat`, the newest change goes to each reader with a
     * HEARTBEAT in the same datagram (send_data_to()), all sharing one count,
     * and the write count toward the next one restarts. Older changes sent in
     * the same pass carry none: the HEARTBEAT after the newest one advertises
     * them too. The write paths pass heartbeat_due().
     *
     * @param[in] with_heartbeat Piggyback a HEARTBEAT on the newest change's
     *   datagram to each reader.
     *
     * Verification:
     * - Correct means: every UNSENT change is attempted once per reader and
     *   leaves UNSENT whether or not the send succeeds; a HEARTBEAT reaches
     *   every matched reader with the piggyback_interval()-th write since the
     *   last HEARTBEAT, and with no write before it.
     * - Covered by: TC-SWR-8.4.9-005, TC-SWR-8.4.15-001, TC-SWR-8.4.15-002.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.4.9.2 (reliable StatefulWriter, UNSENT ->
     *   UNACKNOWLEDGED), §8.4.8.1 (best-effort), §8.4.15.4 (piggybacked
     *   HEARTBEAT)
     */
    void send_unsent_changes(bool with_heartbeat = false) noexcept;

    /// @brief The range a HEARTBEAT advertises: the history's oldest and
    ///   newest change, or lastSN + 1 and lastSN when it is empty (§8.4.2.2).
    void heartbeat_range(SequenceNumber& first, SequenceNumber& last) const noexcept;

    /// @brief Sends a HEARTBEAT for the current history range to one locator.
    void send_heartbeat_to(const EntityId& reader_id, const Locator& dest, bool final_flag) noexcept;

    /**
     * @brief Writes since the last HEARTBEAT that make the next one due: half
     *   the history depth, at least 1; 1 for a service reply writer.
     *
     * Half the depth leaves the other half of the history holding a lost
     * change while the reader's NACK and the repair travel, so a reader that
     * answers the HEARTBEAT promptly gets the change itself rather than a GAP.
     * A service reply is the last write of its exchange, and the requester
     * acknowledges it only in answer to a HEARTBEAT; carrying one on every
     * reply brings that acknowledgement back within a round trip, so the
     * reply's block returns to the pool (release_replies_on_delivery())
     * before the next request's reply needs one, instead of after a
     * heartbeatPeriod.
     */
    uint32_t piggyback_interval() const noexcept;

    /// @brief True when a RELIABLE writer has made piggyback_interval() writes
    ///   since its last HEARTBEAT, so the next send should carry one.
    bool heartbeat_due() const noexcept;

    /// @brief Fills `out` (kRtpsHeartbeatTailroom octets) with a non-final
    ///   HEARTBEAT to `reader_id` for the current range and heartbeatCount_.
    void encode_heartbeat(Octet* out, const EntityId& reader_id) const noexcept;

    /// @brief True when every matched reader has acknowledged `sn`.
    bool all_readers_acked(const SequenceNumber& sn) const noexcept;

    /**
     * @brief Drops every change no reader still needs.
     *
     * Runs after each event that can finish a change: an ACKNACK, a send
     * pass, and an unmatch. Two rules release a change, and dropping it
     * returns its payload block to the pool instead of holding it until
     * KEEP_LAST evicts it:
     * - VOLATILE durability: a reader that matches later is sent only later
     *   changes, so a change every matched reader is done with
     *   (ReaderProxy::is_done_with) is never sent again. With no reader
     *   matched, every change goes.
     * - A service reply writer (release_replies_on_delivery()): a reply
     *   goes once its requester's matched readers are done with it
     *   (reply_delivered()), whatever the durability.
     * Otherwise a TRANSIENT_LOCAL writer keeps its history for readers that
     * match later.
     *
     * Verification:
     * - Correct means: after the call the history holds exactly the changes
     *   neither rule releases; a NACK for a dropped change is answered with GAP.
     * - Covered by: TC-SWR-8.4.7-003 through TC-SWR-8.4.7-008.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.4.7.1 (is_acked_by_all), §8.7.2.2.1 (VOLATILE
     *   durability); OMG DDS 1.4 §2.2.3.4 (DURABILITY)
     */
    void release_finished_changes() noexcept;

    /// @brief True when `reply` names a requester whose matched readers are
    ///   all done with it, and at least one of them is matched.
    bool reply_delivered(const CacheChange& reply, bool reliable) const noexcept;

    Guid            guid_;
    ReliabilityKind reliability_;
    DurabilityKind  durability_;
    ITransport&     transport_;

    HistoryCache&   writer_cache_;
    ReaderProxy*    proxies_;
    uint8_t         max_readers_;

    CacheChange    staging_{};     ///< Staging area for new_change().
    int32_t        heartbeatCount_ = 0;
    SequenceNumber nextSN_         = SequenceNumber::from_int64(1);
    uint32_t       nack_delay_ms_;
    uint32_t       hb_period_ms_;
    uint32_t       lastHbSentMs_   = 0;
    uint32_t       writesSinceHb_  = 0;  ///< RELIABLE writes since any HEARTBEAT.
    bool           releaseRepliesOnDelivery_ = false;  ///< Service reply writer.
    FragmentRepair fragRepair_{};                      ///< Pending NACK_FRAG repair.

    /// Called when a matched reader started over (set_rematch_listener()).
    struct {
        RematchListener fn = nullptr;
        void* context = nullptr;
    } rematchListener_;
    PayloadBuilder payloadBuilder_ = nullptr;  ///< Builds payloads of on-demand changes.
    void* payloadBuilderUser_ = nullptr;
    MessageBuilder builder_;
};

namespace detail {
/// @brief The storage of a StatefulWriterStorage, as a base class so it is
/// constructed before the StatefulWriter base that points into it.
template<uint8_t Depth, uint8_t MaxReaders>
struct StatefulWriterSlots {
    HistoryCacheStorage<Depth> history;
    ReaderProxy                proxies[MaxReaders];
};
}  // namespace detail

/**
 * @brief A StatefulWriter with storage for a KEEP_LAST(Depth) history and
 *   MaxReaders matched readers.
 *
 * Holds only the storage; every operation is StatefulWriter's.
 *
 * @tparam Depth      KEEP_LAST depth (§8.3.3), at least 1.
 * @tparam MaxReaders Most readers matched at once, at least 1.
 */
template<uint8_t Depth = 1, uint8_t MaxReaders = RTPS_MAX_MATCHED_READERS>
class StatefulWriterStorage : private detail::StatefulWriterSlots<Depth, MaxReaders>,
                              public StatefulWriter {
    static_assert(Depth >= 1, "Depth must be >= 1");
    static_assert(MaxReaders >= 1, "MaxReaders must be >= 1");

public:
    /**
     * @brief Creates a writer with no matched readers and an empty history.
     *
     * @param[in] guid          This writer's GUID (§8.2.4).
     * @param[in] reliability   RELIABLE or BEST_EFFORT (§8.3.3).
     * @param[in] durability    VOLATILE or TRANSIENT_LOCAL (§8.3.3).
     * @param[in] transport     Transport the writer sends on; outlives the writer.
     * @param[in] nack_delay_ms nackResponseDelay — §8.4.1.1, p. 104.
     * @param[in] hb_period_ms  heartbeatPeriod — §8.4.1.1, p. 104.
     */
    StatefulWriterStorage(const Guid& guid, ReliabilityKind reliability, DurabilityKind durability,
                          ITransport& transport,
                          uint32_t nack_delay_ms = RTPS_NACK_RESPONSE_DELAY_MS,
                          uint32_t hb_period_ms  = RTPS_HEARTBEAT_PERIOD_MS) noexcept
        : StatefulWriter(guid, reliability, durability, transport, this->history, this->proxies,
                         MaxReaders, nack_delay_ms, hb_period_ms) {}
};

} // namespace rtps
