/**
 * @file rtps/stateful_writer.hpp
 * @brief StatefulWriter — the reliable RTPS writer of §8.4.1–§8.4.9.
 *        (OMG formal/2022-04-01)
 *
 * History depth and matched readers
 * ---------------------------------
 * A writer keeps KEEP_LAST(depth) changes (§8.3.3) and one ReaderProxy per
 * matched reader (§8.4.1.1), and each proxy tracks one ChangeForReader per
 * history slot. All of that storage is fixed-size and chosen at build time.
 *
 * One compiled writer, storage by template
 * ----------------------------------------
 * StatefulWriter is a plain class: every operation lives once in
 * src/rtps/stateful_writer.cpp and works on storage it is handed (a
 * HistoryCache, an array of ReaderProxy, and their ChangeForReader rows).
 * StatefulWriterStorage<Depth, MaxReaders> owns exactly that storage and
 * derives from StatefulWriter, so writers of different depths differ only in
 * their data layout, never in their code:
 *
 *   StatefulWriterStorage<>     — KEEP_LAST(1), RTPS_MAX_MATCHED_READERS readers
 *   StatefulWriterStorage<4>    — KEEP_LAST(4)
 *   StatefulWriterStorage<8,4>  — KEEP_LAST(8), 4 readers
 *
 * Everything outside the object that owns a writer uses StatefulWriter&.
 *
 * IStatefulWriter
 * ---------------
 * RTPSParticipant stores its writers in one table and calls the operations
 * of IStatefulWriter on them; rmw_rosette publishes through the same
 * interface.
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
 *   TRANSIENT_LOCAL replay       — §8.7.2.2, p. 196
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
// IStatefulWriter — the operations RTPSParticipant and rmw_rosette call
// on a writer they hold in a table.
// ============================================================

struct IStatefulWriter {
    virtual ~IStatefulWriter() = default;

    /// §8.4.3, p. 118 — process incoming ACKNACK.
    virtual void on_acknack(const ParseContext&  ctx,
                            const ParsedAckNack& msg) noexcept = 0;

    /// §8.4.4, p. 122 — process incoming NACK_FRAG.
    virtual void on_nack_frag(const ParseContext&   ctx,
                              const ParsedNackFrag& msg) noexcept = 0;

    /// Periodic tick: retransmit deferred NACKs + send HEARTBEAT.
    virtual void tick(uint32_t now_ms) noexcept = 0;

    /// §8.2.4 — GUID of this writer endpoint.
    virtual const Guid& guid() const noexcept = 0;

    /// §8.4.1.4, p. 108 — add a matched reader.
    virtual bool matched_reader_add(const Guid&    reader_guid,
                                    const Locator* unicast_locators,
                                    uint8_t        locator_count,
                                    bool           expects_inline_qos) noexcept = 0;

    /// §8.4.1.4, p. 108 — remove a matched reader.
    virtual void matched_reader_remove(const Guid& reader_guid) noexcept = 0;

    /// Number of currently matched readers (§8.4.7.1 matched_readers).
    virtual uint8_t matched_reader_count() const noexcept = 0;

    /// §8.4.1.4, p. 107 — add a change and send it; see StatefulWriter::write_async().
    virtual bool write_async(ChangeKind       kind,
                             const Octet*     payload_data,
                             uint32_t         payload_size,
                             const InlineQos& inline_qos = InlineQos{},
                             SequenceNumber*  out_sn      = nullptr) noexcept = 0;

    /// §8.4.1.4 — like write_async(), but takes ownership of a pool buffer
    /// the caller already filled (payload.size set), avoiding a copy. The
    /// payload is the CDR body; the writer adds the encapsulation header.
    virtual bool write_async_owned(ChangeKind       kind,
                                   BufRef&&         payload,
                                   const InlineQos& inline_qos = InlineQos{},
                                   SequenceNumber*  out_sn     = nullptr) noexcept = 0;

    /// True when every change in the history is acknowledged by every
    /// matched reader (§8.4.7.1 unacked_changes empty for all proxies).
    /// Backs DDS wait_for_acknowledgments / rmw_publisher_wait_for_all_acked.
    virtual bool acknowledged_by_all() const noexcept = 0;

    /// Milliseconds until this writer's next timed action (heartbeat or a
    /// deferred NACK repair) is due at `now_ms`; 0 when overdue.
    virtual uint32_t ms_until_next_timer(uint32_t now_ms) const noexcept = 0;
};

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
class StatefulWriter : public IStatefulWriter {
public:
    StatefulWriter(const StatefulWriter&)            = delete;
    StatefulWriter& operator=(const StatefulWriter&) = delete;

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
     * When the history is full, KEEP_LAST evicts its oldest change, and every
     * matched proxy stops tracking it first. The added change becomes UNSENT
     * on every matched proxy; add_change() itself sends nothing.
     *
     * @param[in] change Change from new_change() (or built by the caller);
     *   the history takes ownership of its payload.
     * @return `true` on success.
     *
     * Verification:
     * - Correct means: sequence numbers increase by one per change, and no
     *   proxy tracks a change the history no longer holds.
     * - Covered by: TC-SWR-8.4.1.4-001, TC-SWR-8.2.2-001, TC-SWR-8.2.2-002.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.4.1.4 (add_change), §8.3.3 (KEEP_LAST)
     */
    bool add_change(CacheChange&& change) noexcept;

    /**
     * @brief Removes a change from the history and from every matched proxy.
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
                     SequenceNumber*  out_sn     = nullptr) noexcept override;

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
                           SequenceNumber*  out_sn     = nullptr) noexcept override;

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
     * @brief Matches a remote reader.
     *
     * A TRANSIENT_LOCAL writer then marks every cached change UNSENT for the
     * new reader and sends them at once, in sequence order (§8.7.2.2). A
     * VOLATILE writer sends the new reader only changes written after this.
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
                            bool           expects_inline_qos = false) noexcept override;

    /// @brief Unmatches a remote reader; unknown GUIDs are ignored (§8.4.1.4).
    void matched_reader_remove(const Guid& reader_guid) noexcept override;

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
     * REQUESTED (repaired by tick() after nackResponseDelay), GAPs NACKed
     * numbers the reader never had a ChangeForReader for, and answers a
     * preemptive ACKNACK (no NACK bits, base below the next number) with a
     * directed HEARTBEAT.
     *
     * @param[in] ctx Parse context; supplies the sender's GUID prefix.
     * @param[in] msg The decoded ACKNACK.
     *
     * Verification:
     * - Covered by: TC-SWR-8.4.3-001, TC-SWR-8.4.3-002, TC-SWR-8.4.9-001,
     *   TC-SWR-8.2.2-003.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.4.3, §8.4.9.2.8, §8.4.9.2.12, §8.4.15.4
     */
    void on_acknack(const ParseContext& ctx, const ParsedAckNack& msg) noexcept override;

    /**
     * @brief Processes a NACK_FRAG: the whole change is resent after nackResponseDelay.
     *
     * @param[in] ctx Parse context; supplies the sender's GUID prefix.
     * @param[in] msg The decoded NACK_FRAG.
     *
     * Verification:
     * - Covered by: TC-SWR-8.4.4-001.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.4.4, §8.3.7.7 (NACK_FRAG)
     */
    void on_nack_frag(const ParseContext& ctx, const ParsedNackFrag& msg) noexcept override;

    /**
     * @brief Runs the writer's timed behavior.
     *
     * First, for each reader whose nackResponseDelay has elapsed, resends
     * its REQUESTED changes (GAPping any the history evicted) and follows
     * them with a directed final HEARTBEAT. Then, once per heartbeatPeriod,
     * sends a HEARTBEAT to each reader that still has unacknowledged changes.
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
    void tick(uint32_t now_ms) noexcept override;

    const Guid&     guid()        const noexcept override { return guid_; }
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
    bool acknowledged_by_all() const noexcept override;

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
    uint32_t ms_until_next_timer(uint32_t now_ms) const noexcept override;

    /// @brief Number of matched readers (§8.4.7.1 matched_readers).
    uint8_t matched_reader_count() const noexcept override;

protected:
    /**
     * @brief Binds the writer to storage owned by the derived object.
     *
     * Gives every proxy its row of `history.capacity()` ChangeForReader slots.
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
     * @param[in] proxy_changes `max_readers` rows of `history.capacity()` slots.
     * @param[in] max_readers   Number of proxies, at least 1.
     * @param[in] nack_delay_ms nackResponseDelay (§8.4.1.1).
     * @param[in] hb_period_ms  heartbeatPeriod (§8.4.1.1).
     */
    StatefulWriter(const Guid& guid, ReliabilityKind reliability, DurabilityKind durability,
                   ITransport& transport, HistoryCache& history, ReaderProxy* proxies,
                   ChangeForReader* proxy_changes, uint8_t max_readers,
                   uint32_t nack_delay_ms, uint32_t hb_period_ms) noexcept;

    ~StatefulWriter() noexcept override = default;

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
     * @brief Answers the NACKed changes that are irrelevant to `proxy` with one GAP.
     *
     * A VOLATILE writer's HEARTBEAT advertises its whole history, so a
     * reader that matched mid-stream NACKs changes written before it
     * matched. Those changes have no ChangeForReader in its proxy and are
     * never sent to it; without a GAP the reader requests them until
     * HEARTBEAT firstSN passes them and then reports them as lost samples
     * (Fast DDS raises SAMPLE_LOST, which rclpy prints as "A message was
     * lost"). Sequence numbers not yet written are ignored.
     *
     * @param[in] proxy     Reader that sent the ACKNACK.
     * @param[in] requested The ACKNACK's readerSNState.
     *
     * Verification:
     * - Correct means: every requested SN below the next SN to be written
     *   that the proxy does not track is named in one GAP; tracked SNs are
     *   left to the nackResponseDelay repair path.
     * - Covered by: TC-SWR-8.4.9-001.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.4.9.2.12 (irrelevant changes are sent as GAP),
     *   §8.7.2.2.1 (VOLATILE durability)
     */
    void send_irrelevant_gap(const ReaderProxy& proxy, const SequenceNumberSet& requested) noexcept;

    /// @brief The matched proxy for `g`, or null.
    ReaderProxy* find_proxy(const Guid& g) noexcept;

    /// @brief An unmatched proxy slot, or null when all are in use.
    ReaderProxy* find_free_slot() noexcept;

    /**
     * @brief Sends one change to one matched reader, as DATA or DATA_FRAGs.
     *
     * The change's ChangeForReader status is the caller's to update: a
     * send the transport refuses, or a reader with no locator, is treated
     * as a datagram lost on the wire, which reliable repair recovers.
     *
     * @param[in] ch    Change from the writer history.
     * @param[in] proxy Reader to send to, at its best unicast locator.
     * @return `true` when every datagram was handed to the transport.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.4.9.2 (reliable StatefulWriter), §8.4.14.1
     */
    bool send_data_to(const CacheChange& ch, ReaderProxy& proxy) noexcept;

    /**
     * @brief Sends a change too large for one datagram as DATA_FRAG submessages.
     *
     * Splits the serializedPayload (the stored payload, preceded by the
     * CDR_LE header when the change is not preframed) into fragments whose
     * size is a multiple of 4 octets, so each RTPS message, one DATA_FRAG
     * per datagram, fits the transport's datagram limit. Fragment 1 carries
     * the encapsulation header; inline QoS rides in every fragment, so a
     * reader holding any fragment knows the sample's related identity. A
     * NACK_FRAG or NACK for the change resends every fragment.
     *
     * @param[in] ch    Change from the writer history.
     * @param[in] proxy Reader to send to.
     * @param[in] dest  The reader's locator.
     * @return `true` when every fragment was handed to the transport.
     *
     * Verification:
     * - Correct means: every datagram is within the limit and reassembly
     *   restores the header plus payload.
     * - Covered by: TC-SWR-8.4.14-001; tests/ros2/run_rmw_frag_loopback.sh.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.4.14.1, §9.4.5.4 (DATA_FRAG)
     */
    bool send_fragments_to(const CacheChange& ch, ReaderProxy& proxy, const Locator& dest) noexcept;

    /**
     * @brief Sends every UNSENT change to every matched reader.
     *
     * Each change is marked sent before it goes to the transport, as the
     * repair path marks a change underway before resending it. A send that
     * fails (pool exhausted, transport refused, reader without a locator)
     * is thereby a lost datagram: a reliable reader requests it again, a
     * best-effort reader loses it, and the loop always terminates.
     *
     * Verification:
     * - Correct means: every UNSENT change is attempted once per reader and
     *   leaves UNSENT whether or not the send succeeds.
     * - Covered by: TC-SWR-8.4.9-005.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.4.9.2 (reliable StatefulWriter, UNSENT ->
     *   UNACKNOWLEDGED), §8.4.8.1 (best-effort)
     */
    void send_unsent_changes() noexcept;

    /// @brief Sends a HEARTBEAT for the current history range to one locator.
    void send_heartbeat_to(const EntityId& reader_id, const Locator& dest, bool final_flag) noexcept;

    /// @brief True when every matched reader has acknowledged `sn`.
    bool all_readers_acked(const SequenceNumber& sn) const noexcept;

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
    MessageBuilder builder_;
};

namespace detail {
/// @brief The storage of a StatefulWriterStorage, as a base class so it is
/// constructed before the StatefulWriter base that points into it.
template<uint8_t Depth, uint8_t MaxReaders>
struct StatefulWriterSlots {
    HistoryCacheStorage<Depth> history;
    ReaderProxy                proxies[MaxReaders];
    ChangeForReader            proxy_changes[MaxReaders][Depth];
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
                         &this->proxy_changes[0][0], MaxReaders, nack_delay_ms, hb_period_ms) {}
};

} // namespace rtps
