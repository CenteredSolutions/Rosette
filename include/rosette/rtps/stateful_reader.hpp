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
 * StatefulReader runs over a caller-provided WriterProxy array, so one
 * compiled reader serves readers of every capacity: user readers match at
 * most RTPS_MAX_MATCHED_WRITERS writers, while the SEDP builtin readers match
 * one announcer per remote participant (RTPS_MAX_PEERS). Declare a reader as
 * BoundedStatefulReader<N>, which owns the array; pass it around as
 * StatefulReader.
 *
 * Fragment reassembly
 * -------------------
 * DATA_FRAG submessages go to a DefaultFragmentAssembler (§8.4.14). A
 * completed sample is accepted exactly as a DATA would be; HEARTBEAT_FRAG is
 * answered with a NACK_FRAG built from the assembler's missing fragments,
 * and un-matching a writer discards its partial samples.
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

#include "types.hpp"
#include "history_cache.hpp"
#include "writer_proxy.hpp"
#include "fragment_assembler.hpp"
#include "message_builder.hpp"
#include "message_parser.hpp"


namespace rtps {

struct ITransport;

// ============================================================
// StatefulReader
// ============================================================

class StatefulReader {
public:

    /// Consumer callback — fired when a complete CacheChange arrives. The
    /// change is valid only for the duration of the call; a consumer that
    /// needs the payload later retains its buffer (BufRef::retain()), as
    /// rmw_rosette's ChangeRing does. The consumer is the reader's history:
    /// the reader itself keeps no delivered changes.
    using OnChangeAvailable = void (*)(const CacheChange& change,
                                      void* user_data) noexcept;

    StatefulReader(const StatefulReader&)            = delete;
    StatefulReader& operator=(const StatefulReader&) = delete;

    // ----------------------------------------------------------
    // §8.4.7.4  matched_writer_add()
    //
    // "Adds the WriterProxy to the set of matched writers."
    //  — §8.4.7.4, p. 141
    // ----------------------------------------------------------
    bool matched_writer_add(const Guid&    writer_guid,
                            const Locator* unicast_locators,
                            uint8_t        locator_count) noexcept;

    // ----------------------------------------------------------
    // §8.4.7.4  matched_writer_remove()
    //
    // "Removes the WriterProxy from the set of matched writers."
    //  — §8.4.7.4, p. 141
    //
    // Also discards any in-progress fragment assembly for this writer
    // by calling assembler_.discard_writer(), returning pool buffers.
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
     * @param[in,out] proxy      Writer to acknowledge; its count is not
     *   advanced here (the caller calls on_acknack_sent()).
     * @param[in]     final_flag F-flag: the writer need not answer with a
     *   HEARTBEAT.
     *
     * Verification:
     * - Correct means: bitmapBase is at most the lowest MISSING change, every
     *   set bit is MISSING, and on a narrow link exactly one bit is set.
     * - Covered by: TC-SRR-8.4.7-002, TC-SRR-8.4.7-003,
     *   tests/ros2/run_rmw_frag_loopback.sh (stock peer).
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.4.7.3, §8.4.12 (reader behavior), §8.3.7.1 (ACKNACK)
     */
    void send_acknack(WriterProxy& proxy,
                      bool         final_flag = false) noexcept;

    // ----------------------------------------------------------
    // tick()
    //
    // Periodic maintenance: sends the ACKNACKs whose heartbeatResponseDelay
    // has elapsed. Call at least as often as ms_until_next_timer() asks.
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

    uint8_t matched_writer_count() const noexcept;

    // ----------------------------------------------------------
    // ms_until_next_timer()
    //
    // How long tick() can be deferred: the time left on the earliest
    // pending heartbeatResponseDelay (§8.4.7.1), or UINT32_MAX when no
    // ACKNACK is pending. The participant driver sleeps no longer than this.
    //
    // Verification: TC-SRR-8.4.7-001.
    // ----------------------------------------------------------
    uint32_t ms_until_next_timer(uint32_t now_ms) const noexcept;

    /// @brief Most writers this reader can match at once.
    uint8_t max_matched_writers() const noexcept { return proxy_capacity_; }

protected:

    /**
     * @brief Creates a reader over caller-provided WriterProxy storage.
     *
     * Only BoundedStatefulReader constructs a StatefulReader, passing the
     * array it owns; code that uses a reader holds a StatefulReader& or *.
     *
     * @param[in] guid            This reader's GUID (§8.2.4).
     * @param[in] reliability     RELIABLE or BEST_EFFORT (§8.3.3).
     * @param[in] transport       Transport for ACKNACK and NACK_FRAG; must
     *   outlive the reader.
     * @param[in] proxies         Storage for one WriterProxy per matched
     *   writer. It must outlive the reader and is not accessed during
     *   construction, so it may be a member of a derived class.
     * @param[in] proxy_capacity  Number of entries in `proxies` (1-255).
     * @param[in] on_change_avail Callback fired on each new complete change;
     *   null discards delivered changes.
     * @param[in] user_data       Forwarded to on_change_avail.
     * @param[in] hb_response_ms  heartbeatResponseDelay in milliseconds
     *   (§8.4.7.1); 0 answers a HEARTBEAT immediately.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.4.7.1 (StatefulReader attributes)
     */
    StatefulReader(const Guid&       guid,
                   ReliabilityKind   reliability,
                   ITransport&       transport,
                   WriterProxy*      proxies,
                   uint8_t           proxy_capacity,
                   OnChangeAvailable on_change_avail,
                   void*             user_data,
                   uint32_t          hb_response_ms) noexcept;

    ~StatefulReader() noexcept = default;

private:

    WriterProxy* find_proxy(const Guid& guid) noexcept;
    WriterProxy* find_proxy_by_sender(const GuidPrefix& prefix,
                                      const EntityId&   writer_eid) noexcept;
    WriterProxy* find_free_slot() noexcept;

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

    WriterProxy*       proxies_;          ///< Owned by the BoundedStatefulReader.
    uint8_t            proxy_capacity_;

    /// Fragment reassembler — §8.4.14, p. 156.
    /// One slot per matched writer; 256-fragment bitmap; pool-backed.
    DefaultFragmentAssembler assembler_;

    uint32_t       hb_response_ms_;
    MessageBuilder builder_;
};

/**
 * @brief A StatefulReader that owns storage for `kMaxWriters` WriterProxies.
 *
 * The protocol logic lives once in StatefulReader (stateful_reader.cpp);
 * this template only adds the array, so readers of different capacities
 * share all code and differ only in size.
 *
 * @tparam kMaxWriters Most writers matched at once (1-255).
 */
template <uint8_t kMaxWriters>
class BoundedStatefulReader final : public StatefulReader {
    static_assert(kMaxWriters >= 1u, "a reader must be able to match a writer");

public:
    /**
     * @brief Creates a reader with no matched writers.
     *
     * The parameters are those of StatefulReader's constructor, less the
     * proxy storage this class provides.
     */
    BoundedStatefulReader(const Guid&       guid,
                          ReliabilityKind   reliability,
                          ITransport&       transport,
                          OnChangeAvailable on_change_avail = nullptr,
                          void*             user_data       = nullptr,
                          uint32_t          hb_response_ms  =
                              RTPS_HEARTBEAT_RESPONSE_DELAY_MS) noexcept
        : StatefulReader(guid, reliability, transport, proxy_storage_, kMaxWriters,
                         on_change_avail, user_data, hb_response_ms)
    {}

private:
    // Constructed after the base, which only records its address.
    WriterProxy proxy_storage_[kMaxWriters];
};

/// Reader for user topics: up to RTPS_MAX_MATCHED_WRITERS matched writers.
using DefaultStatefulReader = BoundedStatefulReader<RTPS_MAX_MATCHED_WRITERS>;

} // namespace rtps
