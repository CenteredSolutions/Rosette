/**
 * @file rtps/stateful_reader.hpp
 * @brief StatefulReader — reliable RTPS reader with per-writer proxy tracking.
 *
 * Implements the StatefulReader behaviour model defined in §8.4.7–§8.4.9
 * of the DDSI-RTPS 2.5 specification (OMG formal/2022-04-01).
 *
 * Fragment reassembly
 * -------------------
 * DATA_FRAG handling is delegated entirely to a DefaultFragmentAssembler
 * member (fragment_assembler.hpp, §8.4.14).  The old inline FragSlot with
 * its 32-fragment bitmask limit is removed; FragmentAssembler supports up
 * to 256 fragments per sample using the full FragmentNumberSet bitmap
 * encoding from §9.4.2.8.
 *
 * on_data_frag() passes each incoming fragment to assembler_.on_data_frag().
 * When the assembler signals completion it returns a fully-populated
 * CacheChange which is then accepted identically to a directly-received
 * DATA — proxy bookkeeping, cache insertion, and application callback all
 * follow the same path.
 *
 * on_heartbeat_frag() calls assembler_.missing_fragments() to build the
 * NACK_FRAG bitmap, then sends it via the transport.
 *
 * matched_writer_remove() now also calls assembler_.discard_writer() so
 * that partial assembly buffers are returned to the pool when a writer
 * is un-matched.
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
// Configuration
// ============================================================

#ifndef RTPS_MAX_MATCHED_WRITERS
#  define RTPS_MAX_MATCHED_WRITERS 8
#endif

// ============================================================
// StatefulReader
// ============================================================

class StatefulReader {
public:

    /// Application callback — fired when a complete CacheChange arrives.
    /// Valid only for the duration of the call; copy data before returning.
    using OnChangeAvailable = void (*)(const CacheChange& change,
                                      void* user_data) noexcept;

    // ----------------------------------------------------------
    // Constructor
    //
    // @param guid            This reader's GUID (§8.2.4).
    // @param reliability     RELIABLE or BEST_EFFORT (§8.3.3).
    // @param transport       Transport for sending ACKNACK / NACK_FRAG.
    // @param on_change_avail Callback fired on each new complete change.
    // @param user_data       Forwarded to on_change_avail.
    // @param hb_response_ms  heartbeatResponseDelay in ms — §8.4.7.1, p. 135.
    // ----------------------------------------------------------
    StatefulReader(const Guid&       guid,
                   ReliabilityKind   reliability,
                   ITransport&       transport,
                   OnChangeAvailable on_change_avail = nullptr,
                   void*             user_data       = nullptr,
                   uint32_t          hb_response_ms  =
                       RTPS_HEARTBEAT_RESPONSE_DELAY_MS) noexcept;

    StatefulReader(const StatefulReader&)            = delete;
    StatefulReader& operator=(const StatefulReader&) = delete;
    ~StatefulReader() noexcept;

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
    DefaultWriterProxy* matched_writer_lookup(const Guid& writer_guid) noexcept;

    // ----------------------------------------------------------
    // §8.4.8  on_data()
    //
    // "Process an incoming DATA submessage."  — §8.4.8, p. 142
    //
    // Steps:
    //   1. Resolve WriterProxy by (guidPrefix + writerId).
    //      RELIABLE readers ignore unmatched writers.
    //   2. Duplicate check: discard if writerSN <= highestSNReceived.
    //   3. Allocate pool block; copy serialized payload.
    //   4. proxy.received_change_set(writerSN).
    //   5. accept_change() → cache insert + callback.
    // ----------------------------------------------------------
    void on_data(const ParseContext& ctx,
                 const ParsedData&   msg) noexcept;

    // ----------------------------------------------------------
    // §8.4.8  on_data_frag()
    //
    // "Process an incoming DATA_FRAG submessage."  — §8.4.8, p. 142
    //
    // Delegates to assembler_.on_data_frag().  When the assembler
    // reports completion (returns true), the assembled CacheChange is
    // passed to accept_change() identically to a full DATA reception.
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
    // Delegates to proxy.on_gap().  Also calls
    // assembler_.discard(writer_guid, sn) for each irrelevant SN that
    // has a partial assembly in progress — those fragments will never
    // complete.
    // ----------------------------------------------------------
    void on_gap(const ParseContext& ctx,
                const ParsedGap&   msg) noexcept;

    // ----------------------------------------------------------
    // §8.4.7.3  send_acknack()
    //
    // "Sends an ACKNACK to the matched writer."  — §8.4.7.3, p. 140
    //
    // Builds readerSNState from proxy.missing_changes().  Sends a pure
    // ACK (empty bitmap, F-flag set) when nothing is missing.
    // ----------------------------------------------------------
    void send_acknack(DefaultWriterProxy& proxy,
                      bool                final_flag = false) noexcept;

    // ----------------------------------------------------------
    // tick()
    //
    // Periodic maintenance.  Call every RTPS_TICK_PERIOD_MS.
    //   1. Fire deferred ACKNACKs for proxies whose deadline has passed.
    //   2. Evict stale ChangeFromWriter entries.
    // ----------------------------------------------------------
    void tick(uint32_t now_ms) noexcept;

    // ----------------------------------------------------------
    // Accessors
    // ----------------------------------------------------------

    const Guid&              guid()        const noexcept { return guid_; }
    ReliabilityKind          reliability() const noexcept { return reliability_; }
    /// Received changes, kept only for readers without an OnChangeAvailable
    /// callback (pull consumers); a callback receives each change instead.
    const ReaderHistoryCache& cache()      const noexcept { return reader_cache_; }

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

private:

    DefaultWriterProxy* find_proxy(const Guid& guid) noexcept;
    DefaultWriterProxy* find_proxy_by_sender(const GuidPrefix& prefix,
                                             const EntityId&   writer_eid) noexcept;
    DefaultWriterProxy* find_free_slot() noexcept;

    /// Insert a complete CacheChange into the history cache and fire callback.
    void accept_change(CacheChange&&       change,
                       DefaultWriterProxy& proxy) noexcept;

    // ── Data members ─────────────────────────────────────────

    Guid              guid_;
    ReliabilityKind   reliability_;
    ITransport&       transport_;
    OnChangeAvailable on_change_avail_ = nullptr;
    void*             user_data_       = nullptr;

    ReaderHistoryCache    reader_cache_;
    DefaultWriterProxy    proxies_[RTPS_MAX_MATCHED_WRITERS];

    /// Fragment reassembler — §8.4.14, p. 156.
    /// One slot per matched writer; 256-fragment bitmap; pool-backed.
    DefaultFragmentAssembler assembler_;

    uint32_t       hb_response_ms_;
    MessageBuilder builder_;
};

} // namespace rtps
