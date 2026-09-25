/**
 * @file rtps/stateful_writer.hpp
 * @brief StatefulWriter<CacheDepth, MaxReaders> — §8.4.1–§8.4.6.
 *        (OMG formal/2022-04-01)
 *
 * History depth
 * -------------
 * The spec defines KEEP_LAST(N) history at §8.3.3, p. 68.  N is the
 * number of changes retained in the writer's HistoryCache.
 *
 * CacheDepth is a compile-time template parameter so each writer pays
 * only for what it needs — no runtime dispatch, no heap.
 *
 *   StatefulWriter<>     — KEEP_LAST(1), 8 readers (defaults)
 *   StatefulWriter<4>    — KEEP_LAST(4), 8 readers
 *   StatefulWriter<8,4>  — KEEP_LAST(8), 4 readers
 *
 * ReaderProxy coupling (§8.4.1.1, p. 103)
 * -----------------------------------------
 * "For each change in the Writer's HistoryCache the ReaderProxy
 *  maintains a ChangeForReader object."
 *
 * Therefore the proxy's change-tracking depth must equal CacheDepth.
 * ReaderProxy<CacheDepth> is instantiated for all proxies automatically.
 *
 * IStatefulWriter — type erasure for participant storage
 * -------------------------------------------------------
 * RTPSParticipant stores writers of different depths in one table and
 * calls operations on all of them uniformly.  IStatefulWriter is a
 * pure-virtual base providing only the operations the participant needs.
 * Everything else is accessed through the concrete StatefulWriter<D,R>*
 * returned by create_writer<D,R>().
 *
 * The .cpp file is eliminated: with all logic in the header template
 * body the compiler instantiates only the sizes actually used, and
 * link-time deduplication merges identical instantiations.
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
#include "gap_encoding.hpp"
#include "transport.hpp"
#include "log.hpp"
#include <string.h>


namespace rtps {

// ============================================================
// IStatefulWriter — type-erased interface for participant storage.
//
// Contains only the operations RTPSParticipant calls without knowing
// the concrete CacheDepth or MaxReaders of the writer.
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

    // ------------------------------------------------------------------
    // Depth-independent write path.
    //
    // Deliberately included here even though every other member above is
    // an RTPSParticipant-internal operation: new_change()/write_async()'s
    // signatures never mention CacheDepth — only the
    // writer's internal HistoryCache storage does — so callers that only
    // hold an IStatefulWriter* (because they chose a writer's depth at
    // runtime from a small closed set of instantiations, e.g. an rmw
    // implementation matching an rmw_qos_profile_t.depth request) can
    // still publish without needing the concrete StatefulWriter<D,R>
    // type. See rmw_rosette's qos.hpp for the runtime dispatch this
    // exists for.
    // ------------------------------------------------------------------

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
// StatefulWriter<CacheDepth, MaxReaders>
//
// CacheDepth  — number of simultaneously retained CacheChange slots.
//               KEEP_LAST(CacheDepth), §8.3.3, p. 68.  Default = 1.
//
// MaxReaders  — maximum simultaneously matched readers.
//               Default = RTPS_MAX_MATCHED_READERS.
//
// ReaderProxy<CacheDepth> is used for all proxies so that each proxy
// can track exactly one ChangeForReader per cache slot (§8.4.1.1).
// ============================================================

template<uint8_t CacheDepth = 1,
         uint8_t MaxReaders = RTPS_MAX_MATCHED_READERS>
class StatefulWriter : public IStatefulWriter {
    static_assert(CacheDepth >= 1, "CacheDepth must be >= 1");
    static_assert(MaxReaders >= 1, "MaxReaders must be >= 1");

public:

    static constexpr uint8_t kCacheDepth = CacheDepth;
    static constexpr uint8_t kMaxReaders = MaxReaders;

    /// Concrete HistoryCache type for this writer's depth.
    using Cache = HistoryCache<CacheDepth>;

    /// Proxy depth equals cache depth — §8.4.1.1, p. 103.
    using Proxy = ReaderProxy<CacheDepth>;

    // ----------------------------------------------------------
    // Constructor
    //
    // @param guid           This writer's GUID (§8.2.4).
    // @param reliability    RELIABLE or BEST_EFFORT (§8.3.3).
    // @param durability     VOLATILE or TRANSIENT_LOCAL (§8.3.3).
    // @param transport      Transport abstraction.
    // @param nack_delay_ms  nackResponseDelay — §8.4.1.1, p. 104.
    // @param hb_period_ms   heartbeatPeriod   — §8.4.1.1, p. 104.
    // ----------------------------------------------------------
    StatefulWriter(const Guid&      guid,
                   ReliabilityKind  reliability,
                   DurabilityKind   durability,
                   ITransport&      transport,
                   uint32_t         nack_delay_ms = RTPS_NACK_RESPONSE_DELAY_MS,
                   uint32_t         hb_period_ms  = RTPS_HEARTBEAT_PERIOD_MS) noexcept
        : guid_(guid)
        , reliability_(reliability)
        , durability_(durability)
        , transport_(transport)
        , nack_delay_ms_(nack_delay_ms)
        , hb_period_ms_(hb_period_ms)
    {}

    ~StatefulWriter() noexcept override = default;

    StatefulWriter(const StatefulWriter&)            = delete;
    StatefulWriter& operator=(const StatefulWriter&) = delete;

    // ----------------------------------------------------------
    // §8.4.1.4  new_change()
    //
    // "Creates a new CacheChange with a new sequenceNumber."
    //  — §8.4.1.4, p. 107
    //
    // Allocates a pool buffer for the serialized payload and fills
    // the CacheChange header fields.  The caller writes CDR bytes
    // into out_payload then passes the change to add_change().
    //
    // @param kind         ChangeKind (ALIVE, NOT_ALIVE_DISPOSED, …).
    // @param out_payload  Set to the allocated pool buffer on success.
    // @param payload_size Optional hint for the expected payload size; the
    //                     pool may return a larger buffer.
    // @returns            Pointer to the staging CacheChange, or nullptr
    //                     if the pool is exhausted.
    // ----------------------------------------------------------
    CacheChange* new_change(
        ChangeKind kind,
        BufRef& out_payload,
        size_t payload_size = kPoolMaxBlockSize
    ) noexcept {
        if (staging_.payload.valid()) {
            staging_.payload.reset();
        }
        out_payload = buf_alloc(payload_size);
        if (!out_payload.valid()) {
            RTPS_LOG_E("writer", "buf_alloc failed (%zu) for new_change (pool exhausted)", payload_size);
            return nullptr;
        }
        staging_.kind           = kind;
        staging_.writerGuid     = guid_;
        staging_.sequenceNumber = SEQUENCENUMBER_UNKNOWN; // Assigned in add_change.
        staging_.inlineQos      = {};
        staging_.preframed      = false;
        staging_.payload.block  = out_payload.block;
        staging_.payload.data   = out_payload.data;
        staging_.payload.size   = out_payload.size;
        out_payload.retain();
        return &staging_;
    }

    // ----------------------------------------------------------
    // §8.4.1.4  add_change()
    //
    // "Adds the change to the Writer's HistoryCache.  Assigns a new
    //  sequenceNumber.  Updates all ReaderProxy change sets."
    //  — §8.4.1.4, p. 107
    //
    // With CacheDepth > 1:
    //   When the cache is full, the oldest CacheChange is evicted
    //   (§8.3.3.1, p. 68).  The corresponding ChangeForReader entry
    //   in every proxy is also removed so that proxies do not track
    //   changes that are no longer in the cache.
    //
    // @param change  CacheChange produced by new_change(); ownership
    //                transfers to the cache on success.
    // @returns true on success.
    // ----------------------------------------------------------
    bool add_change(CacheChange&& change) noexcept {

        change.sequenceNumber = nextSN_;
        ++nextSN_;

        // Evict the oldest proxy entry before the cache evicts the slot.
        // §8.3.3.1, p. 68: KEEP_LAST(N) evicts the oldest change.
        if (writer_cache_.size() == CacheDepth) {
            const SequenceNumber oldest = writer_cache_.get_seq_num_min();
            if (oldest != SEQUENCENUMBER_UNKNOWN) {
                for (uint8_t i = 0; i < MaxReaders; ++i)
                    if (proxies_[i].is_matched())
                        proxies_[i].remove_change(oldest);
            }
        }

        CacheChange* inserted = writer_cache_.add_change(
            static_cast<CacheChange&&>(change));
        if (!inserted) {
            RTPS_LOG_E("writer", "cache add_change failed (SN=%ld:%lu)",
                       change.sequenceNumber.high,
                       (unsigned long)change.sequenceNumber.low);

            return false;
        }

        RTPS_LOG_D("writer", "add_change SN=%ld:%lu size=%lu depth=%d",
                   inserted->sequenceNumber.high,
                   (unsigned long)inserted->sequenceNumber.low,
                   (unsigned long)inserted->payload.size,
                   writer_cache_.size());

        // Mark the new change UNSENT on every active proxy.
        for (uint8_t i = 0; i < MaxReaders; ++i)
            if (proxies_[i].is_matched())
                proxies_[i].add_change(inserted->sequenceNumber);


        return true;
    }

    // ----------------------------------------------------------
    // remove_change()
    //
    // "Removes the change from the Writer's HistoryCache."
    //  — §8.4.1.4, p. 108
    //
    // @param sn  SequenceNumber to remove.
    // ----------------------------------------------------------
    void remove_change(const SequenceNumber& sn) noexcept {

        writer_cache_.remove_change(sn);
        for (uint8_t i = 0; i < MaxReaders; ++i)
            if (proxies_[i].is_matched())
                proxies_[i].remove_change(sn);

    }

    // ----------------------------------------------------------
    // write_async()
    //
    // Non-blocking stateful write:
    //   - creates a change
    //   - adds it to history
    //   - sends unsent DATA immediately
    //   - returns without waiting for ACKs
    //
    // Useful for builtin reliable endpoints (e.g. SEDP), where
    // retransmit/repair is still handled by ACKNACK + tick().
    // ----------------------------------------------------------
    bool write_async(ChangeKind         kind,
                     const Octet*       payload_data,
                     uint32_t           payload_size,
                     const InlineQos&   inline_qos = InlineQos{},
                     SequenceNumber*    out_sn = nullptr) noexcept override
    {
        if (!payload_data || payload_size == 0) {
            return false;
        }

        BufRef payload_buf;
        CacheChange* ch = new_change(kind, payload_buf, payload_size);
        if (!ch) {
            RTPS_LOG_E("writer", "write_async: new_change failed");
            return false;
        }

        const uint32_t sz = (payload_size < payload_buf.capacity())
                            ? payload_size : payload_buf.capacity();
        memcpy(payload_buf.data, payload_data, sz);
        payload_buf.size = sz;
        ch->payload.size = sz;
        ch->inlineQos    = inline_qos;

        if (!add_change(static_cast<CacheChange&&>(*ch))) {
            payload_buf.reset();
            return false;
        }

        if (out_sn) {
            *out_sn = nextSN_ - 1;
        }
        send_unsent_changes();
        return true;
    }

    // ----------------------------------------------------------
    // write_async_owned()
    //
    // Zero-copy variant of write_async(): the change takes over `payload`,
    // a pool buffer the caller serialized into, so large samples are not
    // copied a second time. Samples above the transport's datagram limit
    // are sent as DATA_FRAG like any other change.
    //
    // @returns false when `payload` is empty or the history rejects it.
    // ----------------------------------------------------------
    bool write_async_owned(ChangeKind       kind,
                           BufRef&&         payload,
                           const InlineQos& inline_qos = InlineQos{},
                           SequenceNumber*  out_sn     = nullptr) noexcept override
    {
        if (!payload.valid() || payload.size == 0) return false;
        if (staging_.payload.valid()) staging_.payload.reset();
        staging_.kind           = kind;
        staging_.writerGuid     = guid_;
        staging_.sequenceNumber = SEQUENCENUMBER_UNKNOWN;
        staging_.inlineQos      = inline_qos;
        staging_.preframed      = false;
        staging_.payload        = static_cast<BufRef&&>(payload);
        if (!add_change(static_cast<CacheChange&&>(staging_))) return false;
        if (out_sn) *out_sn = nextSN_ - 1;
        send_unsent_changes();
        return true;
    }

    // ----------------------------------------------------------
    // write_preframed_async()
    //
    // Non-blocking write for payloads that are already encoded as
    // RTPS serializedPayload bytes (including representation header).
    //
    // This path allocates TX headroom so send_data_to() can prepend
    // DATA/RTPS headers in-place (zero-copy), avoiding append_data()
    // encapsulation rewriting.
    // ----------------------------------------------------------
    bool write_preframed_async(ChangeKind       kind,
                               const Octet*     serialized_payload,
                               uint32_t         payload_size,
                               SequenceNumber*  out_sn = nullptr,
                               const InlineQos& inline_qos = InlineQos{}) noexcept
    {
        if (!serialized_payload || payload_size == 0) {
            return false;
        }

        if (staging_.payload.valid()) {
            staging_.payload.reset();
        }
        // Headroom covers the RTPS/DATA headers plus the largest inline QoS,
        // so send_data_to() can always frame this change in place.
        BufRef payload_buf = buf_alloc_with_headroom(payload_size,
                                                     kRtpsDataHeadroom + kMaxInlineQosSize);
        if (!payload_buf.valid()) {
            RTPS_LOG_E("writer", "write_preframed_async: buf_alloc_with_headroom failed");
            return false;
        }

        const uint32_t sz = (payload_size < payload_buf.capacity())
                            ? payload_size : payload_buf.capacity();
        memcpy(payload_buf.data, serialized_payload, sz);
        payload_buf.size = sz;

        staging_.kind           = kind;
        staging_.writerGuid     = guid_;
        staging_.sequenceNumber = SEQUENCENUMBER_UNKNOWN;
        staging_.inlineQos      = inline_qos;
        staging_.preframed      = true;
        staging_.payload.block  = payload_buf.block;
        staging_.payload.data   = payload_buf.data;
        staging_.payload.size   = payload_buf.size;
        payload_buf.retain();

        if (!add_change(static_cast<CacheChange&&>(staging_))) {
            payload_buf.reset();
            return false;
        }

        if (out_sn) {
            *out_sn = nextSN_ - 1;
        }
        send_unsent_changes();
        return true;
    }

    // ----------------------------------------------------------
    // §8.4.1.4  matched_reader_add()
    //
    // "Adds the ReaderProxy to the set of matched readers."
    //  — §8.4.1.4, p. 108
    //
    // TRANSIENT_LOCAL replay (§8.7.2.2, p. 196):
    //   All CacheDepth slots currently in the cache are marked UNSENT
    //   on the new proxy.  send_unsent_changes() replays them in SN
    //   order.
    // ----------------------------------------------------------
    bool matched_reader_add(const Guid&    reader_guid,
                            const Locator* unicast_locators,
                            uint8_t        locator_count,
                            bool           expects_inline_qos = false) noexcept override
    {

        if (find_proxy(reader_guid)) { return false; }
        Proxy* slot = find_free_slot();
        if (!slot) {
            RTPS_LOG_W("writer", "reader proxy table full (max=%d)", MaxReaders);

            return false;
        }

        slot->init(reader_guid, unicast_locators, locator_count,
                   expects_inline_qos);

        RTPS_LOG_I("writer", "reader matched %02x%02x%02x%02x entity=%02x%02x%02x%02x",
                   reader_guid.prefix.value[0], reader_guid.prefix.value[1],
                   reader_guid.prefix.value[2], reader_guid.prefix.value[3],
                   reader_guid.entityId.entityKey[0], reader_guid.entityId.entityKey[1],
                   reader_guid.entityId.entityKey[2], reader_guid.entityId.entityKind);

        // TRANSIENT_LOCAL: replay every cached change to the new reader,
        // and push it now rather than waiting for the next write.
        if (durability_ == DurabilityKind::TRANSIENT_LOCAL) {
            writer_cache_.replay_to_new_reader(
                [&](const CacheChange& c) noexcept {
                    slot->add_change(c.sequenceNumber);
                });
            send_unsent_changes();
        }

        return true;
    }

    void matched_reader_remove(const Guid& reader_guid) noexcept override {

        Proxy* p = find_proxy(reader_guid);
        if (p) p->clear();

    }

    Proxy* matched_reader_lookup(const Guid& reader_guid) noexcept {
        return find_proxy(reader_guid);
    }

    // ----------------------------------------------------------
    // §8.4.2.2  send_heartbeat()
    //
    // "Sends a HEARTBEAT Submessage advertising the range of SNs
    //  available in the HistoryCache."  — §8.4.2.2, p. 113
    //
    // With CacheDepth > 1, firstSN and lastSN span the full window
    // of retained changes.  The reader uses this range to identify
    // which SNs it can still request via ACKNACK.
    //
    // @param final_flag  F-flag: reader need not reply if up-to-date.
    // @param directed_to Non-GUID_UNKNOWN: send only to this reader.
    // ----------------------------------------------------------
    void send_heartbeat(bool       final_flag  = false,
                        const Guid directed_to = GUID_UNKNOWN) noexcept
    {
        ++heartbeatCount_;
        RTPS_LOG_D("writer", "HEARTBEAT count=%lu final=%d",
                   (unsigned long)heartbeatCount_, (int)final_flag);
        SequenceNumber fsn = writer_cache_.get_seq_num_min();
        SequenceNumber lsn = writer_cache_.get_seq_num_max();
        // Empty cache: first > last signals "nothing available" — §8.4.2.2.
        if (fsn == SEQUENCENUMBER_UNKNOWN) {
            lsn = nextSN_ - 1; fsn = nextSN_;
        }
        for (uint8_t i = 0; i < MaxReaders; ++i) {
            Proxy& proxy = proxies_[i];
            if (!proxy.is_matched()) continue;
            if (directed_to != GUID_UNKNOWN &&
                proxy.remoteReaderGuid != directed_to) continue;
            const Locator& dest = proxy.best_locator();
            if (dest == LOCATOR_INVALID) continue;
            if (!builder_.begin(guid_.prefix)) continue;
            const EntityId& rid = (directed_to != GUID_UNKNOWN)
                ? proxy.remoteReaderGuid.entityId : entity_id::UNKNOWN;
            bool ok = builder_.append_heartbeat(
                rid, guid_.entityId, fsn, lsn,
                heartbeatCount_, final_flag);
            if (ok) {
                BufRef buf = builder_.finish();
                transport_.send_unicast(buf, dest);
                buf.reset();
            } else { builder_.abort(); }
        }
    }

    // ----------------------------------------------------------
    // §8.4.3  on_acknack()  — §8.4.3, p. 118
    // ----------------------------------------------------------
    void on_acknack(const ParseContext&  ctx,
                    const ParsedAckNack& msg) noexcept override
    {

        Guid sg{ ctx.senderGuidPrefix, msg.readerId };
        Proxy* proxy = find_proxy(sg);
        if (!proxy) {
            RTPS_LOG_T("writer", "ACKNACK from unknown reader %02x%02x%02x%02x, ignored",
                       ctx.senderGuidPrefix.value[0], ctx.senderGuidPrefix.value[1],
                       ctx.senderGuidPrefix.value[2], ctx.senderGuidPrefix.value[3]);

            return;
        }

        // Discard duplicate / reordered ACKNACKs.
        if (msg.count <= proxy->lastAcknackCount) { return; }
        proxy->lastAcknackCount = msg.count;

        // Cumulative ACK: all SNs below bitmapBase are acknowledged.
        if (msg.readerSNState.bitmapBase > SEQUENCENUMBER_ZERO)
            proxy->acked_changes_set(msg.readerSNState.bitmapBase - 1);

        RTPS_LOG_D("writer", "ACKNACK base=%d:%lu count=%lu from %02x%02x%02x%02x",
                   msg.readerSNState.bitmapBase.high,
                   (unsigned long)msg.readerSNState.bitmapBase.low,
                   (unsigned long)msg.count,
                   ctx.senderGuidPrefix.value[0], ctx.senderGuidPrefix.value[1],
                   ctx.senderGuidPrefix.value[2], ctx.senderGuidPrefix.value[3]);

        // NACKed SNs: arm the nackResponseDelay timer. NACKed SNs this reader
        // never had a ChangeForReader for are irrelevant to it: GAP them now.
        proxy->requested_changes_set(msg.readerSNState, rosette::platform::monotonic_ms());
        send_irrelevant_gap(*proxy, msg.readerSNState);

        // An ACKNACK that requests nothing yet acknowledges less than this
        // writer has written is a reader announcing itself (Fast DDS and
        // Cyclone DDS send one, with bitmapBase 0 or 1, on matching). Answer
        // with a HEARTBEAT directed at it so it learns the available range
        // now instead of at the next heartbeat period (§8.4.15.4).
        if (msg.readerSNState.numBits == 0u &&
            msg.readerSNState.bitmapBase < nextSN_) {
            send_heartbeat(false, sg);
        }


    }

    // ----------------------------------------------------------
    // §8.4.4  on_nack_frag()  — §8.4.4, p. 122
    // ----------------------------------------------------------
    void on_nack_frag(const ParseContext&   ctx,
                      const ParsedNackFrag& msg) noexcept override
    {

        Guid sg{ ctx.senderGuidPrefix, msg.readerId };
        Proxy* proxy = find_proxy(sg);
        if (!proxy) { return; }

        const uint32_t now_ms = rosette::platform::monotonic_ms();
        if (!proxy->nackPending) {
            proxy->nackPending        = true;
            proxy->nackPendingSinceMs = now_ms;
        }
        // Treat as a whole-sample NACK for the affected SN.
        SequenceNumberSet sn_set{};
        sn_set.bitmapBase = msg.writerSN;
        sn_set.numBits    = 1;
        sn_set.bitmap[0]  = 0x80000000u;   // bit 0 = bitmapBase itself.
        proxy->requested_changes_set(sn_set, now_ms);

    }

    // ----------------------------------------------------------
    // tick()
    //
    // Periodic maintenance.  Call every RTPS_TICK_PERIOD_MS.
    //
    // §8.4.1.1, p. 104  nackResponseDelay — deferred retransmit.
    // §8.4.2.2, p. 113  heartbeatPeriod   — periodic HEARTBEAT.
    // ----------------------------------------------------------
    void tick(uint32_t now_ms) noexcept override {


        // ── Phase 1: NACK retransmit ──────────────────────────
        for (uint8_t i = 0; i < MaxReaders; ++i) {
            Proxy& proxy = proxies_[i];
            if (!proxy.is_matched()) continue;
            if (!proxy.nack_delay_elapsed(now_ms, nack_delay_ms_)) continue;

            bool any_sent = false;
            while (true) {
                const ChangeForReader* req = proxy.next_requested_change();
                if (!req) break;
                const CacheChange* ch =
                    writer_cache_.get_change(req->sequenceNumber);
                if (ch) {
                    proxy.mark_underway(req->sequenceNumber);
                    send_data_to(*ch, proxy);
                    any_sent = true;
                } else {
                    // Change evicted from the history: it is irrelevant now.
                    SequenceNumberSet evicted{};
                    evicted.add(req->sequenceNumber);
                    send_gap(proxy, evicted);
                    proxy.acked_changes_set(req->sequenceNumber);
                }
            }
            proxy.clear_nack_pending();

            // Post-retransmit directed HEARTBEAT (F=true).
            // §8.4.2.2, p. 113.
            if (any_sent) {
                ++heartbeatCount_;
                SequenceNumber fsn = writer_cache_.get_seq_num_min();
                SequenceNumber lsn = writer_cache_.get_seq_num_max();
                if (fsn == SEQUENCENUMBER_UNKNOWN) {
                    lsn = nextSN_ - 1; fsn = nextSN_;
                }
                const Locator& dest = proxy.best_locator();
                if (dest != LOCATOR_INVALID &&
                    builder_.begin(guid_.prefix)) {
                    bool ok = builder_.append_heartbeat(
                        proxy.remoteReaderGuid.entityId,
                        guid_.entityId, fsn, lsn,
                        heartbeatCount_, /*final=*/true);
                    if (ok) {
                        BufRef buf = builder_.finish();
                        transport_.send_unicast(buf, dest);
                        buf.reset();
                    } else { builder_.abort(); }
                }
            }
        }

        // ── Phase 2: Periodic HEARTBEAT to announcing readers ─
        // §8.4.9.2 T5-T7: a ReaderProxy is "announcing" only while it has
        // unacknowledged changes, and only announcing proxies receive the
        // periodic HEARTBEAT. A fully acknowledged reader hears nothing
        // until the next write, so an idle link stays quiet.
        if ((now_ms - lastHbSentMs_) >= hb_period_ms_) {
            lastHbSentMs_ = now_ms;
            ++heartbeatCount_;
            SequenceNumber fsn = writer_cache_.get_seq_num_min();
            SequenceNumber lsn = writer_cache_.get_seq_num_max();
            if (fsn == SEQUENCENUMBER_UNKNOWN) {
                lsn = nextSN_ - 1; fsn = nextSN_;
            }
            for (uint8_t i = 0; i < MaxReaders; ++i) {
                Proxy& proxy = proxies_[i];
                if (!proxy.is_matched() || !proxy.is_announcing()) continue;
                const Locator& dest = proxy.best_locator();
                if (dest == LOCATOR_INVALID) continue;
                if (!builder_.begin(guid_.prefix)) continue;
                bool ok = builder_.append_heartbeat(
                    entity_id::UNKNOWN, guid_.entityId,
                    fsn, lsn, heartbeatCount_, /*final=*/false);
                if (ok) {
                    BufRef buf = builder_.finish();
                    transport_.send_unicast(buf, dest);
                    buf.reset();
                } else { builder_.abort(); }
            }
        }

    }

    // ----------------------------------------------------------
    // Accessors
    // ----------------------------------------------------------

    const Guid&     guid()        const noexcept override { return guid_; }
    ReliabilityKind reliability() const noexcept { return reliability_; }
    DurabilityKind  durability()  const noexcept { return durability_; }
    const Cache&    cache()       const noexcept { return writer_cache_; }

    /// nack_response_delay() — §8.4.1.1, p. 104.
    uint32_t nack_response_delay()            const noexcept { return nack_delay_ms_; }
    void     nack_response_delay(uint32_t ms)       noexcept { nack_delay_ms_ = ms; }

    /// heartbeat_period() — §8.4.1.1, p. 104.
    uint32_t heartbeat_period()               const noexcept { return hb_period_ms_; }
    void     heartbeat_period(uint32_t ms)          noexcept { hb_period_ms_ = ms; }

    // ----------------------------------------------------------
    // acknowledged_by_all()
    //
    // §8.4.7.1: a reliable writer knows, per ReaderProxy, which changes are
    // acknowledged. This reports whether the newest change in the history
    // (and therefore every older one, acknowledgments being cumulative) is
    // acknowledged by every matched reader. A BEST_EFFORT writer never
    // waits, so it always reports true.
    //
    // Verification: TC-SWR-8.4.7-001.
    // ----------------------------------------------------------
    bool acknowledged_by_all() const noexcept override {
        if (reliability_ != ReliabilityKind::RELIABLE) return true;
        const SequenceNumber lsn = writer_cache_.get_seq_num_max();
        if (lsn == SEQUENCENUMBER_UNKNOWN) return true;
        return all_readers_acked(lsn);
    }

    // ----------------------------------------------------------
    // ms_until_next_timer()
    //
    // How long tick() can be deferred: the sooner of the next periodic
    // HEARTBEAT (only while some matched reader has unacknowledged changes,
    // §8.4.9.2 T5-T7) and the end of any pending nackResponseDelay
    // (§8.4.7.1). The participant driver sleeps no longer than this, so a
    // fully acknowledged writer never wakes the CPU on its own.
    //
    // Verification: TC-SWR-8.4.7-002, TC-SWR-8.4.9-004.
    // ----------------------------------------------------------
    uint32_t ms_until_next_timer(uint32_t now_ms) const noexcept override {
        uint32_t next = UINT32_MAX;
        bool any_announcing = false;
        for (uint8_t i = 0; i < MaxReaders; ++i) {
            const Proxy& proxy = proxies_[i];
            if (!proxy.is_matched()) continue;
            if (proxy.is_announcing()) any_announcing = true;
            if (proxy.nackPending) {
                const uint32_t elapsed = now_ms - proxy.nackPendingSinceMs;
                const uint32_t left = elapsed >= nack_delay_ms_ ? 0u : nack_delay_ms_ - elapsed;
                if (left < next) next = left;
            }
        }
        if (any_announcing) {
            const uint32_t elapsed = now_ms - lastHbSentMs_;
            const uint32_t left = elapsed >= hb_period_ms_ ? 0u : hb_period_ms_ - elapsed;
            if (left < next) next = left;
        }
        return next;
    }

    uint8_t matched_reader_count() const noexcept override {
        uint8_t n = 0;
        for (uint8_t i = 0; i < MaxReaders; ++i)
            if (proxies_[i].is_matched()) ++n;
        return n;
    }

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
    void send_gap(const Proxy& proxy, const SequenceNumberSet& irrelevant) noexcept {
        SequenceNumber gap_start{};
        SequenceNumberSet gap_list{};
        if (!encode_gap(irrelevant, gap_start, gap_list)) return;
        const Locator& dest = proxy.best_locator();
        if (dest == LOCATOR_INVALID || !builder_.begin(guid_.prefix)) return;
        if (builder_.append_gap(proxy.remoteReaderGuid.entityId, guid_.entityId,
                                gap_start, gap_list)) {
            BufRef buf = builder_.finish();
            transport_.send_unicast(buf, dest);
            buf.reset();
        } else {
            builder_.abort();
        }
    }

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
    void send_irrelevant_gap(const Proxy& proxy, const SequenceNumberSet& requested) noexcept {
        SequenceNumberSet irrelevant{};
        requested.for_each([&](const SequenceNumber& sn) noexcept {
            if (sn < nextSN_ && !proxy.tracks(sn)) (void)irrelevant.add(sn);
        });
        send_gap(proxy, irrelevant);
    }

    // ── Internal helpers ──────────────────────────────────────

    Proxy* find_proxy(const Guid& g) noexcept {
        for (uint8_t i = 0; i < MaxReaders; ++i)
            if (proxies_[i].is_matched() &&
                proxies_[i].remoteReaderGuid == g)
                return &proxies_[i];
        return nullptr;
    }
    Proxy* find_free_slot() noexcept {
        for (uint8_t i = 0; i < MaxReaders; ++i)
            if (!proxies_[i].is_matched()) return &proxies_[i];
        return nullptr;
    }
    bool send_data_to(const CacheChange& ch, Proxy& proxy) noexcept {
        const Locator& dest = proxy.best_locator();
        if (dest == LOCATOR_INVALID) return false;

        // A sample whose DATA message would exceed the transport's datagram
        // limit goes out as DATA_FRAG submessages instead (§8.4.14.1).
        const uint32_t data_datagram = sizeof(RTPSMessageHeader) + sizeof(SubmessageHeader) +
            sizeof(DataSubmessageFixed) + inline_qos_serialized_size(ch.inlineQos) +
            (ch.preframed ? 0u : kEncapsulationSize) + ch.payload.size;
        if (data_datagram > transport_.max_datagram_size()) {
            return send_fragments_to(ch, proxy, dest);
        }

        // ----------------------------------------------------------
        // Zero-copy TX path (§9.4.5.3, p. 204)
        //
        // If the payload BufRef was allocated with buf_alloc_with_headroom()
        // and has at least kRtpsDataHeadroom bytes of headroom available,
        // we prepend the RTPS message header and DATA submessage header
        // directly into the payload block — no scratch buffer, no copy.
        //
        // headroom_available() == (ch.payload.data - block->payload()),
        // i.e. the reserved bytes before the CDR data.
        // ----------------------------------------------------------
        const uint16_t inline_qos_size = inline_qos_serialized_size(ch.inlineQos);
        if (ch.payload.valid() &&
            ch.payload.headroom_available() >= kRtpsDataHeadroom + inline_qos_size)
        {
            // Work on a non-owning view — we retain the block before
            // prepending so the CacheChange still owns the original ref.
            ch.payload.retain();
            BufRef view{};
            view.block = ch.payload.block;
            view.data  = ch.payload.data;
            view.size  = ch.payload.size;
            // view.capacity() is correct because it is computed from block.

            // Inline QoS sits between the fixed DATA fields and the
            // serializedPayload — §9.4.5.3, Table 9.14, p. 204.
            if (inline_qos_size > 0u) {
                Octet* iq = view.prepend(inline_qos_size);
                if (!iq) { view.reset(); return false; }
                serialize_inline_qos(iq, ch.inlineQos);
            }

            // Prepend DATA submessage header in-place.
            // §9.4.5.3, Table 9.14, p. 204.
            auto* sm = reinterpret_cast<DataSubmessageFixed*>(
                view.prepend(sizeof(DataSubmessageFixed)));
            if (!sm) { view.reset(); return false; }

            sm->extraFlags        = 0;
            sm->octetsToInlineQos = 16u;  // No inline QoS — §9.4.5.3.
            sm->readerId          = proxy.remoteReaderGuid.entityId;
            sm->writerId          = guid_.entityId;
            sm->writerSN          = ch.sequenceNumber;

            // Prepend SubmessageHeader — §9.4.5.1, p. 200.
            auto* smh = reinterpret_cast<SubmessageHeader*>(
                view.prepend(sizeof(SubmessageHeader)));
            if (!smh) { view.reset(); return false; }

            smh->submessageId = SubmessageKind::DATA;
            smh->flags        = static_cast<Octet>(
                                FLAG_E |
                                (ch.kind == ChangeKind::ALIVE
                                    ? data_flags::D : data_flags::K) |
                                (inline_qos_size > 0u ? data_flags::Q : 0u));
            smh->submessageLength = static_cast<uint16_t>(
                view.size - sizeof(SubmessageHeader));

            // Prepend RTPS message header — §9.4.2, p. 222.
            auto* mh = reinterpret_cast<RTPSMessageHeader*>(
                view.prepend(sizeof(RTPSMessageHeader)));
            if (!mh) { view.reset(); return false; }
            *mh = RTPSMessageHeader{};
            mh->guidPrefix = guid_.prefix;
            mh->vendorId = VENDORID_THIS_IMPL;

            bool sent = transport_.send_unicast(view, dest);
            view.reset();   // Release the extra retain() taken above.
            if (sent) proxy.mark_sent(ch.sequenceNumber);
            return sent;
        }

        // ----------------------------------------------------------
        // Copy path — payload has no headroom; use MessageBuilder.
        // ----------------------------------------------------------
        if (!builder_.begin(guid_.prefix, data_datagram)) return false;
        bool ok = builder_.append_data(
            proxy.remoteReaderGuid.entityId, guid_.entityId,
            ch.sequenceNumber, ch.payload,
            ch.kind != ChangeKind::ALIVE, ch.inlineQos);
        if (!ok) { builder_.abort(); return false; }
        BufRef buf = builder_.finish();
        bool sent = transport_.send_unicast(buf, dest);
        buf.reset();
        if (sent) proxy.mark_sent(ch.sequenceNumber);
        return sent;
    }
    /// Octets of the CDR encapsulation header (§10.2) prepended to samples
    /// that are not preframed.
    static constexpr uint16_t kEncapsulationSize = 4u;

    // ----------------------------------------------------------
    // send_fragments_to()
    //
    // §8.4.14.1 (sending fragmented data), §9.4.5.4 (DATA_FRAG): splits the
    // sample's serializedPayload into fragments so that each RTPS message,
    // one DATA_FRAG per datagram, fits transport_.max_datagram_size(). The
    // serializedPayload is the stored payload, preceded by the CDR_LE
    // header when the change is not preframed; fragment 1 carries that
    // header. Inline QoS rides in every fragment, so a reader holding any
    // fragment knows the sample's key and related sample identity.
    // Fragment sizes are multiples of 4 octets.
    //
    // A NACK_FRAG or NACK for the SN resends every fragment.
    //
    // Verification: TC-SWR-8.4.14-001 (every datagram within the limit;
    // reassembly restores the payload), and on-wire checks in
    // tests/ros2/run_rmw_frag_loopback.sh.
    // ----------------------------------------------------------
    bool send_fragments_to(const CacheChange& ch, Proxy& proxy, const Locator& dest) noexcept {
        static constexpr Octet kCdrLe[kEncapsulationSize] = {0x00u, 0x01u, 0x00u, 0x00u};
        const uint16_t prefix_len = ch.preframed ? 0u : kEncapsulationSize;
        const uint32_t data_size = prefix_len + ch.payload.size;
        const uint32_t overhead = sizeof(RTPSMessageHeader) + sizeof(SubmessageHeader) +
            sizeof(DataFragSubmessageFixed) + inline_qos_serialized_size(ch.inlineQos);
        const uint32_t limit = transport_.max_datagram_size();
        if (limit < overhead + 64u) {
            RTPS_LOG_E("writer", "datagram limit %lu leaves no room for fragments",
                       (unsigned long)limit);
            return false;
        }
        uint32_t frag_size = (limit - overhead) & ~3u;
        if (frag_size > 0xFFFCu) frag_size = 0xFFFCu;
        const uint32_t frag_count = (data_size + frag_size - 1u) / frag_size;

        bool all_sent = true;
        for (uint32_t k = 1u; k <= frag_count; ++k) {
            const uint32_t offset = (k - 1u) * frag_size;
            const uint32_t len = (data_size - offset < frag_size) ? data_size - offset : frag_size;
            uint16_t pre_len = 0u;
            const Octet* pre = nullptr;
            if (offset < prefix_len) {
                pre = kCdrLe + offset;
                pre_len = static_cast<uint16_t>(
                    (prefix_len - offset < len) ? prefix_len - offset : len);
            }
            const uint32_t body_offset = (offset + pre_len) - prefix_len;
            const uint16_t body_len = static_cast<uint16_t>(len - pre_len);

            if (!builder_.begin(guid_.prefix, overhead + len)) { all_sent = false; break; }
            const bool ok = builder_.append_data_frag_parts(
                proxy.remoteReaderGuid.entityId, guid_.entityId, ch.sequenceNumber, k,
                static_cast<uint16_t>(frag_size), data_size, pre, pre_len,
                ch.payload.data + body_offset, body_len, ch.inlineQos);
            if (!ok) { builder_.abort(); all_sent = false; break; }
            BufRef buf = builder_.finish();
            all_sent = transport_.send_unicast(buf, dest) && all_sent;
            buf.reset();
        }
        if (all_sent) proxy.mark_sent(ch.sequenceNumber);
        return all_sent;
    }

    void send_unsent_changes() noexcept {
        for (uint8_t i = 0; i < MaxReaders; ++i) {
            Proxy& proxy = proxies_[i];
            if (!proxy.is_matched()) continue;
            while (true) {
                const ChangeForReader* u = proxy.next_unsent_change();
                if (!u) break;
                const CacheChange* ch =
                    writer_cache_.get_change(u->sequenceNumber);
                if (!ch) { proxy.acked_changes_set(u->sequenceNumber); continue; }
                send_data_to(*ch, proxy);
            }
        }
    }
    bool all_readers_acked(const SequenceNumber& sn) const noexcept {
        for (uint8_t i = 0; i < MaxReaders; ++i)
            if (proxies_[i].is_matched() &&
                proxies_[i].highestAckedSN < sn)
                return false;
        return true;
    }

    // ── Data members ──────────────────────────────────────────

    Guid            guid_;
    ReliabilityKind reliability_;
    DurabilityKind  durability_;
    ITransport&     transport_;

    Cache  writer_cache_;          ///< HistoryCache<CacheDepth>
    Proxy  proxies_[MaxReaders];   ///< ReaderProxy<CacheDepth>[MaxReaders]

    CacheChange    staging_{};     ///< Staging area for new_change().
    int32_t        heartbeatCount_ = 0;
    SequenceNumber nextSN_         = SequenceNumber::from_int64(1);
    uint32_t       nack_delay_ms_;
    uint32_t       hb_period_ms_;
    uint32_t       lastHbSentMs_   = 0;
    MessageBuilder builder_;
};

// ============================================================
// Convenience aliases
// ============================================================

/// Original KEEP_LAST(1) writer — matches the previous API exactly.
/// All existing call sites require zero changes.
using WriterKL1 = StatefulWriter<1>;

} // namespace rtps
