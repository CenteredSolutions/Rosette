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
#include "transport.hpp"
#include "log.hpp"
#include <string.h>

#ifndef RTPS_NO_FREERTOS
#  include "FreeRTOS.h"
#  include "task.h"
#  include "semphr.h"
#endif

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
};

// ============================================================
// Configuration defaults
// ============================================================

#ifndef RTPS_MAX_MATCHED_READERS
#  define RTPS_MAX_MATCHED_READERS 8
#endif
#ifndef RTPS_RELIABLE_SEND_TIMEOUT_MS
#  define RTPS_RELIABLE_SEND_TIMEOUT_MS 5000u
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
    {
#ifndef RTPS_NO_FREERTOS
        mutex_ = xSemaphoreCreateMutex();
#endif
    }

    ~StatefulWriter() noexcept override {
#ifndef RTPS_NO_FREERTOS
        if (mutex_) vSemaphoreDelete(mutex_);
#endif
    }

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
        lock();
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
            unlock();
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

        unlock();
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
        lock();
        writer_cache_.remove_change(sn);
        for (uint8_t i = 0; i < MaxReaders; ++i)
            if (proxies_[i].is_matched())
                proxies_[i].remove_change(sn);
        unlock();
    }

    // ----------------------------------------------------------
    // write_reliable()
    //
    // High-level blocking write for RELIABLE writers.
    // Calls new_change() + add_change(), sends to all readers, then
    // blocks on a FreeRTOS task notification until the specific SN
    // written here is acknowledged by all readers or the timeout
    // expires.
    //
    // With CacheDepth > 1 multiple writes can be in flight; each
    // write_reliable() waits only for its own SN.
    //
    // @param kind         ChangeKind.
    // @param payload_data CDR bytes to send.
    // @param payload_size Byte count.
    // @param timeout_ms   0 = use RTPS_RELIABLE_SEND_TIMEOUT_MS.
    // @returns true if all readers acknowledged within the timeout.
    // ----------------------------------------------------------
    bool write_reliable(ChangeKind   kind,
                        const Octet* payload_data,
                        uint32_t     payload_size,
                        uint32_t     timeout_ms = 0) noexcept
    {
        return write_reliable(kind, payload_data, payload_size, InlineQos{}, nullptr,
                              timeout_ms);
    }

    bool write_reliable(ChangeKind         kind,
                        const Octet*       payload_data,
                        uint32_t           payload_size,
                        const InlineQos&   inline_qos,
                        SequenceNumber*    out_sn = nullptr,
                        uint32_t           timeout_ms = 0) noexcept
    {
        if (!payload_data || payload_size == 0) {
            return false;
        }

        BufRef payload_buf;
        CacheChange* ch = new_change(kind, payload_buf, payload_size);
        if (!ch) {
            RTPS_LOG_E("writer", "write_reliable: new_change failed");
            return false;
        }

        const uint32_t sz = (payload_size < payload_buf.capacity())
                            ? payload_size : payload_buf.capacity();
        memcpy(payload_buf.data, payload_data, sz);
        payload_buf.size = sz;
        ch->payload.size = sz;
        ch->inlineQos    = inline_qos;

#ifndef RTPS_NO_FREERTOS
        lock();
        writeTask_ = xTaskGetCurrentTaskHandle();
        unlock();
#endif

        if (!add_change(static_cast<CacheChange&&>(*ch))) {
            payload_buf.reset();
            return false;
        }

        // Capture the SN that add_change() assigned (nextSN_ - 1).
        const SequenceNumber my_sn = nextSN_ - 1;
        if (out_sn) *out_sn = my_sn;

        send_unsent_changes();

        if (reliability_ == ReliabilityKind::BEST_EFFORT ||
            matched_reader_count() == 0) {
            return true;
        }

        const uint32_t to = timeout_ms ? timeout_ms
                                       : RTPS_RELIABLE_SEND_TIMEOUT_MS;

#ifndef RTPS_NO_FREERTOS
        const TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(to);
        while (!all_readers_acked(my_sn)) {
            const TickType_t now = xTaskGetTickCount();
            if (now >= deadline) {
                RTPS_LOG_W("writer", "write_reliable timeout after %lums SN=%ld:%lu",
                           (unsigned long)to,
                           my_sn.high, (unsigned long)my_sn.low);
                lock(); writeTask_ = nullptr; unlock();
                return false;
            }
            ulTaskNotifyTake(pdTRUE, deadline - now);
        }
        lock(); writeTask_ = nullptr; unlock();
#else
        (void)my_sn; (void)to;
#endif
        return true;
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
                     SequenceNumber*    out_sn = nullptr) noexcept
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
    // write_preframed_async()
    //
    // Non-blocking write for payloads that are already encoded as
    // RTPS serializedPayload bytes (including representation header).
    //
    // This path allocates TX headroom so send_data_to() can prepend
    // DATA/RTPS headers in-place (zero-copy), avoiding append_data()
    // encapsulation rewriting.
    // ----------------------------------------------------------
    bool write_preframed_async(ChangeKind      kind,
                               const Octet*    serialized_payload,
                               uint32_t        payload_size,
                               SequenceNumber* out_sn = nullptr) noexcept
    {
        if (!serialized_payload || payload_size == 0) {
            return false;
        }

        if (staging_.payload.valid()) {
            staging_.payload.reset();
        }
        BufRef payload_buf = buf_alloc_with_headroom(payload_size, kRtpsDataHeadroom);
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
        staging_.inlineQos      = {};
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
        lock();
        if (find_proxy(reader_guid)) { unlock(); return false; }
        Proxy* slot = find_free_slot();
        if (!slot) {
            RTPS_LOG_W("writer", "reader proxy table full (max=%d)", MaxReaders);
            unlock();
            return false;
        }

        slot->init(reader_guid, unicast_locators, locator_count,
                   expects_inline_qos);

        RTPS_LOG_I("writer", "reader matched %02x%02x%02x%02x entity=%02x%02x%02x%02x",
                   reader_guid.prefix.value[0], reader_guid.prefix.value[1],
                   reader_guid.prefix.value[2], reader_guid.prefix.value[3],
                   reader_guid.entityId.entityKey[0], reader_guid.entityId.entityKey[1],
                   reader_guid.entityId.entityKey[2], reader_guid.entityId.entityKind);

        // TRANSIENT_LOCAL: replay every cached change to the new reader.
        if (durability_ == DurabilityKind::TRANSIENT_LOCAL) {
            writer_cache_.replay_to_new_reader(
                [&](const CacheChange& c) noexcept {
                    slot->add_change(c.sequenceNumber);
                });
        }
        unlock();
        return true;
    }

    void matched_reader_remove(const Guid& reader_guid) noexcept override {
        lock();
        Proxy* p = find_proxy(reader_guid);
        if (p) p->clear();
        unlock();
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
        lock();
        Guid sg{ ctx.senderGuidPrefix, msg.readerId };
        Proxy* proxy = find_proxy(sg);
        if (!proxy) {
            RTPS_LOG_T("writer", "ACKNACK from unknown reader %02x%02x%02x%02x, ignored",
                       ctx.senderGuidPrefix.value[0], ctx.senderGuidPrefix.value[1],
                       ctx.senderGuidPrefix.value[2], ctx.senderGuidPrefix.value[3]);
            unlock();
            return;
        }

        // Discard duplicate / reordered ACKNACKs.
        if (msg.count <= proxy->lastAcknackCount) { unlock(); return; }
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

        // NACKed SNs: arm the nackResponseDelay timer.
        proxy->requested_changes_set(msg.readerSNState, RTPS_GET_MS());

        // Unblock write_reliable() if all readers are now satisfied.
        const SequenceNumber lsn = writer_cache_.get_seq_num_max();
        if (lsn != SEQUENCENUMBER_UNKNOWN && all_readers_acked(lsn)) {
#ifndef RTPS_NO_FREERTOS
            if (writeTask_) xTaskNotifyGive(writeTask_);
#endif
        }
        unlock();
    }

    // ----------------------------------------------------------
    // §8.4.4  on_nack_frag()  — §8.4.4, p. 122
    // ----------------------------------------------------------
    void on_nack_frag(const ParseContext&   ctx,
                      const ParsedNackFrag& msg) noexcept override
    {
        lock();
        Guid sg{ ctx.senderGuidPrefix, msg.readerId };
        Proxy* proxy = find_proxy(sg);
        if (!proxy) { unlock(); return; }

        const uint32_t now_ms = RTPS_GET_MS();
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
        unlock();
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
        lock();

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
                    // Change evicted from cache — send a GAP.
                    // §9.4.5.11, p. 258.
                    const Locator& dest = proxy.best_locator();
                    if (dest != LOCATOR_INVALID &&
                        builder_.begin(guid_.prefix)) {
                        SequenceNumberSet gl{};
                        gl.bitmapBase = req->sequenceNumber;
                        bool ok = builder_.append_gap(
                            proxy.remoteReaderGuid.entityId,
                            guid_.entityId, req->sequenceNumber, gl);
                        if (ok) {
                            BufRef buf = builder_.finish();
                            transport_.send_unicast(buf, dest);
                            buf.reset();
                        } else { builder_.abort(); }
                    }
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

        // ── Phase 2: Periodic broadcast HEARTBEAT ────────────
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
                if (!proxy.is_matched()) continue;
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
        unlock();
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

    uint8_t matched_reader_count() const noexcept {
        uint8_t n = 0;
        for (uint8_t i = 0; i < MaxReaders; ++i)
            if (proxies_[i].is_matched()) ++n;
        return n;
    }

private:

    // ── Mutex helpers ─────────────────────────────────────────
#ifndef RTPS_NO_FREERTOS
    void lock()   noexcept { if (mutex_) xSemaphoreTake(mutex_, portMAX_DELAY); }
    void unlock() noexcept { if (mutex_) xSemaphoreGive(mutex_); }
#else
    void lock()   noexcept {}
    void unlock() noexcept {}
#endif

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
        if (ch.inlineQos.empty() &&
            ch.payload.valid() &&
            ch.payload.headroom_available() >= kRtpsDataHeadroom)
        {
            // Work on a non-owning view — we retain the block before
            // prepending so the CacheChange still owns the original ref.
            ch.payload.retain();
            BufRef view{};
            view.block = ch.payload.block;
            view.data  = ch.payload.data;
            view.size  = ch.payload.size;
            // view.capacity() is correct because it is computed from block.

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
            smh->flags        = FLAG_E |
                                (ch.kind == ChangeKind::ALIVE
                                    ? data_flags::D : data_flags::K);
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
        if (!builder_.begin(guid_.prefix)) return false;
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

#ifndef RTPS_NO_FREERTOS
    SemaphoreHandle_t mutex_     = nullptr;
    TaskHandle_t      writeTask_ = nullptr;
#endif
};

// ============================================================
// Convenience aliases
// ============================================================

/// Original KEEP_LAST(1) writer — matches the previous API exactly.
/// All existing call sites require zero changes.
using WriterKL1 = StatefulWriter<1>;

} // namespace rtps
