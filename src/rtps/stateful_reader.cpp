/**
 * @file rtps/stateful_reader.cpp
 * @brief StatefulReader implementation — §8.4.7–§8.4.9 (OMG formal/2022-04-01).
 */

#include "stateful_reader.hpp"
#include "stateful_writer.hpp"   // ITransport
#include "log.hpp"
#include <string.h>


namespace rtps {

// ============================================================
// Constructor / Destructor
// ============================================================

StatefulReader::StatefulReader(const Guid&       guid,
                               ReliabilityKind   reliability,
                               ITransport&       transport,
                               OnChangeAvailable on_change_avail,
                               void*             user_data,
                               uint32_t          hb_response_ms) noexcept
    : guid_(guid)
    , reliability_(reliability)
    , transport_(transport)
    , on_change_avail_(on_change_avail)
    , user_data_(user_data)
    , hb_response_ms_(hb_response_ms)
{
}

StatefulReader::~StatefulReader() noexcept {
    // assembler_ destructor clears all FragmentedChange slots and
    // releases their pool buffers — no explicit call needed here.
}

// ============================================================
// §8.4.7.4  matched_writer_add()
// ============================================================

bool StatefulReader::matched_writer_add(const Guid&    writer_guid,
                                        const Locator* unicast_locators,
                                        uint8_t        locator_count) noexcept
{
    if (find_proxy(writer_guid)) return false;
    DefaultWriterProxy* slot = find_free_slot();
    if (!slot) {
        RTPS_LOG_W("reader", "writer proxy table full (max=%d)",
                   RTPS_MAX_MATCHED_WRITERS);
        return false;
    }
    slot->init(writer_guid, unicast_locators, locator_count);
    RTPS_LOG_I("reader", "writer matched %02x%02x%02x%02x entity=%02x%02x%02x%02x",
               writer_guid.prefix.value[0], writer_guid.prefix.value[1],
               writer_guid.prefix.value[2], writer_guid.prefix.value[3],
               writer_guid.entityId.entityKey[0], writer_guid.entityId.entityKey[1],
               writer_guid.entityId.entityKey[2], writer_guid.entityId.entityKind);
    return true;
}

// ============================================================
// §8.4.7.4  matched_writer_remove()
//
// "Removes the WriterProxy from the set of matched writers."
//  — §8.4.7.4, p. 141
//
// Discard any in-progress fragment assembly for this writer so that
// the pool buffer is returned immediately rather than waiting for the
// slot to be evicted by a future alloc.
// ============================================================

void StatefulReader::matched_writer_remove(const Guid& writer_guid) noexcept {
    DefaultWriterProxy* proxy = find_proxy(writer_guid);
    if (proxy) proxy->clear();

    // Release partial assembly buffers for this writer — §8.4.14.
    assembler_.discard_writer(writer_guid);
}

DefaultWriterProxy* StatefulReader::matched_writer_lookup(
        const Guid& writer_guid) noexcept
{
    return find_proxy(writer_guid);
}

// ============================================================
// §8.4.8  on_data()
//
// "Process an incoming DATA submessage."  — §8.4.8, p. 142
// ============================================================

void StatefulReader::on_data(const ParseContext& ctx,
                             const ParsedData&   msg) noexcept
{
    DefaultWriterProxy* proxy = find_proxy_by_sender(
        ctx.senderGuidPrefix, msg.writerId);

    if (!proxy && reliability_ == ReliabilityKind::RELIABLE) {
        RTPS_LOG_D("reader", "DATA from unmatched writer %02x%02x%02x%02x, dropped (RELIABLE)",
                   ctx.senderGuidPrefix.value[0], ctx.senderGuidPrefix.value[1],
                   ctx.senderGuidPrefix.value[2], ctx.senderGuidPrefix.value[3]);
        return;
    }

    // Duplicate check — §8.4.8, p. 142.
    if (proxy && msg.writerSN <= proxy->highestSNReceived) {
        RTPS_LOG_T("reader", "duplicate DATA SN=%d:%lu from %02x%02x%02x%02x, dropped",
                   msg.writerSN.high, (unsigned long)msg.writerSN.low,
                   ctx.senderGuidPrefix.value[0], ctx.senderGuidPrefix.value[1],
                   ctx.senderGuidPrefix.value[2], ctx.senderGuidPrefix.value[3]);
        return;
    }

    if (!msg.dataFlag && !msg.keyFlag) return;

    // Allocate the smallest pool class that fits the payload.
    BufRef payload = buf_alloc(msg.serializedPayloadSize);
    if (!payload.valid()) {
        RTPS_LOG_E("reader", "buf_alloc failed for DATA payload size=%lu",
                   (unsigned long)msg.serializedPayloadSize);
        return;
    }

    const uint32_t copy_size =
        (msg.serializedPayloadSize <= payload.capacity())
        ? msg.serializedPayloadSize : payload.capacity();

    if (copy_size > 0 && msg.serializedPayload)
        memcpy(payload.data, msg.serializedPayload, copy_size);
    payload.size = copy_size;

    CacheChange change{};
    change.kind            = msg.keyFlag ? ChangeKind::NOT_ALIVE_DISPOSED
                                         : ChangeKind::ALIVE;
    change.writerGuid      = Guid{ ctx.senderGuidPrefix, msg.writerId };
    change.sequenceNumber  = msg.writerSN;
    change.sourceTimestamp = ctx.sourceTimestamp;
    change.inlineQos       = msg.inlineQos;
    change.payload         = static_cast<BufRef&&>(payload);

    RTPS_LOG_D("reader", "DATA accepted SN=%d:%lu from %02x%02x%02x%02x size=%lu",
               msg.writerSN.high, (unsigned long)msg.writerSN.low,
               ctx.senderGuidPrefix.value[0], ctx.senderGuidPrefix.value[1],
               ctx.senderGuidPrefix.value[2], ctx.senderGuidPrefix.value[3],
               (unsigned long)copy_size);

    if (proxy) {
        proxy->received_change_set(msg.writerSN);
        accept_change(static_cast<CacheChange&&>(change), *proxy);
    } else {
        reader_cache_.add_change(static_cast<CacheChange&&>(change));
    }
}

// ============================================================
// §8.4.8  on_data_frag()
//
// "Process an incoming DATA_FRAG submessage."  — §8.4.8, p. 142
//
// Delegates to FragmentAssembler::on_data_frag() (§8.4.14, p. 156).
// When the assembler returns true the sample is complete and
// out_change holds the fully assembled CacheChange.
// ============================================================

void StatefulReader::on_data_frag(const ParseContext&   ctx,
                                  const ParsedDataFrag& msg) noexcept
{
    const Guid writer_guid{ ctx.senderGuidPrefix, msg.writerId };

    DefaultWriterProxy* proxy = find_proxy(writer_guid);
    if (!proxy && reliability_ == ReliabilityKind::RELIABLE) {
        RTPS_LOG_D("reader", "DATA_FRAG from unmatched writer, dropped (RELIABLE)");
        return;
    }

    if (msg.fragmentStartingNum == 0 || msg.fragmentSize == 0 ||
        msg.dataSize == 0) {
        RTPS_LOG_W("reader", "DATA_FRAG invalid fields: startFrag=%lu fragSize=%lu dataSize=%lu",
                   (unsigned long)msg.fragmentStartingNum,
                   (unsigned long)msg.fragmentSize,
                   (unsigned long)msg.dataSize);
        return;
    }

    RTPS_LOG_T("reader", "DATA_FRAG SN=%d:%lu frag=%lu/%lu size=%lu",
               msg.writerSN.high, (unsigned long)msg.writerSN.low,
               (unsigned long)msg.fragmentStartingNum,
               (unsigned long)msg.fragmentsInSubmessage,
               (unsigned long)msg.dataSize);

    // Attempt to deliver this fragment to the assembler.
    // §8.4.14, p. 156: on_data_frag() returns true when the sample
    // is complete; out_change is populated with the assembled payload.
    CacheChange out_change{};
    const bool complete = assembler_.on_data_frag(
        writer_guid,
        msg.writerSN,
        msg.fragmentStartingNum,
        msg.fragmentsInSubmessage,
        msg.fragmentSize,
        msg.dataSize,
        msg.fragmentPayload,
        msg.fragmentPayloadSize,
        ctx.sourceTimestamp,
        out_change);

    if (!complete) return;

    // Sample is complete — treat identically to a full DATA reception.
    if (proxy) {
        proxy->received_change_set(msg.writerSN);
        accept_change(static_cast<CacheChange&&>(out_change), *proxy);
    } else {
        reader_cache_.add_change(static_cast<CacheChange&&>(out_change));
    }
}

// ============================================================
// §8.4.9  on_heartbeat()
//
// "Process an incoming HEARTBEAT submessage."  — §8.4.9, p. 145
//
// FinalFlag rule — §8.3.7.5, p. 91:
//   F=0 → MUST send ACKNACK.
//   F=1 + missing changes → MUST send ACKNACK (with NACK bitmap).
//   F=1 + up-to-date → MAY suppress ACKNACK.
// ============================================================

void StatefulReader::on_heartbeat(const ParseContext&    ctx,
                                  const ParsedHeartbeat& msg) noexcept
{
    DefaultWriterProxy* proxy = find_proxy_by_sender(
        ctx.senderGuidPrefix, msg.writerId);
    if (!proxy) {
        RTPS_LOG_T("reader", "HEARTBEAT from unknown writer %02x%02x%02x%02x, ignored",
                   ctx.senderGuidPrefix.value[0], ctx.senderGuidPrefix.value[1],
                   ctx.senderGuidPrefix.value[2], ctx.senderGuidPrefix.value[3]);
        return;
    }

    if (msg.readerId != entity_id::UNKNOWN &&
        msg.readerId != guid_.entityId) return;

    const uint32_t now_ms = rosette::platform::monotonic_ms();
    const bool needs_acknack = proxy->on_heartbeat(
        msg.firstSN, msg.lastSN, msg.count,
        msg.finalFlag, now_ms, hb_response_ms_);

    RTPS_LOG_D("reader", "HEARTBEAT first=%d:%lu last=%d:%lu final=%d needs_ack=%d",
               msg.firstSN.high, (unsigned long)msg.firstSN.low,
               msg.lastSN.high,  (unsigned long)msg.lastSN.low,
               (int)msg.finalFlag, (int)needs_acknack);

    if (needs_acknack && hb_response_ms_ == 0) {
        send_acknack(*proxy, !proxy->has_missing_changes());
        proxy->on_acknack_sent(proxy->highestSNReceived);
    }
}

// ============================================================
// §8.4.9 / §8.3.7.4  on_heartbeat_frag()
//
// "Process an incoming HEARTBEAT_FRAG submessage."
//  — §8.3.7.4, p. 90; §8.4.9, p. 145
//
// The writer is advertising that it has sent fragments up to
// lastFragmentNum for the given writerSN.  We ask the assembler
// which fragments we are missing and send a NACK_FRAG if any.
//
// §8.4.12, p. 153:
//   "The Reader sends a NACK_FRAG for any fragment numbers that
//    it has not received."
// ============================================================

void StatefulReader::on_heartbeat_frag(const ParseContext&       ctx,
                                       const ParsedHeartbeatFrag& msg) noexcept
{
    const Guid writer_guid{ ctx.senderGuidPrefix, msg.writerId };
    DefaultWriterProxy* proxy = find_proxy(writer_guid);
    if (!proxy) return;

    if (msg.readerId != entity_id::UNKNOWN &&
        msg.readerId != guid_.entityId) return;

    // Ask the assembler for the missing fragment set.
    FragmentNumberSet missing{};
    const bool has_slot = assembler_.missing_fragments(
        writer_guid, msg.writerSN, msg.lastFragmentNum, missing);

    if (!has_slot || missing.empty()) return;

    // Send NACK_FRAG — §9.4.5.8, p. 218.
    const Locator& dest = proxy->best_locator();
    if (dest == LOCATOR_INVALID) return;

    if (!builder_.begin(guid_.prefix)) return;

    bool ok = builder_.append_nack_frag(
        guid_.entityId,
        msg.writerId,
        msg.writerSN,
        missing,
        ++proxy->acknackCount);

    if (ok) {
        BufRef buf = builder_.finish();
        transport_.send_unicast(buf, dest);
        buf.reset();
    } else {
        builder_.abort();
    }
}

// ============================================================
// §8.4.9  on_gap()
//
// "Process an incoming GAP submessage."  — §8.4.9, p. 145
//
// §9.4.5.11, p. 258: irrelevant SNs:
//   [gapStart .. gapList.bitmapBase - 1]
//   ∪ { individual SNs in gapList bitmap }
//
// For each irrelevant SN that has a partial assembly in progress,
// we discard the assembler slot — those fragments will never arrive.
// ============================================================

void StatefulReader::on_gap(const ParseContext& ctx,
                            const ParsedGap&   msg) noexcept
{
    DefaultWriterProxy* proxy = find_proxy_by_sender(
        ctx.senderGuidPrefix, msg.writerId);
    if (!proxy) {
        RTPS_LOG_T("reader", "GAP from unknown writer, ignored");
        return;
    }

    if (msg.readerId != entity_id::UNKNOWN &&
        msg.readerId != guid_.entityId) return;

    RTPS_LOG_D("reader", "GAP start=%d:%lu from %02x%02x%02x%02x",
               msg.gapStart.high, (unsigned long)msg.gapStart.low,
               ctx.senderGuidPrefix.value[0], ctx.senderGuidPrefix.value[1],
               ctx.senderGuidPrefix.value[2], ctx.senderGuidPrefix.value[3]);

    const Guid writer_guid{ ctx.senderGuidPrefix, msg.writerId };

    // Discard any partial assemblies for SNs declared irrelevant.
    // Contiguous range [gapStart .. bitmapBase - 1].
    if (!msg.gapList.empty()) {
        for (SequenceNumber sn = msg.gapStart;
             sn < msg.gapList.bitmapBase;
             sn += 1)
        {
            assembler_.discard(writer_guid, sn);
        }
    } else {
        assembler_.discard(writer_guid, msg.gapStart);
    }
    // Individual SNs in the bitmap.
    msg.gapList.for_each([&](const SequenceNumber& sn) noexcept {
        assembler_.discard(writer_guid, sn);
    });

    proxy->on_gap(msg.gapStart, msg.gapList);

    // If all previously missing changes are now LOST, send a pure ACK.
    if (proxy->acknackPending && !proxy->has_missing_changes()) {
        if (hb_response_ms_ == 0) {
            send_acknack(*proxy, /*final=*/true);
            proxy->on_acknack_sent(proxy->highestSNReceived);
        }
    }
}

// ============================================================
// §8.4.7.3  send_acknack()
//
// "Sends an ACKNACK to the matched writer."  — §8.4.7.3, p. 140
// ============================================================

void StatefulReader::send_acknack(DefaultWriterProxy& proxy,
                                  bool                final_flag) noexcept
{
    const Locator& dest = proxy.best_locator();
    if (dest == LOCATOR_INVALID) return;

    SequenceNumberSet sn_state{};
    proxy.missing_changes(sn_state);

    if (sn_state.numBits == 0) {
        sn_state.bitmapBase = proxy.highestSNReceived + 1;
        final_flag = true;
    }

    if (!builder_.begin(guid_.prefix)) return;

    builder_.append_info_dst(proxy.remoteWriterGuid.prefix);

    bool ok = builder_.append_acknack(
        guid_.entityId,
        proxy.remoteWriterGuid.entityId,
        sn_state,
        proxy.acknackCount + 1,
        final_flag);

    if (ok) {
        RTPS_LOG_D("reader", "ACKNACK sent base=%d:%lu final=%d",
                   sn_state.bitmapBase.high, (unsigned long)sn_state.bitmapBase.low,
                   (int)final_flag);
        BufRef buf = builder_.finish();
        transport_.send_unicast(buf, dest);
        buf.reset();
    } else {
        RTPS_LOG_W("reader", "ACKNACK builder failed (pool exhausted?)");
        builder_.abort();
    }
}

// ============================================================
// tick()
// ============================================================

void StatefulReader::tick(uint32_t now_ms) noexcept {
    for (uint8_t i = 0; i < RTPS_MAX_MATCHED_WRITERS; ++i) {
        DefaultWriterProxy& proxy = proxies_[i];
        if (!proxy.is_matched()) continue;

        if (proxy.acknack_response_due(now_ms)) {
            send_acknack(proxy, !proxy.has_missing_changes());
            proxy.on_acknack_sent(proxy.highestSNReceived);
        }

        proxy.evict_stale(proxy.highestSNAcknowledged);
    }
}

// ============================================================
// Internal helpers
// ============================================================

uint32_t StatefulReader::ms_until_next_timer(uint32_t now_ms) const noexcept {
    uint32_t next = UINT32_MAX;
    for (uint8_t i = 0; i < RTPS_MAX_MATCHED_WRITERS; ++i) {
        const DefaultWriterProxy& proxy = proxies_[i];
        if (!proxy.is_matched() || !proxy.acknackPending) continue;
        // Wrap-safe "is the deadline in the past" test, as in acknack_response_due().
        const uint32_t left = proxy.heartbeatResponseDeadlineMs - now_ms;
        const uint32_t wait = (left > 0x80000000u) ? 0u : left;
        if (wait < next) next = wait;
    }
    return next;
}

uint8_t StatefulReader::matched_writer_count() const noexcept {
    uint8_t n = 0;
    for (uint8_t i = 0; i < RTPS_MAX_MATCHED_WRITERS; ++i)
        if (proxies_[i].is_matched()) ++n;
    return n;
}

DefaultWriterProxy* StatefulReader::find_proxy(const Guid& guid) noexcept {
    for (uint8_t i = 0; i < RTPS_MAX_MATCHED_WRITERS; ++i)
        if (proxies_[i].is_matched() &&
            proxies_[i].remoteWriterGuid == guid)
            return &proxies_[i];
    return nullptr;
}

DefaultWriterProxy* StatefulReader::find_proxy_by_sender(
        const GuidPrefix& prefix, const EntityId& writer_eid) noexcept
{
    return find_proxy(Guid{ prefix, writer_eid });
}

DefaultWriterProxy* StatefulReader::find_free_slot() noexcept {
    for (uint8_t i = 0; i < RTPS_MAX_MATCHED_WRITERS; ++i)
        if (!proxies_[i].is_matched()) return &proxies_[i];
    return nullptr;
}

void StatefulReader::accept_change(CacheChange&&       change,
                                   DefaultWriterProxy& /*proxy*/) noexcept
{
    // Fire callback before moving into cache: the application sees
    // the payload while the BufRef is still in the caller's hand.
    // If the application needs the data beyond the callback it must
    // call change.payload.retain() before returning.
    if (on_change_avail_) on_change_avail_(change, user_data_);
    reader_cache_.add_change(static_cast<CacheChange&&>(change));
}

} // namespace rtps
