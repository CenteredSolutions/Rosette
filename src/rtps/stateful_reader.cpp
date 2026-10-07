/**
 * @file rtps/stateful_reader.cpp
 * @brief StatefulReader implementation — §8.4.7–§8.4.9 (OMG formal/2022-04-01).
 */

#include "stateful_reader.hpp"
#include "gap_encoding.hpp"
#include "stateful_writer.hpp"   // ITransport
#include "log.hpp"
#include <string.h>


namespace rtps {

namespace {

/**
 * @brief Derives the ChangeKind of an arriving DATA.
 *
 * The inline QoS StatusInfo says whether a change withdraws its instance:
 * DISPOSED takes precedence over UNREGISTERED, as a disposed instance is
 * also gone for this reader. Without StatusInfo, a DATA whose payload is the
 * serialized key (K flag) is a disposal, and any other DATA is ALIVE.
 *
 * @param[in] msg The DATA submessage.
 * @return ALIVE, NOT_ALIVE_DISPOSED or NOT_ALIVE_UNREGISTERED.
 *
 * Verification:
 * - Correct means: StatusInfo DISPOSED or UNREGISTERED always yields a
 *   NOT_ALIVE kind, whatever the D and K flags.
 * - Covered by: TC-SRR-8.4.8-003, TC-SEDP-8.5.4-009.
 *
 * References:
 * - OMG DDSI-RTPS 2.5 §8.2.1.2 (ChangeKind_t), §9.6.3.3 (StatusInfo_t)
 */
ChangeKind change_kind_of(const ParsedData& msg) noexcept
{
    if (msg.inlineQos.hasStatusInfo) {
        if ((msg.inlineQos.statusInfo & status_info::DISPOSED) != 0u)
            return ChangeKind::NOT_ALIVE_DISPOSED;
        if ((msg.inlineQos.statusInfo & status_info::UNREGISTERED) != 0u)
            return ChangeKind::NOT_ALIVE_UNREGISTERED;
    }
    return msg.keyFlag ? ChangeKind::NOT_ALIVE_DISPOSED : ChangeKind::ALIVE;
}

}  // namespace

StatefulReader::StatefulReader(const Guid& guid, ReliabilityKind reliability, ITransport& transport,
                               WriterProxyPool& pool, uint8_t min_writers, uint8_t max_writers,
                               FragmentAssemblyPool& assembly_pool, uint16_t assembly_limit,
                               OnChangeAvailable on_change_avail, void* user_data,
                               uint32_t hb_response_ms) noexcept
    : guid_(guid),
      reliability_(reliability),
      transport_(transport),
      on_change_avail_(on_change_avail),
      user_data_(user_data),
      pool_(pool),
      assembler_(assembly_pool, assembly_limit),
      hb_response_ms_(hb_response_ms) {
    writers_.floor = min_writers;
    writers_.cap = max_writers;
}

StatefulReader::~StatefulReader() noexcept {
    while (writers_.head != kNoProxy) {
        const ProxyIndex index = writers_.head;
        pool_[index].clear();
        (void)pool_.release(writers_, index);
    }
}

// ============================================================
// §8.4.7.4  matched_writer_add()
// ============================================================

bool StatefulReader::matched_writer_add(const Guid& writer_guid, const Locator* unicast_locators,
                                        uint8_t locator_count,
                                        const VendorId& writer_vendor) noexcept {
    if (find_proxy(writer_guid)) {
        return false;
    }

    const ProxyIndex index = pool_.bind(writers_);
    if (index == kNoProxy) {
        RTPS_LOG_W("reader", "no writer proxy (matched=%u cap=%u pool free=%u reserved=%u)",
                   (unsigned)writers_.count, (unsigned)writers_.cap, (unsigned)pool_.free_count(),
                   (unsigned)pool_.reserved());
        return false;
    }

    WriterProxy* slot = &pool_[index];

    slot->init(writer_guid, unicast_locators, locator_count);
    slot->acknackCount = acknack_count_floor_;
    RTPS_LOG_I("reader", "writer matched %02x%02x%02x%02x entity=%02x%02x%02x%02x",
               writer_guid.prefix.value[0], writer_guid.prefix.value[1],
               writer_guid.prefix.value[2], writer_guid.prefix.value[3],
               writer_guid.entityId.entityKey[0], writer_guid.entityId.entityKey[1],
               writer_guid.entityId.entityKey[2], writer_guid.entityId.entityKind);

    if (reliability_ == ReliabilityKind::RELIABLE) {
        send_preemptive_acknack(*slot, writer_vendor);
    }

    return true;
}

// ============================================================
// §8.4.7.4  matched_writer_remove()
//
// "Removes the WriterProxy from the set of matched writers."
//  — §8.4.7.4, p. 141
//
// The proxy's ACKNACK count raises the reader's floor, so a later match
// with the same writer counts on from it (§8.3.7.1). Any in-progress
// fragment assembly for this writer is discarded so that the pool buffer
// is returned immediately rather than waiting for the slot to be evicted
// by a future alloc.
// ============================================================

void StatefulReader::matched_writer_remove(const Guid& writer_guid) noexcept {
    WriterProxy* proxy = find_proxy(writer_guid);
    if (proxy) {
        if (proxy->acknackCount > acknack_count_floor_) {
            acknack_count_floor_ = proxy->acknackCount;
        }

        proxy->clear();
        (void)pool_.release(writers_, pool_.index_of(*proxy));
    }

    // Release partial assembly buffers for this writer — §8.4.14.
    assembler_.discard_writer(writer_guid);
}

WriterProxy* StatefulReader::matched_writer_lookup(
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
    WriterProxy* proxy = find_proxy_by_sender(
        ctx.senderGuidPrefix, msg.writerId);

    if (!proxy && reliability_ == ReliabilityKind::RELIABLE) {
        RTPS_LOG_D("reader", "DATA from unmatched writer %02x%02x%02x%02x, dropped (RELIABLE)",
                   ctx.senderGuidPrefix.value[0], ctx.senderGuidPrefix.value[1],
                   ctx.senderGuidPrefix.value[2], ctx.senderGuidPrefix.value[3]);
        return;
    }

    if (proxy && !is_new_change(*proxy, msg.writerSN)) {
        RTPS_LOG_T("reader", "DATA SN=%d:%lu from %02x%02x%02x%02x not new, dropped",
                   msg.writerSN.high, (unsigned long)msg.writerSN.low,
                   ctx.senderGuidPrefix.value[0], ctx.senderGuidPrefix.value[1],
                   ctx.senderGuidPrefix.value[2], ctx.senderGuidPrefix.value[3]);
        return;
    }

    const ChangeKind kind = change_kind_of(msg);

    // A DATA without payload or key (inline QoS only) carries nothing to
    // deliver unless it withdraws an instance; its sequence number is
    // accounted for either way so it is never requested again.
    if (!msg.dataFlag && !msg.keyFlag && kind == ChangeKind::ALIVE) {
        if (proxy) account_undelivered(*proxy, msg.writerSN);
        return;
    }

    // A payload no pool block can hold is never deliverable; see
    // account_undelivered().
    if (msg.serializedPayloadSize > kPoolMaxBlockSize) {
        RTPS_LOG_W("reader", "DATA SN=%d:%lu of %lu octets exceeds the largest pool block, lost",
                   msg.writerSN.high, (unsigned long)msg.writerSN.low,
                   (unsigned long)msg.serializedPayloadSize);
        if (proxy) account_undelivered(*proxy, msg.writerSN);
        return;
    }

    // Copy the payload into the smallest pool class that fits it. A
    // disposal sent as inline QoS alone has no payload to copy.
    BufRef payload{};
    uint32_t copy_size = 0;
    if (msg.dataFlag || msg.keyFlag) {
        payload = buf_alloc(msg.serializedPayloadSize);
        if (!payload.valid()) {
            RTPS_LOG_E("reader", "buf_alloc failed for DATA payload size=%lu",
                       (unsigned long)msg.serializedPayloadSize);
            return;
        }
        copy_size = (msg.serializedPayloadSize <= payload.capacity())
                    ? msg.serializedPayloadSize : payload.capacity();
        if (copy_size > 0 && msg.serializedPayload)
            memcpy(payload.data, msg.serializedPayload, copy_size);
        payload.size = copy_size;
    }

    CacheChange change{};
    change.kind            = kind;
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
        const bool repair = proxy->is_missing(msg.writerSN);
        record_change(*proxy, msg.writerSN);
        deliver(change);
        if (repair) request_next_repair(*proxy);
    } else {
        deliver(change);
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

    WriterProxy* proxy = find_proxy(writer_guid);
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

    // A fragment of a sample already delivered (the writer resends whole
    // samples on NACK_FRAG) must not assemble and deliver it again.
    if (proxy && !is_new_change(*proxy, msg.writerSN)) {
        assembler_.discard(writer_guid, msg.writerSN);
        return;
    }

    // Every fragment carries the sample's total size, so a sample no pool
    // block can hold is recognized on the first fragment that arrives.
    if (msg.dataSize > kPoolMaxBlockSize) {
        RTPS_LOG_W("reader", "DATA_FRAG SN=%d:%lu of %lu octets exceeds the largest pool block, lost",
                   msg.writerSN.high, (unsigned long)msg.writerSN.low,
                   (unsigned long)msg.dataSize);
        if (proxy) account_undelivered(*proxy, msg.writerSN);
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
        out_change,
        msg.inlineQosPresent ? &msg.inlineQos : nullptr);

    if (!complete) return;

    // Sample is complete — treat identically to a full DATA reception.
    if (proxy) {
        const bool repair = proxy->is_missing(msg.writerSN);
        record_change(*proxy, msg.writerSN);
        deliver(out_change);
        if (repair) request_next_repair(*proxy);
    } else {
        deliver(out_change);
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
    WriterProxy* proxy = find_proxy_by_sender(
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
        proxy->on_acknack_sent();
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
    WriterProxy* proxy = find_proxy(writer_guid);
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
    WriterProxy* proxy = find_proxy_by_sender(
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
    assembler_.discard_range(writer_guid, msg.gapStart,
                             gap_run_end(msg.gapStart, msg.gapList));
    msg.gapList.for_each([&](const SequenceNumber& sn) noexcept {
        assembler_.discard(writer_guid, sn);
    });

    const bool had_missing = proxy->has_missing_changes();
    proxy->irrelevant_change_set(msg.gapStart, msg.gapList);
    if (had_missing) request_next_repair(*proxy);

    // If all previously missing changes are now LOST, send a pure ACK.
    if (proxy->acknackPending && !proxy->has_missing_changes()) {
        if (hb_response_ms_ == 0) {
            send_acknack(*proxy, /*final=*/true);
            proxy->on_acknack_sent();
        }
    }
}

// ============================================================
// §8.4.7.3  send_acknack()
//
// "Sends an ACKNACK to the matched writer."  — §8.4.7.3, p. 140
// ============================================================

void StatefulReader::send_acknack(WriterProxy& proxy,
                                  bool         final_flag) noexcept
{
    const Locator& dest = proxy.best_locator();
    if (dest == LOCATOR_INVALID) return;

    SequenceNumberSet missing{};
    proxy.missing_changes(missing, narrow_repairs() ? 1u : SequenceNumberSet::kMaxBits);

    // A MISSING change with fragments already here is requested fragment by
    // fragment (NACK_FRAG) and left out of the ACKNACK, which would have the
    // writer resend every fragment, including the ones that arrived. The
    // bitmap keeps its base, so the cumulative acknowledgement is unchanged.
    SequenceNumberSet sn_state{};
    sn_state.bitmapBase = missing.bitmapBase;
    SequenceNumber partial[kMaxNackFragsPerAcknack]{};
    FragmentNumberSet partial_missing[kMaxNackFragsPerAcknack]{};
    uint8_t partial_count = 0;
    missing.for_each([&](const SequenceNumber& sn) noexcept {
        if (partial_count < kMaxNackFragsPerAcknack
            && assembler_.missing_fragments(proxy.remoteWriterGuid, sn, UINT32_MAX,
                                            partial_missing[partial_count])
            && !partial_missing[partial_count].empty()) {
            partial[partial_count++] = sn;
            return;
        }

        (void)sn_state.set(static_cast<uint32_t>(sn - missing.bitmapBase));
    });
    if (sn_state.empty() && partial_count == 0u) {
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
    for (uint8_t i = 0; ok && i < partial_count; ++i) {
        ok = builder_.append_nack_frag(guid_.entityId, proxy.remoteWriterGuid.entityId,
                                       partial[i], partial_missing[i], proxy.acknackCount + 1);
    }

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

bool StatefulReader::narrow_repairs() const noexcept
{
    return repairs_must_fit_datagram(transport_);
}

void StatefulReader::send_preemptive_acknack(WriterProxy& proxy,
                                             const VendorId& writer_vendor) noexcept {
    const Locator& dest = proxy.best_locator();
    if (dest == LOCATOR_INVALID || !builder_.begin(guid_.prefix)) {
        return;
    }

    const bool zero_base =
        writer_vendor == VENDORID_EPROSIMA || writer_vendor == VENDORID_RTI_CONNEXT;
    SequenceNumberSet sn_state{};
    sn_state.bitmapBase = zero_base ? SEQUENCENUMBER_ZERO : SequenceNumber::from_int64(1);

    builder_.append_info_dst(proxy.remoteWriterGuid.prefix);
    if (!builder_.append_acknack(guid_.entityId, proxy.remoteWriterGuid.entityId, sn_state,
                                 proxy.acknackCount + 1, /*final_flag=*/false)) {
        RTPS_LOG_W("reader", "preemptive ACKNACK builder failed (pool exhausted?)");
        builder_.abort();
        return;
    }

    BufRef buf = builder_.finish();
    transport_.send_unicast(buf, dest);
    buf.reset();
    proxy.on_acknack_sent();
}

void StatefulReader::request_next_repair(WriterProxy& proxy) noexcept
{
    if (!narrow_repairs() || !proxy.has_missing_changes()) return;
    send_acknack(proxy, /*final=*/false);
    proxy.on_acknack_sent();
}

// ============================================================
// tick()
// ============================================================

void StatefulReader::tick(uint32_t now_ms) noexcept {
    for (WriterProxy& proxy : matched()) {
        if (proxy.acknack_response_due(now_ms)) {
            send_acknack(proxy, !proxy.has_missing_changes());
            proxy.on_acknack_sent();
        }
    }
}

// ============================================================
// Internal helpers
// ============================================================

uint32_t StatefulReader::ms_until_next_timer(uint32_t now_ms) const noexcept {
    uint32_t next = UINT32_MAX;
    for (const WriterProxy& proxy : matched()) {
        if (!proxy.acknackPending) {
            continue;
        }

        // Wrap-safe "is the deadline in the past" test, as in acknack_response_due().
        const uint32_t left = proxy.heartbeatResponseDeadlineMs - now_ms;
        const uint32_t wait = (left > 0x80000000u) ? 0u : left;
        if (wait < next) {
            next = wait;
        }
    }
    return next;
}

WriterProxy* StatefulReader::find_proxy(const Guid& guid) noexcept {
    for (WriterProxy& proxy : matched()) {
        if (proxy.remoteWriterGuid == guid) {
            return &proxy;
        }
    }

    return nullptr;
}

WriterProxy* StatefulReader::find_proxy_by_sender(
        const GuidPrefix& prefix, const EntityId& writer_eid) noexcept
{
    return find_proxy(Guid{ prefix, writer_eid });
}

bool StatefulReader::is_new_change(const WriterProxy&    proxy,
                                   const SequenceNumber& sn) const noexcept
{
    if (reliability_ == ReliabilityKind::RELIABLE) return proxy.is_new_change(sn);
    return proxy.available_changes_max() < sn;
}

void StatefulReader::record_change(WriterProxy& proxy, const SequenceNumber& sn) noexcept
{
    if (reliability_ != ReliabilityKind::RELIABLE) proxy.lost_changes_update(sn);
    proxy.received_change_set(sn);
}

void StatefulReader::account_undelivered(WriterProxy& proxy, const SequenceNumber& sn) noexcept
{
    const bool repair = proxy.is_missing(sn);
    record_change(proxy, sn);
    if (repair) request_next_repair(proxy);
}

void StatefulReader::deliver(const CacheChange& change) noexcept
{
    if (on_change_avail_) on_change_avail_(change, user_data_);
}

} // namespace rtps
