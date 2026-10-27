/**
 * @file stateful_writer.cpp
 * @brief StatefulWriter behavior (§8.4.1–§8.4.9), compiled once for every depth.
 *
 * See rtps/stateful_writer.hpp. The writer's history belongs to
 * SharedPoolStatefulWriter<Depth> or StatefulWriterStorage<Depth, MaxReaders>,
 * and its proxies to a ReaderProxyPool; this file sees them only through the
 * references bound at construction and the writer's list of bound proxies.
 */

#include "rosette/rtps/stateful_writer.hpp"

#include <string.h>

#include "rosette/rtps/gap_encoding.hpp"
#include "rosette/rtps/log.hpp"

namespace rosette::rtps {
namespace {

/// Octets of the CDR encapsulation header (§10.2) prepended to samples that
/// are not preframed.
constexpr uint16_t kEncapsulationSize = 4u;

}  // namespace

StatefulWriter::StatefulWriter(const Guid& guid, ReliabilityKind reliability,
                               DurabilityKind durability, ITransport& transport,
                               HistoryCache& history, ReaderProxyPool& pool, uint8_t min_readers,
                               uint8_t max_readers, uint32_t nack_delay_ms,
                               uint32_t hb_period_ms) noexcept
    : guid_(guid),
      reliability_(reliability),
      durability_(durability),
      transport_(transport),
      writer_cache_(history),
      pool_(pool),
      nack_delay_ms_(nack_delay_ms),
      hb_period_ms_(hb_period_ms) {
    readers_.floor = min_readers;
    readers_.cap = max_readers;
}

StatefulWriter::~StatefulWriter() noexcept {
    while (readers_.head != kNoProxy) {
        const ProxyIndex index = readers_.head;
        pool_[index].clear();
        (void)pool_.release(readers_, index);
    }
}

CacheChange* StatefulWriter::new_change(ChangeKind kind, BufRef& out_payload,
                                        size_t payload_size) noexcept {
    if (staging_.payload.valid())
        staging_.payload.reset();

    out_payload = buf_alloc(static_cast<uint32_t>(payload_size));
    if (!out_payload.valid()) {
        RTPS_LOG_E("writer", "buf_alloc failed (%zu) for new_change (pool exhausted)", payload_size);
        return nullptr;
    }
    staging_.kind           = kind;
    staging_.writerGuid     = guid_;
    staging_.sequenceNumber = SEQUENCENUMBER_UNKNOWN;  // Assigned in add_change.
    staging_.inlineQos      = {};
    staging_.preframed      = false;
    staging_.payload.block  = out_payload.block;
    staging_.payload.data   = out_payload.data;
    staging_.payload.size   = out_payload.size;
    out_payload.retain();
    return &staging_;
}

bool StatefulWriter::add_change(CacheChange&& change) noexcept {
    change.sequenceNumber = nextSN_;
    ++nextSN_;

    // A full history evicts its oldest change (KEEP_LAST, §8.3.3.1). Every
    // matched proxy has been sent less than the new change, so it is UNSENT
    // everywhere without touching the proxies.
    if (writer_cache_.size() == writer_cache_.capacity()) {
        note_eviction();
    }

    CacheChange* inserted = writer_cache_.add_change(static_cast<CacheChange&&>(change));
    if (!inserted) {
        RTPS_LOG_E("writer", "cache add_change failed (SN=%ld:%lu)", change.sequenceNumber.high,
                   static_cast<unsigned long>(change.sequenceNumber.low));
        return false;
    }

    RTPS_LOG_D("writer", "add_change SN=%ld:%lu size=%lu depth=%d", inserted->sequenceNumber.high,
               static_cast<unsigned long>(inserted->sequenceNumber.low),
               static_cast<unsigned long>(inserted->payload.size), writer_cache_.size());
    if (reliability_ == ReliabilityKind::RELIABLE) {
        ++writesSinceHb_;
    }

    return true;
}

void StatefulWriter::remove_change(const SequenceNumber& sn) noexcept {
    writer_cache_.remove_change(sn);
}

bool StatefulWriter::write_async(ChangeKind kind, const Octet* payload_data, uint32_t payload_size,
                                 const InlineQos& inline_qos, SequenceNumber* out_sn) noexcept {
    if (!payload_data || payload_size == 0) return false;

    BufRef payload_buf;
    CacheChange* ch = new_change(kind, payload_buf, payload_size);
    if (!ch) {
        RTPS_LOG_E("writer", "write_async: new_change failed");
        return false;
    }

    const uint32_t sz = (payload_size < payload_buf.capacity()) ? payload_size : payload_buf.capacity();
    memcpy(payload_buf.data, payload_data, sz);
    payload_buf.size = sz;
    ch->payload.size = sz;
    ch->inlineQos    = inline_qos;

    if (!add_change(static_cast<CacheChange&&>(*ch))) {
        payload_buf.reset();
        return false;
    }
    if (out_sn) *out_sn = nextSN_ - 1;
    send_unsent_changes(heartbeat_due());
    return true;
}

bool StatefulWriter::write_async_owned(ChangeKind kind, BufRef&& payload,
                                       const InlineQos& inline_qos,
                                       SequenceNumber* out_sn) noexcept {
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
    send_unsent_changes(heartbeat_due());
    return true;
}

bool StatefulWriter::write_preframed_async(ChangeKind kind, const Octet* serialized_payload,
                                           uint32_t payload_size, SequenceNumber* out_sn,
                                           const InlineQos& inline_qos) noexcept {
    if (!serialized_payload || payload_size == 0) return false;
    if (staging_.payload.valid()) staging_.payload.reset();

    // Headroom covers the RTPS/DATA headers plus the largest inline QoS, and
    // tailroom one HEARTBEAT, so send_data_to() can always frame this change
    // in place with a piggybacked HEARTBEAT.
    BufRef payload_buf = buf_alloc_with_headroom(payload_size + kRtpsHeartbeatTailroom,
                                                 kRtpsDataHeadroom + kMaxInlineQosSize);
    if (!payload_buf.valid()) {
        RTPS_LOG_E("writer", "write_preframed_async: buf_alloc_with_headroom failed");
        return false;
    }

    const uint32_t sz = (payload_size < payload_buf.capacity()) ? payload_size : payload_buf.capacity();
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
    if (out_sn) *out_sn = nextSN_ - 1;
    send_unsent_changes(heartbeat_due());
    return true;
}

// Cold, like send_built_data_to(): on-demand changes are SEDP announcements,
// written once per endpoint and sent on discovery and repair, so this path is
// optimized for flash rather than speed.
[[gnu::cold]] bool StatefulWriter::write_on_demand_async(ChangeKind kind,
                                                         const InlineQos& inline_qos,
                                                         SequenceNumber* out_sn) noexcept {
    if (payloadBuilder_ == nullptr) {
        RTPS_LOG_E("writer", "write_on_demand_async: no payload builder installed");
        return false;
    }

    if (staging_.payload.valid()) {
        staging_.payload.reset();
    }

    staging_.kind = kind;
    staging_.writerGuid = guid_;
    staging_.sequenceNumber = SEQUENCENUMBER_UNKNOWN;
    staging_.inlineQos = inline_qos;
    staging_.preframed = true;
    if (!add_change(static_cast<CacheChange&&>(staging_))) {
        return false;
    }

    if (out_sn) {
        *out_sn = nextSN_ - 1;
    }

    send_unsent_changes(heartbeat_due());
    return true;
}

bool StatefulWriter::matched_reader_add(const Guid& reader_guid, const Locator* unicast_locators,
                                        uint8_t locator_count, bool expects_inline_qos,
                                        ReliabilityKind reader_reliability) noexcept {
    if (find_proxy(reader_guid)) {
        return false;
    }

    const ProxyIndex index = pool_.bind(readers_);
    if (index == kNoProxy) {
        RTPS_LOG_W("writer", "no reader proxy (matched=%u cap=%u pool free=%u reserved=%u)",
                   static_cast<unsigned>(readers_.count), static_cast<unsigned>(readers_.cap),
                   static_cast<unsigned>(pool_.free_count()),
                   static_cast<unsigned>(pool_.reserved()));
        return false;
    }

    ReaderProxy* slot = &pool_[index];
    const bool   reader_reliable = reader_reliability == ReliabilityKind::RELIABLE;
    slot->init(reader_guid, unicast_locators, locator_count, expects_inline_qos,
               first_relevant_for_new_reader(), reader_reliable);

    RTPS_LOG_I("writer", "reader matched %02x%02x%02x%02x entity=%02x%02x%02x%02x%s",
               reader_guid.prefix.value[0], reader_guid.prefix.value[1],
               reader_guid.prefix.value[2], reader_guid.prefix.value[3],
               reader_guid.entityId.entityKey[0], reader_guid.entityId.entityKey[1],
               reader_guid.entityId.entityKey[2], reader_guid.entityId.entityKind,
               reader_reliable ? "" : " (best-effort)");

    // TRANSIENT_LOCAL: the cached changes go to the new reader from tick(),
    // a budget at a time, rather than waiting for the next write (§8.7.2.2).
    if (durability_ == DurabilityKind::TRANSIENT_LOCAL && slot->has_unsent_changes(writer_cache_)) {
        slot->replaying = true;
        replayPending_ = true;
    }

    return true;
}

void StatefulWriter::matched_reader_remove(const Guid& reader_guid) noexcept {
    ReaderProxy* p = find_proxy(reader_guid);
    if (p) {
        if (fragRepair_.proxy == p) {
            fragRepair_ = FragmentRepair{};
        }

        p->clear();
        (void)pool_.release(readers_, pool_.index_of(*p));
    }

    // A reader that was still waiting for a proxy holds none, yet replies
    // were kept for it (reply_delivered()), so its requester's cleanup runs
    // whether or not a proxy was bound.
    if (releaseRepliesOnDelivery_) {
        drop_replies_to_departed(reader_guid.prefix);
    }

    if (p || releaseRepliesOnDelivery_) {
        release_finished_changes();
    }
}

void StatefulWriter::on_waiting_reader_bound() noexcept {
    if (releaseRepliesOnDelivery_) {
        release_finished_changes();
    }
}

void StatefulWriter::drop_replies_to_departed(const GuidPrefix& requester) noexcept {
    for (const ReaderProxy& proxy : matched()) {
        if (proxy.remoteReaderGuid.prefix == requester) {
            return;
        }
    }

    if (requester_has_waiting_reader(requester)) {
        return;
    }

    writer_cache_.for_each_change([&](CacheChange& c) noexcept {
        if (c.inlineQos.hasRelatedSampleIdentity
            && c.inlineQos.relatedSampleIdentity.writerGuid.prefix == requester) {
            c.clear();
        }
    });
}

void StatefulWriter::heartbeat_range(SequenceNumber& first, SequenceNumber& last) const noexcept {
    first = writer_cache_.get_seq_num_min();
    last = writer_cache_.get_seq_num_max();

    // Empty history: firstSN = lastSN + 1 signals "nothing available" (§8.4.2.2).
    if (first == SEQUENCENUMBER_UNKNOWN) {
        last = nextSN_ - 1;
        first = nextSN_;
    }
}

void StatefulWriter::send_heartbeat_to(const EntityId& reader_id, const Locator& dest,
                                       bool final_flag) noexcept {
    SequenceNumber fsn{};
    SequenceNumber lsn{};
    heartbeat_range(fsn, lsn);
    if (!builder_.begin(guid_.prefix)) return;
    if (builder_.append_heartbeat(reader_id, guid_.entityId, fsn, lsn, heartbeatCount_, final_flag)) {
        BufRef buf = builder_.finish();
        transport_.send_unicast(buf, dest);
        buf.reset();
    } else {
        builder_.abort();
    }
}

void StatefulWriter::send_heartbeat(bool final_flag, const Guid directed_to) noexcept {
    ++heartbeatCount_;
    RTPS_LOG_D("writer", "HEARTBEAT count=%lu final=%d",
               static_cast<unsigned long>(heartbeatCount_), static_cast<int>(final_flag));
    for (ReaderProxy& proxy : matched()) {
        if (!serves_reliably(proxy)
            || (directed_to != GUID_UNKNOWN && proxy.remoteReaderGuid != directed_to)) {
            continue;
        }

        const Locator& dest = proxy.best_locator();
        if (dest == LOCATOR_INVALID) {
            continue;
        }

        const EntityId& rid =
            (directed_to != GUID_UNKNOWN) ? proxy.remoteReaderGuid.entityId : entity_id::UNKNOWN;
        send_heartbeat_to(rid, dest, final_flag);
    }
}

void StatefulWriter::on_acknack(const ParseContext& ctx, const ParsedAckNack& msg) noexcept {
    Guid sg{ctx.senderGuidPrefix, msg.readerId};
    ReaderProxy* proxy = find_proxy(sg);
    if (!proxy) {
        RTPS_LOG_T("writer", "ACKNACK from unknown reader %02x%02x%02x%02x, ignored",
                   ctx.senderGuidPrefix.value[0], ctx.senderGuidPrefix.value[1],
                   ctx.senderGuidPrefix.value[2], ctx.senderGuidPrefix.value[3]);
        return;
    }

    // Best-effort behavior toward this reader has no ACKNACK to act on
    // (§8.4.9.1); acting on one would hold changes for a reader that is
    // never repaired.
    if (!serves_reliably(*proxy)) {
        return;
    }

    // A cumulative ACK below the last one means the reader started over
    // after its participant lost this one's lease and matched this writer
    // again. Its count may have restarted too, so an ACK of nothing passes
    // the duplicate check; a delayed ACKNACK that acknowledged something
    // does not, and is discarded with the other duplicates. Base 0 and base
    // 1 both acknowledge nothing (Fast DDS announces a reader with base 0).
    const SequenceNumber base_minus_one = msg.readerSNState.bitmapBase - 1;
    const SequenceNumber committed =
        SEQUENCENUMBER_ZERO < base_minus_one ? base_minus_one : SEQUENCENUMBER_ZERO;
    const bool newer = msg.count > proxy->lastAcknackCount;
    const bool started_over =
        committed < proxy->highestAckedSN && (newer || committed == SEQUENCENUMBER_ZERO);
    if (started_over) {
        rematch_reader(*proxy);
    } else if (!newer) {
        return;
    }

    proxy->lastAcknackCount = msg.count;

    // Cumulative ACK: all SNs below bitmapBase are acknowledged.
    const bool acked_nothing_before = proxy->highestAckedSN == SEQUENCENUMBER_ZERO;
    if (msg.readerSNState.bitmapBase > SEQUENCENUMBER_ZERO) {
        proxy->acked_changes_set(msg.readerSNState.bitmapBase - 1);
    }

    RTPS_LOG_D("writer", "ACKNACK base=%d:%lu count=%lu from %02x%02x%02x%02x",
               static_cast<int>(msg.readerSNState.bitmapBase.high),
               static_cast<unsigned long>(msg.readerSNState.bitmapBase.low),
               static_cast<unsigned long>(msg.count), ctx.senderGuidPrefix.value[0],
               ctx.senderGuidPrefix.value[1], ctx.senderGuidPrefix.value[2],
               ctx.senderGuidPrefix.value[3]);

    // A TRANSIENT_LOCAL writer replays its history to a reader that started
    // over, as matched_reader_add() does for a new one (§8.7.2.2).
    if (started_over && durability_ == DurabilityKind::TRANSIENT_LOCAL
        && proxy->has_unsent_changes(writer_cache_)) {
        proxy->replaying = true;
        replayPending_ = true;
    }

    // NACKed SNs: repair what the reader can have after nackResponseDelay,
    // GAP the rest now.
    answer_nack(*proxy, msg.readerSNState, rosette::platform::monotonic_ms());

    // An ACKNACK that requests nothing, from a reader that has acknowledged
    // nothing yet, while this writer has written more, is a reader announcing
    // itself (Fast DDS and Cyclone DDS send one, with bitmapBase 0 or 1, on
    // matching). Answer with a HEARTBEAT directed at it so it learns the
    // available range now instead of at the next heartbeat period. A reader
    // that has acknowledged before is only behind a stream in flight: the
    // next piggybacked HEARTBEAT covers it, and answering every such ACK
    // would turn each HEARTBEAT into a HEARTBEAT/ACKNACK exchange per round
    // trip for as long as the writer keeps writing.
    if (msg.readerSNState.numBits == 0u && acked_nothing_before
        && msg.readerSNState.bitmapBase < nextSN_) {
        send_heartbeat(false, sg);
    }

    release_finished_changes();

    // Last, so the listener may write to this writer once the ACKNACK is
    // fully applied.
    if (started_over && rematchListener_.fn != nullptr) {
        rematchListener_.fn(rematchListener_.context);
    }
}

void StatefulWriter::on_nack_frag(const ParseContext& ctx, const ParsedNackFrag& msg) noexcept {
    Guid sg{ctx.senderGuidPrefix, msg.readerId};
    ReaderProxy* proxy = find_proxy(sg);
    if (!proxy || !serves_reliably(*proxy) || msg.count <= proxy->lastNackFragCount) {
        return;
    }

    proxy->lastNackFragCount = msg.count;
    record_fragment_repair(*proxy, msg.writerSN, msg.fragmentNumberState);

    // The change itself is requested like a NACKed one, so the repair waits
    // out nackResponseDelay and a change no longer held is answered with GAP.
    SequenceNumberSet sn_set{};
    sn_set.bitmapBase = msg.writerSN;
    sn_set.numBits    = 1;
    sn_set.bitmap[0]  = 0x80000000u;  // bit 0 = bitmapBase itself.
    answer_nack(*proxy, sn_set, rosette::platform::monotonic_ms());
}

void StatefulWriter::record_fragment_repair(const ReaderProxy& proxy, const SequenceNumber& sn,
                                            const FragmentNumberSet& fragments) noexcept {
    uint64_t mask = 0u;
    bool whole = fragments.empty();
    fragments.for_each([&](uint32_t frag) noexcept {
        if (frag < 1u || frag > 64u) {
            whole = true;
            return;
        }

        mask |= uint64_t{1} << (frag - 1u);
    });

    const bool same = fragRepair_.proxy == &proxy && fragRepair_.sn == sn;
    if (whole) {
        // A request the mask cannot express: the whole change goes.
        if (same) {
            fragRepair_ = FragmentRepair{};
        }

        return;
    }

    if (same) {
        fragRepair_.fragments |= mask;
    } else if (fragRepair_.proxy == nullptr) {
        fragRepair_ = FragmentRepair{&proxy, sn, mask};
    }
}

void StatefulWriter::tick(uint32_t now_ms) noexcept {
    // Phase 0: the next part of any TRANSIENT_LOCAL replay.
    if (replayPending_) {
        drain_replays();
    }

    // Phase 1: repair what readers NACKed, once nackResponseDelay has passed.
    for (ReaderProxy& proxy : matched()) {
        if (!proxy.nack_delay_elapsed(now_ms, nack_delay_ms_)) continue;

        bool any_sent = false;
        SequenceNumberSet evicted{};
        while (true) {
            const SequenceNumber sn = proxy.next_requested_change();
            if (sn == SEQUENCENUMBER_UNKNOWN) break;
            proxy.mark_underway(sn);
            const CacheChange* ch = writer_cache_.get_change(sn);
            if (ch) {
                const bool fragments_only = fragRepair_.proxy == &proxy && fragRepair_.sn == sn;
                send_data_to(*ch, proxy, false, fragments_only ? &fragRepair_.fragments : nullptr);
                any_sent = true;
            } else {
                // Evicted or removed since it was requested: irrelevant now.
                (void)evicted.add(sn);
            }
        }
        send_gap(proxy, evicted);
        proxy.clear_nack_pending();
        if (fragRepair_.proxy == &proxy) {
            fragRepair_ = FragmentRepair{};
        }

        // Post-retransmit directed HEARTBEAT (F=true), §8.4.2.2.
        if (any_sent) {
            ++heartbeatCount_;
            const Locator& dest = proxy.best_locator();
            if (dest != LOCATOR_INVALID) {
                send_heartbeat_to(proxy.remoteReaderGuid.entityId, dest, /*final=*/true);
            }
        }
    }

    // Phase 2: periodic HEARTBEAT to announcing readers. §8.4.9.2 T5-T7: a
    // ReaderProxy is "announcing" only while it has unacknowledged changes,
    // and only announcing proxies receive the periodic HEARTBEAT. A fully
    // acknowledged reader hears nothing until the next write, so an idle
    // link stays quiet. A reader served best-effort never acknowledges, and
    // is never announcing (§8.4.9.1).
    if ((now_ms - lastHbSentMs_) >= hb_period_ms_) {
        lastHbSentMs_ = now_ms;
        writesSinceHb_ = 0u;
        ++heartbeatCount_;
        for (ReaderProxy& proxy : matched()) {
            if (!serves_reliably(proxy) || !proxy.is_announcing(writer_cache_)) {
                continue;
            }

            const Locator& dest = proxy.best_locator();
            if (dest == LOCATOR_INVALID) {
                continue;
            }

            send_heartbeat_to(entity_id::UNKNOWN, dest, /*final=*/false);
        }
    }
}

bool StatefulWriter::acknowledged_by_all() const noexcept {
    if (reliability_ != ReliabilityKind::RELIABLE) return true;
    const SequenceNumber lsn = writer_cache_.get_seq_num_max();
    if (lsn == SEQUENCENUMBER_UNKNOWN) return true;
    return all_readers_acked(lsn);
}

bool StatefulWriter::acknowledged_by_participant(const GuidPrefix& prefix) const noexcept {
    if (reliability_ != ReliabilityKind::RELIABLE) {
        return true;
    }

    const SequenceNumber lsn = writer_cache_.get_seq_num_max();
    if (lsn == SEQUENCENUMBER_UNKNOWN) {
        return true;
    }

    for (const ReaderProxy& proxy : matched()) {
        if (serves_reliably(proxy) && proxy.remoteReaderGuid.prefix == prefix
            && proxy.highestAckedSN < lsn) {
            return false;
        }
    }

    return true;
}

uint32_t StatefulWriter::ms_until_next_timer(uint32_t now_ms) const noexcept {
    if (replayPending_) {
        return 0u;
    }

    uint32_t next = UINT32_MAX;
    bool any_announcing = false;
    for (const ReaderProxy& proxy : matched()) {
        if (serves_reliably(proxy) && proxy.is_announcing(writer_cache_)) {
            any_announcing = true;
        }

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

void StatefulWriter::send_gap(const ReaderProxy& proxy, const SequenceNumberSet& irrelevant) noexcept {
    SequenceNumber gap_start{};
    SequenceNumberSet gap_list{};
    if (!encode_gap(irrelevant, gap_start, gap_list)) return;
    const Locator& dest = proxy.best_locator();
    if (dest == LOCATOR_INVALID || !builder_.begin(guid_.prefix)) return;
    if (builder_.append_gap(proxy.remoteReaderGuid.entityId, guid_.entityId, gap_start, gap_list)) {
        BufRef buf = builder_.finish();
        transport_.send_unicast(buf, dest);
        buf.reset();
    } else {
        builder_.abort();
    }
}

void StatefulWriter::answer_nack(ReaderProxy& proxy, const SequenceNumberSet& requested,
                                 uint32_t now_ms) noexcept {
    SequenceNumberSet repairable{};
    SequenceNumberSet irrelevant{};
    requested.for_each([&](const SequenceNumber& sn) noexcept {
        // Beyond the history, or not yet sent to this reader: the send already
        // due (a write in progress, a replay) answers it.
        if (!(sn < nextSN_) || proxy.is_unsent(sn)) {
            return;
        }

        if (proxy.is_relevant(sn) && writer_cache_.get_change(sn)) {
            (void)repairable.add(sn);
        } else {
            (void)irrelevant.add(sn);
        }
    });
    proxy.requested_changes_set(repairable, now_ms);
    send_gap(proxy, irrelevant);
}

SequenceNumber StatefulWriter::first_relevant_for_new_reader() const noexcept {
    return durability_ == DurabilityKind::TRANSIENT_LOCAL ? SequenceNumber::from_int64(1) : nextSN_;
}

void StatefulWriter::rematch_reader(ReaderProxy& proxy) noexcept {
    [[maybe_unused]] const Guid& g = proxy.remoteReaderGuid;
    RTPS_LOG_I("writer", "reader %02x%02x%02x%02x entity=%02x%02x%02x%02x started over, rematched",
               g.prefix.value[0], g.prefix.value[1], g.prefix.value[2], g.prefix.value[3],
               g.entityId.entityKey[0], g.entityId.entityKey[1], g.entityId.entityKey[2],
               g.entityId.entityKind);
    proxy.restart(first_relevant_for_new_reader());
    if (fragRepair_.proxy == &proxy) {
        fragRepair_ = FragmentRepair{};
    }
}

ReaderProxy* StatefulWriter::find_proxy(const Guid& g) noexcept {
    for (ReaderProxy& proxy : matched()) {
        if (proxy.remoteReaderGuid == g) {
            return &proxy;
        }
    }

    return nullptr;
}

// Kept out of line: at -O3 it would be inlined whole into
// send_unsent_changes(), costing about 2.5 KB of flash on a Cortex-M0+ build
// for no measurable gain on a path bounded by the transport.
[[gnu::noinline]] bool StatefulWriter::send_data_to(const CacheChange& ch, ReaderProxy& proxy,
                                                    bool heartbeat,
                                                    const uint64_t* only_fragments) noexcept {
    if (!ch.payload.valid() && payloadBuilder_ != nullptr) {
        return send_built_data_to(ch, proxy, heartbeat, only_fragments);
    }

    const Locator& dest = proxy.best_locator();
    if (dest == LOCATOR_INVALID) return false;

    // A sample whose DATA message would exceed the transport's datagram limit
    // goes out as DATA_FRAG submessages instead (§8.4.14.1).
    const uint16_t inline_qos_size = inline_qos_serialized_size(ch.inlineQos);
    const uint32_t data_datagram = static_cast<uint32_t>(
        sizeof(RTPSMessageHeader) + sizeof(SubmessageHeader) + sizeof(DataSubmessageFixed)
        + inline_qos_size + (ch.preframed ? 0u : kEncapsulationSize) + ch.payload.size);
    const uint32_t limit = transport_.max_datagram_size();
    if (data_datagram > limit) {
        return send_fragments_to(ch, proxy, dest, heartbeat, only_fragments);
    }

    // A payload allocated with headroom is framed in place (§9.4.5.3); the
    // HEARTBEAT then needs tailroom in the same block and a payload that
    // ends on a 4-octet boundary (§9.4.1), since in-place bytes cannot be
    // padded. A copied DATA is padded to the boundary instead. Either way the
    // message must fit the datagram limit; otherwise the HEARTBEAT goes in a
    // datagram of its own.
    const bool in_place = ch.payload.valid()
                       && ch.payload.headroom_available() >= kRtpsDataHeadroom + inline_qos_size;
    const uint32_t pad = in_place ? 0u : (4u - (ch.payload.size & 3u)) & 3u;
    const bool framable = !in_place
                       || (ch.payload.capacity() - ch.payload.size >= kRtpsHeartbeatTailroom
                           && (ch.payload.size & 3u) == 0u);
    const bool inline_hb = heartbeat && framable
                        && data_datagram + pad + kRtpsHeartbeatTailroom <= limit;

    bool sent = false;
    if (in_place) {
        sent = send_data_in_place(ch, proxy, dest, inline_hb);
    } else if (builder_.begin(guid_.prefix,
                              data_datagram + (inline_hb ? pad + kRtpsHeartbeatTailroom : 0u))) {
        SequenceNumber first{};
        SequenceNumber last{};
        heartbeat_range(first, last);
        const bool ok = builder_.append_data(proxy.remoteReaderGuid.entityId, guid_.entityId,
                                             ch.sequenceNumber, ch.payload,
                                             ch.kind != ChangeKind::ALIVE, ch.inlineQos,
                                             /*pad_to_boundary=*/inline_hb)
                     && (!inline_hb
                         || builder_.append_heartbeat(proxy.remoteReaderGuid.entityId,
                                                      guid_.entityId, first, last,
                                                      heartbeatCount_, /*final=*/false));
        if (ok) {
            BufRef buf = builder_.finish();
            sent = transport_.send_unicast(buf, dest);
            buf.reset();
        } else {
            builder_.abort();
        }
    }

    if (heartbeat && !inline_hb) {
        send_heartbeat_to(proxy.remoteReaderGuid.entityId, dest, /*final=*/false);
    }

    return sent;
}

// Cold: on-demand changes are sent on discovery and repair, never per sample.
[[gnu::cold]] bool StatefulWriter::send_built_data_to(const CacheChange& ch, ReaderProxy& proxy,
                                                      bool heartbeat,
                                                      const uint64_t* only_fragments) noexcept {
    CacheChange built{};
    built.kind = ch.kind;
    built.writerGuid = ch.writerGuid;
    built.sequenceNumber = ch.sequenceNumber;
    built.sourceTimestamp = ch.sourceTimestamp;
    built.inlineQos = ch.inlineQos;
    built.preframed = ch.preframed;
    built.payload = payloadBuilder_(payloadBuilderUser_, ch);
    if (!built.payload.valid()) {
        RTPS_LOG_W("writer", "payload of SN=%ld:%lu could not be built; left for repair",
                   ch.sequenceNumber.high, static_cast<unsigned long>(ch.sequenceNumber.low));
        const Locator& dest = proxy.best_locator();
        if (heartbeat && dest != LOCATOR_INVALID) {
            send_heartbeat_to(proxy.remoteReaderGuid.entityId, dest, /*final=*/false);
        }

        return false;
    }

    // `built` owns the block, so it returns to the pool when this returns.
    return send_data_to(built, proxy, heartbeat, only_fragments);
}

bool StatefulWriter::frame_data_in_place(BufRef& view, const CacheChange& ch, uint32_t payload_size,
                                         const EntityId& reader_id) const noexcept {
    // Inline QoS sits between the fixed DATA fields and the
    // serializedPayload — §9.4.5.3, Table 9.14.
    const uint16_t inline_qos_size = inline_qos_serialized_size(ch.inlineQos);
    Octet* iq = inline_qos_size > 0u ? view.prepend(inline_qos_size) : nullptr;
    if (iq) {
        serialize_inline_qos(iq, ch.inlineQos);
    }

    auto*  sm = aligned_as<DataSubmessageFixed>(view.prepend(sizeof(DataSubmessageFixed)));
    auto*  smh = aligned_as<SubmessageHeader>(view.prepend(sizeof(SubmessageHeader)));
    auto* mh = reinterpret_cast<RTPSMessageHeader*>(view.prepend(sizeof(RTPSMessageHeader)));
    if ((inline_qos_size > 0u && !iq) || !sm || !smh || !mh) {
        return false;
    }

    sm->extraFlags        = 0;
    sm->octetsToInlineQos = 16u;  // Fixed DATA fields after this one — §9.4.5.3.
    sm->readerId = reader_id;
    sm->writerId          = guid_.entityId;
    sm->writerSN          = ch.sequenceNumber;

    smh->submessageId = SubmessageKind::DATA;
    smh->flags = static_cast<Octet>(FLAG_E |
                                    (ch.kind == ChangeKind::ALIVE ? data_flags::D : data_flags::K) |
                                    (inline_qos_size > 0u ? data_flags::Q : 0u));
    smh->submessageLength =
        static_cast<uint16_t>(sizeof(DataSubmessageFixed) + inline_qos_size + payload_size);

    *mh = RTPSMessageHeader{};
    mh->guidPrefix = guid_.prefix;
    mh->vendorId   = VENDORID_THIS_IMPL;
    return true;
}

bool StatefulWriter::send_data_in_place(const CacheChange& ch, ReaderProxy& proxy,
                                        const Locator& dest, bool heartbeat) noexcept {
    // Work on a non-owning view; retain the block first so the CacheChange
    // keeps its own reference. The view's prepends and append only write
    // outside [payload.data, payload.data + payload.size), so the stored
    // change is unchanged for the next reader or a repair.
    ch.payload.retain();
    BufRef view{};
    view.block = ch.payload.block;
    view.data = ch.payload.data;
    view.size = ch.payload.size;

    const bool framed =
        frame_data_in_place(view, ch, ch.payload.size, proxy.remoteReaderGuid.entityId);
    Octet* hb = framed && heartbeat ? view.append(kRtpsHeartbeatTailroom) : nullptr;
    if (!framed || (heartbeat && !hb)) {
        view.reset();
        return false;
    }

    if (hb) {
        encode_heartbeat(hb, proxy.remoteReaderGuid.entityId);
    }

    const bool sent = transport_.send_unicast(view, dest);
    view.reset();  // Release the extra retain() taken above.
    return sent;
}

bool StatefulWriter::send_fragments_to(const CacheChange& ch, ReaderProxy& proxy,
                                       const Locator& dest, bool heartbeat,
                                       const uint64_t* only_fragments) noexcept {
    static constexpr Octet kCdrLe[kEncapsulationSize] = {0x00u, 0x01u, 0x00u, 0x00u};
    const uint16_t prefix_len = ch.preframed ? 0u : kEncapsulationSize;
    const uint32_t data_size = prefix_len + ch.payload.size;
    const uint32_t overhead = sizeof(RTPSMessageHeader) + sizeof(SubmessageHeader) +
                              sizeof(DataFragSubmessageFixed) +
                              inline_qos_serialized_size(ch.inlineQos);
    const uint32_t limit = transport_.max_datagram_size();
    if (limit < overhead + 64u) {
        RTPS_LOG_E("writer", "datagram limit %lu leaves no room for fragments",
                   static_cast<unsigned long>(limit));
        return false;
    }

    uint32_t frag_size = (limit - overhead) & ~3u;
    if (frag_size > 0xFFFCu) frag_size = 0xFFFCu;
    const uint32_t frag_count = (data_size + frag_size - 1u) / frag_size;
    SequenceNumber first{};
    SequenceNumber last{};
    heartbeat_range(first, last);

    bool all_sent = true;
    bool hb_sent = false;
    for (uint32_t k = 1u; k <= frag_count; ++k) {
        if (only_fragments && (k > 64u || ((*only_fragments >> (k - 1u)) & 1u) == 0u)) {
            continue;
        }

        const uint32_t offset = (k - 1u) * frag_size;
        const uint32_t len = (data_size - offset < frag_size) ? data_size - offset : frag_size;
        uint16_t pre_len = 0u;
        const Octet* pre = nullptr;
        if (offset < prefix_len) {
            pre = kCdrLe + offset;
            pre_len = static_cast<uint16_t>((prefix_len - offset < len) ? prefix_len - offset : len);
        }

        const uint32_t body_offset = (offset + pre_len) - prefix_len;
        const uint16_t body_len = static_cast<uint16_t>(len - pre_len);

        // The last fragment carries the HEARTBEAT when it ends on a 4-octet
        // boundary (§9.4.1; fragment bytes are not padded) and its datagram
        // has room for it.
        const bool with_hb = heartbeat && k == frag_count && (len & 3u) == 0u
                          && overhead + len + kRtpsHeartbeatTailroom <= limit;
        if (!builder_.begin(guid_.prefix,
                            overhead + len + (with_hb ? kRtpsHeartbeatTailroom : 0u))) {
            all_sent = false;
            break;
        }

        const bool ok = builder_.append_data_frag_parts(
                            proxy.remoteReaderGuid.entityId, guid_.entityId, ch.sequenceNumber, k,
                            static_cast<uint16_t>(frag_size), data_size, pre, pre_len,
                            ch.payload.data + body_offset, body_len, ch.inlineQos)
                     && (!with_hb
                         || builder_.append_heartbeat(proxy.remoteReaderGuid.entityId,
                                                      guid_.entityId, first, last,
                                                      heartbeatCount_, /*final=*/false));
        if (!ok) {
            builder_.abort();
            all_sent = false;
            break;
        }

        BufRef buf = builder_.finish();
        all_sent = transport_.send_unicast(buf, dest) && all_sent;
        buf.reset();
        hb_sent = hb_sent || with_hb;
    }

    if (heartbeat && !hb_sent) {
        send_heartbeat_to(proxy.remoteReaderGuid.entityId, dest, /*final=*/false);
    }

    return all_sent;
}

void StatefulWriter::send_unsent_changes(bool with_heartbeat) noexcept {
    if (with_heartbeat) {
        ++heartbeatCount_;
        writesSinceHb_ = 0u;
    }

    const SequenceNumber newest = writer_cache_.get_seq_num_max();
    for (ReaderProxy& proxy : matched()) {
        if (proxy.replaying) {
            continue;
        }

        while (true) {
            const SequenceNumber sn = proxy.next_unsent_change(writer_cache_);
            if (sn == SEQUENCENUMBER_UNKNOWN) break;
            proxy.mark_sent(sn);
            send_data_to(*writer_cache_.get_change(sn), proxy,
                         with_heartbeat && sn == newest && serves_reliably(proxy));
        }
    }

    release_finished_changes();
}

uint32_t StatefulWriter::piggyback_interval() const noexcept {
    if (releaseRepliesOnDelivery_) {
        return 1u;
    }

    const uint32_t half_depth = writer_cache_.capacity() / 2u;
    return half_depth > 0u ? half_depth : 1u;
}

bool StatefulWriter::heartbeat_due() const noexcept {
    return reliability_ == ReliabilityKind::RELIABLE && writesSinceHb_ >= piggyback_interval();
}

void StatefulWriter::encode_heartbeat(Octet* out, const EntityId& reader_id) const noexcept {
    static_assert(sizeof(SubmessageHeader) + sizeof(HeartbeatSubmessage) == kRtpsHeartbeatTailroom,
                  "kRtpsHeartbeatTailroom must hold exactly one HEARTBEAT submessage");

    // Same layout MessageBuilder::append_heartbeat() writes (§9.4.5.5).
    SubmessageHeader smh{};
    smh.submessageId     = SubmessageKind::HEARTBEAT;
    smh.flags            = FLAG_E;
    smh.submessageLength = heartbeat_payload_size();

    HeartbeatSubmessage body{};
    body.readerId = reader_id;
    body.writerId = guid_.entityId;
    heartbeat_range(body.firstSN, body.lastSN);
    body.count = heartbeatCount_;

    memcpy(out, &smh, sizeof(smh));
    memcpy(out + sizeof(smh), &body, sizeof(body));
}

void StatefulWriter::note_eviction() noexcept {
    if (reliability_ != ReliabilityKind::RELIABLE) {
        return;
    }

    // The writer's own changes fill its history, so KEEP_LAST evicts the
    // lowest sequence number (HistoryCache::add_change()).
    const SequenceNumber oldest_sn = writer_cache_.get_seq_num_min();
    const CacheChange*   oldest = writer_cache_.get_change(oldest_sn);
    if (oldest == nullptr) {
        return;
    }

    const bool undelivered = releaseRepliesOnDelivery_ ? !reply_delivered(*oldest)
                                                       : !all_readers_done_with(oldest_sn);
    if (!undelivered) {
        return;
    }

    // Logged at counts 1, 2, 4, 8, ...: a publisher of latest values that
    // outpaces a slow reader evicts on every write, and the log must not
    // flood a serial console while the count stays exact.
    ++evictedUnacknowledged_;
    if ((evictedUnacknowledged_ & (evictedUnacknowledged_ - 1u)) == 0u) {
        [[maybe_unused]] const EntityId& id = guid_.entityId;
        RTPS_LOG_W(
            "writer",
            "%02x%02x%02x%02x evicted unacknowledged SN=%lu from %u-deep history (%lu so far)",
            id.entityKey[0], id.entityKey[1], id.entityKey[2], id.entityKind,
            static_cast<unsigned long>(oldest_sn.low),
            static_cast<unsigned>(writer_cache_.capacity()),
            static_cast<unsigned long>(evictedUnacknowledged_));
    }
}

bool StatefulWriter::all_readers_acked(const SequenceNumber& sn) const noexcept {
    for (const ReaderProxy& proxy : matched()) {
        if (serves_reliably(proxy) && proxy.highestAckedSN < sn) {
            return false;
        }
    }

    return true;
}

bool StatefulWriter::all_readers_done_with(const SequenceNumber& sn) const noexcept {
    for (const ReaderProxy& proxy : matched()) {
        if (!proxy.is_done_with(sn, serves_reliably(proxy))) {
            return false;
        }
    }

    return true;
}

void StatefulWriter::release_finished_changes() noexcept {
    const bool is_volatile = durability_ == DurabilityKind::VOLATILE;
    if (!is_volatile && !releaseRepliesOnDelivery_) {
        return;
    }

    writer_cache_.for_each_change([&](CacheChange& c) noexcept {
        if (releaseRepliesOnDelivery_ && reply_delivered(c)) {
            c.clear();
            return;
        }

        if (!is_volatile) {
            return;
        }

        if (all_readers_done_with(c.sequenceNumber)) {
            c.clear();
        }
    });
}

bool StatefulWriter::reply_delivered(const CacheChange& reply) const noexcept {
    if (!reply.inlineQos.hasRelatedSampleIdentity) {
        return false;
    }

    // The requester's reply reader shares the GUID prefix of the request
    // writer the reply names: both belong to the client's participant.
    const GuidPrefix& requester = reply.inlineQos.relatedSampleIdentity.writerGuid.prefix;
    bool requester_matched = false;
    for (const ReaderProxy& proxy : matched()) {
        if (proxy.remoteReaderGuid.prefix != requester) {
            continue;
        }

        if (!proxy.is_done_with(reply.sequenceNumber, serves_reliably(proxy))) {
            return false;
        }

        requester_matched = true;
    }

    // A reader still waiting for a proxy will bind and need the reply, and the
    // matched readers' acknowledgments say nothing about it.
    return requester_matched && !requester_has_waiting_reader(requester);
}

bool StatefulWriter::requester_has_waiting_reader(const GuidPrefix& requester) const noexcept {
    return waitingReaderQuery_.fn != nullptr
           && waitingReaderQuery_.fn(waitingReaderQuery_.context, guid_, requester);
}

}  // namespace rosette::rtps
