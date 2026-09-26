/**
 * @file stateful_writer.cpp
 * @brief StatefulWriter behavior (§8.4.1–§8.4.9), compiled once for every depth.
 *
 * See rtps/stateful_writer.hpp. The writer's storage (history and proxies)
 * belongs to StatefulWriterStorage<Depth, MaxReaders>; this file sees it only
 * through the references and counts bound at construction.
 */

#include "rosette/rtps/stateful_writer.hpp"

#include <string.h>

#include "rosette/rtps/gap_encoding.hpp"
#include "rosette/rtps/log.hpp"

namespace rtps {
namespace {

/// Octets of the CDR encapsulation header (§10.2) prepended to samples that
/// are not preframed.
constexpr uint16_t kEncapsulationSize = 4u;

}  // namespace

StatefulWriter::StatefulWriter(const Guid& guid, ReliabilityKind reliability,
                               DurabilityKind durability, ITransport& transport,
                               HistoryCache& history, ReaderProxy* proxies,
                               uint8_t max_readers, uint32_t nack_delay_ms,
                               uint32_t hb_period_ms) noexcept
    : guid_(guid),
      reliability_(reliability),
      durability_(durability),
      transport_(transport),
      writer_cache_(history),
      proxies_(proxies),
      max_readers_(max_readers),
      nack_delay_ms_(nack_delay_ms),
      hb_period_ms_(hb_period_ms) {}

CacheChange* StatefulWriter::new_change(ChangeKind kind, BufRef& out_payload,
                                        size_t payload_size) noexcept {
    if (staging_.payload.valid()) staging_.payload.reset();
    out_payload = buf_alloc(payload_size);
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
    CacheChange* inserted = writer_cache_.add_change(static_cast<CacheChange&&>(change));
    if (!inserted) {
        RTPS_LOG_E("writer", "cache add_change failed (SN=%ld:%lu)", change.sequenceNumber.high,
                   (unsigned long)change.sequenceNumber.low);
        return false;
    }

    RTPS_LOG_D("writer", "add_change SN=%ld:%lu size=%lu depth=%d", inserted->sequenceNumber.high,
               (unsigned long)inserted->sequenceNumber.low, (unsigned long)inserted->payload.size,
               writer_cache_.size());
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
    send_unsent_changes();
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
    send_unsent_changes();
    return true;
}

bool StatefulWriter::write_preframed_async(ChangeKind kind, const Octet* serialized_payload,
                                           uint32_t payload_size, SequenceNumber* out_sn,
                                           const InlineQos& inline_qos) noexcept {
    if (!serialized_payload || payload_size == 0) return false;
    if (staging_.payload.valid()) staging_.payload.reset();

    // Headroom covers the RTPS/DATA headers plus the largest inline QoS, so
    // send_data_to() can always frame this change in place.
    BufRef payload_buf = buf_alloc_with_headroom(payload_size, kRtpsDataHeadroom + kMaxInlineQosSize);
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
    send_unsent_changes();
    return true;
}

bool StatefulWriter::matched_reader_add(const Guid& reader_guid, const Locator* unicast_locators,
                                        uint8_t locator_count, bool expects_inline_qos) noexcept {
    if (find_proxy(reader_guid)) return false;
    ReaderProxy* slot = find_free_slot();
    if (!slot) {
        RTPS_LOG_W("writer", "reader proxy table full (max=%d)", max_readers_);
        return false;
    }

    // TRANSIENT_LOCAL: every cached change is relevant to the new reader
    // (§8.7.2.2). VOLATILE: only changes written from now on (§8.7.2.2.1).
    const SequenceNumber first_relevant = durability_ == DurabilityKind::TRANSIENT_LOCAL
                                              ? SequenceNumber::from_int64(1)
                                              : nextSN_;
    slot->init(reader_guid, unicast_locators, locator_count, expects_inline_qos, first_relevant);

    RTPS_LOG_I("writer", "reader matched %02x%02x%02x%02x entity=%02x%02x%02x%02x",
               reader_guid.prefix.value[0], reader_guid.prefix.value[1],
               reader_guid.prefix.value[2], reader_guid.prefix.value[3],
               reader_guid.entityId.entityKey[0], reader_guid.entityId.entityKey[1],
               reader_guid.entityId.entityKey[2], reader_guid.entityId.entityKind);

    // TRANSIENT_LOCAL: push the cached changes now rather than waiting for
    // the next write (§8.7.2.2).
    if (durability_ == DurabilityKind::TRANSIENT_LOCAL) send_unsent_changes();
    return true;
}

void StatefulWriter::matched_reader_remove(const Guid& reader_guid) noexcept {
    ReaderProxy* p = find_proxy(reader_guid);
    if (p) p->clear();
}

void StatefulWriter::send_heartbeat_to(const EntityId& reader_id, const Locator& dest,
                                       bool final_flag) noexcept {
    SequenceNumber fsn = writer_cache_.get_seq_num_min();
    SequenceNumber lsn = writer_cache_.get_seq_num_max();
    // Empty history: firstSN = lastSN + 1 signals "nothing available" (§8.4.2.2).
    if (fsn == SEQUENCENUMBER_UNKNOWN) {
        lsn = nextSN_ - 1;
        fsn = nextSN_;
    }
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
    RTPS_LOG_D("writer", "HEARTBEAT count=%lu final=%d", (unsigned long)heartbeatCount_,
               (int)final_flag);
    for (uint8_t i = 0; i < max_readers_; ++i) {
        ReaderProxy& proxy = proxies_[i];
        if (!proxy.is_matched()) continue;
        if (directed_to != GUID_UNKNOWN && proxy.remoteReaderGuid != directed_to) continue;
        const Locator& dest = proxy.best_locator();
        if (dest == LOCATOR_INVALID) continue;
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

    // Discard duplicate / reordered ACKNACKs.
    if (msg.count <= proxy->lastAcknackCount) return;
    proxy->lastAcknackCount = msg.count;

    // Cumulative ACK: all SNs below bitmapBase are acknowledged.
    if (msg.readerSNState.bitmapBase > SEQUENCENUMBER_ZERO) {
        proxy->acked_changes_set(msg.readerSNState.bitmapBase - 1);
    }

    RTPS_LOG_D("writer", "ACKNACK base=%d:%lu count=%lu from %02x%02x%02x%02x",
               msg.readerSNState.bitmapBase.high, (unsigned long)msg.readerSNState.bitmapBase.low,
               (unsigned long)msg.count, ctx.senderGuidPrefix.value[0],
               ctx.senderGuidPrefix.value[1], ctx.senderGuidPrefix.value[2],
               ctx.senderGuidPrefix.value[3]);

    // NACKed SNs: repair what the reader can have after nackResponseDelay,
    // GAP the rest now.
    answer_nack(*proxy, msg.readerSNState, rosette::platform::monotonic_ms());

    // An ACKNACK that requests nothing yet acknowledges less than this writer
    // has written is a reader announcing itself (Fast DDS and Cyclone DDS
    // send one, with bitmapBase 0 or 1, on matching). Answer with a HEARTBEAT
    // directed at it so it learns the available range now instead of at the
    // next heartbeat period (§8.4.15.4).
    if (msg.readerSNState.numBits == 0u && msg.readerSNState.bitmapBase < nextSN_) {
        send_heartbeat(false, sg);
    }
}

void StatefulWriter::on_nack_frag(const ParseContext& ctx, const ParsedNackFrag& msg) noexcept {
    Guid sg{ctx.senderGuidPrefix, msg.readerId};
    ReaderProxy* proxy = find_proxy(sg);
    if (!proxy) return;

    // Treat as a whole-sample NACK for the affected SN.
    SequenceNumberSet sn_set{};
    sn_set.bitmapBase = msg.writerSN;
    sn_set.numBits    = 1;
    sn_set.bitmap[0]  = 0x80000000u;  // bit 0 = bitmapBase itself.
    answer_nack(*proxy, sn_set, rosette::platform::monotonic_ms());
}

void StatefulWriter::tick(uint32_t now_ms) noexcept {
    // Phase 1: repair what readers NACKed, once nackResponseDelay has passed.
    for (uint8_t i = 0; i < max_readers_; ++i) {
        ReaderProxy& proxy = proxies_[i];
        if (!proxy.is_matched()) continue;
        if (!proxy.nack_delay_elapsed(now_ms, nack_delay_ms_)) continue;

        bool any_sent = false;
        SequenceNumberSet evicted{};
        while (true) {
            const SequenceNumber sn = proxy.next_requested_change();
            if (sn == SEQUENCENUMBER_UNKNOWN) break;
            proxy.mark_underway(sn);
            const CacheChange* ch = writer_cache_.get_change(sn);
            if (ch) {
                send_data_to(*ch, proxy);
                any_sent = true;
            } else {
                // Evicted or removed since it was requested: irrelevant now.
                (void)evicted.add(sn);
            }
        }
        send_gap(proxy, evicted);
        proxy.clear_nack_pending();

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
    // link stays quiet.
    if ((now_ms - lastHbSentMs_) >= hb_period_ms_) {
        lastHbSentMs_ = now_ms;
        ++heartbeatCount_;
        for (uint8_t i = 0; i < max_readers_; ++i) {
            ReaderProxy& proxy = proxies_[i];
            if (!proxy.is_matched() || !proxy.is_announcing(writer_cache_)) continue;
            const Locator& dest = proxy.best_locator();
            if (dest == LOCATOR_INVALID) continue;
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

uint32_t StatefulWriter::ms_until_next_timer(uint32_t now_ms) const noexcept {
    uint32_t next = UINT32_MAX;
    bool any_announcing = false;
    for (uint8_t i = 0; i < max_readers_; ++i) {
        const ReaderProxy& proxy = proxies_[i];
        if (!proxy.is_matched()) continue;
        if (proxy.is_announcing(writer_cache_)) any_announcing = true;
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

uint8_t StatefulWriter::matched_reader_count() const noexcept {
    uint8_t n = 0;
    for (uint8_t i = 0; i < max_readers_; ++i) {
        if (proxies_[i].is_matched()) ++n;
    }
    return n;
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
        if (!(sn < nextSN_)) return;
        if (proxy.is_relevant(sn) && writer_cache_.get_change(sn)) {
            (void)repairable.add(sn);
        } else {
            (void)irrelevant.add(sn);
        }
    });
    proxy.requested_changes_set(repairable, now_ms);
    send_gap(proxy, irrelevant);
}

ReaderProxy* StatefulWriter::find_proxy(const Guid& g) noexcept {
    for (uint8_t i = 0; i < max_readers_; ++i) {
        if (proxies_[i].is_matched() && proxies_[i].remoteReaderGuid == g) return &proxies_[i];
    }
    return nullptr;
}

ReaderProxy* StatefulWriter::find_free_slot() noexcept {
    for (uint8_t i = 0; i < max_readers_; ++i) {
        if (!proxies_[i].is_matched()) return &proxies_[i];
    }
    return nullptr;
}

bool StatefulWriter::send_data_to(const CacheChange& ch, ReaderProxy& proxy) noexcept {
    const Locator& dest = proxy.best_locator();
    if (dest == LOCATOR_INVALID) return false;

    // A sample whose DATA message would exceed the transport's datagram limit
    // goes out as DATA_FRAG submessages instead (§8.4.14.1).
    const uint32_t data_datagram = sizeof(RTPSMessageHeader) + sizeof(SubmessageHeader) +
                                   sizeof(DataSubmessageFixed) +
                                   inline_qos_serialized_size(ch.inlineQos) +
                                   (ch.preframed ? 0u : kEncapsulationSize) + ch.payload.size;
    if (data_datagram > transport_.max_datagram_size()) {
        return send_fragments_to(ch, proxy, dest);
    }

    // Zero-copy path (§9.4.5.3): a payload allocated with headroom gets the
    // RTPS message header, DATA submessage header, and inline QoS prepended
    // in place in its own pool block — no scratch buffer, no copy.
    const uint16_t inline_qos_size = inline_qos_serialized_size(ch.inlineQos);
    if (ch.payload.valid() && ch.payload.headroom_available() >= kRtpsDataHeadroom + inline_qos_size) {
        // Work on a non-owning view; retain the block first so the
        // CacheChange keeps its own reference.
        ch.payload.retain();
        BufRef view{};
        view.block = ch.payload.block;
        view.data  = ch.payload.data;
        view.size  = ch.payload.size;

        // Inline QoS sits between the fixed DATA fields and the
        // serializedPayload — §9.4.5.3, Table 9.14.
        if (inline_qos_size > 0u) {
            Octet* iq = view.prepend(inline_qos_size);
            if (!iq) {
                view.reset();
                return false;
            }
            serialize_inline_qos(iq, ch.inlineQos);
        }

        auto* sm = reinterpret_cast<DataSubmessageFixed*>(view.prepend(sizeof(DataSubmessageFixed)));
        if (!sm) {
            view.reset();
            return false;
        }
        sm->extraFlags        = 0;
        sm->octetsToInlineQos = 16u;  // Fixed DATA fields after this one — §9.4.5.3.
        sm->readerId          = proxy.remoteReaderGuid.entityId;
        sm->writerId          = guid_.entityId;
        sm->writerSN          = ch.sequenceNumber;

        auto* smh = reinterpret_cast<SubmessageHeader*>(view.prepend(sizeof(SubmessageHeader)));
        if (!smh) {
            view.reset();
            return false;
        }
        smh->submessageId = SubmessageKind::DATA;
        smh->flags = static_cast<Octet>(FLAG_E |
                                        (ch.kind == ChangeKind::ALIVE ? data_flags::D : data_flags::K) |
                                        (inline_qos_size > 0u ? data_flags::Q : 0u));
        smh->submessageLength = static_cast<uint16_t>(view.size - sizeof(SubmessageHeader));

        auto* mh = reinterpret_cast<RTPSMessageHeader*>(view.prepend(sizeof(RTPSMessageHeader)));
        if (!mh) {
            view.reset();
            return false;
        }
        *mh = RTPSMessageHeader{};
        mh->guidPrefix = guid_.prefix;
        mh->vendorId   = VENDORID_THIS_IMPL;

        const bool sent = transport_.send_unicast(view, dest);
        view.reset();  // Release the extra retain() taken above.
        return sent;
    }

    // Copy path: the payload has no headroom; frame it with the builder.
    if (!builder_.begin(guid_.prefix, data_datagram)) return false;
    const bool ok = builder_.append_data(proxy.remoteReaderGuid.entityId, guid_.entityId,
                                         ch.sequenceNumber, ch.payload,
                                         ch.kind != ChangeKind::ALIVE, ch.inlineQos);
    if (!ok) {
        builder_.abort();
        return false;
    }
    BufRef buf = builder_.finish();
    const bool sent = transport_.send_unicast(buf, dest);
    buf.reset();
    return sent;
}

bool StatefulWriter::send_fragments_to(const CacheChange& ch, ReaderProxy& proxy,
                                       const Locator& dest) noexcept {
    static constexpr Octet kCdrLe[kEncapsulationSize] = {0x00u, 0x01u, 0x00u, 0x00u};
    const uint16_t prefix_len = ch.preframed ? 0u : kEncapsulationSize;
    const uint32_t data_size = prefix_len + ch.payload.size;
    const uint32_t overhead = sizeof(RTPSMessageHeader) + sizeof(SubmessageHeader) +
                              sizeof(DataFragSubmessageFixed) +
                              inline_qos_serialized_size(ch.inlineQos);
    const uint32_t limit = transport_.max_datagram_size();
    if (limit < overhead + 64u) {
        RTPS_LOG_E("writer", "datagram limit %lu leaves no room for fragments", (unsigned long)limit);
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
            pre_len = static_cast<uint16_t>((prefix_len - offset < len) ? prefix_len - offset : len);
        }
        const uint32_t body_offset = (offset + pre_len) - prefix_len;
        const uint16_t body_len = static_cast<uint16_t>(len - pre_len);

        if (!builder_.begin(guid_.prefix, overhead + len)) {
            all_sent = false;
            break;
        }
        const bool ok = builder_.append_data_frag_parts(
            proxy.remoteReaderGuid.entityId, guid_.entityId, ch.sequenceNumber, k,
            static_cast<uint16_t>(frag_size), data_size, pre, pre_len,
            ch.payload.data + body_offset, body_len, ch.inlineQos);
        if (!ok) {
            builder_.abort();
            all_sent = false;
            break;
        }
        BufRef buf = builder_.finish();
        all_sent = transport_.send_unicast(buf, dest) && all_sent;
        buf.reset();
    }
    return all_sent;
}

void StatefulWriter::send_unsent_changes() noexcept {
    for (uint8_t i = 0; i < max_readers_; ++i) {
        ReaderProxy& proxy = proxies_[i];
        if (!proxy.is_matched()) continue;
        while (true) {
            const SequenceNumber sn = proxy.next_unsent_change(writer_cache_);
            if (sn == SEQUENCENUMBER_UNKNOWN) break;
            proxy.mark_sent(sn);
            send_data_to(*writer_cache_.get_change(sn), proxy);
        }
    }
}

bool StatefulWriter::all_readers_acked(const SequenceNumber& sn) const noexcept {
    for (uint8_t i = 0; i < max_readers_; ++i) {
        if (proxies_[i].is_matched() && proxies_[i].highestAckedSN < sn) return false;
    }
    return true;
}

}  // namespace rtps
