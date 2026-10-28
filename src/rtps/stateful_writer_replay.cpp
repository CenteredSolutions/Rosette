/**
 * @file stateful_writer_replay.cpp
 * @brief StatefulWriter's TRANSIENT_LOCAL replay (§8.7.2.2), sent from tick() a budget at a time.
 *
 * A TRANSIENT_LOCAL writer owes every newly matched reader its whole history.
 * Sending it from matched_reader_add() would block the caller, which is the
 * participant parsing the SEDP or SPDP datagram that caused the match, for as
 * long as the history takes to go out: a SEDP announcer replays one
 * announcement per local endpoint to every new peer. Instead the proxy is
 * marked replaying and tick() sends at most RTPS_REPLAY_DATAGRAMS_PER_TICK
 * datagrams of it per call, so one new peer costs the pump a bounded slice
 * per pass.
 *
 * A writer that builds payloads on demand (the SEDP announcers) also packs
 * the replay: each datagram is the first change's own block, framed in place,
 * with as many of the following changes appended as fit in what is left of
 * that block. The block is whatever size class the first payload needed;
 * packing never takes a larger one. Each following change is sized before it
 * is built, so a change is built only for the datagram that sends it. See
 * rtps/stateful_writer.hpp (drain_replays(), send_replay_datagram()).
 */

#include "rosette/rtps/stateful_writer.hpp"

#include <string.h>

namespace rosette::rtps {

void StatefulWriter::drain_replays() noexcept {
    uint32_t spent = 0;
    bool     pending = false;
    uint32_t resume = replayCursor_;

    // Sends one replaying proxy datagrams until the budget is spent, and
    // records where the next pass starts: just after the last proxy served.
    // An attempt is charged the datagrams it sent, or one when it sent none (a
    // payload that could not be built or framed is a lost datagram the reader
    // repairs), so a starved pool cannot turn a pass into history-sized work.
    const auto serve = [&](ReaderProxy& proxy, uint32_t index) noexcept {
        if (!proxy.replaying) {
            return;
        }

        while (spent < RTPS_REPLAY_DATAGRAMS_PER_TICK) {
            const uint32_t before = datagramsSent_;
            if (!send_replay_datagram(proxy)) {
                break;
            }

            const uint32_t sent = datagramsSent_ - before;
            spent += sent > 0u ? sent : 1u;
            resume = index + 1u;
        }

        if (proxy.has_unsent_changes(writer_cache_)) {
            pending = true;
        } else {
            proxy.replaying = false;
        }
    };

    // The proxies from the cursor on, then those before it: a rotation of
    // match order.
    for (const bool from_cursor : {true, false}) {
        uint32_t index = 0;
        for (ReaderProxy& proxy : matched()) {
            if ((index >= replayCursor_) == from_cursor) {
                serve(proxy, index);
            }

            ++index;
        }
    }

    replayCursor_ = resume;
    replayPending_ = pending;
    release_finished_changes();
}

bool StatefulWriter::send_replay_datagram(ReaderProxy& proxy) noexcept {
    const SequenceNumber sn = proxy.next_unsent_change(writer_cache_);
    const CacheChange*   ch = sn == SEQUENCENUMBER_UNKNOWN ? nullptr : writer_cache_.get_change(sn);
    if (ch == nullptr) {
        return false;
    }

    const EntityId&      reader_id = proxy.remoteReaderGuid.entityId;
    const SequenceNumber newest = writer_cache_.get_seq_num_max();
    const bool           reliable = serves_reliably(proxy);
    const auto           heartbeat_with = [&](const SequenceNumber& last) noexcept {
        if (!reliable || last != newest) {
            return false;
        }

        ++heartbeatCount_;
        writesSinceHb_ = 0u;
        return true;
    };

    proxy.mark_sent(sn);
    if (payloadBuilder_ == nullptr || ch->payload.valid()) {
        send_data_to(*ch, proxy, heartbeat_with(sn));
        return true;
    }

    BufRef datagram = payloadBuilder_(payloadBuilderUser_, *ch);

    // In place only when the block has the headroom and the DATA fits one
    // datagram; otherwise send_data_to() frames it by copy or as DATA_FRAG.
    const Locator& dest = proxy.best_locator();
    const uint16_t inline_qos_size = inline_qos_serialized_size(ch->inlineQos);
    const uint32_t first_payload = datagram.size;
    const bool     in_place = datagram.valid() && dest != LOCATOR_INVALID
                          && datagram.headroom_available() >= kRtpsDataHeadroom + inline_qos_size
                          && sizeof(RTPSMessageHeader) + sizeof(SubmessageHeader)
                                     + sizeof(DataSubmessageFixed) + inline_qos_size + first_payload
                                 <= transport_.max_datagram_size();
    if (!in_place) {
        if (!datagram.valid()) {
            // Not built (pool exhausted): a lost datagram the reader repairs.
            if (heartbeat_with(sn) && dest != LOCATOR_INVALID) {
                send_heartbeat_to(reader_id, dest, /*final=*/false);
            }

            return true;
        }

        CacheChange built{};
        built.kind = ch->kind;
        built.writerGuid = ch->writerGuid;
        built.sequenceNumber = ch->sequenceNumber;
        built.sourceTimestamp = ch->sourceTimestamp;
        built.inlineQos = ch->inlineQos;
        built.preframed = ch->preframed;
        built.payload = static_cast<BufRef&&>(datagram);
        send_data_to(built, proxy, heartbeat_with(sn));
        return true;
    }

    if (!frame_data_in_place(datagram, *ch, first_payload, reader_id)) {
        // A lost datagram the reader repairs, as a failed build is.
        datagram.reset();
        if (heartbeat_with(sn)) {
            send_heartbeat_to(reader_id, dest, /*final=*/false);
        }

        return true;
    }

    SequenceNumber last = sn;
    const bool aligned = pack_following_changes(proxy, datagram, (first_payload & 3u) == 0u, last);

    const bool heartbeat = heartbeat_with(last);
    Octet*     hb = heartbeat && aligned
                        && datagram.size + kRtpsHeartbeatTailroom <= transport_.max_datagram_size()
                        ? datagram.append(kRtpsHeartbeatTailroom)
                        : nullptr;
    if (hb) {
        encode_heartbeat(hb, reader_id);
    }

    send_counted(datagram, dest);
    datagram.reset();
    if (heartbeat && !hb) {
        send_heartbeat_to(reader_id, dest, /*final=*/false);
    }

    return true;
}

bool StatefulWriter::pack_following_changes(ReaderProxy& proxy, BufRef& datagram, bool aligned,
                                            SequenceNumber& last) noexcept {
    // A builder may reserve more headroom than the change's inline QoS took;
    // move the framed message to the start of the block, so the room left
    // for the following changes is the whole rest of the block.
    Octet* const block_start = datagram.block->payload();
    if (datagram.data != block_start) {
        memmove(block_start, datagram.data, datagram.size);
        datagram.data = block_start;
    }

    // Append the following changes while each fits in the block in hand,
    // sizing each before building it. Every submessage starts on a 4-octet
    // boundary (§9.4.1), and in-place payloads are not padded, so packing
    // stops after one whose length is not a multiple of 4.
    const uint32_t room = datagram.capacity() < transport_.max_datagram_size()
                              ? datagram.capacity()
                              : transport_.max_datagram_size();
    while (aligned && payloadSizer_ != nullptr) {
        const SequenceNumber next = proxy.next_unsent_change(writer_cache_);
        const CacheChange*   next_ch =
            next == SEQUENCENUMBER_UNKNOWN ? nullptr : writer_cache_.get_change(next);
        if (next_ch == nullptr || next_ch->payload.valid()) {
            break;
        }

        const uint32_t size = payloadSizer_(payloadBuilderUser_, *next_ch);
        if (size == 0u
            || datagram.size + sizeof(SubmessageHeader) + sizeof(DataSubmessageFixed)
                       + inline_qos_serialized_size(next_ch->inlineQos) + size
                   > room) {
            break;
        }

        BufRef payload = payloadBuilder_(payloadBuilderUser_, *next_ch);
        if (!payload.valid()
            || !append_built_data(datagram, *next_ch, payload, proxy.remoteReaderGuid.entityId)) {
            break;
        }

        proxy.mark_sent(next);
        last = next;
        aligned = (payload.size & 3u) == 0u;
    }

    return aligned;
}

bool StatefulWriter::append_built_data(BufRef& datagram, const CacheChange& ch,
                                       const BufRef&   payload,
                                       const EntityId& reader_id) const noexcept {
    const uint16_t inline_qos_size = inline_qos_serialized_size(ch.inlineQos);
    const uint32_t body =
        static_cast<uint32_t>(sizeof(DataSubmessageFixed)) + inline_qos_size + payload.size;
    const uint32_t need = static_cast<uint32_t>(sizeof(SubmessageHeader)) + body;
    if (body > 0xFFFFu || datagram.size + need > transport_.max_datagram_size()) {
        return false;
    }

    Octet* out = datagram.append(need);
    if (out == nullptr) {
        return false;
    }

    // §9.4.5.3, as frame_data_in_place() writes it; copied in, since the
    // append position carries no alignment guarantee for the structs.
    SubmessageHeader smh{};
    smh.submessageId = SubmessageKind::DATA;
    smh.flags =
        static_cast<Octet>(FLAG_E | (ch.kind == ChangeKind::ALIVE ? data_flags::D : data_flags::K)
                           | (inline_qos_size > 0u ? data_flags::Q : 0u));
    smh.submessageLength = static_cast<uint16_t>(body);

    DataSubmessageFixed fixed{};
    fixed.extraFlags = 0;
    fixed.octetsToInlineQos = 16u;
    fixed.readerId = reader_id;
    fixed.writerId = guid_.entityId;
    fixed.writerSN = ch.sequenceNumber;

    memcpy(out, &smh, sizeof(smh));
    out += sizeof(smh);
    memcpy(out, &fixed, sizeof(fixed));
    out += sizeof(fixed);
    if (inline_qos_size > 0u) {
        serialize_inline_qos(out, ch.inlineQos);
        out += inline_qos_size;
    }

    memcpy(out, payload.data, payload.size);
    return true;
}

}  // namespace rosette::rtps
