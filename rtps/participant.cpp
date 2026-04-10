/**
 * @file rtps/participant.cpp
 * @brief RTPSParticipant implementation — §8.5.1 (OMG formal/2022-04-01).
 */

#include "participant.hpp"
#include <new>
#include <string.h>

namespace rtps {

// ============================================================
// Constructor
// ============================================================

RTPSParticipant::RTPSParticipant(uint8_t     domain_id,
                                 uint8_t     participant_id,
                                 ITransport& transport,
                                 VendorId    vendor_id) noexcept
    : domain_id_(domain_id)
    , participant_id_(participant_id)
    , transport_(transport)
    , match_table_(on_endpoint_match, on_endpoint_unmatch, this)
    , spdp_writer_(local_participant_data_,
                   transport,
                   spdp_multicast_locator(domain_id))
    , spdp_reader_(on_spdp_new_peer, on_spdp_peer_removed, this)
    , sedp_writer_(transport)
    , sedp_reader_(match_table_)
{
    memset(writer_iface_, 0, sizeof(writer_iface_));
    memset(writer_alive_, 0, sizeof(writer_alive_));
    memset(reader_alive_, 0, sizeof(reader_alive_));

    // Participant GUID — §8.5.1.2, p. 151.
    participant_guid_.prefix   = rtps_platform_make_guid_prefix(participant_id);
    participant_guid_.entityId = entity_id::PARTICIPANT;

    // SPDPdiscoveredParticipantData — §8.5.3.2, Table 8.73, p. 160.
    local_participant_data_.guidPrefix      = participant_guid_.prefix;
    local_participant_data_.protocolVersion = PROTOCOLVERSION_2_5;
    local_participant_data_.vendorId        = vendor_id;
    local_participant_data_.leaseDuration   =
        Duration::from_ms(RTPS_SPDP_LEASE_DURATION_MS);

    Locator mt_uc = metatraffic_unicast_locator();
    local_participant_data_.metatrafficUnicastLocatorList[0] = mt_uc;
    local_participant_data_.metatrafficUnicastLocatorCount   = 1;

    Locator def_uc = default_unicast_locator();
    local_participant_data_.defaultUnicastLocatorList[0] = def_uc;
    local_participant_data_.defaultUnicastLocatorCount   = 1;

    local_participant_data_.availableBuiltinEndpoints =
        builtin_endpoint::DISC_BUILTIN_ENDPOINT_PARTICIPANT_ANNOUNCER  |
        builtin_endpoint::DISC_BUILTIN_ENDPOINT_PARTICIPANT_DETECTOR   |
        builtin_endpoint::DISC_BUILTIN_ENDPOINT_PUBLICATIONS_ANNOUNCER |
        builtin_endpoint::DISC_BUILTIN_ENDPOINT_PUBLICATIONS_DETECTOR  |
        builtin_endpoint::DISC_BUILTIN_ENDPOINT_SUBSCRIPTIONS_ANNOUNCER|
        builtin_endpoint::DISC_BUILTIN_ENDPOINT_SUBSCRIPTIONS_DETECTOR;

    // ParseOps — §8.3.4, p. 69.
    parse_ops_ = ParseOps{};
    parse_ops_.on_data            = dispatch_data;
    parse_ops_.on_data_frag       = dispatch_data_frag;
    parse_ops_.on_heartbeat       = dispatch_heartbeat;
    parse_ops_.on_heartbeat_frag  = dispatch_heartbeat_frag;
    parse_ops_.on_acknack         = dispatch_acknack;
    parse_ops_.on_nack_frag       = dispatch_nack_frag;
    parse_ops_.on_gap             = dispatch_gap;
}

// ============================================================
// create_reader()
// ============================================================

StatefulReader* RTPSParticipant::create_reader(
        const ReaderConfig&               cfg,
        StatefulReader::OnChangeAvailable on_change_avail,
        void*                             user_data) noexcept
{
    int slot = -1;
    for (int i = 0; i < RTPS_MAX_USER_READERS; ++i)
        if (!reader_alive_[i]) { slot = i; break; }
    if (slot < 0) return nullptr;

    const Guid reader_guid{ participant_guid_.prefix, cfg.entityId };

    StatefulReader* r = new (reader_storage_[slot]) StatefulReader(
        reader_guid, cfg.reliability, transport_,
        on_change_avail, user_data, cfg.hbResponseMs);

    reader_alive_[slot] = true;

    DiscoveredReaderData rd{};
    rd.readerGuid      = reader_guid;
    rd.topicName       = cfg.topicName;
    rd.typeName        = cfg.typeName;
    rd.reliabilityKind = cfg.reliability;
    rd.durabilityKind  = cfg.durability;
    rd.topicKind       = cfg.topicKind;
    rd.unicastLocatorList[0] = default_unicast_locator();
    rd.unicastLocatorCount   = 1;
    sedp_writer_.register_reader(rd);
    match_table_.add_reader(rd);

    return r;
}

// ============================================================
// register_writer_endpoint()  (called from create_writer<D,R>())
// ============================================================

void RTPSParticipant::register_writer_endpoint(const Guid&         writer_guid,
                                               const WriterConfig& cfg) noexcept
{
    DiscoveredWriterData wd{};
    wd.writerGuid      = writer_guid;
    wd.topicName       = cfg.topicName;
    wd.typeName        = cfg.typeName;
    wd.reliabilityKind = cfg.reliability;
    wd.durabilityKind  = cfg.durability;
    wd.topicKind       = cfg.topicKind;
    wd.unicastLocatorList[0] = default_unicast_locator();
    wd.unicastLocatorCount   = 1;
    sedp_writer_.register_writer(wd);
    match_table_.add_writer(wd);
}

// ============================================================
// parse_received() / tick() / announce()
// ============================================================

ParseResult RTPSParticipant::parse_received(const Octet* buf,
                                            uint32_t     len) noexcept
{
    return parse_message(buf, len, participant_guid_.prefix,
                         parse_ops_, this);
}

void RTPSParticipant::tick(uint32_t now_ms) noexcept {
    spdp_writer_.tick(now_ms);
    spdp_reader_.tick(now_ms);
    for (int i = 0; i < RTPS_MAX_USER_WRITERS; ++i)
        if (writer_alive_[i] && writer_iface_[i])
            writer_iface_[i]->tick(now_ms);
    for (int i = 0; i < RTPS_MAX_USER_READERS; ++i)
        if (reader_alive_[i])
            reinterpret_cast<StatefulReader*>(reader_storage_[i])->tick(now_ms);
}

void RTPSParticipant::announce() noexcept {
    spdp_writer_.announce();
}

// ============================================================
// Discovery callbacks
// ============================================================

void RTPSParticipant::on_spdp_new_peer(const PeerEntry& peer,
                                       void*            self) noexcept
{
    auto* p = static_cast<RTPSParticipant*>(self);
    if (peer.data.metatrafficUnicastLocatorCount == 0) return;
    const Locator& dest = peer.data.metatrafficUnicastLocatorList[0];
    p->sedp_writer_.announce_to_peer(p->participant_guid_.prefix, dest);
    p->spdp_reader_.mark_sedp_announced(peer.data.guidPrefix);
}

void RTPSParticipant::on_spdp_peer_removed(const PeerEntry& peer,
                                           void*            self) noexcept
{
    auto* p = static_cast<RTPSParticipant*>(self);

    // Remove all writer and reader endpoints announced by this participant
    // and fire on_unmatch for every affected proxy pair.
    //
    // §8.5.3.2, p. 160:
    //   "If a participant does not receive a refresh within leaseDuration,
    //    the Participant removes all information about the remote Participant
    //    and its endpoints."
    //
    // EndpointMatchTable::remove_participant() iterates all known endpoints
    // whose GUID prefix matches the gone participant, calls remove_writer /
    // remove_reader on each, and fires on_unmatch (→ on_endpoint_unmatch)
    // for every dissolved match — which in turn calls matched_reader_remove /
    // matched_writer_remove on the affected local endpoints.
    p->match_table_.remove_participant(peer.data.guidPrefix);
}

void RTPSParticipant::on_endpoint_match(const DiscoveredWriterData& wd,
                                        const DiscoveredReaderData& rd,
                                        void*                       self) noexcept
{
    auto* p = static_cast<RTPSParticipant*>(self);

    // Local writer matched to a remote reader.
    if (wd.writerGuid.prefix == p->participant_guid_.prefix) {
        IStatefulWriter* iw = p->find_writer_iface(wd.writerGuid.entityId);
        if (iw) iw->matched_reader_add(rd.readerGuid,
                                       rd.unicastLocatorList,
                                       rd.unicastLocatorCount,
                                       rd.expectsInlineQos);
    }

    // Local reader matched to a remote writer.
    if (rd.readerGuid.prefix == p->participant_guid_.prefix) {
        StatefulReader* r = p->find_reader(rd.readerGuid.entityId);
        if (r) r->matched_writer_add(wd.writerGuid,
                                     wd.unicastLocatorList,
                                     wd.unicastLocatorCount);
    }
}

void RTPSParticipant::on_endpoint_unmatch(const Guid& writer_guid,
                                          const Guid& reader_guid,
                                          void*       self) noexcept
{
    auto* p = static_cast<RTPSParticipant*>(self);
    if (writer_guid.prefix == p->participant_guid_.prefix) {
        IStatefulWriter* iw = p->find_writer_iface(writer_guid.entityId);
        if (iw) iw->matched_reader_remove(reader_guid);
    }
    if (reader_guid.prefix == p->participant_guid_.prefix) {
        StatefulReader* r = p->find_reader(reader_guid.entityId);
        if (r) r->matched_writer_remove(writer_guid);
    }
}

// ============================================================
// ParseOps trampolines
// ============================================================

void RTPSParticipant::dispatch_data(const ParseContext& ctx,
                                    const ParsedData&   msg) noexcept
{
    auto* p = static_cast<RTPSParticipant*>(ctx.user_data);
    const uint32_t now_ms = RTPS_GET_MS();

    if (msg.writerId == entity_id::SPDP_BUILTIN_PARTICIPANT_WRITER) {
        p->spdp_reader_.on_data(ctx.senderGuidPrefix,
                                msg.serializedPayload,
                                msg.serializedPayloadSize, now_ms);
        return;
    }
    if (msg.writerId == entity_id::SEDP_BUILTIN_PUBLICATIONS_ANNOUNCER) {
        p->sedp_reader_.on_publication_data(ctx.senderGuidPrefix,
                                            msg.serializedPayload,
                                            msg.serializedPayloadSize);
        return;
    }
    if (msg.writerId == entity_id::SEDP_BUILTIN_SUBSCRIPTIONS_ANNOUNCER) {
        p->sedp_reader_.on_subscription_data(ctx.senderGuidPrefix,
                                             msg.serializedPayload,
                                             msg.serializedPayloadSize);
        return;
    }
    for (int i = 0; i < RTPS_MAX_USER_READERS; ++i) {
        if (!p->reader_alive_[i]) continue;
        auto* r = reinterpret_cast<StatefulReader*>(p->reader_storage_[i]);
        if (r->guid().entityId == msg.readerId ||
            msg.readerId == entity_id::UNKNOWN)
            r->on_data(ctx, msg);
    }
}

void RTPSParticipant::dispatch_data_frag(const ParseContext&   ctx,
                                         const ParsedDataFrag& msg) noexcept
{
    auto* p = static_cast<RTPSParticipant*>(ctx.user_data);
    for (int i = 0; i < RTPS_MAX_USER_READERS; ++i) {
        if (!p->reader_alive_[i]) continue;
        auto* r = reinterpret_cast<StatefulReader*>(p->reader_storage_[i]);
        if (r->guid().entityId == msg.readerId ||
            msg.readerId == entity_id::UNKNOWN)
            r->on_data_frag(ctx, msg);
    }
}

void RTPSParticipant::dispatch_heartbeat(const ParseContext&    ctx,
                                         const ParsedHeartbeat& msg) noexcept
{
    auto* p = static_cast<RTPSParticipant*>(ctx.user_data);
    for (int i = 0; i < RTPS_MAX_USER_READERS; ++i) {
        if (!p->reader_alive_[i]) continue;
        auto* r = reinterpret_cast<StatefulReader*>(p->reader_storage_[i]);
        if (r->guid().entityId == msg.readerId ||
            msg.readerId == entity_id::UNKNOWN)
            r->on_heartbeat(ctx, msg);
    }
}

void RTPSParticipant::dispatch_heartbeat_frag(const ParseContext&       ctx,
                                              const ParsedHeartbeatFrag& msg) noexcept
{
    auto* p = static_cast<RTPSParticipant*>(ctx.user_data);
    for (int i = 0; i < RTPS_MAX_USER_READERS; ++i) {
        if (!p->reader_alive_[i]) continue;
        auto* r = reinterpret_cast<StatefulReader*>(p->reader_storage_[i]);
        if (r->guid().entityId == msg.readerId ||
            msg.readerId == entity_id::UNKNOWN)
            r->on_heartbeat_frag(ctx, msg);
    }
}

void RTPSParticipant::dispatch_acknack(const ParseContext&  ctx,
                                       const ParsedAckNack& msg) noexcept
{
    auto* p = static_cast<RTPSParticipant*>(ctx.user_data);
    IStatefulWriter* iw = p->find_writer_iface(msg.writerId);
    if (iw) iw->on_acknack(ctx, msg);
}

void RTPSParticipant::dispatch_nack_frag(const ParseContext&   ctx,
                                         const ParsedNackFrag& msg) noexcept
{
    auto* p = static_cast<RTPSParticipant*>(ctx.user_data);
    IStatefulWriter* iw = p->find_writer_iface(msg.writerId);
    if (iw) iw->on_nack_frag(ctx, msg);
}

void RTPSParticipant::dispatch_gap(const ParseContext& ctx,
                                   const ParsedGap&   msg) noexcept
{
    auto* p = static_cast<RTPSParticipant*>(ctx.user_data);
    for (int i = 0; i < RTPS_MAX_USER_READERS; ++i) {
        if (!p->reader_alive_[i]) continue;
        auto* r = reinterpret_cast<StatefulReader*>(p->reader_storage_[i]);
        if (r->guid().entityId == msg.readerId ||
            msg.readerId == entity_id::UNKNOWN)
            r->on_gap(ctx, msg);
    }
}

// ============================================================
// Internal helpers
// ============================================================

int RTPSParticipant::find_free_writer_slot() const noexcept {
    for (int i = 0; i < RTPS_MAX_USER_WRITERS; ++i)
        if (!writer_alive_[i]) return i;
    return -1;
}

IStatefulWriter* RTPSParticipant::find_writer_iface(const EntityId& eid) noexcept {
    for (int i = 0; i < RTPS_MAX_USER_WRITERS; ++i) {
        if (!writer_alive_[i] || !writer_iface_[i]) continue;
        if (writer_iface_[i]->guid().entityId == eid)
            return writer_iface_[i];
    }
    return nullptr;
}

StatefulReader* RTPSParticipant::find_reader(const EntityId& eid) noexcept {
    for (int i = 0; i < RTPS_MAX_USER_READERS; ++i) {
        if (!reader_alive_[i]) continue;
        auto* r = reinterpret_cast<StatefulReader*>(reader_storage_[i]);
        if (r->guid().entityId == eid) return r;
    }
    return nullptr;
}

Locator RTPSParticipant::metatraffic_unicast_locator() const noexcept {
    const uint32_t port = 7400u + 250u * domain_id_ + 10u + 2u * participant_id_;
    return make_udpv4_locator(rtps_platform_get_ipv4_addr(),
                              static_cast<uint16_t>(port));
}

Locator RTPSParticipant::default_unicast_locator() const noexcept {
    const uint32_t port = 7400u + 250u * domain_id_ + 1u + 2u * participant_id_;
    return make_udpv4_locator(rtps_platform_get_ipv4_addr(),
                              static_cast<uint16_t>(port));
}

} // namespace rtps
