/**
 * @file rtps/participant.cpp
 * @brief RTPSParticipant implementation — §8.5.1 (OMG formal/2022-04-01).
 */

#include "participant.hpp"
#include "log.hpp"
#include <new>
#include <string.h>

namespace rtps {

namespace {

bool locator_address_is_unspecified(const Locator& locator) noexcept
{
    if (locator.kind == locator_kind::UDPv4) {
        return locator.address[12] == 0u &&
               locator.address[13] == 0u &&
               locator.address[14] == 0u &&
               locator.address[15] == 0u;
    }
    if (locator.kind == locator_kind::UDPv6) {
        for (uint8_t i = 0; i < 16u; ++i) {
            if (locator.address[i] != 0u) {
                return false;
            }
        }
        return true;
    }
    return true;
}

bool append_unique_locator(Locator* out,
                           uint8_t& count,
                           uint8_t cap,
                           const Locator& locator) noexcept
{
    if ((out == nullptr) || (count >= cap)) {
        return false;
    }

    if (locator_address_is_unspecified(locator)) {
        return false;
    }

    for (uint8_t i = 0; i < count; ++i) {
        if (out[i] == locator) {
            return true;
        }
    }

    out[count++] = locator;
    return true;
}

bool remove_locator(Locator* list,
                    uint8_t& count,
                    const Locator& locator) noexcept
{
    if ((list == nullptr) || (count == 0u)) {
        return false;
    }

    for (uint8_t i = 0; i < count; ++i) {
        if (list[i] != locator) {
            continue;
        }

        for (uint8_t j = static_cast<uint8_t>(i + 1u); j < count; ++j) {
            list[j - 1u] = list[j];
        }
        --count;
        list[count] = Locator{};
        return true;
    }

    return false;
}

uint16_t calc_metatraffic_unicast_port(uint8_t domain_id,
                                       uint8_t participant_id) noexcept
{
    return static_cast<uint16_t>(7400u + 250u * domain_id + 10u + 2u * participant_id);
}

uint16_t calc_default_unicast_port(uint8_t domain_id,
                                   uint8_t participant_id) noexcept
{
    return static_cast<uint16_t>(7400u + 250u * domain_id + 11u + 2u * participant_id);
}

Locator make_udpv6_locator_from_ptr(const uint8_t ipv6[16], uint16_t port) noexcept
{
    Locator l{};
    l.kind = locator_kind::UDPv6;
    l.port = port;
    if (ipv6 != nullptr) {
        memcpy(l.address, ipv6, 16u);
    }
    return l;
}

uint8_t copy_locators(Locator* out,
                      uint8_t cap,
                      const Locator* src,
                      uint8_t src_count) noexcept
{
    if ((out == nullptr) || (src == nullptr) || (cap == 0u)) {
        return 0u;
    }

    const uint8_t n = (src_count < cap) ? src_count : cap;
    for (uint8_t i = 0; i < n; ++i) {
        out[i] = src[i];
    }
    return n;
}

} // namespace

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
    , participant_guid_{rtps_platform_make_guid_prefix(participant_id),
                        entity_id::PARTICIPANT}
    , spdp_multicast_locator_(LOCATOR_INVALID)
    , match_table_(on_endpoint_match, on_endpoint_unmatch, this)
    , spdp_writer_(local_participant_data_,
                   transport,
                   LOCATOR_INVALID)
    , spdp_reader_(on_spdp_new_peer, on_spdp_peer_removed, this)
    , sedp_writer_(participant_guid_.prefix, transport)
    , sedp_reader_(match_table_)
{
    memset(writer_iface_, 0, sizeof(writer_iface_));
    memset(writer_alive_, 0, sizeof(writer_alive_));
    memset(reader_alive_, 0, sizeof(reader_alive_));

    // SPDPdiscoveredParticipantData — §8.5.3.2, Table 8.73, p. 160.
    local_participant_data_.guidPrefix      = participant_guid_.prefix;
    local_participant_data_.protocolVersion = PROTOCOLVERSION_2_5;
    local_participant_data_.vendorId        = vendor_id;
    local_participant_data_.leaseDuration   =
        Duration::from_ms(RTPS_SPDP_LEASE_DURATION_MS);

    local_participant_data_.metatrafficUnicastLocatorCount = 0u;
    local_participant_data_.metatrafficMulticastLocatorCount = 0u;
    local_participant_data_.defaultUnicastLocatorCount = 0u;
    local_participant_data_.defaultMulticastLocatorCount = 0u;

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

    refresh_spdp_multicast_locator();
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
    if (slot < 0) {
        RTPS_LOG_E("participant", "reader table full (max=%d)", RTPS_MAX_USER_READERS);
        return nullptr;
    }

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
    rd.unicastLocatorCount =
        append_default_unicast_locators(rd.unicastLocatorList,
                                        RTPS_MAX_LOCATORS_PER_PARTICIPANT);
    sedp_writer_.register_reader(rd);
    match_table_.add_reader(rd);

    RTPS_LOG_I("participant", "reader created: slot=%d topic=%s type=%s",
               slot, cfg.topicName.str, cfg.typeName.str);
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
    wd.unicastLocatorCount =
        append_default_unicast_locators(wd.unicastLocatorList,
                                        RTPS_MAX_LOCATORS_PER_PARTICIPANT);
    sedp_writer_.register_writer(wd);
    match_table_.add_writer(wd);
}

// ============================================================
// parse_received() / tick() / announce()
// ============================================================

ParseResult RTPSParticipant::parse_received(const Octet* buf,
                                            uint32_t     len) noexcept
{
    const ParseResult r = parse_message(buf, len, participant_guid_.prefix,
                                        parse_ops_, this);
    if (r != ParseResult::OK) {
        RTPS_LOG_W("participant", "parse_message failed: result=%d len=%lu",
                   (int)r, (unsigned long)len);
    }
    return r;
}

void RTPSParticipant::tick(uint32_t now_ms) noexcept {
    if (!has_addresses()) {
        return;
    }

    spdp_writer_.tick(now_ms);
    spdp_reader_.tick(now_ms);
    sedp_writer_.tick(now_ms);
    for (int i = 0; i < RTPS_MAX_USER_WRITERS; ++i)
        if (writer_alive_[i] && writer_iface_[i])
            writer_iface_[i]->tick(now_ms);
    for (int i = 0; i < RTPS_MAX_USER_READERS; ++i)
        if (reader_alive_[i])
            reinterpret_cast<StatefulReader*>(reader_storage_[i])->tick(now_ms);
}

void RTPSParticipant::announce() noexcept {
    if (!has_addresses()) {
        return;
    }

    spdp_writer_.announce();
}

bool RTPSParticipant::add_ipv4_address(uint32_t ipv4_be) noexcept
{
    const uint16_t meta_port = transport_.local_metatraffic_unicast_port() != 0u
        ? transport_.local_metatraffic_unicast_port()
        : calc_metatraffic_unicast_port(domain_id_, participant_id_);
    const uint16_t user_port = transport_.local_default_unicast_port() != 0u
        ? transport_.local_default_unicast_port()
        : calc_default_unicast_port(domain_id_, participant_id_);
    const Locator meta = make_udpv4_locator(
        ipv4_be, meta_port);
    const Locator user = make_udpv4_locator(
        ipv4_be, user_port);
    return add_address_locator_pair(meta, user);
}

bool RTPSParticipant::add_ipv6_address(const uint8_t ipv6[16]) noexcept
{
    const uint16_t meta_port = transport_.local_metatraffic_unicast_port() != 0u
        ? transport_.local_metatraffic_unicast_port()
        : calc_metatraffic_unicast_port(domain_id_, participant_id_);
    const uint16_t user_port = transport_.local_default_unicast_port() != 0u
        ? transport_.local_default_unicast_port()
        : calc_default_unicast_port(domain_id_, participant_id_);
    const Locator meta = make_udpv6_locator_from_ptr(
        ipv6, meta_port);
    const Locator user = make_udpv6_locator_from_ptr(
        ipv6, user_port);
    return add_address_locator_pair(meta, user);
}

bool RTPSParticipant::remove_ipv4_address(uint32_t ipv4_be) noexcept
{
    const uint16_t meta_port = transport_.local_metatraffic_unicast_port() != 0u
        ? transport_.local_metatraffic_unicast_port()
        : calc_metatraffic_unicast_port(domain_id_, participant_id_);
    const uint16_t user_port = transport_.local_default_unicast_port() != 0u
        ? transport_.local_default_unicast_port()
        : calc_default_unicast_port(domain_id_, participant_id_);
    const Locator meta = make_udpv4_locator(
        ipv4_be, meta_port);
    const Locator user = make_udpv4_locator(
        ipv4_be, user_port);
    return remove_address_locator_pair(meta, user);
}

bool RTPSParticipant::remove_ipv6_address(const uint8_t ipv6[16]) noexcept
{
    const uint16_t meta_port = transport_.local_metatraffic_unicast_port() != 0u
        ? transport_.local_metatraffic_unicast_port()
        : calc_metatraffic_unicast_port(domain_id_, participant_id_);
    const uint16_t user_port = transport_.local_default_unicast_port() != 0u
        ? transport_.local_default_unicast_port()
        : calc_default_unicast_port(domain_id_, participant_id_);
    const Locator meta = make_udpv6_locator_from_ptr(
        ipv6, meta_port);
    const Locator user = make_udpv6_locator_from_ptr(
        ipv6, user_port);
    return remove_address_locator_pair(meta, user);
}

void RTPSParticipant::clear_addresses() noexcept
{
    local_participant_data_.metatrafficUnicastLocatorCount = 0u;
    local_participant_data_.defaultUnicastLocatorCount = 0u;
    refresh_spdp_multicast_locator();
}

bool RTPSParticipant::has_addresses() const noexcept
{
    return local_participant_data_.metatrafficUnicastLocatorCount > 0u &&
           local_participant_data_.defaultUnicastLocatorCount > 0u;
}

// ============================================================
// Discovery callbacks
// ============================================================

void RTPSParticipant::on_spdp_new_peer(const PeerEntry& peer,
                                       void*            self) noexcept
{
    auto* p = static_cast<RTPSParticipant*>(self);
    RTPS_LOG_I("participant", "SPDP new peer %02x%02x%02x%02x%02x%02x",
               peer.data.guidPrefix.value[0], peer.data.guidPrefix.value[1],
               peer.data.guidPrefix.value[2], peer.data.guidPrefix.value[3],
               peer.data.guidPrefix.value[4], peer.data.guidPrefix.value[5]);
    if (peer.data.metatrafficUnicastLocatorCount == 0) {
        RTPS_LOG_W("participant", "new peer has no metatraffic locator, skipping SEDP");
        return;
    }
    p->sedp_writer_.on_peer_discovered(peer.data);
    const Locator& dest = peer.data.metatrafficUnicastLocatorList[0];
    p->sedp_writer_.announce_to_peer(peer.data.guidPrefix, dest);
    p->spdp_reader_.mark_sedp_announced(peer.data.guidPrefix);
}

void RTPSParticipant::on_spdp_peer_removed(const PeerEntry& peer,
                                           void*            self) noexcept
{
    auto* p = static_cast<RTPSParticipant*>(self);
    RTPS_LOG_I("participant", "SPDP peer removed %02x%02x%02x%02x%02x%02x",
               peer.data.guidPrefix.value[0], peer.data.guidPrefix.value[1],
               peer.data.guidPrefix.value[2], peer.data.guidPrefix.value[3],
               peer.data.guidPrefix.value[4], peer.data.guidPrefix.value[5]);

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
    p->sedp_writer_.on_peer_removed(peer.data.guidPrefix);
}

void RTPSParticipant::on_endpoint_match(const DiscoveredWriterData& wd,
                                        const DiscoveredReaderData& rd,
                                        void*                       self) noexcept
{
    auto* p = static_cast<RTPSParticipant*>(self);
    RTPS_LOG_I("participant", "endpoint match: topic=%s type=%s",
               wd.topicName.str, wd.typeName.str);

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
    RTPS_LOG_I("participant", "endpoint unmatch: writer %02x%02x%02x%02x reader %02x%02x%02x%02x",
               writer_guid.prefix.value[0], writer_guid.prefix.value[1],
               writer_guid.prefix.value[2], writer_guid.prefix.value[3],
               reader_guid.prefix.value[0], reader_guid.prefix.value[1],
               reader_guid.prefix.value[2], reader_guid.prefix.value[3]);
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
        // Ignore our own SPDP announcements (multicast loopback on host sockets).
        if (ctx.senderGuidPrefix == p->participant_guid_.prefix) {
            return;
        }
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
    RTPS_LOG_T("participant", "HEARTBEAT writer=%02x%02x%02x%02x first=%d:%lu last=%d:%lu",
               ctx.senderGuidPrefix.value[0], ctx.senderGuidPrefix.value[1],
               ctx.senderGuidPrefix.value[2], ctx.senderGuidPrefix.value[3],
               msg.firstSN.high, (unsigned long)msg.firstSN.low,
               msg.lastSN.high,  (unsigned long)msg.lastSN.low);
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
    RTPS_LOG_T("participant", "ACKNACK reader=%02x%02x%02x%02x base=%d:%lu count=%lu",
               ctx.senderGuidPrefix.value[0], ctx.senderGuidPrefix.value[1],
               ctx.senderGuidPrefix.value[2], ctx.senderGuidPrefix.value[3],
               msg.readerSNState.bitmapBase.high,
               (unsigned long)msg.readerSNState.bitmapBase.low,
               (unsigned long)msg.count);
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
    if (IStatefulWriter* builtin = sedp_writer_.find_writer_iface(eid)) {
        return builtin;
    }
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

uint8_t RTPSParticipant::append_metatraffic_unicast_locators(Locator* out,
                                                             uint8_t cap) const noexcept
{
    return copy_locators(out, cap,
                         local_participant_data_.metatrafficUnicastLocatorList,
                         local_participant_data_.metatrafficUnicastLocatorCount);
}

uint8_t RTPSParticipant::append_metatraffic_multicast_locators(Locator* out,
                                                               uint8_t cap) const noexcept
{
    return copy_locators(out, cap,
                         local_participant_data_.metatrafficMulticastLocatorList,
                         local_participant_data_.metatrafficMulticastLocatorCount);
}

uint8_t RTPSParticipant::append_default_unicast_locators(Locator* out,
                                                         uint8_t cap) const noexcept
{
    return copy_locators(out, cap,
                         local_participant_data_.defaultUnicastLocatorList,
                         local_participant_data_.defaultUnicastLocatorCount);
}

bool RTPSParticipant::add_address_locator_pair(const Locator& meta,
                                               const Locator& user) noexcept
{
    const uint8_t meta_count_before = local_participant_data_.metatrafficUnicastLocatorCount;
    const uint8_t user_count_before = local_participant_data_.defaultUnicastLocatorCount;

    const bool meta_ok = append_unique_locator(
        local_participant_data_.metatrafficUnicastLocatorList,
        local_participant_data_.metatrafficUnicastLocatorCount,
        RTPS_MAX_LOCATORS_PER_PARTICIPANT,
        meta);
    if (!meta_ok) {
        return false;
    }

    const bool user_ok = append_unique_locator(
        local_participant_data_.defaultUnicastLocatorList,
        local_participant_data_.defaultUnicastLocatorCount,
        RTPS_MAX_LOCATORS_PER_PARTICIPANT,
        user);
    if (!user_ok) {
        local_participant_data_.metatrafficUnicastLocatorCount = meta_count_before;
        local_participant_data_.defaultUnicastLocatorCount = user_count_before;
        return false;
    }

    refresh_spdp_multicast_locator();
    return true;
}

bool RTPSParticipant::remove_address_locator_pair(const Locator& meta,
                                                  const Locator& user) noexcept
{
    bool changed = false;
    changed |= remove_locator(local_participant_data_.metatrafficUnicastLocatorList,
                              local_participant_data_.metatrafficUnicastLocatorCount,
                              meta);
    changed |= remove_locator(local_participant_data_.defaultUnicastLocatorList,
                              local_participant_data_.defaultUnicastLocatorCount,
                              user);
    if (changed) {
        refresh_spdp_multicast_locator();
    }
    return changed;
}

void RTPSParticipant::refresh_spdp_multicast_locator() noexcept
{
    bool have_ipv4 = false;
    bool have_ipv6 = false;
    for (uint8_t i = 0; i < local_participant_data_.metatrafficUnicastLocatorCount; ++i) {
        const Long kind = local_participant_data_.metatrafficUnicastLocatorList[i].kind;
        have_ipv4 = have_ipv4 || (kind == locator_kind::UDPv4);
        have_ipv6 = have_ipv6 || (kind == locator_kind::UDPv6);
    }

    if (have_ipv4) {
        spdp_multicast_locator_ = spdp_multicast_locator_v4(domain_id_);
        local_participant_data_.metatrafficMulticastLocatorList[0] = spdp_multicast_locator_;
        local_participant_data_.metatrafficMulticastLocatorCount = 1u;
    } else if (have_ipv6) {
        spdp_multicast_locator_ = spdp_multicast_locator_v6(domain_id_);
        local_participant_data_.metatrafficMulticastLocatorList[0] = spdp_multicast_locator_;
        local_participant_data_.metatrafficMulticastLocatorCount = 1u;
    } else {
        spdp_multicast_locator_ = LOCATOR_INVALID;
        local_participant_data_.metatrafficMulticastLocatorCount = 0u;
    }

    spdp_writer_.set_dest(spdp_multicast_locator_);
}

} // namespace rtps
