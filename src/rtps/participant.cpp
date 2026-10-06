/**
 * @file rtps/participant.cpp
 * @brief RTPSParticipant implementation — §8.5.1 (OMG formal/2022-04-01).
 */

#include "participant.hpp"

#include <atomic>
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
    if ((out == nullptr) || locator_address_is_unspecified(locator)) {
        return false;
    }

    // A locator already listed is found before the capacity check, so
    // announcing an address again succeeds on a full list.
    for (uint8_t i = 0; i < count; ++i) {
        if (out[i] == locator) {
            return true;
        }
    }

    if (count >= cap) {
        return false;
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
// make_participant_guid_prefix()  — §8.2.4.2, §9.3.1.5
// ============================================================

GuidPrefix make_participant_guid_prefix(uint8_t participant_id) noexcept
{
    static std::atomic<uint8_t> instance{0};
    GuidPrefix prefix{};
    prefix.value[0] = VENDORID_THIS_IMPL.vendorId[0];
    prefix.value[1] = VENDORID_THIS_IMPL.vendorId[1];
    rosette::platform::device_unique_id(&prefix.value[2]);
    prefix.value[10] = instance.fetch_add(1u);
    prefix.value[11] = participant_id;
    return prefix;
}

// ============================================================

RTPSParticipant::RTPSParticipant(uint8_t domain_id, uint8_t participant_id, ITransport& transport,
                                 VendorId vendor_id) noexcept
    : domain_id_(domain_id),
      participant_id_(participant_id),
      transport_(transport),
      participant_guid_{make_participant_guid_prefix(participant_id), entity_id::PARTICIPANT},
      spdp_multicast_locator_(LOCATOR_INVALID),
      match_table_(on_endpoint_match, on_endpoint_unmatch, this, on_endpoints_changed),
      spdp_writer_(local_participant_data_, transport, LOCATOR_INVALID),
      spdp_reader_(on_spdp_new_peer, on_spdp_peer_removed, this),
      sedp_writer_(participant_guid_.prefix, transport, sedp_endpoint_source()),
      sedp_reader_(participant_guid_.prefix, transport, match_table_) {
    memset(writers_, 0, sizeof(writers_));
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

    // Only pairs with a local endpoint need proxies; remote-to-remote
    // matches would only consume match records.
    match_table_.set_local_prefix(participant_guid_.prefix);
    match_table_.set_local_subnets(local_subnets_);
    match_table_.set_participant_defaults([](const GuidPrefix& prefix, const Locator*& locators,
                                             uint8_t& count, void* self) noexcept {
        return static_cast<const RTPSParticipant*>(self)->peer_default_locators(prefix, locators,
                                                                                count);
    });
    spdp_reader_.set_local_subnets(local_subnets_);

    refresh_spdp_multicast_locator();
}

// ============================================================
// assign_entity_id()  — §8.2.4.3, §9.3.1.2 Table 9.1
// ============================================================

EntityId RTPSParticipant::assign_entity_id(const EntityId& requested,
                                           bool            is_writer,
                                           TopicKind       topic_kind) noexcept
{
    if (!(requested == entity_id::UNKNOWN)) {
        return requested;
    }

    const bool keyed = (topic_kind == TopicKind::WITH_KEY);
    const uint32_t key = next_entity_key_++;

    EntityId id{};
    id.entityKey[0] = static_cast<Octet>((key >> 16u) & 0xFFu);
    id.entityKey[1] = static_cast<Octet>((key >> 8u) & 0xFFu);
    id.entityKey[2] = static_cast<Octet>(key & 0xFFu);
    id.entityKind   = is_writer ? (keyed ? 0x02u : 0x03u)
                                : (keyed ? 0x07u : 0x04u);
    return id;
}

// ============================================================
// create_reader()
// ============================================================

StatefulReader* RTPSParticipant::create_reader(
        const ReaderConfig&               cfg,
        StatefulReader::OnChangeAvailable on_change_avail,
        void*                             user_data) noexcept
{
    if (!endpoint_names_fit(cfg.topicName, cfg.typeName)) return nullptr;

    int slot = -1;
    for (int i = 0; i < RTPS_MAX_USER_READERS; ++i)
        if (!reader_alive_[i]) { slot = i; break; }
    if (slot < 0) {
        RTPS_LOG_E("participant", "reader table full (max=%d)", RTPS_MAX_USER_READERS);
        return nullptr;
    }

    const Guid reader_guid{ participant_guid_.prefix,
                            assign_entity_id(cfg.entityId, false, cfg.topicKind) };

    StatefulReader* r = new (reader_storage_[slot]) DefaultStatefulReader(
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
    // The match table records the reader before SEDP announces it: a
    // rebuilding SedpWriter reads it back from there for every send.
    match_table_.add_reader(rd);
    sedp_writer_.register_reader(rd);
    owe_parked_peers();

    RTPS_LOG_I("participant", "reader created: slot=%d topic=%s type=%s",
               slot, cfg.topicName.str, cfg.typeName.str);
    return r;
}

// ============================================================
// delete_writer() / delete_reader()
// ============================================================

bool RTPSParticipant::delete_writer(StatefulWriter* writer) noexcept
{
    if (!writer) return false;
    for (int i = 0; i < RTPS_MAX_USER_WRITERS; ++i) {
        if (!writer_alive_[i] || writers_[i] != writer) continue;
        const Guid guid = writer->guid();
        sedp_writer_.unregister_writer(guid);
        owe_parked_peers();
        match_table_.remove_writer(guid);
        writer->~StatefulWriter();
        writers_[i] = nullptr;
        writer_alive_[i] = false;
        RTPS_LOG_I("participant", "writer deleted: slot=%d", i);
        return true;
    }
    return false;
}

bool RTPSParticipant::delete_reader(StatefulReader* reader) noexcept
{
    if (!reader) return false;
    for (int i = 0; i < RTPS_MAX_USER_READERS; ++i) {
        if (!reader_alive_[i] || user_reader(i) != reader) continue;
        const Guid guid = reader->guid();
        sedp_writer_.unregister_reader(guid);
        owe_parked_peers();
        match_table_.remove_reader(guid);
        user_reader(i)->~DefaultStatefulReader();
        reader_alive_[i] = false;
        RTPS_LOG_I("participant", "reader deleted: slot=%d", i);
        return true;
    }
    return false;
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
    // The match table records the writer before SEDP announces it: a
    // rebuilding SedpWriter reads it back from there for every send.
    match_table_.add_writer(wd);
    sedp_writer_.register_writer(wd);
    owe_parked_peers();
}

// ============================================================
// SEDP endpoint source
// ============================================================

LocalEndpointSource RTPSParticipant::sedp_endpoint_source() noexcept {
    if (RTPS_SEDP_REBUILD_ANNOUNCEMENTS == 0) {
        return LocalEndpointSource{};
    }

    return LocalEndpointSource{describe_local_writer, describe_local_reader, this};
}

[[gnu::cold]] bool RTPSParticipant::describe_local_writer(void* user, const Guid& guid,
                                                          DiscoveredWriterData& out) noexcept {
    const auto& self = *static_cast<const RTPSParticipant*>(user);
    if (!self.match_table_.local_writer_data(guid, out)) {
        return false;
    }

    out.unicastLocatorCount = self.append_default_unicast_locators(
        out.unicastLocatorList, RTPS_MAX_LOCATORS_PER_PARTICIPANT);
    return true;
}

[[gnu::cold]] bool RTPSParticipant::describe_local_reader(void* user, const Guid& guid,
                                                          DiscoveredReaderData& out) noexcept {
    const auto& self = *static_cast<const RTPSParticipant*>(user);
    if (!self.match_table_.local_reader_data(guid, out)) {
        return false;
    }

    out.unicastLocatorCount = self.append_default_unicast_locators(
        out.unicastLocatorList, RTPS_MAX_LOCATORS_PER_PARTICIPANT);
    return true;
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
    serve_owed_peer(now_ms);
    sedp_writer_.tick(now_ms);
    sedp_reader_.tick(now_ms);
    for (int i = 0; i < RTPS_MAX_USER_WRITERS; ++i)
        if (writer_alive_[i] && writers_[i])
            writers_[i]->tick(now_ms);
    for (int i = 0; i < RTPS_MAX_USER_READERS; ++i)
        if (reader_alive_[i])
            user_reader(i)->tick(now_ms);
}

uint32_t RTPSParticipant::ms_until_next_timer(uint32_t now_ms) const noexcept {
    if (!has_addresses()) return UINT32_MAX;
    uint32_t next = spdp_writer_.ms_until_next_announce(now_ms);
    auto consider = [&next](uint32_t candidate) noexcept {
        if (candidate < next) next = candidate;
    };
    consider(spdp_reader_.ms_until_next_expiry(now_ms));
    // An owed parked peer waits for a peer to settle; look again a few
    // times per settle period.
    bool owed = false;
    spdp_reader_.for_each_peer(
        [&owed](const PeerEntry& peer) noexcept { owed = owed || peer.owed; });
    if (owed) {
        consider(RTPS_PEER_SETTLE_MS / 4u);
    }

    consider(sedp_writer_.ms_until_next_timer(now_ms));
    consider(sedp_reader_.ms_until_next_timer(now_ms));
    for (int i = 0; i < RTPS_MAX_USER_WRITERS; ++i)
        if (writer_alive_[i] && writers_[i])
            consider(writers_[i]->ms_until_next_timer(now_ms));
    for (int i = 0; i < RTPS_MAX_USER_READERS; ++i)
        if (reader_alive_[i])
            consider(user_reader(i)->ms_until_next_timer(now_ms));
    return next;
}

void RTPSParticipant::announce() noexcept {
    if (!has_addresses()) {
        return;
    }

    spdp_writer_.announce();
}

bool RTPSParticipant::add_ipv4_address(uint32_t ipv4_be, uint8_t prefix_len) noexcept {
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
    return add_address_locator_pair(meta, user, prefix_len);
}

bool RTPSParticipant::add_ipv6_address(const uint8_t ipv6[16], uint8_t prefix_len) noexcept {
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
    return add_address_locator_pair(meta, user, prefix_len);
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
    local_subnets_.clear();
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
    // SPDP reports a new peer, and every announcement of one still
    // pending; it is activated when the active tier has room or a settled
    // peer can be parked for it (participant_peer_tiers.cpp).
    auto* p = static_cast<RTPSParticipant*>(self);
    PeerEntry* known = p->spdp_reader_.find_peer(peer.guidPrefix);
    if (known != nullptr) {
        (void)p->admit_peer(*known, rosette::platform::monotonic_ms());
    }
}

void RTPSParticipant::on_spdp_peer_removed(const PeerEntry& peer,
                                           void*            self) noexcept
{
    auto* p = static_cast<RTPSParticipant*>(self);
    RTPS_LOG_I("participant", "SPDP peer removed %02x%02x%02x%02x%02x%02x",
               peer.guidPrefix.value[0], peer.guidPrefix.value[1], peer.guidPrefix.value[2],
               peer.guidPrefix.value[3], peer.guidPrefix.value[4], peer.guidPrefix.value[5]);

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
    p->match_table_.remove_participant(peer.guidPrefix);
    p->sedp_writer_.on_peer_removed(peer.guidPrefix);
    p->sedp_reader_.on_peer_removed(peer.guidPrefix);
    p->notify_graph_change();
}

void RTPSParticipant::on_endpoints_changed(void* self) noexcept
{
    static_cast<RTPSParticipant*>(self)->notify_graph_change();
}

void RTPSParticipant::on_endpoint_match(const MatchedEndpoint& writer,
                                        const MatchedEndpoint& reader,
                                        const TopicName&       topic,
                                        void*                  self) noexcept
{
    auto* p = static_cast<RTPSParticipant*>(self);
    RTPS_LOG_I("participant", "endpoint match: topic=%s", topic.str);

    // Local writer matched to a reader.
    if (writer.guid.prefix == p->participant_guid_.prefix) {
        StatefulWriter* iw = p->find_writer(writer.guid.entityId);
        if (iw) {
            const Locator* locs = reader.locators;
            uint8_t count = reader.locatorCount;
            p->resolve_endpoint_locators(reader.guid.prefix, locs, count);
            iw->matched_reader_add(reader.guid, locs, count, reader.expectsInlineQos);
        }
    }

    // Local reader matched to a writer.
    if (reader.guid.prefix == p->participant_guid_.prefix) {
        StatefulReader* r = p->find_reader(reader.guid.entityId);
        if (r) {
            const Locator* locs = writer.locators;
            uint8_t count = writer.locatorCount;
            p->resolve_endpoint_locators(writer.guid.prefix, locs, count);
            r->matched_writer_add(writer.guid, locs, count, p->peer_vendor_id(writer.guid.prefix));
        }
    }
}

// An endpoint announcing its participant's defaults keeps the same number of
// them as the peer record does, so EndpointMatchTable finds them equal.
static_assert(RTPS_MAX_REMOTE_ENDPOINT_LOCATORS == RTPS_MAX_PEER_LOCATORS,
              "endpoint and participant locator limits must match");

bool RTPSParticipant::peer_default_locators(const GuidPrefix& prefix, const Locator*& locators,
                                            uint8_t& count) const noexcept {
    // Only an active peer has endpoints in the match table, and its slot
    // holds the default list.
    bool found = false;
    spdp_reader_.for_each_peer([&](const PeerEntry& peer) noexcept {
        const DiscoveredParticipant* data = spdp_reader_.active_data(peer);
        if (peer.guidPrefix != prefix || data == nullptr) {
            return;
        }

        locators = data->defaultUnicastLocatorList;
        count = data->defaultUnicastLocatorCount;
        found = true;
    });

    return found;
}

void RTPSParticipant::resolve_endpoint_locators(const GuidPrefix& remote_prefix,
                                                const Locator*& locators,
                                                uint8_t& count) const noexcept {
    if (count > 0u) {
        return;
    }

    if (remote_prefix == participant_guid_.prefix) {
        locators = local_participant_data_.defaultUnicastLocatorList;
        count = local_participant_data_.defaultUnicastLocatorCount;
        return;
    }

    peer_default_locators(remote_prefix, locators, count);
}

VendorId RTPSParticipant::peer_vendor_id(const GuidPrefix& remote_prefix) const noexcept {
    VendorId vendor = VENDORID_UNKNOWN;
    spdp_reader_.for_each_peer([&](const PeerEntry& peer) noexcept {
        if (peer.guidPrefix == remote_prefix) {
            vendor = peer.vendorId;
        }
    });

    return vendor;
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
        StatefulWriter* iw = p->find_writer(writer_guid.entityId);
        if (iw) iw->matched_reader_remove(reader_guid);
    }
    if (reader_guid.prefix == p->participant_guid_.prefix) {
        StatefulReader* r = p->find_reader(reader_guid.entityId);
        if (r) r->matched_writer_remove(writer_guid);
    }
}

// ============================================================
// Reader-side submessage routing
// ============================================================

/**
 * @brief Calls `deliver` on every local reader a reader-side submessage is for.
 *
 * DATA, DATA_FRAG, HEARTBEAT, HEARTBEAT_FRAG and GAP all travel from a writer
 * to a reader, and are routed the same way. A submessage from a SEDP
 * announcer goes to the builtin SEDP reader matched with it, whatever
 * readerId it names: builtin EntityIds are reserved (§9.3.1.2), and stock
 * implementations address SEDP traffic to ENTITYID_UNKNOWN as often as to
 * the detector. Any other submessage goes to the user reader named by
 * readerId, or to every user reader when readerId is ENTITYID_UNKNOWN
 * (§8.3.7, readerId semantics); each reader then ignores writers it has not
 * matched.
 *
 * @param[in] writer_id Submessage writerId.
 * @param[in] reader_id Submessage readerId.
 * @param[in] deliver   Callable taking StatefulReader&.
 *
 * Verification:
 * - Correct means: SEDP traffic never reaches a user reader, and user
 *   traffic reaches exactly the addressed readers.
 * - Covered by: tests/ros2/run_rmw_interop_loopback.sh (discovery and topic
 *   traffic against Fast DDS). A unit test would parse a SEDP DATA with
 *   readerId ENTITYID_UNKNOWN and check that no user reader sees it.
 *
 * References:
 * - OMG DDSI-RTPS 2.5 §8.3.7 (submessage readerId), §9.3.1.2 (builtin
 *   EntityIds)
 */
template <typename Deliver>
void RTPSParticipant::for_each_addressed_reader(const EntityId& writer_id,
                                                const EntityId& reader_id,
                                                Deliver&&       deliver) noexcept
{
    if (StatefulReader* builtin = sedp_reader_.reader_for_writer(writer_id)) {
        deliver(*builtin);
        return;
    }
    for (int i = 0; i < RTPS_MAX_USER_READERS; ++i) {
        if (!reader_alive_[i]) continue;
        StatefulReader& r = *user_reader(i);
        if (r.guid().entityId == reader_id || reader_id == entity_id::UNKNOWN)
            deliver(r);
    }
}

// ============================================================
// ParseOps trampolines
// ============================================================

/// True for the writerId of a remote participant's SEDP announcer (§8.5.4.3).
static bool is_sedp_announcer(const EntityId& writer_id) noexcept {
    return writer_id == entity_id::SEDP_BUILTIN_PUBLICATIONS_ANNOUNCER
           || writer_id == entity_id::SEDP_BUILTIN_SUBSCRIPTIONS_ANNOUNCER;
}

void RTPSParticipant::dispatch_data(const ParseContext& ctx,
                                    const ParsedData&   msg) noexcept
{
    auto* p = static_cast<RTPSParticipant*>(ctx.user_data);
    const uint32_t now_ms = rosette::platform::monotonic_ms();

    if (msg.writerId == entity_id::SPDP_BUILTIN_PARTICIPANT_WRITER) {
        // Ignore our own SPDP announcements (multicast loopback on host sockets).
        if (ctx.senderGuidPrefix == p->participant_guid_.prefix) {
            return;
        }
        // A peer leaving cleanly disposes its SPDP instance (§8.5.3.2).
        if (msg.inlineQos.withdraws_instance()) {
            const GuidPrefix gone = msg.inlineQos.hasKeyHash
                ? msg.inlineQos.key_hash_guid().prefix : ctx.senderGuidPrefix;
            p->spdp_reader_.remove_peer(gone);
            return;
        }
        p->spdp_reader_.on_data(ctx.senderGuidPrefix,
                                msg.serializedPayload,
                                msg.serializedPayloadSize, now_ms);
        return;
    }

    if (is_sedp_announcer(msg.writerId)) {
        p->on_announcer_traffic(ctx.senderGuidPrefix, msg.writerId, msg.writerSN, true, now_ms);
    }

    p->for_each_addressed_reader(msg.writerId, msg.readerId, [&](StatefulReader& r) noexcept {
        r.on_data(ctx, msg);
    });
}

void RTPSParticipant::dispatch_data_frag(const ParseContext&   ctx,
                                         const ParsedDataFrag& msg) noexcept
{
    auto* p = static_cast<RTPSParticipant*>(ctx.user_data);
    p->for_each_addressed_reader(msg.writerId, msg.readerId, [&](StatefulReader& r) noexcept {
        r.on_data_frag(ctx, msg);
    });
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
    if (is_sedp_announcer(msg.writerId)) {
        p->on_announcer_traffic(ctx.senderGuidPrefix, msg.writerId, msg.lastSN, false,
                                rosette::platform::monotonic_ms());
    }

    p->for_each_addressed_reader(msg.writerId, msg.readerId, [&](StatefulReader& r) noexcept {
        r.on_heartbeat(ctx, msg);
    });
}

void RTPSParticipant::dispatch_heartbeat_frag(const ParseContext&       ctx,
                                              const ParsedHeartbeatFrag& msg) noexcept
{
    auto* p = static_cast<RTPSParticipant*>(ctx.user_data);
    p->for_each_addressed_reader(msg.writerId, msg.readerId, [&](StatefulReader& r) noexcept {
        r.on_heartbeat_frag(ctx, msg);
    });
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
    StatefulWriter* iw = p->find_writer(msg.writerId);
    if (iw) iw->on_acknack(ctx, msg);
}

void RTPSParticipant::dispatch_nack_frag(const ParseContext&   ctx,
                                         const ParsedNackFrag& msg) noexcept
{
    auto* p = static_cast<RTPSParticipant*>(ctx.user_data);
    StatefulWriter* iw = p->find_writer(msg.writerId);
    if (iw) iw->on_nack_frag(ctx, msg);
}

void RTPSParticipant::dispatch_gap(const ParseContext& ctx,
                                   const ParsedGap&   msg) noexcept
{
    auto* p = static_cast<RTPSParticipant*>(ctx.user_data);
    p->for_each_addressed_reader(msg.writerId, msg.readerId, [&](StatefulReader& r) noexcept {
        r.on_gap(ctx, msg);
    });
}

// ============================================================
// Internal helpers
// ============================================================

bool RTPSParticipant::endpoint_names_fit(const TopicName& topic,
                                         const TopicName& type) noexcept
{
    if (topic.fits() && type.fits()) return true;
    RTPS_LOG_E("participant", "endpoint name longer than %d octets refused: topic=%s type=%s",
               RTPS_MAX_TOPIC_NAME_LEN - 1, topic.str, type.str);
    return false;
}

int RTPSParticipant::find_free_writer_slot() const noexcept {
    for (int i = 0; i < RTPS_MAX_USER_WRITERS; ++i)
        if (!writer_alive_[i]) return i;
    return -1;
}

StatefulWriter* RTPSParticipant::find_writer(const EntityId& eid) noexcept {
    if (StatefulWriter* builtin = sedp_writer_.find_writer(eid)) {
        return builtin;
    }
    for (int i = 0; i < RTPS_MAX_USER_WRITERS; ++i) {
        if (!writer_alive_[i] || !writers_[i]) continue;
        if (writers_[i]->guid().entityId == eid)
            return writers_[i];
    }
    return nullptr;
}

StatefulReader* RTPSParticipant::find_reader(const EntityId& eid) noexcept {
    for (int i = 0; i < RTPS_MAX_USER_READERS; ++i) {
        if (!reader_alive_[i]) continue;
        if (user_reader(i)->guid().entityId == eid) return user_reader(i);
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

bool RTPSParticipant::add_address_locator_pair(const Locator& meta, const Locator& user,
                                               uint8_t prefix_len) noexcept {
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
    if (!user_ok || !local_subnets_.add(meta, prefix_len)) {
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
    changed |= local_subnets_.remove(meta);
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
