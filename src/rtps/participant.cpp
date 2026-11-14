// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file rtps/participant.cpp
 * @brief RTPSParticipant implementation — §8.5.1 (OMG formal/2022-04-01).
 */

#include "participant.hpp"

#include <string.h>
#include <algorithm>
#include <atomic>
#include <new>
#include "log.hpp"
#include "rosette/support/fnv1a.hpp"

namespace rosette::rtps {

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
    uint8_t unique_id[8] = {};
    rosette::platform::device_unique_id(unique_id);
    const uint32_t folded = rosette::support::fnv1a32(unique_id, sizeof(unique_id));
    const uint32_t nonce = rosette::platform::boot_nonce();
    for (int i = 0; i < 4; ++i) {
        prefix.value[2 + i] = static_cast<uint8_t>(folded >> (24 - 8 * i));
        prefix.value[6 + i] = static_cast<uint8_t>(nonce >> (24 - 8 * i));
    }

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

    // Every endpoint slot's floor is promised from the start, so an
    // endpoint's first matches never depend on creation order. The
    // static_asserts in participant.hpp prove both reservations fit.
    (void)reader_proxies_.reserve(RTPS_MAX_USER_WRITERS * RTPS_MIN_MATCHES_PER_ENDPOINT);
    (void)writer_proxies_.reserve(RTPS_MAX_USER_READERS * RTPS_MIN_MATCHES_PER_ENDPOINT);

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
    match_table_.set_may_match(may_bind_pair);
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
    timers_stale_ = true;
    if (!local_endpoint_fits(cfg.topicName, cfg.typeName, /*is_writer=*/false)) {
        return nullptr;
    }

    int slot = -1;
    for (int i = 0; i < RTPS_MAX_USER_READERS; ++i)
        if (!reader_alive_[i]) { slot = i; break; }
    if (slot < 0) {
        RTPS_LOG_E("participant", "reader table full (max=%d)", RTPS_MAX_USER_READERS);
        return nullptr;
    }

    uint8_t floor = 0u;
    uint8_t cap = 0u;
    if (!reserve_endpoint_floor(writer_proxies_, cfg.minMatchedWriters, cfg.maxMatchedWriters,
                                floor, cap)) {
        return nullptr;
    }

    const Guid reader_guid{ participant_guid_.prefix,
                            assign_entity_id(cfg.entityId, false, cfg.topicKind) };

    StatefulReader* r = new (reader_storage_[slot]) SharedPoolStatefulReader(
        reader_guid, cfg.reliability, transport_, writer_proxies_, floor, cap, assembly_slots_,
        RTPS_ASSEMBLY_SLOTS_PER_READER, on_change_avail, user_data, cfg.hbResponseMs);

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
    timers_stale_ = true;
    for (int i = 0; i < RTPS_MAX_USER_WRITERS; ++i) {
        if (!writer_alive_[i] || writers_[i] != writer) continue;
        const Guid guid = writer->guid();
        sedp_writer_.unregister_writer(guid);
        owe_parked_peers();
        match_table_.remove_writer(guid);
        const uint8_t floor = writer->min_matched_readers();
        evicted_by_deleted_writers_ += writer->evicted_unacknowledged();
        writer->~StatefulWriter();
        release_endpoint_floor(reader_proxies_, floor);
        rebind_due_ = true;
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
    timers_stale_ = true;
    for (int i = 0; i < RTPS_MAX_USER_READERS; ++i) {
        if (!reader_alive_[i] || user_reader(i) != reader) continue;
        const Guid guid = reader->guid();
        sedp_writer_.unregister_reader(guid);
        owe_parked_peers();
        match_table_.remove_reader(guid);
        const uint8_t floor = reader->min_matched_writers();
        user_reader(i)->~SharedPoolStatefulReader();
        release_endpoint_floor(writer_proxies_, floor);
        rebind_due_ = true;
        reader_alive_[i] = false;
        RTPS_LOG_I("participant", "reader deleted: slot=%d", i);
        return true;
    }
    return false;
}

// ============================================================
// Endpoint floors in the proxy pools
// ============================================================

bool RTPSParticipant::reserve_endpoint_floor(ProxyPool& pool, uint8_t requested_min,
                                             uint8_t requested_max, uint8_t& floor,
                                             uint8_t& cap) noexcept {
    floor = requested_min > RTPS_MIN_MATCHES_PER_ENDPOINT ? requested_min
                                                          : RTPS_MIN_MATCHES_PER_ENDPOINT;
    cap = requested_max > floor ? requested_max : floor;
    if (cap == 0u) {
        cap = 1u;
    }

    const uint8_t above_slot = static_cast<uint8_t>(floor - RTPS_MIN_MATCHES_PER_ENDPOINT);
    if (!pool.reserve(above_slot)) {
        RTPS_LOG_E("participant", "proxy pool cannot reserve a floor of %u (free=%u reserved=%u)",
                   static_cast<unsigned>(floor), static_cast<unsigned>(pool.free_count()),
                   static_cast<unsigned>(pool.reserved()));
        return false;
    }

    return true;
}

void RTPSParticipant::release_endpoint_floor(ProxyPool& pool, uint8_t floor) noexcept {
    pool.unreserve(static_cast<uint8_t>(floor - RTPS_MIN_MATCHES_PER_ENDPOINT));
}

// ============================================================
// register_writer_endpoint()  (called from create_writer<D>())
// ============================================================

void RTPSParticipant::register_writer_endpoint(const Guid&         writer_guid,
                                               const WriterConfig& cfg) noexcept
{
    timers_stale_ = true;
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
        RTPS_LOG_W("participant", "parse_message failed: result=%d len=%lu", static_cast<int>(r),
                   static_cast<unsigned long>(len));
    }
    return r;
}

void RTPSParticipant::release_replies_to_bound_readers() noexcept {
    for (int i = 0; i < RTPS_MAX_USER_WRITERS; ++i) {
        if (writer_alive_[i] && writers_[i]) {
            writers_[i]->on_waiting_reader_bound();
        }
    }
}

void RTPSParticipant::tick(uint32_t now_ms) noexcept {
    if (!has_addresses()) {
        return;
    }

    if (!timers_stale_ && static_cast<int32_t>(next_timer_due_ms_ - now_ms) > 0) {
        return;
    }

    // Cleared before the walk, so a match or peer change made during it
    // leaves the timers stale and the next pass walks again.
    timers_stale_ = false;

    uint32_t next = spdp_writer_.tick(now_ms);
    next = std::min(next, spdp_reader_.tick(now_ms));
    serve_owed_peer(now_ms);
    next = std::min(next, sedp_writer_.tick(now_ms));
    next = std::min(next, sedp_reader_.tick(now_ms));
    finish_peer_refreshes();
    // Both steps are flag checks until a match frees room: a rebind is due
    // only after a proxy frees, and a rescan only after a pair was turned
    // away and a record or waiting place has freed since. Waiting pairs are
    // older than any pair the rescan finds, so a tick that binds one skips
    // the rescan.
    if (rebind_due_) {
        rebind_due_ = match_table_.match_oldest_waiting();
        if (rebind_due_) {
            release_replies_to_bound_readers();
        }
    }

    if (!rebind_due_ && match_table_.rescan_due()) {
        match_table_.rescan_unrecorded();
    }

    // The user endpoints come after every step that can match them, so the
    // wait each one returns already counts what those steps did.
    for (int i = 0; i < RTPS_MAX_USER_WRITERS; ++i) {
        if (writer_alive_[i] && writers_[i]) {
            next = std::min(next, writers_[i]->tick(now_ms));
        }
    }

    for (int i = 0; i < RTPS_MAX_USER_READERS; ++i) {
        if (reader_alive_[i]) {
            next = std::min(next, user_reader(i)->tick(now_ms));
        }
    }

    // An owed parked peer waits for a peer to settle, and a reactivated
    // peer's refresh for its replay; look again a few times per settle period.
    if (peers_settling()) {
        next = std::min<uint32_t>(next, RTPS_PEER_SETTLE_MS / 4u);
    }

    if (rebind_due_ || match_table_.rescan_due()) {
        next = 0u;
    }

    // The SPDP period bounds the wait whenever there is an address; the cap
    // keeps the deadline comparison in tick() wrap-safe regardless.
    next_timer_due_ms_ = now_ms + std::min<uint32_t>(next, 0x7FFFFFFFu);
}

uint32_t RTPSParticipant::ms_until_next_timer(uint32_t now_ms) const noexcept {
    if (!has_addresses()) {
        return UINT32_MAX;
    }

    const int32_t left = static_cast<int32_t>(next_timer_due_ms_ - now_ms);
    return timers_stale_ || left <= 0 ? 0u : static_cast<uint32_t>(left);
}

bool RTPSParticipant::peers_settling() const noexcept {
    bool settling = false;
    spdp_reader_.for_each_peer([&settling](const PeerEntry& peer) noexcept {
        const bool refreshing =
            peer.active() && (peer.parkedPublicationsSn != 0u || peer.parkedSubscriptionsSn != 0u);
        settling = settling || peer.owed || refreshing;
    });
    return settling;
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
    timers_stale_ = true;
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
    p->timers_stale_ = true;
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

bool RTPSParticipant::on_endpoint_match(const MatchedEndpoint&       writer,
                                        const MatchedEndpoint&       reader,
                                        [[maybe_unused]] const char* topic, void* self) noexcept {
    auto* p = static_cast<RTPSParticipant*>(self);
    bool bound = true;

    // Local writer matched to a reader.
    if (writer.guid.prefix == p->participant_guid_.prefix) {
        StatefulWriter* iw = p->find_writer(writer.guid.entityId);
        if (iw && !iw->matched_reader_lookup(reader.guid)) {
            const Locator* locs = reader.locators;
            uint8_t count = reader.locatorCount;
            p->resolve_endpoint_locators(reader.guid.prefix, locs, count);
            bound = iw->matched_reader_add(reader.guid, locs, count, reader.expectsInlineQos,
                                           reader.reliability);
        }
    }

    // Local reader matched to a writer.
    if (reader.guid.prefix == p->participant_guid_.prefix) {
        StatefulReader* r = p->find_reader(reader.guid.entityId);
        if (r && !r->matched_writer_lookup(writer.guid)) {
            const Locator* locs = writer.locators;
            uint8_t count = writer.locatorCount;
            p->resolve_endpoint_locators(writer.guid.prefix, locs, count);
            bound = r->matched_writer_add(writer.guid, locs, count,
                                          p->peer_vendor_id(writer.guid.prefix))
                    && bound;
        }
    }

    if (bound) {
        RTPS_LOG_I("participant", "endpoint match: topic=%s", topic);
    } else {
        RTPS_LOG_I("participant", "endpoint match waits for a proxy: topic=%s", topic);
    }

    return bound;
}

bool RTPSParticipant::may_bind_pair(const Guid& writer_guid, const Guid& reader_guid,
                                    void* self) noexcept {
    auto* p = static_cast<RTPSParticipant*>(self);
    if (writer_guid.prefix == p->participant_guid_.prefix) {
        StatefulWriter* w = p->find_writer(writer_guid.entityId);
        if (w && !w->matched_reader_lookup(reader_guid) && !w->may_add_reader()) {
            return false;
        }
    }

    if (reader_guid.prefix == p->participant_guid_.prefix) {
        StatefulReader* r = p->find_reader(reader_guid.entityId);
        if (r && !r->matched_writer_lookup(writer_guid) && !r->may_add_writer()) {
            return false;
        }
    }

    return true;
}

// An endpoint announcing its participant's defaults keeps the same number of
// them as the peer record does, so EndpointMatchTable finds them equal.
static_assert(RTPS_MAX_REMOTE_ENDPOINT_LOCATORS == RTPS_MAX_PEER_LOCATORS,
              "endpoint and participant locator limits must match");

bool RTPSParticipant::peer_default_locators(const GuidPrefix& prefix, const Locator*& locators,
                                            uint8_t& count) const noexcept {
    // Only an active peer has endpoints in the match table, and its slot
    // holds the default list.
    const PeerEntry* peer = spdp_reader_.find_peer(prefix);
    const DiscoveredParticipant* data = peer ? spdp_reader_.active_data(*peer) : nullptr;
    if (data == nullptr) {
        return false;
    }

    locators = data->defaultUnicastLocatorList;
    count = data->defaultUnicastLocatorCount;
    return true;
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
    const PeerEntry* peer = spdp_reader_.find_peer(remote_prefix);
    return peer ? peer->vendorId : VENDORID_UNKNOWN;
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
        if (iw) {
            iw->matched_reader_remove(reader_guid);
        }
    }

    if (reader_guid.prefix == p->participant_guid_.prefix) {
        StatefulReader* r = p->find_reader(reader_guid.entityId);
        if (r) {
            r->matched_writer_remove(writer_guid);
        }
    }

    // The freed proxy may let a waiting match bind at the next tick(), which
    // is then due at once.
    p->rebind_due_ = true;
    p->timers_stale_ = true;
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
    timers_stale_ = true;
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

        // A renewed lease moves that peer's expiry later, which the timers
        // survive: tick() walks early and finds nothing. Only a lease
        // shorter than the time left could expire unseen. A peer that is
        // new, or still asking for an active slot, marks the timers stale
        // itself (on_spdp_new_peer()), so the hundred renewals a large
        // network sends between two timers cost no walk.
        const PeerEntry* renewed = p->spdp_reader_.on_data(
            ctx.senderGuidPrefix, msg.serializedPayload, msg.serializedPayloadSize, now_ms);
        if (renewed != nullptr && renewed->leaseMs < p->ms_until_next_timer(now_ms)) {
            p->timers_stale_ = true;
        }

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
               static_cast<int>(msg.firstSN.high), static_cast<unsigned long>(msg.firstSN.low),
               static_cast<int>(msg.lastSN.high), static_cast<unsigned long>(msg.lastSN.low));
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
               static_cast<int>(msg.readerSNState.bitmapBase.high),
               static_cast<unsigned long>(msg.readerSNState.bitmapBase.low),
               static_cast<unsigned long>(msg.count));
    p->timers_stale_ = true;
    StatefulWriter* iw = p->find_writer(msg.writerId);
    if (iw) {
        iw->on_acknack(ctx, msg);
    }
}

void RTPSParticipant::dispatch_nack_frag(const ParseContext&   ctx,
                                         const ParsedNackFrag& msg) noexcept
{
    auto* p = static_cast<RTPSParticipant*>(ctx.user_data);
    p->timers_stale_ = true;
    StatefulWriter* iw = p->find_writer(msg.writerId);
    if (iw) {
        iw->on_nack_frag(ctx, msg);
    }
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

bool RTPSParticipant::local_endpoint_fits(const TopicName& topic, const TopicName& type,
                                          bool is_writer) const noexcept {
    if (!topic.fits() || !type.fits()) {
        RTPS_LOG_E("participant", "endpoint name longer than %d octets refused: topic=%s type=%s",
                   RTPS_MAX_TOPIC_NAME_LEN - 1, topic.str, type.str);
        return false;
    }

    if (!match_table_.has_room_for_local(topic, type, is_writer)) {
        RTPS_LOG_E("participant",
                   "%s topic=%s refused: no %s record (RTPS_MAX_LOCAL_%s) or name room "
                   "(RTPS_LOCAL_NAME_BYTES=%lu, %lu used) in the match table",
                   is_writer ? "writer" : "reader", topic.str, is_writer ? "writer" : "reader",
                   is_writer ? "WRITERS" : "READERS",
                   static_cast<unsigned long>(RTPS_LOCAL_NAME_BYTES),
                   static_cast<unsigned long>(match_table_.local_name_bytes_used()));
        return false;
    }

    return true;
}

int RTPSParticipant::find_free_writer_slot(bool shallow, bool fits) const noexcept {
    if (!fits) {
        return -1;
    }

    const uint32_t first = shallow ? kDeepWriterSlots : 0u;
    const uint32_t end = shallow ? RTPS_MAX_USER_WRITERS : kDeepWriterSlots;
    for (uint32_t i = first; i < end; ++i) {
        if (!writer_alive_[i]) {
            return static_cast<int>(i);
        }
    }

    return -1;
}

Octet* RTPSParticipant::writer_slot_storage(int slot) noexcept {
    const uint32_t i = static_cast<uint32_t>(slot);
    return i < kDeepWriterSlots ? deep_writer_storage_.slot(i)
                                : shallow_writer_storage_.slot(i - kDeepWriterSlots);
}

uint32_t RTPSParticipant::evicted_unacknowledged() const noexcept {
    uint32_t evicted = evicted_by_deleted_writers_;
    for (uint32_t i = 0u; i < RTPS_MAX_USER_WRITERS; ++i) {
        if (writer_alive_[i] && writers_[i] != nullptr) {
            evicted += writers_[i]->evicted_unacknowledged();
        }
    }

    return evicted;
}

uint32_t RTPSParticipant::free_writer_slots(bool shallow) const noexcept {
    const uint32_t first = shallow ? kDeepWriterSlots : 0u;
    const uint32_t end = shallow ? RTPS_MAX_USER_WRITERS : kDeepWriterSlots;
    uint32_t       free = 0u;
    for (uint32_t i = first; i < end; ++i) {
        if (!writer_alive_[i]) {
            ++free;
        }
    }

    return free;
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
    timers_stale_ = true;
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
    timers_stale_ = true;
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

}  // namespace rosette::rtps
