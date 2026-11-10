// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file test_participant_8_5_3_peer_tiers.cpp
 * @brief Known and active peers: admission, parking and reactivation
 *        (src/rtps/participant_peer_tiers.cpp).
 *
 * Built into participant_tier_spec_tests with RTPS_MAX_PEERS=2,
 * RTPS_MAX_KNOWN_PEERS=6 and RTPS_PEER_SETTLE_MS=100, so a third peer
 * already finds the active tier full.
 */

#include "participant_test_support.hpp"

#include <map>

namespace spec_test {

extern uint32_t g_participant_test_now_ms;

#define CHECK(COND, MSG)  \
    do {                  \
        if (!(COND)) {    \
            err = (MSG);  \
            return false; \
        }                 \
    } while (0)

using namespace participant_support;

namespace {

static_assert(RTPS_MAX_PEERS == 2 && RTPS_PEER_SETTLE_MS == 100u,
              "these tests assume two active slots and a 100 ms settle time");

/// Delivers a peer's SPDP announcement at `now_ms`.
bool announce(RTPSParticipant& p, const GuidPrefix& prefix, Octet host, uint32_t now_ms) {
    std::vector<Octet> payload{};
    const Locator meta = make_udpv4_locator(0xC0A80100u | host, 7410u);
    const Locator user = make_udpv4_locator(0xC0A80100u | host, 7411u);
    if (!build_spdp_payload(prefix, 30u, meta, payload, user)) {
        return false;
    }

    g_participant_test_now_ms = now_ms;
    return p.spdp_reader().on_data(prefix, payload.data(), static_cast<uint32_t>(payload.size()),
                                   now_ms);
}

/// Delivers, at `now_ms`, one SEDP announcement from a peer's announcer. The
/// endpoint's entity key is `key`, or `sn` when `key` is 0.
bool deliver_announcement(RTPSParticipant& p, const GuidPrefix& prefix, bool publication,
                          const char* topic, int64_t sn, uint32_t now_ms, Octet key = 0u) {
    const Octet        entity_key = key != 0u ? key : static_cast<Octet>(sn);
    std::vector<Octet> payload{};
    if (publication) {
        DiscoveredWriterData wd{};
        wd.writerGuid = Guid{prefix, make_entity(0x71u, 0x72u, entity_key, 0x03u)};
        wd.topicName.assign(topic);
        wd.typeName.assign("pkg::msg::dds_::T_");
        wd.reliabilityKind = ReliabilityKind::RELIABLE;
        if (!build_publication_payload(wd, payload)) {
            return false;
        }
    } else {
        DiscoveredReaderData rd{};
        rd.readerGuid = Guid{prefix, make_entity(0x61u, 0x62u, entity_key, 0x04u)};
        rd.topicName.assign(topic);
        rd.typeName.assign("pkg::msg::dds_::T_");
        rd.reliabilityKind = ReliabilityKind::RELIABLE;
        if (!build_subscription_payload(rd, payload)) {
            return false;
        }
    }

    const EntityId writer = publication ? entity_id::SEDP_BUILTIN_PUBLICATIONS_ANNOUNCER
                                        : entity_id::SEDP_BUILTIN_SUBSCRIPTIONS_ANNOUNCER;
    const EntityId reader = publication ? entity_id::SEDP_BUILTIN_PUBLICATIONS_DETECTOR
                                        : entity_id::SEDP_BUILTIN_SUBSCRIPTIONS_DETECTOR;
    std::vector<Octet> msg{};
    if (!build_data_message(prefix, reader, writer, payload, msg, SequenceNumber::from_int64(sn))) {
        return false;
    }

    g_participant_test_now_ms = now_ms;
    return p.parse_received(msg.data(), static_cast<uint32_t>(msg.size())) == ParseResult::OK;
}

/// Delivers a peer's ACKNACK acknowledging changes 1..`acked` of one local SEDP writer.
bool acknowledge_sedp(RTPSParticipant& p, const GuidPrefix& prefix, bool publications,
                      int64_t acked) {
    SequenceNumberSet state{};
    state.bitmapBase = SequenceNumber::from_int64(acked + 1);
    state.numBits = 0u;
    const EntityId reader = publications ? entity_id::SEDP_BUILTIN_PUBLICATIONS_DETECTOR
                                         : entity_id::SEDP_BUILTIN_SUBSCRIPTIONS_DETECTOR;
    const EntityId writer = publications ? entity_id::SEDP_BUILTIN_PUBLICATIONS_ANNOUNCER
                                         : entity_id::SEDP_BUILTIN_SUBSCRIPTIONS_ANNOUNCER;
    std::vector<Octet> msg{};
    if (!build_acknack_message(prefix, reader, writer, state, 1, true, msg)) {
        return false;
    }

    return p.parse_received(msg.data(), static_cast<uint32_t>(msg.size())) == ParseResult::OK;
}

/// The peer's record, or null.
const PeerEntry* record(RTPSParticipant& p, const GuidPrefix& prefix) {
    return p.spdp_reader().find_peer(prefix);
}

bool is_active(RTPSParticipant& p, const GuidPrefix& prefix) {
    const PeerEntry* e = record(p, prefix);
    return e != nullptr && e->active();
}

bool is_parked(RTPSParticipant& p, const GuidPrefix& prefix) {
    const PeerEntry* e = record(p, prefix);
    return e != nullptr && e->parked;
}

/// Delivers a GAP from a peer's publications announcer declaring [start, base) irrelevant.
bool deliver_publications_gap(RTPSParticipant& p, const GuidPrefix& prefix, int64_t start,
                              int64_t base) {
    SequenceNumberSet irrelevant{};
    irrelevant.bitmapBase = SequenceNumber::from_int64(base);
    irrelevant.numBits = 0u;
    MessageBuilder builder{};
    if (!builder.begin(prefix)
        || !builder.append_gap(entity_id::SEDP_BUILTIN_PUBLICATIONS_DETECTOR,
                               entity_id::SEDP_BUILTIN_PUBLICATIONS_ANNOUNCER,
                               SequenceNumber::from_int64(start), irrelevant)) {
        builder.abort();
        return false;
    }

    BufRef                   msg = builder.finish();
    const std::vector<Octet> bytes(msg.data, msg.data + msg.size);
    msg.reset();
    return p.parse_received(bytes.data(), static_cast<uint32_t>(bytes.size())) == ParseResult::OK;
}

/// Orders GUIDs by their octets, prefix first, as the host graph cache keys them.
struct GuidOrder {
    bool operator()(const Guid& x, const Guid& y) const { return memcmp(&x, &y, sizeof(Guid)) < 0; }
};

/// Delivers, at `now_ms`, a HEARTBEAT from a peer's publications announcer holding [first, last].
bool deliver_publications_heartbeat(RTPSParticipant& p, const GuidPrefix& prefix, int64_t first,
                                    int64_t last, uint32_t now_ms) {
    MessageBuilder builder{};
    if (!builder.begin(prefix)
        || !builder.append_heartbeat(entity_id::SEDP_BUILTIN_PUBLICATIONS_DETECTOR,
                                     entity_id::SEDP_BUILTIN_PUBLICATIONS_ANNOUNCER,
                                     SequenceNumber::from_int64(first),
                                     SequenceNumber::from_int64(last), 1, true)) {
        builder.abort();
        return false;
    }

    BufRef                   msg = builder.finish();
    const std::vector<Octet> bytes(msg.data, msg.data + msg.size);
    msg.reset();
    g_participant_test_now_ms = now_ms;
    return p.parse_received(bytes.data(), static_cast<uint32_t>(bytes.size())) == ParseResult::OK;
}

/// A graph observer that lists endpoints as the host graph cache does and counts the notices.
struct GraphLog {
    std::map<Guid, bool, GuidOrder> listed{};  ///< Endpoint GUID to whether it is confirmed.
    int                             endpoint_gone = 0;
    int                             participant_gone = 0;
    int                             refreshing = 0;
    int                             refreshed = 0;
    GraphEndpointObserver           observer{};

    GraphLog() {
        observer.user = this;
        observer.on_endpoint = [](void* u, const Guid& g, const char*, const char*, bool) noexcept {
            static_cast<GraphLog*>(u)->listed[g] = true;
        };
        observer.on_endpoint_gone = [](void* u, const Guid& g) noexcept {
            auto* log = static_cast<GraphLog*>(u);
            log->listed.erase(g);
            ++log->endpoint_gone;
        };
        observer.on_participant_gone = [](void* u, const GuidPrefix& prefix) noexcept {
            auto* log = static_cast<GraphLog*>(u);
            log->forget(prefix, /*unconfirmed_only=*/false);
            ++log->participant_gone;
        };
        observer.on_participant_refreshing = [](void* u, const GuidPrefix& prefix) noexcept {
            auto* log = static_cast<GraphLog*>(u);
            for (auto& entry : log->listed) {
                if (entry.first.prefix == prefix) {
                    entry.second = false;
                }
            }

            ++log->refreshing;
        };
        observer.on_participant_refreshed = [](void* u, const GuidPrefix& prefix) noexcept {
            auto* log = static_cast<GraphLog*>(u);
            log->forget(prefix, /*unconfirmed_only=*/true);
            ++log->refreshed;
        };
    }

    void forget(const GuidPrefix& prefix, bool unconfirmed_only) {
        for (auto it = listed.begin(); it != listed.end();) {
            const bool drop = it->first.prefix == prefix && !(unconfirmed_only && it->second);
            it = drop ? listed.erase(it) : std::next(it);
        }
    }

    bool lists(const Guid& guid) const { return listed.count(guid) != 0u; }
};

/// The GUID deliver_announcement() gives a peer's writer announced at `sn`.
Guid announced_writer(const GuidPrefix& prefix, int64_t sn) {
    return Guid{prefix, make_entity(0x71u, 0x72u, static_cast<Octet>(sn), 0x03u)};
}

}  // namespace

// A peer that finds the active tier full waits until a settled peer can be
// parked for it; a parked peer's SPDP renews its lease; a parked peer whose
// SEDP announcer sends a newer change is reactivated, displacing a settled
// peer, and its announcement reaches the match table.
static bool tc_part_8_5_3_011(std::string& err) {
    CaptureTransport tx{};
    RTPSParticipant p(0u, 1u, tx);
    CHECK(p.add_ipv4_address(0xC0A8010Au), "local address setup should succeed");
    const GuidPrefix a = make_prefix(0xA1u, 0xA2u, 0xA3u);
    const GuidPrefix b = make_prefix(0xB1u, 0xB2u, 0xB3u);
    const GuidPrefix c = make_prefix(0xC1u, 0xC2u, 0xC3u);

    CHECK(announce(p, a, 0x11u, 1000u) && announce(p, b, 0x12u, 1010u),
          "the first two peers are recorded");
    CHECK(is_active(p, a) && is_active(p, b), "the first two peers take the two active slots");

    CHECK(announce(p, c, 0x13u, 1050u), "the third peer is recorded");
    CHECK(!is_active(p, c) && !is_parked(p, c),
          "with both active peers inside their settle time, the third waits");
    CHECK(p.spdp_reader().active_count() == 2u, "the active tier never exceeds its slots");

    CHECK(announce(p, c, 0x13u, 1200u), "the third peer announces again");
    CHECK(is_active(p, c), "its next announcement is admitted");
    CHECK(is_parked(p, a) && is_active(p, b),
          "the peer settled longest is parked for it, the other stays active");

    const uint32_t expiry = record(p, a)->leaseExpiryMs;
    CHECK(announce(p, a, 0x11u, 1300u), "the parked peer's announcement is accepted");
    CHECK(is_parked(p, a) && record(p, a)->leaseExpiryMs == expiry + 300u,
          "a parked peer's announcement renews its lease and nothing else");

    CHECK(deliver_announcement(p, a, true, "rt/new_topic", 1, 1400u),
          "the parked peer announces a new endpoint");
    CHECK(is_active(p, a), "an announcement above the parked number reactivates the peer");
    CHECK(is_parked(p, b) && is_active(p, c),
          "the peer settled longest is parked for it; the one inside its settle time stays");
    CHECK(p.match_table().count("rt/new_topic", true) == 1u,
          "the announcement that reactivated the peer is processed");
    return true;
}

// A peer is parked only once settled: not while its readers have
// unacknowledged SEDP, never while one of its endpoints is in a match with a
// local endpoint, but a match with the discovery writer alone does not keep
// it active; parking it unmatches that writer's reader proxy.
static bool tc_part_8_5_3_012(std::string& err) {
    CaptureTransport tx{};
    RTPSParticipant p(0u, 1u, tx);
    CHECK(p.add_ipv4_address(0xC0A8010Au), "local address setup should succeed");

    WriterConfig wcfg{};
    wcfg.topicName.assign("rt/discovery");
    wcfg.typeName.assign("pkg::msg::dds_::T_");
    wcfg.reliability = ReliabilityKind::RELIABLE;
    auto* discovery = p.create_writer<1>(wcfg);
    CHECK(discovery != nullptr, "the discovery writer is created");
    p.set_discovery_writer(discovery->guid());
    ReaderConfig rcfg{};
    rcfg.topicName.assign("rt/control");
    rcfg.typeName.assign("pkg::msg::dds_::T_");
    rcfg.reliability = ReliabilityKind::RELIABLE;
    CHECK(p.create_reader(rcfg, on_reader_change, nullptr) != nullptr, "the reader is created");

    const GuidPrefix a = make_prefix(0xA1u, 0xA2u, 0xA3u);
    const GuidPrefix b = make_prefix(0xB1u, 0xB2u, 0xB3u);
    const GuidPrefix c = make_prefix(0xC1u, 0xC2u, 0xC3u);
    CHECK(announce(p, a, 0x11u, 1000u) && announce(p, b, 0x12u, 1000u), "two peers are active");
    CHECK(deliver_announcement(p, a, false, "rt/discovery", 1, 1000u),
          "peer A reads the discovery topic");
    CHECK(deliver_announcement(p, b, true, "rt/control", 1, 1000u),
          "peer B writes the topic the local reader reads");
    const Guid a_reader{a, make_entity(0x61u, 0x62u, 1u, 0x04u)};
    CHECK(discovery->matched_reader_lookup(a_reader) != nullptr,
          "the discovery writer matches peer A's reader");

    CHECK(announce(p, c, 0x13u, 2000u), "a third peer is recorded");
    CHECK(!is_active(p, c) && is_active(p, a) && is_active(p, b),
          "no peer is parked while A has unacknowledged SEDP and B is in a match");

    CHECK(acknowledge_sedp(p, a, true, 1) && acknowledge_sedp(p, a, false, 1),
          "peer A acknowledges both announcements");
    CHECK(announce(p, c, 0x13u, 2100u), "the third peer announces again");
    CHECK(is_active(p, c) && is_parked(p, a) && is_active(p, b),
          "A, matched only with the discovery writer, is parked; B is not");
    CHECK(discovery->matched_reader_lookup(a_reader) == nullptr,
          "parking A unmatches its reader of the discovery writer");
    return true;
}

// A peer parked while it reads only the discovery writer receives no sample
// written after: discovery_info_changed() owes it the change, tick()
// reactivates it in place of a settled peer, and its replayed reader is
// matched to the discovery writer again, whose history then reaches it.
static bool tc_part_8_5_3_013(std::string& err) {
    CaptureTransport tx{};
    RTPSParticipant p(0u, 1u, tx);
    CHECK(p.add_ipv4_address(0xC0A8010Au), "local address setup should succeed");

    WriterConfig wcfg{};
    wcfg.topicName.assign("rt/discovery");
    wcfg.typeName.assign("pkg::msg::dds_::T_");
    wcfg.reliability = ReliabilityKind::RELIABLE;
    auto* discovery = p.create_writer<1>(wcfg);
    CHECK(discovery != nullptr, "the discovery writer is created");
    p.set_discovery_writer(discovery->guid());

    const GuidPrefix a = make_prefix(0xA1u, 0xA2u, 0xA3u);
    const GuidPrefix b = make_prefix(0xB1u, 0xB2u, 0xB3u);
    const GuidPrefix c = make_prefix(0xC1u, 0xC2u, 0xC3u);
    CHECK(announce(p, a, 0x11u, 1000u) && announce(p, b, 0x12u, 1000u), "two peers are active");
    CHECK(deliver_announcement(p, a, false, "rt/discovery", 1, 1000u),
          "peer A reads the discovery topic");
    CHECK(acknowledge_sedp(p, a, true, 1) && acknowledge_sedp(p, a, false, 0),
          "peer A acknowledges the announcement");

    CHECK(announce(p, c, 0x13u, 2000u), "a third peer is recorded");
    CHECK(is_parked(p, a) && is_active(p, c),
          "A, matched only with the discovery writer, is parked");
    CHECK(!record(p, a)->owed, "a peer is owed nothing when it is parked");

    p.discovery_info_changed();
    const PeerEntry* owed_peer = record(p, a);
    CHECK(owed_peer != nullptr, "peer A keeps its record");
    CHECK(owed_peer->owed, "a change of the discovery writer's contents is owed to A");

    CHECK(acknowledge_sedp(p, c, true, 1) && acknowledge_sedp(p, c, false, 0),
          "peer C acknowledges the announcement");
    g_participant_test_now_ms = 2200u;
    p.tick(2200u);
    CHECK(is_active(p, a) && !record(p, a)->owed,
          "the next tick reactivates the owed peer in place of a settled one");

    const Guid a_reader{a, make_entity(0x61u, 0x62u, 1u, 0x04u)};
    CHECK(deliver_announcement(p, a, false, "rt/discovery", 1, 2200u),
          "the reactivated peer replays its reader");
    CHECK(discovery->matched_reader_lookup(a_reader) != nullptr,
          "its reader of the discovery writer is matched again");
    return true;
}

// A parked peer's lease is renewed only by a well-formed announcement of its
// own GUID: an empty payload, an unknown representation, a parameter list
// cut short, or a payload naming another participant leaves it unchanged.
static bool tc_part_8_5_3_014(std::string& err) {
    CaptureTransport tx{};
    RTPSParticipant p(0u, 1u, tx);
    CHECK(p.add_ipv4_address(0xC0A8010Au), "local address setup should succeed");
    const GuidPrefix a = make_prefix(0xA1u, 0xA2u, 0xA3u);
    const GuidPrefix b = make_prefix(0xB1u, 0xB2u, 0xB3u);
    const GuidPrefix c = make_prefix(0xC1u, 0xC2u, 0xC3u);
    CHECK(announce(p, a, 0x11u, 1000u) && announce(p, b, 0x12u, 1010u)
              && announce(p, c, 0x13u, 1050u) && announce(p, c, 0x13u, 1200u),
          "three peers are recorded");
    CHECK(is_parked(p, a), "the peer settled longest is parked for the third");

    std::vector<Octet> valid{};
    CHECK(build_spdp_payload(a, 30u, make_udpv4_locator(0xC0A80111u, 7410u), valid),
          "SPDP payload build failed");
    std::vector<Octet> bad_representation = valid;
    bad_representation[1] = 0x7Fu;
    const std::vector<Octet> truncated(valid.begin(), valid.begin() + 8);
    std::vector<Octet> foreign{};
    CHECK(build_spdp_payload(make_prefix(0xD1u, 0xD2u, 0xD3u), 30u,
                             make_udpv4_locator(0xC0A80114u, 7410u), foreign),
          "SPDP payload build failed");

    const PeerEntry* expiring = record(p, a);
    CHECK(expiring != nullptr, "peer A keeps its record");
    const uint32_t expiry = expiring->leaseExpiryMs;
    SpdpReader& reader = p.spdp_reader();
    CHECK(!reader.on_data(a, nullptr, 0u, 1300u), "an empty payload is rejected");
    CHECK(!reader.on_data(a, bad_representation.data(),
                          static_cast<uint32_t>(bad_representation.size()), 1300u),
          "an unknown representation is rejected");
    CHECK(!reader.on_data(a, truncated.data(), static_cast<uint32_t>(truncated.size()), 1300u),
          "a parameter list cut short is rejected");
    reader.on_data(a, foreign.data(), static_cast<uint32_t>(foreign.size()), 1300u);
    CHECK(is_parked(p, a) && record(p, a)->leaseExpiryMs == expiry,
          "none of them renews the parked peer's lease");

    CHECK(reader.on_data(a, valid.data(), static_cast<uint32_t>(valid.size()), 1300u),
          "the peer's own announcement is accepted");
    CHECK(is_parked(p, a) && record(p, a)->leaseExpiryMs == expiry + 300u, "and renews its lease");
    return true;
}

// Parking releases a peer's endpoints from the match table but not from the
// graph observer, which lists the network's endpoints: it hears nothing. Only
// the departure of the peer, by lease expiry, is announced, and once.
static bool tc_part_8_5_3_015(std::string& err) {
    CaptureTransport tx{};
    RTPSParticipant  p(0u, 1u, tx);
    CHECK(p.add_ipv4_address(0xC0A8010Au), "local address setup should succeed");
    GraphLog log{};
    p.match_table().set_graph_endpoint_observer(&log.observer);
    const GuidPrefix a = make_prefix(0xA1u, 0xA2u, 0xA3u);
    const GuidPrefix b = make_prefix(0xB1u, 0xB2u, 0xB3u);
    const GuidPrefix c = make_prefix(0xC1u, 0xC2u, 0xC3u);

    CHECK(announce(p, a, 0x11u, 1000u) && announce(p, b, 0x12u, 1010u), "two peers are active");
    CHECK(deliver_announcement(p, a, true, "rt/kept", 1, 1000u), "peer A announces a writer");
    CHECK(log.lists(announced_writer(a, 1)), "the observer lists the writer");

    CHECK(announce(p, c, 0x13u, 1200u), "a third peer is recorded");
    CHECK(is_parked(p, a) && is_active(p, c), "A is parked for the third peer");
    CHECK(p.match_table().count("rt/kept", true) == 0u, "parking releases A's writer record");
    CHECK(log.lists(announced_writer(a, 1)) && log.endpoint_gone == 0 && log.participant_gone == 0
              && log.refreshing == 0,
          "the observer hears nothing of the parking and keeps listing A's writer");

    g_participant_test_now_ms = 40000u;
    p.tick(40000u);
    CHECK(record(p, a) == nullptr, "A's lease expires");
    CHECK(!log.lists(announced_writer(a, 1)), "the departed peer's writer is no longer listed");
    CHECK(log.endpoint_gone == 0, "a departure is not announced endpoint by endpoint");
    CHECK(log.participant_gone == 3, "each departed peer is announced once");
    return true;
}

// A parked peer that withdrew one endpoint and announced another is
// reactivated by the new announcement. The replay skips the withdrawn one
// with a GAP, so the refresh started at reactivation completes only once
// both announcers have caught up past the numbers kept at parking; it then
// forgets the withdrawn endpoint and keeps the new one.
static bool tc_part_8_5_3_016(std::string& err) {
    CaptureTransport tx{};
    RTPSParticipant  p(0u, 1u, tx);
    CHECK(p.add_ipv4_address(0xC0A8010Au), "local address setup should succeed");
    GraphLog log{};
    p.match_table().set_graph_endpoint_observer(&log.observer);
    const GuidPrefix a = make_prefix(0xA1u, 0xA2u, 0xA3u);
    const GuidPrefix b = make_prefix(0xB1u, 0xB2u, 0xB3u);
    const GuidPrefix c = make_prefix(0xC1u, 0xC2u, 0xC3u);

    CHECK(announce(p, a, 0x11u, 1000u) && announce(p, b, 0x12u, 1010u), "two peers are active");
    CHECK(deliver_announcement(p, a, true, "rt/withdrawn", 1, 1000u), "A announces a writer");
    CHECK(announce(p, c, 0x13u, 1200u) && is_parked(p, a), "A is parked for a third peer");
    CHECK(record(p, a)->parkedPublicationsSn == 1u, "the parked number is kept");

    // While parked, A withdrew writer 1 (SN 2) and announced writer 3 (SN 3).
    CHECK(deliver_announcement(p, a, true, "rt/new", 3, 1400u), "A announces a new writer");
    CHECK(is_active(p, a) && log.refreshing == 1,
          "the announcement reactivates A and refreshes it");
    CHECK(log.lists(announced_writer(a, 3)) && log.lists(announced_writer(a, 1)),
          "the new writer is listed; the withdrawn one is not yet known to be gone");

    g_participant_test_now_ms = 1450u;
    p.tick(1450u);
    CHECK(log.refreshed == 0, "the refresh waits while SNs 1 and 2 are missing");
    CHECK(log.lists(announced_writer(a, 1)), "so the unconfirmed writer stays listed");

    CHECK(deliver_publications_gap(p, a, 1, 3), "A's replay skips SNs 1 and 2");
    g_participant_test_now_ms = 1500u;
    p.tick(1500u);
    CHECK(log.refreshed == 1, "the refresh completes once the replay has caught up");
    CHECK(!log.lists(announced_writer(a, 1)), "the writer withdrawn while parked is forgotten");
    CHECK(log.lists(announced_writer(a, 3)), "the replayed writer stays listed");
    CHECK(record(p, a)->parkedPublicationsSn == 0u && record(p, a)->parkedSubscriptionsSn == 0u,
          "no refresh remains under way");

    g_participant_test_now_ms = 1600u;
    p.tick(1600u);
    CHECK(log.refreshed == 1, "a refresh completes once");
    return true;
}

// An endpoint re-announced while its peer was parked moves to a sequence
// number above the one kept at parking. A refresh started by a HEARTBEAT
// that advertises it does not complete while that announcement is missing,
// even though everything up to the kept number has arrived, so the endpoint
// never leaves the listing.
static bool tc_part_8_5_3_017(std::string& err) {
    CaptureTransport tx{};
    RTPSParticipant  p(0u, 1u, tx);
    CHECK(p.add_ipv4_address(0xC0A8010Au), "local address setup should succeed");
    GraphLog log{};
    p.match_table().set_graph_endpoint_observer(&log.observer);
    const GuidPrefix a = make_prefix(0xA1u, 0xA2u, 0xA3u);
    const GuidPrefix b = make_prefix(0xB1u, 0xB2u, 0xB3u);
    const GuidPrefix c = make_prefix(0xC1u, 0xC2u, 0xC3u);

    CHECK(announce(p, a, 0x11u, 1000u) && announce(p, b, 0x12u, 1010u), "two peers are active");
    CHECK(deliver_announcement(p, a, true, "rt/moved", 1, 1000u)
              && deliver_announcement(p, a, true, "rt/still", 2, 1000u),
          "A announces two writers");
    CHECK(announce(p, c, 0x13u, 1200u) && is_parked(p, a), "A is parked for a third peer");
    CHECK(record(p, a)->parkedPublicationsSn == 2u, "the parked number is kept");

    // While parked, A re-announced writer 1 at SN 3; SN 1 is now irrelevant.
    CHECK(deliver_publications_heartbeat(p, a, 1, 3, 1400u), "A's announcer advertises SN 3");
    CHECK(is_active(p, a) && log.refreshing == 1, "the heartbeat reactivates A and refreshes it");
    CHECK(deliver_publications_gap(p, a, 1, 2)
              && deliver_announcement(p, a, true, "rt/still", 2, 1410u),
          "the replay resolves SNs 1 and 2");
    g_participant_test_now_ms = 1450u;
    p.tick(1450u);
    CHECK(log.refreshed == 0, "the refresh waits for the advertised SN 3");
    CHECK(log.lists(announced_writer(a, 1)), "the re-announced writer stays listed meanwhile");

    CHECK(deliver_announcement(p, a, true, "rt/moved", 3, 1460u, 1u), "SN 3 re-announces writer 1");
    g_participant_test_now_ms = 1500u;
    p.tick(1500u);
    CHECK(log.refreshed == 1, "the refresh completes");
    CHECK(log.lists(announced_writer(a, 1)) && log.lists(announced_writer(a, 2)),
          "both live writers remain listed");
    return true;
}

void append_participant_tests_8_5_3_peer_tiers(std::vector<TestCase>& tests) {
    tests.push_back({"TC-PART-8.5.3-011",
                     "§8.5.3.2/§8.5.4.1: a peer finding the active tier full waits until a "
                     "settled peer is parked for it; a parked peer's SPDP renews its lease; a "
                     "newer SEDP announcement reactivates a parked peer and is processed.",
                     tc_part_8_5_3_011});
    tests.push_back({"TC-PART-8.5.3-012",
                     "§8.5.4.1: a peer is parked only when settled: no unacknowledged SEDP and "
                     "no match except with the discovery writer, whose reader proxy parking "
                     "removes.",
                     tc_part_8_5_3_012});
    tests.push_back({"TC-PART-8.5.3-013",
                     "§8.5.4.1/§8.7.2.2.1: a change of the discovery writer's contents is owed "
                     "to parked peers, which are reactivated and matched to it again.",
                     tc_part_8_5_3_013});
    tests.push_back({"TC-PART-8.5.3-014",
                     "§8.5.3.2/§9.6.2: a parked peer's lease is renewed only by a well-formed "
                     "SPDP payload carrying its own PID_PARTICIPANT_GUID.",
                     tc_part_8_5_3_014});
    tests.push_back({"TC-PART-8.5.3-015",
                     "§8.5.3.2: parking a peer leaves the graph observer's listing unchanged; "
                     "the peer's departure is announced to it once.",
                     tc_part_8_5_3_015});
    tests.push_back({"TC-PART-8.5.3-016",
                     "§8.5.4.1/§8.4.10.4: reactivating a parked peer refreshes its listing; once "
                     "its replay has caught up, an endpoint withdrawn while parked is forgotten.",
                     tc_part_8_5_3_016});
    tests.push_back({"TC-PART-8.5.3-017",
                     "§8.5.4.1/§8.4.10.4: a refresh does not complete while an announcement the "
                     "peer's HEARTBEAT advertises above the parked number is missing.",
                     tc_part_8_5_3_017});
}

}  // namespace spec_test
