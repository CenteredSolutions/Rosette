/**
 * @file test_participant_waiting_matches.cpp
 * @brief Waiting matches: a pair a proxy pool refuses binds later, oldest first.
 *
 * A user endpoint binds the proxy of each match from a participant-wide pool,
 * up to its cap (rtps/proxy_pool.hpp). A pair the pool or the cap refuses is
 * not dropped: the match table keeps it waiting with the ticket it drew when
 * reported, and after a proxy frees, each RTPSParticipant::tick() binds the
 * oldest waiting pair that can now bind, one per tick. The bound endpoint
 * starts the pair as a newly matched one (§8.4.1.4, §8.4.7.4), so a
 * TRANSIENT_LOCAL writer replays its history, a service reply included.
 *
 * The cases cap one endpoint at one or two matches, which exercises the same
 * admission path as an exhausted pool (TC-POOL-8.4.1.1-001 covers the pool's
 * own rule).
 */

#include "participant_test_support.hpp"

namespace spec_test {

#define CHECK(COND, MSG)  \
    do {                  \
        if (!(COND)) {    \
            err = (MSG);  \
            return false; \
        }                 \
    } while (0)

namespace {

using namespace participant_support;

constexpr Octet kPayload[4] = {0x00u, 0x01u, 0x00u, 0x00u};

/// Announces a peer with prefix `prefix` and default unicast port `port`.
bool add_peer(RTPSParticipant& p, const GuidPrefix& prefix, uint16_t port) {
    std::vector<Octet> spdp{};
    return build_spdp_payload(prefix, 30u, make_udpv4_locator(0xC0A80170u, 7410u), spdp,
                              make_udpv4_locator(0xC0A80170u, port))
           && p.spdp_reader().on_data(prefix, spdp.data(), static_cast<uint32_t>(spdp.size()),
                                      100u);
}

/// A remote reader of `pool_topic`, numbered `n` on `prefix`.
Guid remote_reader(const GuidPrefix& prefix, Octet n) {
    return Guid{prefix, make_entity(0x40u, 0x41u, n, 0x07u)};
}

/// Announces `reader` of `topic` through the participant's match table, as SEDP does.
void announce_reader(RTPSParticipant& p, const Guid& reader, const char* topic = "pool_topic") {
    DiscoveredReaderData rd{};
    rd.readerGuid = reader;
    rd.topicName.assign(topic);
    rd.typeName.assign("pkg::msg::PoolType");
    rd.reliabilityKind = ReliabilityKind::RELIABLE;
    rd.durabilityKind = DurabilityKind::TRANSIENT_LOCAL;
    p.match_table().add_reader(rd);
}

/// A reliable TRANSIENT_LOCAL writer of `topic` matching at most `cap` readers.
StatefulWriter* create_capped_writer(RTPSParticipant& p, uint8_t cap,
                                     const char* topic = "pool_topic") {
    WriterConfig wcfg{};
    wcfg.topicName.assign(topic);
    wcfg.typeName.assign("pkg::msg::PoolType");
    wcfg.reliability = ReliabilityKind::RELIABLE;
    wcfg.durability = DurabilityKind::TRANSIENT_LOCAL;
    wcfg.maxMatchedReaders = cap;
    return p.create_writer<4>(wcfg);
}

/// True when `tx` carries a DATA for `reader` with sequence number `sn`.
bool sent_data(const CaptureTransport& tx, const Guid& reader, int64_t sn) {
    struct Want {
        EntityId reader;
        SequenceNumber sn;
        bool seen;
    } want{reader.entityId, SequenceNumber::from_int64(sn), false};

    ParseOps ops{};
    ops.on_data = [](const ParseContext& ctx, const ParsedData& d) noexcept {
        auto* w = static_cast<Want*>(ctx.user_data);
        w->seen = w->seen || (d.readerId == w->reader && d.writerSN == w->sn);
    };
    for (const auto& packet : tx.packets) {
        (void)parse_message(packet.bytes.data(), static_cast<uint32_t>(packet.bytes.size()),
                            GuidPrefix{}, ops, &want);
    }

    return want.seen;
}

// TC-PART-8.4.1.4-001: a reader the writer's cap refuses waits; when a
// proxy frees, the next tick binds the oldest waiting reader, not the one in
// the lowest record, and the writer replays its history to it.
bool tc_part_8_4_1_4_001(std::string& err) {
    CaptureTransport tx{};
    RTPSParticipant p(0u, 1u, tx);
    CHECK(p.add_ipv4_address(0xC0A8010Au), "local address setup");
    const GuidPrefix peer = make_prefix(0xA1u, 0xA2u, 0xA3u);
    CHECK(add_peer(p, peer, 7411u), "the peer is accepted");

    StatefulWriter* w = create_capped_writer(p, 1u);
    CHECK(w != nullptr, "a writer capped at one reader is created");
    CHECK(w->write_async(ChangeKind::ALIVE, kPayload, 4u), "the writer holds one change");

    const Guid a = remote_reader(peer, 1u);
    const Guid b = remote_reader(peer, 2u);
    const Guid c = remote_reader(peer, 3u);
    announce_reader(p, a);
    announce_reader(p, b);
    announce_reader(p, c);
    CHECK(w->matched_reader_count() == 1u && w->matched_reader_lookup(a) != nullptr,
          "the first reader binds; the others wait at the cap");

    // d takes a's freed record, so record order alone would pick it first.
    p.match_table().remove_reader(a);
    const Guid d = remote_reader(peer, 4u);
    announce_reader(p, d);
    CHECK(w->matched_reader_count() == 0u, "a waiting reader binds only in tick()");
    CHECK(p.ms_until_next_timer(200u) == 0u, "a freed proxy makes tick() due at once");

    tx.packets.clear();
    p.tick(200u);
    CHECK(w->matched_reader_lookup(b) != nullptr && w->matched_reader_count() == 1u,
          "the oldest waiting reader binds");
    CHECK(sent_data(tx, b, 1), "the writer replays its history to the reader that bound");

    p.match_table().remove_reader(b);
    const Guid e = remote_reader(peer, 5u);
    announce_reader(p, e);
    p.tick(300u);
    CHECK(w->matched_reader_lookup(c) != nullptr && w->matched_reader_count() == 1u,
          "c, older than d and e, binds next");

    p.tick(400u);
    CHECK(w->matched_reader_lookup(d) == nullptr && w->matched_reader_lookup(e) == nullptr,
          "no reader binds above the cap");
    CHECK(p.ms_until_next_timer(400u) != 0u, "with nothing able to bind, tick() is not due");
    return true;
}

// TC-PART-8.4.1.4-002: when several proxies free at once, each tick binds one
// waiting match, oldest first; a deleted writer's waiting matches are gone.
bool tc_part_8_4_1_4_002(std::string& err) {
    CaptureTransport tx{};
    RTPSParticipant p(0u, 1u, tx);
    CHECK(p.add_ipv4_address(0xC0A8010Au), "local address setup");
    const GuidPrefix peer = make_prefix(0xB1u, 0xB2u, 0xB3u);
    CHECK(add_peer(p, peer, 7411u), "the peer is accepted");

    StatefulWriter* w = create_capped_writer(p, 2u);
    CHECK(w != nullptr, "a writer capped at two readers is created");
    for (Octet n = 1u; n <= 5u; ++n) {
        announce_reader(p, remote_reader(peer, n));
    }

    CHECK(w->matched_reader_count() == 2u, "two readers bind; three wait");
    p.match_table().remove_reader(remote_reader(peer, 1u));
    p.match_table().remove_reader(remote_reader(peer, 2u));

    p.tick(200u);
    CHECK(w->matched_reader_count() == 1u
              && w->matched_reader_lookup(remote_reader(peer, 3u)) != nullptr,
          "the first tick binds only the oldest waiting reader");
    CHECK(p.ms_until_next_timer(200u) == 0u, "a bind leaves tick() due for the next");

    p.tick(201u);
    CHECK(w->matched_reader_count() == 2u
              && w->matched_reader_lookup(remote_reader(peer, 4u)) != nullptr,
          "the second tick binds the next oldest");

    p.tick(202u);
    CHECK(w->matched_reader_lookup(remote_reader(peer, 5u)) == nullptr,
          "the cap holds the last one waiting");

    const ProxyPool& pool = p.reader_proxy_pool();
    CHECK(p.delete_writer(w), "the writer is deleted");
    CHECK(pool.free_count() == pool.capacity(), "its proxies return to the pool");
    CHECK(!p.match_table().has_match_with(peer, GUID_UNKNOWN),
          "its waiting match is dissolved with it");
    return true;
}

// TC-PART-8.4.1.4-003: a reply written while its client's reply reader waits
// stays in the reply writer's history and reaches that reader when it binds;
// a waiting match keeps its peer in a match.
bool tc_part_8_4_1_4_003(std::string& err) {
    CaptureTransport tx{};
    RTPSParticipant p(0u, 1u, tx);
    CHECK(p.add_ipv4_address(0xC0A8010Au), "local address setup");
    const GuidPrefix other = make_prefix(0xC1u, 0xC2u, 0xC3u);
    const GuidPrefix client = make_prefix(0xD1u, 0xD2u, 0xD3u);
    CHECK(add_peer(p, other, 7411u) && add_peer(p, client, 7412u), "both peers are accepted");

    StatefulWriter* replies = create_capped_writer(p, 1u);
    CHECK(replies != nullptr, "a reply writer capped at one reader is created");
    replies->release_replies_on_delivery(true);

    announce_reader(p, remote_reader(other, 1u));
    const Guid client_reader = remote_reader(client, 2u);
    announce_reader(p, client_reader);
    CHECK(replies->matched_reader_lookup(client_reader) == nullptr,
          "the client's reply reader waits at the cap");
    CHECK(p.match_table().has_match_with(client, GUID_UNKNOWN),
          "a waiting match keeps the client's peer in a match");

    InlineQos reply{};
    reply.hasRelatedSampleIdentity = true;
    reply.relatedSampleIdentity.writerGuid = Guid{client, make_entity(0x50u, 0x51u, 0x52u, 0x03u)};
    reply.relatedSampleIdentity.sequenceNumber = SequenceNumber::from_int64(1);
    CHECK(replies->write_async(ChangeKind::ALIVE, kPayload, 4u, reply), "the reply is written");
    CHECK(replies->cache().get_change(SequenceNumber::from_int64(1)) != nullptr,
          "a reply to an unbound client stays in the history");

    p.match_table().remove_reader(remote_reader(other, 1u));
    tx.packets.clear();
    p.tick(200u);
    CHECK(replies->matched_reader_lookup(client_reader) != nullptr,
          "the client's reply reader binds once a proxy frees");
    CHECK(sent_data(tx, client_reader, 1), "the waiting reply reaches the client");
    return true;
}

/// Fills `w`'s waiting matches: reader 1 of `peer` binds and readers
/// 2 .. RTPS_MAX_WAITING_MATCHES + 1 wait, so the next pair that would wait
/// is turned away.
void fill_waiting(RTPSParticipant& p, const GuidPrefix& peer) {
    for (Octet n = 1u; n <= RTPS_MAX_WAITING_MATCHES + 1u; ++n) {
        announce_reader(p, remote_reader(peer, n));
    }
}

// TC-PART-8.4.1.4-004: with RTPS_MAX_WAITING_MATCHES pairs waiting, a
// further pair that would wait is not recorded, while a pair that may bind
// still matches.
bool tc_part_8_4_1_4_004(std::string& err) {
    CaptureTransport tx{};
    RTPSParticipant p(0u, 1u, tx);
    CHECK(p.add_ipv4_address(0xC0A8010Au), "local address setup");
    const GuidPrefix peer = make_prefix(0xE1u, 0xE2u, 0xE3u);
    const GuidPrefix late = make_prefix(0xE4u, 0xE5u, 0xE6u);
    CHECK(add_peer(p, peer, 7411u) && add_peer(p, late, 7412u), "both peers are accepted");

    StatefulWriter* w = create_capped_writer(p, 1u);
    StatefulWriter* spare = create_capped_writer(p, 1u, "spare_topic");
    CHECK(w != nullptr && spare != nullptr, "two writers capped at one reader are created");

    fill_waiting(p, peer);
    announce_reader(p, remote_reader(late, 1u));
    CHECK(!p.match_table().has_match_with(late, GUID_UNKNOWN),
          "the reader beyond the waiting limit has no record");

    const Guid other = remote_reader(late, 2u);
    announce_reader(p, other, "spare_topic");
    CHECK(spare->matched_reader_lookup(other) != nullptr,
          "a pair that may bind matches with the waiting matches full");
    return true;
}

// TC-PART-8.4.1.4-005: a pair turned away at the waiting limit is found
// again once room frees, on a tick that binds no waiting pair, and binds
// after the pairs that waited before it.
bool tc_part_8_4_1_4_005(std::string& err) {
    CaptureTransport tx{};
    RTPSParticipant p(0u, 1u, tx);
    CHECK(p.add_ipv4_address(0xC0A8010Au), "local address setup");
    const GuidPrefix peer = make_prefix(0xF1u, 0xF2u, 0xF3u);
    const GuidPrefix late = make_prefix(0xF4u, 0xF5u, 0xF6u);
    CHECK(add_peer(p, peer, 7411u) && add_peer(p, late, 7412u), "both peers are accepted");

    StatefulWriter* w = create_capped_writer(p, 1u);
    CHECK(w != nullptr, "a writer capped at one reader is created");
    fill_waiting(p, peer);
    const Guid turned_away = remote_reader(late, 1u);
    announce_reader(p, turned_away);
    CHECK(!p.match_table().has_match_with(late, GUID_UNKNOWN), "the late reader is turned away");
    p.tick(100u);
    CHECK(p.ms_until_next_timer(100u) != 0u, "with no room freed, no rescan is due");

    p.match_table().remove_reader(remote_reader(peer, 1u));
    CHECK(p.ms_until_next_timer(200u) == 0u, "a freed record makes tick() due");
    p.tick(200u);
    CHECK(w->matched_reader_lookup(remote_reader(peer, 2u)) != nullptr,
          "the oldest waiting reader binds first");
    CHECK(!p.match_table().has_match_with(late, GUID_UNKNOWN),
          "a tick that binds a waiting pair does not rescan");

    p.tick(201u);
    CHECK(p.match_table().has_match_with(late, GUID_UNKNOWN),
          "the next tick rescans and records the late reader waiting");
    CHECK(w->matched_reader_lookup(turned_away) == nullptr, "it waits behind the older readers");

    // Free the bound proxy once per reader ahead of it; it binds last.
    for (Octet n = 2u; n <= RTPS_MAX_WAITING_MATCHES + 1u; ++n) {
        CHECK(w->matched_reader_lookup(turned_away) == nullptr, "older readers bind first");
        p.match_table().remove_reader(remote_reader(peer, n));
        p.tick(300u + n);
    }

    CHECK(w->matched_reader_lookup(turned_away) != nullptr, "the late reader binds");
    p.tick(400u);
    CHECK(p.ms_until_next_timer(400u) != 0u, "with every pair recorded, no rescan is due");
    return true;
}

// TC-PART-8.4.1.4-006: when room frees for several turned-away pairs at
// once, each rescan binds one of them, so a tick replays at most one history.
bool tc_part_8_4_1_4_006(std::string& err) {
    CaptureTransport tx{};
    RTPSParticipant p(0u, 1u, tx);
    CHECK(p.add_ipv4_address(0xC0A8010Au), "local address setup");
    const GuidPrefix peer = make_prefix(0x91u, 0x92u, 0x93u);
    const GuidPrefix first = make_prefix(0x94u, 0x95u, 0x96u);
    const GuidPrefix late = make_prefix(0x97u, 0x98u, 0x99u);
    CHECK(add_peer(p, peer, 7411u) && add_peer(p, first, 7412u) && add_peer(p, late, 7413u),
          "the peers are accepted");

    // `full` holds the waiting matches with readers that cannot bind.
    StatefulWriter* full = create_capped_writer(p, 1u);
    StatefulWriter* w = create_capped_writer(p, 2u, "pair_topic");
    CHECK(full != nullptr && w != nullptr, "the writers are created");
    fill_waiting(p, peer);

    announce_reader(p, remote_reader(first, 1u), "pair_topic");
    announce_reader(p, remote_reader(first, 2u), "pair_topic");
    announce_reader(p, remote_reader(late, 1u), "pair_topic");
    announce_reader(p, remote_reader(late, 2u), "pair_topic");
    CHECK(w->matched_reader_count() == 2u && !p.match_table().has_match_with(late, GUID_UNKNOWN),
          "two readers bind at the cap and the late two are turned away");

    p.match_table().remove_reader(remote_reader(first, 1u));
    p.match_table().remove_reader(remote_reader(first, 2u));
    p.tick(200u);
    CHECK(w->matched_reader_count() == 1u, "the first rescan binds one late reader");
    CHECK(p.ms_until_next_timer(200u) == 0u, "the rescan stays due for the other");

    p.tick(201u);
    CHECK(w->matched_reader_count() == 2u, "the next rescan binds the other");
    return true;
}

// TC-PART-8.4.1.4-007: a pair that waits while far more than half the ticket
// range of later pairs are reported and dissolved still binds before a pair
// reported after them.
bool tc_part_8_4_1_4_007(std::string& err) {
    CaptureTransport tx{};
    RTPSParticipant p(0u, 1u, tx);
    CHECK(p.add_ipv4_address(0xC0A8010Au), "local address setup");
    const GuidPrefix peer = make_prefix(0xB1u, 0xB2u, 0xB3u);
    CHECK(add_peer(p, peer, 7411u), "the peer is accepted");

    StatefulWriter* w = create_capped_writer(p, 1u);
    CHECK(w != nullptr, "a writer capped at one reader is created");

    const Guid a = remote_reader(peer, 1u);
    const Guid b = remote_reader(peer, 2u);
    const Guid churn = remote_reader(peer, 9u);
    announce_reader(p, a);
    announce_reader(p, b);
    CHECK(w->matched_reader_lookup(a) != nullptr, "the first reader binds; b waits");

    // Each round records one waiting pair and dissolves it, drawing a ticket.
    constexpr uint32_t kRounds = 40000u;  // past half of, and across, the 16-bit range
    for (uint32_t i = 0; i < kRounds; ++i) {
        announce_reader(p, churn);
        p.match_table().remove_reader(churn);
    }

    const Guid late = remote_reader(peer, 3u);
    announce_reader(p, late);
    p.match_table().remove_reader(a);
    p.tick(200u);
    CHECK(w->matched_reader_lookup(b) != nullptr && w->matched_reader_lookup(late) == nullptr,
          "b, which waited through the churn, binds before the pair reported after it");
    return true;
}

}  // namespace

void append_participant_waiting_match_tests(std::vector<TestCase>& tests) {
    tests.push_back({"TC-PART-8.4.1.4-001",
                     "§8.4.1.4: a match refused by the writer's cap waits; after a proxy "
                     "frees, the next tick binds the oldest waiting match and replays the "
                     "TRANSIENT_LOCAL history to it; nothing binds above the cap.",
                     tc_part_8_4_1_4_001});
    tests.push_back({"TC-PART-8.4.1.4-002",
                     "§8.4.1.4: with several proxies free, each tick binds one waiting match, "
                     "oldest first; deleting the writer dissolves its waiting matches.",
                     tc_part_8_4_1_4_002});
    tests.push_back({"TC-PART-8.4.1.4-003",
                     "§8.4.1.4/DDS-RPC §7.8.2: a reply written while its client's reply reader "
                     "waits is kept and delivered when the reader binds; a waiting match keeps "
                     "its peer in a match.",
                     tc_part_8_4_1_4_003});
    tests.push_back({"TC-PART-8.4.1.4-004",
                     "§8.4.1.4: with RTPS_MAX_WAITING_MATCHES pairs waiting, a further pair "
                     "that would wait is not recorded, while a pair that may bind still "
                     "matches.",
                     tc_part_8_4_1_4_004});
    tests.push_back({"TC-PART-8.4.1.4-005",
                     "§8.4.1.4/§8.5.5.1: a pair turned away at the waiting limit is found again "
                     "once room frees, on a tick that binds no waiting pair, and binds after the "
                     "pairs that waited before it.",
                     tc_part_8_4_1_4_005});
    tests.push_back({"TC-PART-8.4.1.4-006",
                     "§8.4.1.4: when room frees for several turned-away pairs at once, each "
                     "rescan binds one of them.",
                     tc_part_8_4_1_4_006});
    tests.push_back({"TC-PART-8.4.1.4-007",
                     "§8.4.1.4/§8.5.5.1: a waiting pair stays older than every pair reported "
                     "after it, however many pairs are reported and dissolved while it waits.",
                     tc_part_8_4_1_4_007});
}

}  // namespace spec_test
