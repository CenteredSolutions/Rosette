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

// TC-PART-8.4.1.4-004: with RTPS_MAX_WAITING_MATCHES pairs waiting, a
// further pair that would wait is not recorded, so it never binds, while a
// pair that may bind still matches.
bool tc_part_8_4_1_4_004(std::string& err) {
    CaptureTransport tx{};
    RTPSParticipant p(0u, 1u, tx);
    CHECK(p.add_ipv4_address(0xC0A8010Au), "local address setup");
    const GuidPrefix peer = make_prefix(0xE1u, 0xE2u, 0xE3u);
    CHECK(add_peer(p, peer, 7411u), "the peer is accepted");

    StatefulWriter* w = create_capped_writer(p, 1u);
    StatefulWriter* spare = create_capped_writer(p, 1u, "spare_topic");
    CHECK(w != nullptr && spare != nullptr, "two writers capped at one reader are created");

    // Reader 1 binds; readers 2 .. kWaiting + 1 wait; the last one is turned away.
    constexpr Octet kWaiting = RTPS_MAX_WAITING_MATCHES;
    const Guid refused = remote_reader(peer, kWaiting + 2u);
    for (Octet n = 1u; n <= kWaiting + 1u; ++n) {
        announce_reader(p, remote_reader(peer, n));
    }

    announce_reader(p, refused);
    const Guid other = remote_reader(peer, kWaiting + 3u);
    announce_reader(p, other, "spare_topic");
    CHECK(spare->matched_reader_lookup(other) != nullptr,
          "a pair that may bind matches with the waiting matches full");

    // Free the bound proxy once per waiting reader; each tick binds the next.
    for (Octet n = 1u; n <= kWaiting + 1u; ++n) {
        p.match_table().remove_reader(remote_reader(peer, n));
        p.tick(200u + n);
    }

    CHECK(w->matched_reader_count() == 0u && w->matched_reader_lookup(refused) == nullptr,
          "the reader turned away at the waiting limit was never recorded, so never binds");
    CHECK(p.ms_until_next_timer(400u) != 0u, "nothing is left to bind");
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
                     "that would wait is not recorded and never binds, while a pair that may "
                     "bind still matches.",
                     tc_part_8_4_1_4_004});
}

}  // namespace spec_test
