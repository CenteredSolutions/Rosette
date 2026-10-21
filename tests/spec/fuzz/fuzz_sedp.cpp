/**
 * @file fuzz_sedp.cpp
 * @brief Fuzz harness for SEDP endpoint-data decoding and withdrawal.
 *
 * SedpReader receives announcements through its builtin StatefulReaders. The
 * harness plays one remote participant's SEDP announcers against a SedpReader
 * (SedpPeer, as the SEDP spec tests do) and interprets the input as a stream
 * of operations: a 12-octet sender prefix, then (op, length, bytes) records.
 * Ops deliver the bytes as a publication or a subscription announcement,
 * announce a heartbeat, or drop and rediscover the peer. Correct means no
 * input makes the reader read out of bounds, overflow a table, or leave the
 * match table inconsistent.
 */

#include "sedp_peer_driver.hpp"

#include <stddef.h>
#include <stdint.h>

#include <vector>

using namespace rtps;

namespace {

struct NullTransport : ITransport {
    bool send_unicast(const BufRef&, const Locator&) noexcept override { return true; }

    bool send_multicast(const BufRef&, const Locator&) noexcept override { return false; }
};

bool on_match(const MatchedEndpoint&, const MatchedEndpoint&, const char*, void*) noexcept {
    return true;
}

EntityId make_entity(Octet k0, Octet k1, Octet k2, Octet kind) {
    EntityId e{};
    e.entityKey[0] = k0;
    e.entityKey[1] = k1;
    e.entityKey[2] = k2;
    e.entityKind = kind;
    return e;
}

Guid make_guid(Octet p0, Octet p1, Octet p2, const EntityId& eid) {
    Guid g{};
    g.prefix.value[0] = p0;
    g.prefix.value[1] = p1;
    g.prefix.value[2] = p2;
    g.entityId = eid;
    return g;
}

/// Local endpoints on the topic the seed corpus announces, so decoded
/// announcements reach the matching code and not only the decoder.
void seed_local_endpoints(EndpointMatchTable& table) {
    table.set_local_prefix(make_guid(0x11u, 0x22u, 0x33u, entity_id::PARTICIPANT).prefix);

    DiscoveredWriterData w{};
    w.writerGuid = make_guid(0x11u, 0x22u, 0x33u, make_entity(1u, 1u, 1u, 0xC2u));
    w.topicName.assign("scan");
    w.typeName.assign("scan");
    w.reliabilityKind = ReliabilityKind::RELIABLE;
    w.durabilityKind = DurabilityKind::VOLATILE;
    table.add_writer(w);

    DiscoveredReaderData r{};
    r.readerGuid = make_guid(0x11u, 0x22u, 0x33u, make_entity(2u, 2u, 2u, 0xC7u));
    r.topicName.assign("scan");
    r.typeName.assign("scan");
    r.reliabilityKind = ReliabilityKind::RELIABLE;
    r.durabilityKind = DurabilityKind::VOLATILE;
    table.add_reader(r);
}

}  // namespace

extern "C" int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size) {
    constexpr size_t kPrefixOctets = 12u;
    if (data == nullptr || size < kPrefixOctets + 2u) {
        return 0;
    }

    NullTransport      transport;
    EndpointMatchTable table(on_match, nullptr, nullptr);
    seed_local_endpoints(table);
    SedpReader reader(GuidPrefix{}, transport, table);

    spec_test::SedpPeer peer;
    for (size_t i = 0; i < kPrefixOctets; ++i) {
        peer.prefix.value[i] = static_cast<Octet>(data[i]);
    }

    peer.meta = make_udpv4_locator(0xC0A80101u, 7410u);
    peer.discover(reader);

    int64_t sn = 1;
    size_t  off = kPrefixOctets;
    while (off + 2u <= size) {
        const uint8_t op = data[off++] & 0x3u;
        size_t        len = data[off++];
        if (len > size - off) {
            len = size - off;
        }

        const std::vector<Octet> payload(data + off, data + off + len);
        off += len;

        switch (op) {
            case 0:
                peer.announce(reader, entity_id::SEDP_BUILTIN_PUBLICATIONS_ANNOUNCER, sn++,
                              payload);
                break;

            case 1:
                peer.announce(reader, entity_id::SEDP_BUILTIN_SUBSCRIPTIONS_ANNOUNCER, sn++,
                              payload);
                break;

            case 2:
                peer.heartbeat(reader, entity_id::SEDP_BUILTIN_PUBLICATIONS_ANNOUNCER, 1, sn,
                               static_cast<int32_t>(sn));
                break;

            default:
                reader.on_peer_removed(peer.prefix);
                peer.discover(reader);
                break;
        }
    }

    return 0;
}
