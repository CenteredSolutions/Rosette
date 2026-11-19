// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file fuzz_stateful_endpoints.cpp
 * @brief Fuzz harness for the StatefulReader and StatefulWriter state machines.
 *
 * A reliable reader and a reliable writer, each matched to a remote counterpart, receive
 * datagrams through parse_message() and the routing the participant applies: reader
 * submessages reach the reader when readerId names it or is ENTITYID_UNKNOWN, and ACKNACK and
 * NACK_FRAG reach the writer when writerId names it. Only states the participant can reach
 * are explored. The transport carries at most kDatagramOctets, so the writer fragments any
 * sample larger than that and answers NACK_FRAG from fragments.
 *
 * The input is a stream of records:
 *
 * | op & 3 | n                      | effect                                                   |
 * |--------|------------------------|----------------------------------------------------------|
 * | 0      | body octets that follow| deliver the body behind an RTPS header                   |
 * | 1      | size / 8               | write a sample of n * 8 octets (at most one pool block)  |
 * | 2      | milliseconds           | advance the clock and tick both endpoints                |
 * | 3      | ignored                | unmatch and rematch the remote endpoints                 |
 *
 * With op bit 2 set, a delivery's first 12 body octets are the datagram's sender prefix in
 * place of the matched remote participant's, and the rest are its submessages.
 *
 * Correct means no input makes either endpoint read or write out of bounds or overflow a
 * sequence-number window, a proxy pool, or the fragment assembler.
 */

#include <stddef.h>
#include <stdint.h>

#include <vector>

#include "rosette/rtps/message_parser.hpp"
#include "rosette/rtps/stateful_reader.hpp"
#include "rosette/rtps/stateful_writer.hpp"
#include "rosette/rtps/transport.hpp"

using namespace rosette::rtps;

// The settable clock from src/test_stateful_writer_clock_stub.cpp; a run depends on its input.
namespace spec_test {
extern uint32_t g_stateful_writer_test_now_ms;
}

namespace {

constexpr size_t kHeaderOctets = 20u;
constexpr size_t kPrefixOctets = 12u;
constexpr size_t kMaxWriteOctets = 255u * 8u;

// Smaller than a pool block, so a sample the pool can hold may still need DATA_FRAG.
constexpr uint32_t kDatagramOctets = 512u;

struct NullTransport : ITransport {
    bool send_unicast(const BufRef&, const Locator&) noexcept override { return true; }

    bool send_multicast(const BufRef&, const Locator&) noexcept override { return true; }

    uint32_t max_datagram_size() const noexcept override { return kDatagramOctets; }
};

Guid make_guid(Octet prefix, Octet k0, Octet kind) {
    Guid g{};
    for (int i = 0; i < 12; ++i) {
        g.prefix.value[i] = prefix;
    }

    g.entityId.entityKey[0] = k0;
    g.entityId.entityKey[1] = k0;
    g.entityId.entityKey[2] = k0;
    g.entityId.entityKind = kind;
    return g;
}

/// The local endpoints, the remote participant they are matched to, and the dispatch targets.
struct Rig {
    NullTransport tx{};
    Locator       remote_loc = make_udpv4_locator(0xC0A80102u, 7411u);

    Guid local_reader = make_guid(0x61u, 0x07u, 0x04u);
    Guid local_writer = make_guid(0x61u, 0x01u, 0x03u);
    Guid remote_reader = make_guid(0x71u, 0x02u, 0x04u);
    Guid remote_writer = make_guid(0x71u, 0x08u, 0x03u);

    DefaultStatefulReader         reader;
    StatefulWriterStorage<4u, 2u> writer;

    Rig()
        : reader(local_reader, ReliabilityKind::RELIABLE, tx, nullptr, nullptr, 5u),
          writer(local_writer, ReliabilityKind::RELIABLE, DurabilityKind::VOLATILE, tx, 5u) {
        match();
    }

    void match() {
        reader.matched_writer_add(remote_writer, &remote_loc, 1u);
        writer.matched_reader_add(remote_reader, &remote_loc, 1u, false);
    }

    void unmatch() {
        reader.matched_writer_remove(remote_writer);
        writer.matched_reader_remove(remote_reader);
    }
};

Rig& rig_of(const ParseContext& ctx) {
    return *static_cast<Rig*>(ctx.user_data);
}

/// The reader when readerId names it or is ENTITYID_UNKNOWN, as the participant routes it.
StatefulReader* addressed_reader(const ParseContext& c, const EntityId& reader_id) {
    Rig& rig = rig_of(c);
    if (reader_id == rig.local_reader.entityId || reader_id == entity_id::UNKNOWN) {
        return &rig.reader;
    }

    return nullptr;
}

/// The writer when writerId names it, as the participant routes ACKNACK and NACK_FRAG.
StatefulWriter* addressed_writer(const ParseContext& c, const EntityId& writer_id) {
    Rig& rig = rig_of(c);
    return writer_id == rig.local_writer.entityId ? &rig.writer : nullptr;
}

void on_data(const ParseContext& c, const ParsedData& m) noexcept {
    if (StatefulReader* r = addressed_reader(c, m.readerId)) {
        r->on_data(c, m);
    }
}

void on_data_frag(const ParseContext& c, const ParsedDataFrag& m) noexcept {
    if (StatefulReader* r = addressed_reader(c, m.readerId)) {
        r->on_data_frag(c, m);
    }
}

void on_heartbeat(const ParseContext& c, const ParsedHeartbeat& m) noexcept {
    if (StatefulReader* r = addressed_reader(c, m.readerId)) {
        r->on_heartbeat(c, m);
    }
}

void on_heartbeat_frag(const ParseContext& c, const ParsedHeartbeatFrag& m) noexcept {
    if (StatefulReader* r = addressed_reader(c, m.readerId)) {
        r->on_heartbeat_frag(c, m);
    }
}

void on_gap(const ParseContext& c, const ParsedGap& m) noexcept {
    if (StatefulReader* r = addressed_reader(c, m.readerId)) {
        r->on_gap(c, m);
    }
}

void on_acknack(const ParseContext& c, const ParsedAckNack& m) noexcept {
    if (StatefulWriter* w = addressed_writer(c, m.writerId)) {
        w->on_acknack(c, m);
    }
}

void on_nack_frag(const ParseContext& c, const ParsedNackFrag& m) noexcept {
    if (StatefulWriter* w = addressed_writer(c, m.writerId)) {
        w->on_nack_frag(c, m);
    }
}

/// RTPS header (§9.4.2) from the matched remote participant, or with `keep_sender` from the
/// prefix in the first 12 body octets, followed by the rest of `body` as submessages.
void deliver(Rig& rig, const uint8_t* body, size_t length, bool keep_sender) {
    const Octet* sender = rig.remote_writer.prefix.value;
    if (keep_sender && length >= kPrefixOctets) {
        sender = body;
        body += kPrefixOctets;
        length -= kPrefixOctets;
    }

    std::vector<Octet> datagram(kHeaderOctets + length, 0u);
    datagram[0] = 'R';
    datagram[1] = 'T';
    datagram[2] = 'P';
    datagram[3] = 'S';
    datagram[4] = 2u;
    datagram[5] = 5u;
    datagram[6] = 0x01u;
    datagram[7] = 0x0Fu;
    for (size_t i = 0; i < kPrefixOctets; ++i) {
        datagram[8u + i] = sender[i];
    }

    for (size_t i = 0; i < length; ++i) {
        datagram[kHeaderOctets + i] = body[i];
    }

    ParseOps ops{};
    ops.on_data = on_data;
    ops.on_data_frag = on_data_frag;
    ops.on_heartbeat = on_heartbeat;
    ops.on_heartbeat_frag = on_heartbeat_frag;
    ops.on_gap = on_gap;
    ops.on_acknack = on_acknack;
    ops.on_nack_frag = on_nack_frag;
    parse_message(datagram.data(), static_cast<uint32_t>(datagram.size()), rig.local_reader.prefix,
                  ops, &rig);
}

void advance(Rig& rig, uint32_t ms) {
    spec_test::g_stateful_writer_test_now_ms += ms;
    rig.reader.tick(spec_test::g_stateful_writer_test_now_ms);
    rig.writer.tick(spec_test::g_stateful_writer_test_now_ms);
    rig.reader.ms_until_next_timer(spec_test::g_stateful_writer_test_now_ms);
    rig.writer.ms_until_next_timer(spec_test::g_stateful_writer_test_now_ms);
}

}  // namespace

extern "C" int LLVMFuzzerTestOneInput(const uint8_t* data, size_t size) {
    if (data == nullptr) {
        return 0;
    }

    spec_test::g_stateful_writer_test_now_ms = 1000u;
    Rig                rig;
    std::vector<Octet> sample(kMaxWriteOctets, 0xA5u);

    size_t off = 0;
    while (off + 2u <= size) {
        const uint8_t op = data[off++];
        const size_t  n = data[off++];

        switch (op & 0x3u) {
            case 0: {
                const size_t length = n > size - off ? size - off : n;
                deliver(rig, data + off, length, (op & 0x4u) != 0u);
                off += length;
                break;
            }

            case 1:
                rig.writer.write_async(ChangeKind::ALIVE, sample.data(),
                                       static_cast<uint32_t>(n * 8u));
                break;

            case 2:
                advance(rig, static_cast<uint32_t>(n));
                break;

            default:
                rig.unmatch();
                rig.match();
                break;
        }
    }

    return 0;
}
