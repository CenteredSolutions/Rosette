/**
 * @file test_stateful_reader_8_4_10_4_locator_reference.cpp
 * @brief WriterProxy addresses its writer through a reference to the matched locator list.
 *
 * A WriterProxy keeps a pointer to one entry of the list it was matched with
 * rather than a copy (see "Locator reference" in rtps/writer_proxy.hpp), so
 * the reader's ACKNACKs follow the owning record when it takes a new address.
 */

#include "spec_test_common.hpp"

#include "rosette/rtps/stateful_reader.hpp"
#include "rosette/rtps/transport.hpp"

namespace spec_test {

extern uint32_t g_stateful_reader_test_now_ms;

#define CHECK(COND, MSG)  \
    do {                  \
        if (!(COND)) {    \
            err = (MSG);  \
            return false; \
        }                 \
    } while (0)

namespace {

struct DestTransport : ITransport {
    std::vector<Locator> dests{};

    bool send_unicast(const BufRef&, const Locator& dest) noexcept override {
        dests.push_back(dest);
        return true;
    }

    bool send_multicast(const BufRef&, const Locator&) noexcept override { return true; }
};

Guid make_guid(Octet p0, const EntityId& eid) {
    Guid g{};
    g.prefix.value[0] = p0;
    g.prefix.value[1] = 0x22u;
    g.entityId = eid;
    return g;
}

void heartbeat(StatefulReader& r, const Guid& reader, const Guid& writer, int32_t count) {
    ParsedHeartbeat hb{};
    hb.readerId = reader.entityId;
    hb.writerId = writer.entityId;
    hb.firstSN = SequenceNumber::from_int64(1);
    hb.lastSN = SequenceNumber::from_int64(1);
    hb.count = count;
    ParseContext ctx{};
    ctx.senderGuidPrefix = writer.prefix;
    g_stateful_reader_test_now_ms = 100u * static_cast<uint32_t>(count);
    r.on_heartbeat(ctx, hb);
}

}  // namespace

// §8.4.10.4: ACKNACKs go to the writer's unicastLocatorList; the proxy reads
// the referenced entry at each send, so they follow it when it changes.
static bool tc_srr_8_4_10_004(std::string& err) {
    DestTransport tx{};
    const Guid reader = make_guid(0x31u, make_entity(1u, 2u, 3u, 0x04u));
    const Guid writer = make_guid(0x42u, make_entity(4u, 5u, 6u, 0x03u));
    const Locator first = make_udpv4_locator(0xC0A80160u, 7411u);
    const Locator moved = make_udpv4_locator(0xC0A80161u, 7411u);
    Locator record[1] = {first};  // Stands in for the owning record.

    DefaultStatefulReader r(reader, ReliabilityKind::RELIABLE, tx);
    CHECK(r.matched_writer_add(writer, record, 1u), "match");
    CHECK(tx.dests.size() == 1u && tx.dests[0] == first,
          "the preemptive ACKNACK goes to the writer's locator");

    record[0] = moved;
    tx.dests.clear();
    heartbeat(r, reader, writer, 1);
    CHECK(tx.dests.size() == 1u && tx.dests[0] == moved,
          "after the record moves, the ACKNACK goes to the new address");
    return true;
}

void append_stateful_reader_tests_8_4_10_4_locator_reference(std::vector<TestCase>& tests) {
    tests.push_back({"TC-SRR-8.4.10-004",
                     "§8.4.10.4: a WriterProxy refers to the writer's locator in the owning "
                     "record; ACKNACKs follow that entry when it changes in place.",
                     tc_srr_8_4_10_004});
}

}  // namespace spec_test
