// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file test_stateful_writer_8_4_1_1_locator_reference.cpp
 * @brief ReaderProxy addresses its reader through a reference to the matched locator list.
 *
 * A ReaderProxy keeps a pointer to one entry of the list it was matched with,
 * owned by the SPDP peer record or an EndpointMatchTable origin, rather than
 * a copy (see "Locator reference" in rtps/reader_proxy.hpp). The writer must
 * therefore send to that entry's current value: a reader whose record takes a
 * new address is reached there without a rematch, and an emptied entry is
 * never sent to.
 */

#include "spec_test_common.hpp"

#include "rosette/rtps/stateful_writer.hpp"

namespace spec_test {

extern uint32_t g_stateful_writer_test_now_ms;

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
    g.prefix.value[1] = 0x11u;
    g.entityId = eid;
    return g;
}

}  // namespace

// §8.4.1.1: the writer addresses the reader at its unicastLocatorList. The
// proxy refers to the list's first UDP entry and reads it at each send.
static bool tc_swr_8_4_1_1_001(std::string& err) {
    DestTransport tx{};
    const Guid writer = make_guid(0xA1u, make_entity(1u, 2u, 3u, 0xC2u));
    const Guid reader = make_guid(0xB1u, make_entity(4u, 5u, 6u, 0xC7u));
    const Locator first = make_udpv4_locator(0xC0A80150u, 7411u);
    const Locator moved = make_udpv4_locator(0xC0A80151u, 7411u);

    // Stands in for the owning record: a shared-memory locator ahead of UDP,
    // as Fast DDS announces.
    Locator record[2] = {};
    record[0].kind = 16;
    record[1] = first;

    StatefulWriterStorage<2, 1> w(writer, ReliabilityKind::RELIABLE, DurabilityKind::VOLATILE, tx,
                                  50u, 10u);
    CHECK(w.matched_reader_add(reader, record, 2u, false), "match");
    const ReaderProxy* proxy = w.matched_reader_lookup(reader);
    CHECK(proxy != nullptr && proxy->unicastLocator == &record[1],
          "the proxy refers to the record's UDP entry, not a copy of it");

    const Octet payload[1] = {0x5Au};
    CHECK(w.write_async(ChangeKind::ALIVE, payload, 1u), "first write");
    CHECK(!tx.dests.empty() && tx.dests.back() == first, "DATA goes to the UDP locator");

    record[1] = moved;
    tx.dests.clear();
    CHECK(w.write_async(ChangeKind::ALIVE, payload, 1u), "second write");
    CHECK(!tx.dests.empty(), "the second write is sent");
    for (const Locator& dest : tx.dests) {
        CHECK(dest == moved, "after the record moves, every send goes to the new address");
    }

    CHECK(w.matched_reader_count() == 1u, "the reader stays matched; no rematch is needed");

    record[1] = Locator{};
    tx.dests.clear();
    g_stateful_writer_test_now_ms = 100u;
    w.tick(100u);
    CHECK(tx.dests.empty(), "an emptied entry (kind RESERVED) is never sent to");
    return true;
}

void append_stateful_writer_tests_8_4_1_1_locator_reference(std::vector<TestCase>& tests) {
    tests.push_back({"TC-SWR-8.4.1.1-001",
                     "§8.4.1.1: a ReaderProxy refers to its reader's first UDP locator in the "
                     "owning record; sends follow that entry when it changes in place, and an "
                     "emptied entry is never sent to.",
                     tc_swr_8_4_1_1_001});
}

}  // namespace spec_test
