#include "spec_test_common.hpp"

#include "rtps/stateful_reader.hpp"
#include "rtps/transport.hpp"

namespace spec_test {

#define CHECK(COND, MSG) do { if (!(COND)) { err = (MSG); return false; } } while (0)

namespace {

struct NullTransport : ITransport {
    bool send_unicast(const BufRef&, const Locator&) noexcept override { return true; }
    bool send_multicast(const BufRef&, const Locator&) noexcept override { return true; }
};

Guid make_guid(Octet p0, Octet p1, Octet p2, const EntityId& eid) {
    Guid g{};
    g.prefix.value[0] = p0;
    g.prefix.value[1] = p1;
    g.prefix.value[2] = p2;
    g.entityId = eid;
    return g;
}

} // namespace

static bool tc_srr_8_4_7_4_001(std::string& err) {
    NullTransport tx{};
    const Guid reader_guid = make_guid(0x11u, 0x12u, 0x13u, make_entity(1u, 2u, 3u, 0xC7u));
    const Guid writer_guid = make_guid(0x21u, 0x22u, 0x23u, make_entity(4u, 5u, 6u, 0xC2u));
    const Locator loc = make_udpv4_locator(0xC0A80114u, 7411u);

    StatefulReader r(reader_guid, ReliabilityKind::RELIABLE, tx);
    CHECK(r.matched_writer_add(writer_guid, &loc, 1u), "first matched_writer_add should succeed");
    CHECK(r.matched_writer_count() == 1u, "matched writer count should be one");
    CHECK(r.matched_writer_lookup(writer_guid) != nullptr, "matched writer should be discoverable");

    CHECK(!r.matched_writer_add(writer_guid, &loc, 1u),
          "duplicate matched_writer_add should be rejected");

    r.matched_writer_remove(writer_guid);
    CHECK(r.matched_writer_count() == 0u, "matched writer count should drop after remove");
    CHECK(r.matched_writer_lookup(writer_guid) == nullptr,
          "removed writer should no longer be discoverable");
    return true;
}

void append_stateful_reader_tests_8_4_7_4(std::vector<TestCase>& tests) {
    tests.push_back({"TC-SRR-8.4.7.4-001",
                     "§8.4.7.4: matched_writer_add creates one WriterProxy for a newly matched writer, and matched_writer_remove removes it from the matched set.",
                     tc_srr_8_4_7_4_001});
}

} // namespace spec_test
