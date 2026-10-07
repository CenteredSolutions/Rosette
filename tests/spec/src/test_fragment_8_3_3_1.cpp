#include "spec_test_common.hpp"

#include "rosette/rtps/fragment_assembler.hpp"

namespace spec_test {

#define CHECK(COND, MSG) do { if (!(COND)) { err = (MSG); return false; } } while (0)

static Guid make_writer_guid(Octet p0, Octet p1, Octet p2, Octet kind) {
    Guid g{};
    g.prefix.value[0] = p0;
    g.prefix.value[1] = p1;
    g.prefix.value[2] = p2;
    g.entityId = make_entity(0x01u, 0x10u, 0x20u, kind);
    return g;
}

static bool tc_fas_8_3_3_1_001(std::string& err) {
    BoundedFragmentAssembler<1> assembler{};
    const Guid writer = make_writer_guid(0x71u, 0x72u, 0x73u, 0xC2u);
    const Timestamp ts{0, 0};
    const Octet frag[4] = {0x01u, 0x02u, 0x03u, 0x04u};
    CacheChange out{};

    const SequenceNumber sn1 = SequenceNumber::from_int64(10);
    const SequenceNumber sn2 = SequenceNumber::from_int64(11);

    CHECK(!assembler.on_data_frag(writer, sn1, 1u, 1u, 4u, 8u, frag, 4u, ts, out),
          "first partial sample should create in-flight slot");
    CHECK(assembler.active_count() == 1u, "expected one active slot");

    CHECK(!assembler.on_data_frag(writer, sn2, 1u, 1u, 4u, 8u, frag, 4u, ts, out),
          "newer sample from same writer should replace old in-flight sample");
    CHECK(assembler.active_count() == 1u, "slot count should remain one under KEEP_LAST(1)");

    FragmentNumberSet old_missing{};
    CHECK(!assembler.missing_fragments(writer, sn1, 2u, old_missing),
          "older sample should be evicted under KEEP_LAST(1)");

    FragmentNumberSet new_missing{};
    CHECK(assembler.missing_fragments(writer, sn2, 2u, new_missing),
          "new sample should remain in assembler state");
    CHECK(!new_missing.empty(), "new sample should still be missing fragment 2");
    return true;
}

void append_fragment_tests_8_3_3_1(std::vector<TestCase>& tests) {
    tests.push_back({"TC-FAS-8.3.3.1-001",
                     "§8.3.3.1 KEEP_LAST(1): a newer in-flight sample from the same writer evicts older partial assembly state.",
                     tc_fas_8_3_3_1_001});
}

} // namespace spec_test
