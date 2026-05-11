#include "spec_test_common.hpp"

#include "rosette/rtps/fragment_assembler.hpp"

namespace spec_test {

#define CHECK(COND, MSG) do { if (!(COND)) { err = (MSG); return false; } } while (0)

static Guid make_writer_guid(Octet p0, Octet p1, Octet p2, Octet kind) {
    Guid g{};
    g.prefix.value[0] = p0;
    g.prefix.value[1] = p1;
    g.prefix.value[2] = p2;
    g.entityId = make_entity(0x55u, 0x66u, 0x77u, kind);
    return g;
}

static bool tc_fas_8_4_12_001(std::string& err) {
    FragmentAssembler<2> assembler{};
    const Guid writer = make_writer_guid(0x91u, 0x92u, 0x93u, 0xC2u);
    const SequenceNumber sn = SequenceNumber::from_int64(77);
    const Timestamp ts{0, 0};

    const Octet frag1[4] = {0xA1u, 0xA2u, 0xA3u, 0xA4u};
    const Octet frag3[4] = {0xC1u, 0xC2u, 0xC3u, 0xC4u};
    CacheChange out{};

    CHECK(!assembler.on_data_frag(writer, sn, 1u, 1u, 4u, 16u, frag1, 4u, ts, out),
          "partial fragment set should not complete");
    CHECK(!assembler.on_data_frag(writer, sn, 3u, 1u, 4u, 16u, frag3, 4u, ts, out),
          "partial fragment set should not complete");

    FragmentNumberSet missing{};
    CHECK(assembler.missing_fragments(writer, sn, 4u, missing),
          "missing_fragments should resolve active assembly slot");
    CHECK(!missing.empty(), "missing fragment set should not be empty");

    uint32_t values[4] = {};
    uint32_t count = 0;
    missing.for_each([&](uint32_t frag_num) {
        if (count < 4u) values[count++] = frag_num;
    });

    CHECK(count == 2u, "expected exactly two missing fragments");
    CHECK(values[0] == 2u && values[1] == 4u,
          "missing set should contain fragments 2 and 4");

    assembler.discard(writer, sn);
    return true;
}

static bool tc_fas_8_4_12_002(std::string& err) {
    FragmentAssembler<2> assembler{};
    const Guid writer = make_writer_guid(0xA9u, 0xAAu, 0xABu, 0xC2u);
    const SequenceNumber sn = SequenceNumber::from_int64(88);

    FragmentNumberSet missing{};
    CHECK(!assembler.missing_fragments(writer, sn, 7u, missing),
          "missing_fragments should return false when no assembly slot exists");
    return true;
}

void append_fragment_tests_8_4_12(std::vector<TestCase>& tests) {
    tests.push_back({"TC-FAS-8.4.12-001",
                     "§8.4.12: NACK_FRAG fragment set is derived from the reader's not-yet-received fragments up to lastFragmentNum.",
                     tc_fas_8_4_12_001});
    tests.push_back({"TC-FAS-8.4.12-002",
                     "§8.4.12 precondition: missing fragment computation is defined only for an existing in-flight fragmented sample.",
                     tc_fas_8_4_12_002});
}

} // namespace spec_test
