#include "spec_test_common.hpp"

#include "rosette/rtps/fragment_assembler.hpp"

namespace spec_test {

#define CHECK(COND, MSG) do { if (!(COND)) { err = (MSG); return false; } } while (0)

static Guid make_writer_guid(Octet p0, Octet p1, Octet p2, Octet kind) {
    Guid g{};
    g.prefix.value[0] = p0;
    g.prefix.value[1] = p1;
    g.prefix.value[2] = p2;
    g.entityId = make_entity(0x11u, 0x22u, 0x33u, kind);
    return g;
}

static bool tc_fas_8_4_14_001(std::string& err) {
    FragmentAssembler<2> assembler{};
    const Guid writer = make_writer_guid(0xA1u, 0xB2u, 0xC3u, 0xC2u);
    const SequenceNumber sn = SequenceNumber::from_int64(101);
    const Timestamp ts{7, 9};
    const Octet frag[4] = {0x10u, 0x20u, 0x30u, 0x40u};

    CacheChange out{};
    const bool complete = assembler.on_data_frag(
        writer, sn, 1u, 1u, 8u, 4u, frag, 4u, ts, out);

    CHECK(complete, "single-fragment sample should complete immediately");
    CHECK(out.writerGuid == writer, "writer GUID should propagate into CacheChange");
    CHECK(out.sequenceNumber == sn, "sequence number should propagate into CacheChange");
    CHECK(out.sourceTimestamp == ts, "source timestamp should propagate into CacheChange");
    CHECK(out.payload.valid(), "completed CacheChange should own payload buffer");
    CHECK(out.payload.size == 4u, "CacheChange payload size should match dataSize");
    CHECK(out.payload.data[0] == 0x10u && out.payload.data[1] == 0x20u &&
          out.payload.data[2] == 0x30u && out.payload.data[3] == 0x40u,
          "payload bytes should match incoming fragment data");

    out.clear();
    return true;
}

static bool tc_fas_8_4_14_002(std::string& err) {
    FragmentAssembler<2> assembler{};
    const Guid writer = make_writer_guid(0xD1u, 0xD2u, 0xD3u, 0xC2u);
    const SequenceNumber sn = SequenceNumber::from_int64(202);
    const Timestamp ts{3, 4};

    const Octet frag2[4] = {0x04u, 0x05u, 0x06u, 0x07u};
    const Octet frag1[4] = {0x00u, 0x01u, 0x02u, 0x03u};
    const Octet frag3[2] = {0x08u, 0x09u};

    CacheChange out{};
    CHECK(!assembler.on_data_frag(writer, sn, 2u, 1u, 4u, 10u, frag2, 4u, ts, out),
          "assembly should remain incomplete after only fragment 2");
    CHECK(!assembler.on_data_frag(writer, sn, 1u, 1u, 4u, 10u, frag1, 4u, ts, out),
          "assembly should remain incomplete until final fragment arrives");
    CHECK(assembler.on_data_frag(writer, sn, 3u, 1u, 4u, 10u, frag3, 2u, ts, out),
          "assembly should complete after all fragments are received");

    CHECK(out.payload.size == 10u, "assembled payload should use declared dataSize");
    for (uint32_t i = 0; i < 10u; ++i) {
        CHECK(out.payload.data[i] == static_cast<Octet>(i), "assembled byte order mismatch");
    }

    out.clear();
    return true;
}

static bool tc_fas_8_4_14_003(std::string& err) {
    FragmentAssembler<2> assembler{};
    const Guid writer = make_writer_guid(0xE1u, 0xE2u, 0xE3u, 0xC2u);
    const SequenceNumber sn = SequenceNumber::from_int64(303);
    const Timestamp ts{0, 0};
    const Octet frag[4] = {0xAAu, 0xBBu, 0xCCu, 0xDDu};
    CacheChange out{};

    const bool ok = assembler.on_data_frag(
        writer, sn, 0u, 1u, 4u, 4u, frag, 4u, ts, out);
    CHECK(!ok, "fragmentStartingNum=0 should be rejected");
    CHECK(assembler.active_count() == 0u, "invalid fragment should not allocate assembly slot");
    return true;
}

static bool tc_fas_8_4_14_004(std::string& err) {
    FragmentAssembler<2> assembler{};
    const Guid writer = make_writer_guid(0xF1u, 0xF2u, 0xF3u, 0xC2u);
    const SequenceNumber sn = SequenceNumber::from_int64(404);
    const Timestamp ts{0, 0};
    const Octet frag[8] = {0};
    CacheChange out{};

    const bool ok = assembler.on_data_frag(
        writer, sn, 1u, 1u, 8u, 2049u, frag, 8u, ts, out);
    CHECK(!ok, "samples needing more than 256 fragments should be rejected");
    CHECK(assembler.active_count() == 0u, "oversized sample should not allocate assembly slot");
    return true;
}

static bool tc_fas_8_4_14_005(std::string& err) {
    FragmentAssembler<3> assembler{};
    const Guid writer_a = make_writer_guid(0x21u, 0x22u, 0x23u, 0xC2u);
    const Guid writer_b = make_writer_guid(0x31u, 0x32u, 0x33u, 0xC2u);
    const SequenceNumber sn = SequenceNumber::from_int64(1);
    const Timestamp ts{0, 0};
    const Octet frag[4] = {1u, 2u, 3u, 4u};
    CacheChange out{};

    CHECK(!assembler.on_data_frag(writer_a, sn, 1u, 1u, 4u, 8u, frag, 4u, ts, out),
          "writer A should have one partial sample");
    CHECK(!assembler.on_data_frag(writer_b, sn, 1u, 1u, 4u, 8u, frag, 4u, ts, out),
          "writer B should have one partial sample");

    assembler.discard(writer_a, sn);
    FragmentNumberSet missing_a{};
    CHECK(!assembler.missing_fragments(writer_a, sn, 2u, missing_a),
          "discard(writer,sn) should clear that assembly slot");

    assembler.discard_writer(writer_b);
    FragmentNumberSet missing_b{};
    CHECK(!assembler.missing_fragments(writer_b, sn, 2u, missing_b),
          "discard_writer(writer) should clear all writer slots");
    CHECK(assembler.active_count() == 0u, "all slots should be released after discard operations");
    return true;
}

void append_fragment_tests_8_4_14(std::vector<TestCase>& tests) {
    tests.push_back({"TC-FAS-8.4.14-001",
                     "§8.4.14: once all fragments are received, FragmentedChange becomes a complete CacheChange.",
                     tc_fas_8_4_14_001});
    tests.push_back({"TC-FAS-8.4.14-002",
                     "§8.4.14: out-of-order DATA_FRAG arrivals are assembled by fragment number and complete only when all fragments are present.",
                     tc_fas_8_4_14_002});
    tests.push_back({"TC-FAS-8.4.14-003",
                     "§9.4.5.4/§8.4.14: fragmentStartingNum is 1-based; zero is invalid and shall not create assembly state.",
                     tc_fas_8_4_14_003});
    tests.push_back({"TC-FAS-8.4.14-004",
                     "§9.4.2.8-linked limit: fragment bitmap supports up to 256 fragments; larger samples are rejected by MVP assembler.",
                     tc_fas_8_4_14_004});
    tests.push_back({"TC-FAS-8.4.14-005",
                     "§8.4.14 lifecycle: discard(writer,sn) and discard_writer(writer) release partial fragment assembly state.",
                     tc_fas_8_4_14_005});
}

} // namespace spec_test
