/**
 * @file test_fragment_8_4_14_shared_pool.cpp
 * @brief Readers sharing one fragment assembly pool (§8.4.14, §8.3.3.1).
 *
 * RTPSParticipant's user readers reassemble in one FragmentAssemblyPool.
 * These cases hold the pool's rules: a reader replaces only its own partial
 * samples, never holds more than its limit, is refused when the pool is full
 * and it holds nothing to replace, and returns its slots when destroyed.
 */

#include "spec_test_common.hpp"

#include "rosette/rtps/fragment_assembler.hpp"

#include <vector>

namespace spec_test {

#define CHECK(COND, MSG)  \
    do {                  \
        if (!(COND)) {    \
            err = (MSG);  \
            return false; \
        }                 \
    } while (0)

namespace {

constexpr uint16_t kFragSize = 300u;
constexpr uint32_t kSampleSize = 600u;  // two fragments
const Octet kFrag[kFragSize] = {};

Guid writer_guid(Octet id) {
    Guid g{};
    g.prefix.value[0] = 0x5Au;
    g.prefix.value[1] = id;
    g.entityId = make_entity(0x00u, 0x00u, id, 0x03u);
    return g;
}

/// Sends fragment `frag` (1 or 2) of the two-fragment sample `sn` from `writer`.
bool send_fragment(FragmentAssembler& a, const Guid& writer, int64_t sn, uint32_t frag,
                   CacheChange& out) {
    return a.on_data_frag(writer, SequenceNumber::from_int64(sn), frag, 1u, kFragSize, kSampleSize,
                          kFrag, kFragSize, TIME_ZERO, out);
}

bool holds(const FragmentAssembler& a, const Guid& writer, int64_t sn) {
    FragmentNumberSet missing{};
    return a.missing_fragments(writer, SequenceNumber::from_int64(sn), UINT32_MAX, missing);
}

}  // namespace

// A reader at its limit replaces its own sample from the same writer first,
// then its own oldest, and never another reader's.
static bool tc_fas_8_4_14_009(std::string& err) {
    FragmentAssemblyPoolStorage<3> pool;
    FragmentAssembler motion(pool, 2u);
    FragmentAssembler map(pool, 2u);
    const Guid cmd = writer_guid(1u);
    const Guid a = writer_guid(2u);
    const Guid b = writer_guid(3u);
    const Guid c = writer_guid(4u);
    CacheChange out{};

    CHECK(!send_fragment(motion, cmd, 1, 1u, out), "the motion command is partial");
    CHECK(!send_fragment(map, a, 1, 1u, out) && !send_fragment(map, b, 1, 1u, out),
          "the map reader takes the other two slots");
    CHECK(pool.active_count() == 3u, "the pool is full");

    CHECK(!send_fragment(map, c, 1, 1u, out), "a third map writer starts a sample");
    CHECK(map.active_count() == 2u, "the map reader never holds more than its limit");
    CHECK(!holds(map, a, 1) && holds(map, b, 1) && holds(map, c, 1),
          "the map reader replaces its own oldest sample");
    CHECK(holds(motion, cmd, 1), "another reader's partial sample is never replaced");

    CHECK(!send_fragment(map, c, 2, 1u, out), "writer c's next sample starts");
    CHECK(holds(map, b, 1) && !holds(map, c, 1) && holds(map, c, 2),
          "a newer sample from the same writer replaces that writer's, not the oldest");

    CHECK(send_fragment(motion, cmd, 1, 2u, out), "the motion command completes");
    CHECK(out.payload.valid() && out.payload.size == kSampleSize, "with its whole payload");
    out.clear();

    CHECK(!send_fragment(map, writer_guid(5u), 1, 1u, out), "a fourth map writer starts a sample");
    CHECK(map.active_count() == 2u && pool.active_count() == 2u,
          "a reader at its limit replaces its own sample even with a slot free");
    return true;
}

// A reader that holds no slot is refused while the pool is full, and a
// destroyed reader returns its slots so the refused one can assemble.
static bool tc_fas_8_4_14_010(std::string& err) {
    FragmentAssemblyPoolStorage<2> pool;
    FragmentAssembler quiet(pool, 2u);
    const Guid writer = writer_guid(7u);
    CacheChange out{};
    {
        FragmentAssembler busy(pool, 2u);
        CHECK(!send_fragment(busy, writer_guid(8u), 1, 1u, out)
                  && !send_fragment(busy, writer_guid(9u), 1, 1u, out),
              "the busy reader fills the pool");
        CHECK(!send_fragment(quiet, writer, 1, 1u, out), "the pool is full");
        CHECK(quiet.active_count() == 0u && pool.active_count() == 2u,
              "a refused sample takes no slot and evicts nothing");
        CHECK(!send_fragment(quiet, writer, 1, 2u, out),
              "a refused sample cannot complete from its remaining fragments");
    }

    CHECK(pool.active_count() == 0u, "a destroyed reader returns its slots");
    CHECK(!send_fragment(quiet, writer, 1, 1u, out) && send_fragment(quiet, writer, 1, 2u, out),
          "the resent sample assembles once a slot is free");
    out.clear();
    return true;
}

// A DATA_FRAG that restates the sample's fragmentSize or dataSize differently
// from the one in assembly is refused and leaves the assembly untouched.
static bool tc_fas_8_4_14_011(std::string& err) {
    FragmentAssemblyPoolStorage<2> pool;
    FragmentAssembler a(pool, 2u);
    const Guid writer = writer_guid(10u);
    CacheChange out{};

    CHECK(!send_fragment(a, writer, 1, 1u, out), "the first fragment starts the sample");
    CHECK(!a.on_data_frag(writer, SequenceNumber::from_int64(1), 2u, 1u, kFragSize / 2u,
                          kSampleSize, kFrag, kFragSize / 2u, TIME_ZERO, out),
          "a different fragmentSize is refused");
    CHECK(!a.on_data_frag(writer, SequenceNumber::from_int64(1), 2u, 1u, kFragSize,
                          kSampleSize + kFragSize, kFrag, kFragSize, TIME_ZERO, out),
          "a different dataSize is refused");

    FragmentNumberSet missing{};
    CHECK(
        a.missing_fragments(writer, SequenceNumber::from_int64(1), 2u, missing) && !missing.empty(),
        "the refused fragments marked nothing received");
    CHECK(send_fragment(a, writer, 1, 2u, out) && out.payload.size == kSampleSize,
          "the consistent fragment completes the sample");
    out.clear();
    return true;
}

// A fragment of a sample older than the writer's one in assembly is refused,
// and the newer partial sample stays.
static bool tc_fas_8_4_14_012(std::string& err) {
    FragmentAssemblyPoolStorage<2> pool;
    FragmentAssembler a(pool, 1u);
    const Guid writer = writer_guid(11u);
    CacheChange out{};

    CHECK(!send_fragment(a, writer, 5, 1u, out), "sample 5 starts");
    CHECK(!send_fragment(a, writer, 4, 1u, out),
          "a delayed fragment of sample 4 does not complete");
    CHECK(holds(a, writer, 5) && !holds(a, writer, 4),
          "sample 4 is refused and sample 5 keeps its slot");
    CHECK(!send_fragment(a, writer, 6, 1u, out) && holds(a, writer, 6) && !holds(a, writer, 5),
          "a newer sample 6 still replaces sample 5");
    return true;
}

// A first DATA_FRAG that cannot store a fragment takes no slot, so it never
// evicts the partial sample a full pool holds.
static bool tc_fas_8_4_14_013(std::string& err) {
    FragmentAssemblyPoolStorage<1> pool;
    FragmentAssembler a(pool, 1u);
    const Guid held = writer_guid(12u);
    const Guid other = writer_guid(13u);
    const SequenceNumber sn = SequenceNumber::from_int64(1);
    CacheChange out{};

    CHECK(!send_fragment(a, held, 1, 1u, out), "the held sample starts");
    CHECK(!a.on_data_frag(other, sn, 1u, 0u, kFragSize, kSampleSize, kFrag, kFragSize, TIME_ZERO,
                          out),
          "a submessage of no fragments is refused");
    CHECK(!a.on_data_frag(other, sn, 3u, 1u, kFragSize, kSampleSize, kFrag, kFragSize, TIME_ZERO,
                          out),
          "a fragment number past the last is refused");
    CHECK(!a.on_data_frag(other, sn, 1u, 1u, kFragSize, kSampleSize, kFrag, kFragSize - 1u,
                          TIME_ZERO, out),
          "a payload shorter than its fragment is refused");
    CHECK(holds(a, held, 1) && !holds(a, other, 1) && pool.active_count() == 1u,
          "none of them took a slot or evicted the held sample");
    return true;
}

// With slots to spare, a reader still keeps one partial sample per writer: a
// newer sample replaces the older, a delayed older fragment is refused, and
// other writers keep their own slots.
static bool tc_fas_8_4_14_014(std::string& err) {
    FragmentAssemblyPoolStorage<4> pool;
    FragmentAssembler a(pool, 4u);
    const Guid first = writer_guid(14u);
    const Guid second = writer_guid(15u);
    CacheChange out{};

    CHECK(!send_fragment(a, first, 5, 1u, out) && !send_fragment(a, second, 1, 1u, out),
          "two writers each start a sample");
    CHECK(!send_fragment(a, first, 6, 1u, out), "the first writer's newer sample starts");
    CHECK(
        a.active_count() == 2u && !holds(a, first, 5) && holds(a, first, 6) && holds(a, second, 1),
        "it replaced that writer's older sample and left the other writer's alone");

    CHECK(!send_fragment(a, first, 5, 2u, out) && !send_fragment(a, first, 5, 1u, out),
          "the delayed fragments of sample 5 do not complete it");
    CHECK(a.active_count() == 2u && !holds(a, first, 5) && holds(a, first, 6),
          "sample 5 is refused and does not start beside sample 6");
    return true;
}

void append_fragment_tests_8_4_14_shared_pool(std::vector<TestCase>& tests) {
    tests.push_back(
        {"TC-FAS-8.4.14-009",
         "§8.4.14/§8.3.3.1: a reader sharing the assembly pool replaces only its own partial "
         "samples, the same writer's first, then its oldest, and never holds more than its limit.",
         tc_fas_8_4_14_009});
    tests.push_back(
        {"TC-FAS-8.4.14-010",
         "§8.4.14: with the assembly pool full, a reader holding no slot is refused without "
         "evicting another reader's sample, and assembles once a reader returns its slots.",
         tc_fas_8_4_14_010});
    tests.push_back(
        {"TC-FAS-8.4.14-011",
         "§9.4.5.4: a DATA_FRAG whose fragmentSize or dataSize differs from the sample in "
         "assembly is refused and leaves the assembly untouched.",
         tc_fas_8_4_14_011});
    tests.push_back(
        {"TC-FAS-8.4.14-012",
         "§8.3.3.1: KEEP_LAST(1) per writer: a fragment of an older sample never evicts the "
         "newer partial sample, and a newer sample replaces the older.",
         tc_fas_8_4_14_012});
    tests.push_back(
        {"TC-FAS-8.4.14-013",
         "§8.4.14: a first DATA_FRAG that cannot store a fragment (no fragments, a number past "
         "the last, a short payload) takes no slot and evicts no other sample.",
         tc_fas_8_4_14_013});
    tests.push_back(
        {"TC-FAS-8.4.14-014",
         "§8.3.3.1: a reader holds one partial sample per writer even with slots to spare; a "
         "newer sample replaces the older and a delayed older fragment is refused.",
         tc_fas_8_4_14_014});
}

}  // namespace spec_test
