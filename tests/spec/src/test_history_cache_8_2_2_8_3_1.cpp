#include "spec_test_common.hpp"

#include "rosette/rtps/history_cache.hpp"

namespace spec_test {

#define CHECK(COND, MSG) do { if (!(COND)) { err = (MSG); return false; } } while (0)

namespace {

Guid make_guid(Octet p0, Octet p1, Octet p2, const EntityId& eid) {
    Guid g{};
    g.prefix.value[0] = p0;
    g.prefix.value[1] = p1;
    g.prefix.value[2] = p2;
    g.entityId = eid;
    return g;
}

CacheChange make_change(const Guid& writer_guid,
                        int64_t sn,
                        Octet payload_byte) {
    CacheChange c{};
    c.kind = ChangeKind::ALIVE;
    c.writerGuid = writer_guid;
    c.sequenceNumber = SequenceNumber::from_int64(sn);
    c.sourceTimestamp = TIME_ZERO;
    c.inlineQos = {};

    BufRef p = buf_alloc(32u);
    if (p.valid()) {
        p.data[0] = payload_byte;
        p.size = 1u;
        c.payload = static_cast<BufRef&&>(p);
    }
    return c;
}

} // namespace

static bool tc_hc_8_3_1_001(std::string& err) {
    HistoryCacheStorage<3> cache{};
    CHECK(cache.is_empty(), "new cache should start empty");
    CHECK(cache.size() == 0u, "new cache size should be zero");
    CHECK(cache.get_seq_num_min() == SEQUENCENUMBER_UNKNOWN,
          "empty cache get_seq_num_min should be UNKNOWN");
    CHECK(cache.get_seq_num_max() == SEQUENCENUMBER_UNKNOWN,
          "empty cache get_seq_num_max should be UNKNOWN");

    const Guid w = make_guid(0x10u, 0x11u, 0x12u, make_entity(1u, 1u, 1u, 0xC2u));
    CHECK(cache.add_change(make_change(w, 5, 0xA5u)) != nullptr,
          "add_change should insert first change");
    CHECK(cache.add_change(make_change(w, 3, 0xA3u)) != nullptr,
          "add_change should insert second change");
    CHECK(!cache.is_empty(), "cache should be non-empty after inserts");
    CHECK(cache.size() == 2u, "cache size should track live changes");
    CHECK(cache.get_seq_num_min().to_int64() == 3, "get_seq_num_min should return minimum SN");
    CHECK(cache.get_seq_num_max().to_int64() == 5, "get_seq_num_max should return maximum SN");
    CHECK(cache.get_change(SequenceNumber::from_int64(3)) != nullptr,
          "get_change should find inserted SN");
    CHECK(cache.get_change(SequenceNumber::from_int64(99)) == nullptr,
          "get_change should return null for absent SN");
    return true;
}

static bool tc_hc_8_3_1_002(std::string& err) {
    HistoryCacheStorage<2> cache{};
    const Guid w = make_guid(0x20u, 0x21u, 0x22u, make_entity(2u, 2u, 2u, 0xC2u));

    CHECK(cache.add_change(make_change(w, 1, 0xB1u)) != nullptr, "add_change #1 should succeed");
    CHECK(cache.add_change(make_change(w, 2, 0xB2u)) != nullptr, "add_change #2 should succeed");
    CHECK(cache.add_change(make_change(w, 3, 0xB3u)) != nullptr,
          "add_change on full cache should evict oldest for same writer");

    CHECK(cache.get_change(SequenceNumber::from_int64(1)) == nullptr,
          "oldest same-writer SN should be evicted when full");
    CHECK(cache.get_change(SequenceNumber::from_int64(2)) != nullptr &&
          cache.get_change(SequenceNumber::from_int64(3)) != nullptr,
          "newest same-writer SNs should remain after eviction");
    return true;
}

static bool tc_hc_8_3_1_003(std::string& err) {
    HistoryCacheStorage<2> cache{};
    const Guid wa = make_guid(0x31u, 0x32u, 0x33u, make_entity(3u, 3u, 3u, 0xC2u));
    const Guid wb = make_guid(0x41u, 0x42u, 0x43u, make_entity(4u, 4u, 4u, 0xC2u));
    const Guid wc = make_guid(0x51u, 0x52u, 0x53u, make_entity(5u, 5u, 5u, 0xC2u));

    CHECK(cache.add_change(make_change(wa, 5, 0xC5u)) != nullptr, "add_change A should succeed");
    CHECK(cache.add_change(make_change(wb, 2, 0xC2u)) != nullptr, "add_change B should succeed");
    CHECK(cache.add_change(make_change(wc, 7, 0xC7u)) != nullptr,
          "add_change with unmatched writer on full cache should evict global oldest");

    CHECK(cache.get_change(SequenceNumber::from_int64(2)) == nullptr,
          "globally oldest SN should be evicted when no same-writer victim exists");
    CHECK(cache.get_change(SequenceNumber::from_int64(5)) != nullptr &&
          cache.get_change(SequenceNumber::from_int64(7)) != nullptr,
          "non-evicted SNs should remain in cache");
    return true;
}

static bool tc_hc_8_3_1_004(std::string& err) {
    HistoryCacheStorage<3> cache{};
    const Guid w = make_guid(0x61u, 0x62u, 0x63u, make_entity(6u, 6u, 6u, 0xC2u));

    CHECK(cache.add_change(make_change(w, 10, 0xDAu)) != nullptr, "add_change #1 should succeed");
    CHECK(cache.add_change(make_change(w, 11, 0xDBu)) != nullptr, "add_change #2 should succeed");
    CHECK(cache.remove_change(SequenceNumber::from_int64(10)),
          "remove_change should return true for present SN");
    CHECK(!cache.remove_change(SequenceNumber::from_int64(77)),
          "remove_change should return false for missing SN");
    CHECK(cache.get_change(SequenceNumber::from_int64(10)) == nullptr,
          "removed SN should no longer be accessible");
    CHECK(cache.size() == 1u, "cache size should decrement after remove_change");
    return true;
}

static bool tc_hc_8_3_1_005(std::string& err) {
    HistoryCacheStorage<4> cache{};
    const Guid w = make_guid(0x71u, 0x72u, 0x73u, make_entity(7u, 7u, 7u, 0xC2u));
    CHECK(cache.add_change(make_change(w, 1, 0xE1u)) != nullptr, "add_change #1 should succeed");
    CHECK(cache.add_change(make_change(w, 4, 0xE4u)) != nullptr, "add_change #2 should succeed");
    CHECK(cache.add_change(make_change(w, 3, 0xE3u)) != nullptr, "add_change #3 should succeed");

    int const_iter_count = 0;
    const auto& ccache = cache;
    ccache.for_each_change([&](const CacheChange& c) noexcept {
        (void)c;
        const_iter_count++;
    });
    CHECK(const_iter_count == 3, "for_each_change const should visit each live change once");

    int mutated = 0;
    cache.for_each_change([&](CacheChange& c) noexcept {
        c.kind = ChangeKind::NOT_ALIVE_DISPOSED;
        mutated++;
    });
    CHECK(mutated == 3, "for_each_change mutable should visit each live change once");
    const CacheChange* c1 = cache.get_change(SequenceNumber::from_int64(1));
    const CacheChange* c3 = cache.get_change(SequenceNumber::from_int64(3));
    const CacheChange* c4 = cache.get_change(SequenceNumber::from_int64(4));
    CHECK(c1 && c3 && c4 &&
          c1->kind == ChangeKind::NOT_ALIVE_DISPOSED &&
          c3->kind == ChangeKind::NOT_ALIVE_DISPOSED &&
          c4->kind == ChangeKind::NOT_ALIVE_DISPOSED,
          "mutable for_each_change updates should persist in cache");
    return true;
}

static bool tc_hc_8_3_1_006(std::string& err) {
    HistoryCacheStorage<4> cache{};
    const Guid w = make_guid(0x81u, 0x82u, 0x83u, make_entity(8u, 8u, 8u, 0xC2u));
    CHECK(cache.add_change(make_change(w, 1, 0xF1u)) != nullptr, "add_change #1 should succeed");
    CHECK(cache.add_change(make_change(w, 3, 0xF3u)) != nullptr, "add_change #2 should succeed");
    CHECK(cache.add_change(make_change(w, 5, 0xF5u)) != nullptr, "add_change #3 should succeed");

    std::vector<int64_t> replayed{};
    cache.replay_to_new_reader([&](const CacheChange& c) noexcept {
        replayed.push_back(c.sequenceNumber.to_int64());
    });
    CHECK(replayed.size() == 3u, "replay_to_new_reader should visit all cached changes");
    CHECK(replayed[0] == 1 && replayed[1] == 3 && replayed[2] == 5,
          "replay_to_new_reader should replay in ascending sequence order");
    return true;
}

static bool tc_hc_8_3_1_007(std::string& err) {
    HistoryCacheStorage<3> cache{};
    const Guid w = make_guid(0x91u, 0x92u, 0x93u, make_entity(9u, 9u, 9u, 0xC2u));
    CHECK(cache.add_change(make_change(w, 21, 0x11u)) != nullptr, "add_change #1 should succeed");
    CHECK(cache.add_change(make_change(w, 22, 0x12u)) != nullptr, "add_change #2 should succeed");
    CHECK(cache.size() == 2u, "precondition: cache should contain entries");

    cache.clear_all();
    CHECK(cache.is_empty(), "clear_all should empty the cache");
    CHECK(cache.size() == 0u, "clear_all should reset size to zero");
    CHECK(cache.get_change(SequenceNumber::from_int64(21)) == nullptr,
          "clear_all should remove existing entries");
    CHECK(cache.get_seq_num_min() == SEQUENCENUMBER_UNKNOWN &&
          cache.get_seq_num_max() == SEQUENCENUMBER_UNKNOWN,
          "clear_all should reset sequence bounds to UNKNOWN");
    return true;
}

void append_history_cache_tests_8_2_2_8_3_1(std::vector<TestCase>& tests) {
    tests.push_back({"TC-HC-8.3.1-001",
                     "§8.3.1: add_change/get_change/get_seq_num_min/get_seq_num_max/is_empty/size expose expected HistoryCache state transitions.",
                     tc_hc_8_3_1_001});
    tests.push_back({"TC-HC-8.3.3.1-001",
                     "§8.3.3.1: when full, add_change evicts the oldest change for the same writer under KEEP_LAST semantics.",
                     tc_hc_8_3_1_002});
    tests.push_back({"TC-HC-8.3.3.1-002",
                     "§8.3.3.1 implementation behavior: when full with no same-writer candidate, add_change evicts the globally oldest change.",
                     tc_hc_8_3_1_003});
    tests.push_back({"TC-HC-8.3.1-002",
                     "§8.3.1: remove_change removes present changes and reports false for absent sequence numbers.",
                     tc_hc_8_3_1_004});
    tests.push_back({"TC-HC-8.3.1-003",
                     "§8.3.1: for_each_change iterates all live CacheChange entries for both const and mutable traversal forms.",
                     tc_hc_8_3_1_005});
    tests.push_back({"TC-HC-8.2.2-001",
                     "§8.2.2 spirit + §8.7.2.2 linkage: replay_to_new_reader yields cached changes in ascending sequence order.",
                     tc_hc_8_3_1_006});
    tests.push_back({"TC-HC-8.3.1-004",
                     "§8.3.1: clear_all removes all CacheChange entries and resets empty/min/max observable state.",
                     tc_hc_8_3_1_007});
}

} // namespace spec_test
