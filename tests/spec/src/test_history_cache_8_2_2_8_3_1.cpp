// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

#include "spec_test_common.hpp"

#include "rosette/rtps/history_cache.hpp"

namespace spec_test {

#define CHECK(COND, MSG) do { if (!(COND)) { err = (MSG); return false; } } while (0)

namespace {

HistoryChange make_change(int64_t sn, Octet payload_byte) {
    HistoryChange c{};
    c.kind = ChangeKind::ALIVE;
    c.sequenceNumber = SequenceNumber::from_int64(sn);

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

    CHECK(cache.add_change(make_change(5, 0xA5u)) != nullptr,
          "add_change should insert first change");
    CHECK(cache.add_change(make_change(3, 0xA3u)) != nullptr,
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

    CHECK(cache.add_change(make_change(1, 0xB1u)) != nullptr, "add_change #1 should succeed");
    CHECK(cache.add_change(make_change(2, 0xB2u)) != nullptr, "add_change #2 should succeed");
    CHECK(cache.add_change(make_change(3, 0xB3u)) != nullptr,
          "add_change on full cache should evict the oldest change");

    CHECK(cache.get_change(SequenceNumber::from_int64(1)) == nullptr,
          "oldest SN should be evicted when full");
    CHECK(cache.get_change(SequenceNumber::from_int64(2)) != nullptr
              && cache.get_change(SequenceNumber::from_int64(3)) != nullptr,
          "newest SNs should remain after eviction");
    return true;
}

static bool tc_hc_8_3_1_003(std::string& err) {
    HistoryCacheStorage<2> cache{};

    CHECK(cache.add_change(make_change(5, 0xC5u)) != nullptr, "add_change SN 5 should succeed");
    CHECK(cache.add_change(make_change(2, 0xC2u)) != nullptr, "add_change SN 2 should succeed");
    CHECK(cache.add_change(make_change(7, 0xC7u)) != nullptr,
          "add_change on a full cache should evict the lowest sequence number");

    CHECK(cache.get_change(SequenceNumber::from_int64(2)) == nullptr,
          "the lowest SN should be evicted, though it was added after SN 5");
    CHECK(cache.get_change(SequenceNumber::from_int64(5)) != nullptr &&
          cache.get_change(SequenceNumber::from_int64(7)) != nullptr,
          "non-evicted SNs should remain in cache");
    return true;
}

static bool tc_hc_8_3_1_004(std::string& err) {
    HistoryCacheStorage<3> cache{};

    CHECK(cache.add_change(make_change(10, 0xDAu)) != nullptr, "add_change #1 should succeed");
    CHECK(cache.add_change(make_change(11, 0xDBu)) != nullptr, "add_change #2 should succeed");
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
    CHECK(cache.add_change(make_change(1, 0xE1u)) != nullptr, "add_change #1 should succeed");
    CHECK(cache.add_change(make_change(4, 0xE4u)) != nullptr, "add_change #2 should succeed");
    CHECK(cache.add_change(make_change(3, 0xE3u)) != nullptr, "add_change #3 should succeed");

    int const_iter_count = 0;
    const auto& ccache = cache;
    ccache.for_each_change([&](const HistoryChange& c) noexcept {
        (void)c;
        const_iter_count++;
    });
    CHECK(const_iter_count == 3, "for_each_change const should visit each live change once");

    int mutated = 0;
    cache.for_each_change([&](HistoryChange& c) noexcept {
        c.kind = ChangeKind::NOT_ALIVE_DISPOSED;
        mutated++;
    });
    CHECK(mutated == 3, "for_each_change mutable should visit each live change once");
    const HistoryChange* c1 = cache.get_change(SequenceNumber::from_int64(1));
    const HistoryChange* c3 = cache.get_change(SequenceNumber::from_int64(3));
    const HistoryChange* c4 = cache.get_change(SequenceNumber::from_int64(4));
    CHECK(c1 && c3 && c4 &&
          c1->kind == ChangeKind::NOT_ALIVE_DISPOSED &&
          c3->kind == ChangeKind::NOT_ALIVE_DISPOSED &&
          c4->kind == ChangeKind::NOT_ALIVE_DISPOSED,
          "mutable for_each_change updates should persist in cache");
    return true;
}

static bool tc_hc_8_3_1_006(std::string& err) {
    HistoryCacheStorage<4> cache{};
    CHECK(cache.add_change(make_change(1, 0xF1u)) != nullptr, "add_change #1 should succeed");
    CHECK(cache.add_change(make_change(3, 0xF3u)) != nullptr, "add_change #2 should succeed");
    CHECK(cache.add_change(make_change(5, 0xF5u)) != nullptr, "add_change #3 should succeed");

    std::vector<int64_t> replayed{};
    cache.replay_to_new_reader(
        [&](const HistoryChange& c) noexcept { replayed.push_back(c.sequenceNumber.to_int64()); });
    CHECK(replayed.size() == 3u, "replay_to_new_reader should visit all cached changes");
    CHECK(replayed[0] == 1 && replayed[1] == 3 && replayed[2] == 5,
          "replay_to_new_reader should replay in ascending sequence order");
    return true;
}

static bool tc_hc_8_3_1_007(std::string& err) {
    HistoryCacheStorage<3> cache{};
    CHECK(cache.add_change(make_change(21, 0x11u)) != nullptr, "add_change #1 should succeed");
    CHECK(cache.add_change(make_change(22, 0x12u)) != nullptr, "add_change #2 should succeed");
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
    tests.push_back(
        {"TC-HC-8.3.3.1-001",
         "§8.3.3.1: when full, add_change evicts the oldest change under KEEP_LAST semantics.",
         tc_hc_8_3_1_002});
    tests.push_back({"TC-HC-8.3.3.1-002",
                     "§8.3.3.1: the change evicted from a full cache is the one with the lowest "
                     "sequence number, whatever the insertion order.",
                     tc_hc_8_3_1_003});
    tests.push_back({"TC-HC-8.3.1-002",
                     "§8.3.1: remove_change removes present changes and reports false for absent sequence numbers.",
                     tc_hc_8_3_1_004});
    tests.push_back({"TC-HC-8.3.1-003",
                     "§8.3.1: for_each_change iterates all live HistoryChange entries for both "
                     "const and mutable traversal forms.",
                     tc_hc_8_3_1_005});
    tests.push_back({"TC-HC-8.2.2-001",
                     "§8.2.2 spirit + §8.7.2.2 linkage: replay_to_new_reader yields cached changes in ascending sequence order.",
                     tc_hc_8_3_1_006});
    tests.push_back({"TC-HC-8.3.1-004",
                     "§8.3.1: clear_all removes all HistoryChange entries and resets empty/min/max "
                     "observable state.",
                     tc_hc_8_3_1_007});
}

} // namespace spec_test
