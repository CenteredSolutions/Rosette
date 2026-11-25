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

static bool max_is(const HistoryCache& cache, int64_t sn) {
    return cache.get_seq_num_max().to_int64() == sn;
}

static bool tc_hc_8_3_1_008(std::string& err) {
    HistoryCacheStorage<3> cache{};
    CHECK(cache.add_change(make_change(7, 0x17u)) != nullptr, "add_change SN 7 should succeed");
    CHECK(cache.add_change(make_change(4, 0x14u)) != nullptr, "add_change SN 4 should succeed");
    CHECK(max_is(cache, 7), "a lower SN added later should leave the maximum at 7");
    CHECK(cache.add_change(make_change(9, 0x19u)) != nullptr, "add_change SN 9 should succeed");
    CHECK(max_is(cache, 9), "a higher SN should become the maximum");

    // Full: SN 4 is evicted; the maximum is the change just added.
    CHECK(cache.add_change(make_change(12, 0x1Cu)) != nullptr, "add_change SN 12 should evict");
    CHECK(max_is(cache, 12), "the change that evicted the oldest should be the maximum");

    CHECK(cache.remove_change(SequenceNumber::from_int64(12)), "remove_change of the newest");
    CHECK(max_is(cache, 9), "removing the newest should expose the next highest SN");
    CHECK(cache.remove_change(SequenceNumber::from_int64(7)), "remove_change of an older SN");
    CHECK(max_is(cache, 9), "removing an older SN should leave the maximum unchanged");
    CHECK(cache.remove_change(SequenceNumber::from_int64(9)), "remove_change of the last change");
    CHECK(cache.get_seq_num_max() == SEQUENCENUMBER_UNKNOWN,
          "an emptied cache should report an UNKNOWN maximum");

    CHECK(cache.add_change(make_change(2, 0x12u)) != nullptr, "add_change after emptying");
    CHECK(max_is(cache, 2), "the first change after emptying should be the maximum");
    cache.clear_all();
    CHECK(cache.get_seq_num_max() == SEQUENCENUMBER_UNKNOWN, "clear_all should reset the maximum");

    // One slot: every insertion evicts the newest change itself.
    HistoryCacheStorage<1> single{};
    CHECK(single.add_change(make_change(5, 0x25u)) != nullptr, "add_change SN 5 to one slot");
    CHECK(single.add_change(make_change(3, 0x23u)) != nullptr, "add_change SN 3 evicts SN 5");
    CHECK(max_is(single, 3), "the maximum should follow the only change held");
    return true;
}

// A writer releases delivered replies and finished volatile changes by
// clearing them in place through the mutable traversal, not remove_change().
static bool tc_hc_8_3_1_009(std::string& err) {
    HistoryCacheStorage<3> cache{};
    CHECK(cache.add_change(make_change(1, 0x31u)) != nullptr, "add_change SN 1 should succeed");
    CHECK(cache.add_change(make_change(2, 0x32u)) != nullptr, "add_change SN 2 should succeed");
    CHECK(cache.add_change(make_change(3, 0x33u)) != nullptr, "add_change SN 3 should succeed");

    auto clear_sn = [&cache](int64_t sn) {
        cache.for_each_change([sn](HistoryChange& c) noexcept {
            if (c.sequenceNumber == SequenceNumber::from_int64(sn)) {
                c.clear();
            }
        });
    };

    clear_sn(1);
    CHECK(max_is(cache, 3), "clearing an older change should leave the maximum at 3");
    clear_sn(3);
    CHECK(max_is(cache, 2), "clearing the newest change should expose SN 2");
    CHECK(cache.add_change(make_change(4, 0x34u)) != nullptr, "add_change SN 4 should succeed");
    CHECK(max_is(cache, 4), "a change added after the traversal should become the maximum");

    cache.for_each_change([](HistoryChange& c) noexcept { c.clear(); });
    CHECK(cache.get_seq_num_max() == SEQUENCENUMBER_UNKNOWN,
          "clearing every change should report an UNKNOWN maximum");
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
    tests.push_back({"TC-HC-8.3.1-005",
                     "§8.3.1: get_seq_num_max follows insertions in any order, KEEP_LAST "
                     "eviction, removal of the newest and of older changes, and clear_all.",
                     tc_hc_8_3_1_008});
    tests.push_back({"TC-HC-8.3.1-006",
                     "§8.3.1: get_seq_num_max follows changes cleared in place by the mutable "
                     "for_each_change, including the newest.",
                     tc_hc_8_3_1_009});
}

} // namespace spec_test
