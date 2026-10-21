/**
 * @file test_proxy_pool_admission.cpp
 * @brief ProxyPool: free list, endpoint lists, floors and caps (rtps/proxy_pool.hpp).
 *
 * A participant's writers share one pool of ReaderProxy and its readers one
 * pool of WriterProxy (§8.4.1.1, §8.4.7.1). These cases check the pool's
 * admission rule on its own: a list below its floor always binds, a list at
 * or above it borrows only proxies no floor is promised, no list exceeds its
 * cap, and `free >= reserved` holds after every operation. The proxies are
 * plain integers, since the pool never touches what it stores.
 */

#include "spec_test_common.hpp"

#include "rosette/rtps/proxy_pool.hpp"

#include <vector>

namespace rtps = rosette::rtps;

namespace spec_test {

#define CHECK(COND, MSG)  \
    do {                  \
        if (!(COND)) {    \
            err = (MSG);  \
            return false; \
        }                 \
    } while (0)

namespace {

using rtps::kNoProxy;
using rtps::ProxyIndex;
using rtps::ProxyList;
using rtps::ProxyPoolStorage;

ProxyList make_list(uint8_t floor, uint8_t cap) {
    ProxyList list{};
    list.floor = floor;
    list.cap = cap;
    return list;
}

/// The indices bound to `list`, in list order.
template <class Pool>
std::vector<ProxyIndex> members(const Pool& pool, const ProxyList& list) {
    std::vector<ProxyIndex> out{};
    for (const int& proxy : pool.bound(list)) {
        out.push_back(pool.index_of(proxy));
    }

    return out;
}

// TC-POOL-8.4.1.1-001: a list below its floor binds even when every
// unpromised proxy is borrowed; above its floor it does not; the cap holds.
bool tc_pool_8_4_1_1_001(std::string& err) {
    ProxyPoolStorage<int, 6> pool;
    CHECK(pool.reserve(2u), "two floors of 1 must fit a pool of 6");

    ProxyList busy = make_list(1u, 8u);
    ProxyList quiet = make_list(1u, 8u);

    // busy takes its floor and then borrows the 4 unpromised proxies.
    for (int i = 0; i < 5; ++i) {
        CHECK(pool.bind(busy) != kNoProxy, "busy must bind its floor and 4 borrowed proxies");
    }

    CHECK(busy.count == 5u, "busy must hold 5 proxies");
    CHECK(pool.free_count() == 1u && pool.reserved() == 1u,
          "the last free proxy must stay promised to quiet's floor");
    CHECK(pool.bind(busy) == kNoProxy, "busy must not take quiet's reserved proxy");
    CHECK(pool.bind(quiet) != kNoProxy, "quiet must bind within its floor");
    CHECK(pool.bind(quiet) == kNoProxy, "quiet above its floor must not bind from an empty pool");

    ProxyPoolStorage<int, 6> capped;
    ProxyList one = make_list(0u, 2u);
    CHECK(capped.bind(one) != kNoProxy && capped.bind(one) != kNoProxy, "cap 2 must bind twice");
    CHECK(capped.bind(one) == kNoProxy, "a list at its cap must not bind");
    CHECK(capped.free_count() == 4u, "a refused bind must take nothing");
    return true;
}

// TC-POOL-8.4.1.1-002: releasing below the floor promises the proxy back to
// that floor; releasing above it returns it to everyone.
bool tc_pool_8_4_1_1_002(std::string& err) {
    ProxyPoolStorage<int, 4> pool;
    CHECK(pool.reserve(2u), "two floors of 1 must fit a pool of 4");

    ProxyList a = make_list(1u, 4u);
    ProxyList b = make_list(1u, 4u);
    const ProxyIndex a0 = pool.bind(a);
    const ProxyIndex a1 = pool.bind(a);
    const ProxyIndex a2 = pool.bind(a);
    CHECK(a0 != kNoProxy && a1 != kNoProxy && a2 != kNoProxy, "a must bind 3");
    CHECK(pool.free_count() == 1u && pool.reserved() == 1u, "one proxy left, promised to b");

    CHECK(pool.release(a, a2), "a must release a borrowed proxy");
    CHECK(pool.reserved() == 1u && pool.free_count() == 2u, "a borrowed proxy returns unpromised");
    CHECK(pool.release(a, a1) && pool.release(a, a0), "a must release the rest");
    CHECK(a.count == 0u && pool.reserved() == 2u,
          "below its floor, a's last proxy is promised back to it");

    CHECK(pool.bind(b) != kNoProxy, "b must bind its floor");
    CHECK(pool.reserved() == 1u, "b's floor is spent");
    return true;
}

// TC-POOL-8.4.1.1-003: reserve() refuses more than the unpromised free proxies.
bool tc_pool_8_4_1_1_003(std::string& err) {
    ProxyPoolStorage<int, 5> pool;
    CHECK(pool.reserve(3u), "3 of 5 must reserve");
    CHECK(!pool.reserve(3u), "3 more of the 2 unpromised must not reserve");
    CHECK(pool.reserved() == 3u, "a refused reserve must change nothing");

    ProxyList borrower = make_list(0u, 5u);
    CHECK(pool.bind(borrower) != kNoProxy, "one unpromised proxy may be borrowed");
    CHECK(!pool.reserve(2u) && pool.reserve(1u), "only the one still unpromised may be reserved");
    pool.unreserve(4u);
    CHECK(pool.reserved() == 0u, "unreserving everything leaves nothing promised");
    return true;
}

// TC-POOL-8.4.1.1-004: a list iterates in bind order, and a list's proxies
// are disjoint from every other list's and from the free proxies.
bool tc_pool_8_4_1_1_004(std::string& err) {
    ProxyPoolStorage<int, 6> pool;
    ProxyList a = make_list(0u, 6u);
    ProxyList b = make_list(0u, 6u);
    const ProxyIndex a0 = pool.bind(a);
    const ProxyIndex b0 = pool.bind(b);
    const ProxyIndex a1 = pool.bind(a);
    const ProxyIndex a2 = pool.bind(a);

    CHECK((members(pool, a) == std::vector<ProxyIndex>{a0, a1, a2}), "a must list a0, a1, a2");
    CHECK((members(pool, b) == std::vector<ProxyIndex>{b0}), "b must list b0 alone");
    CHECK(a0 != b0 && a1 != b0 && a2 != b0, "lists must not share a proxy");

    CHECK(pool.release(a, a1), "a must release its middle proxy");
    CHECK((members(pool, a) == std::vector<ProxyIndex>{a0, a2}), "a must keep a0, a2 in order");
    CHECK(pool.bind(b) == a1, "the proxy freed last must be bound next");
    CHECK((members(pool, b) == std::vector<ProxyIndex>{b0, a1}), "b must append the new proxy");
    return true;
}

// TC-POOL-8.4.1.1-005: release() of a proxy not on the list changes nothing,
// and free >= reserved holds through a long run of binds and releases.
bool tc_pool_8_4_1_1_005(std::string& err) {
    ProxyPoolStorage<int, 8> pool;
    CHECK(pool.reserve(3u), "three floors of 1 must fit a pool of 8");

    ProxyList lists[3] = {make_list(1u, 5u), make_list(1u, 5u), make_list(1u, 5u)};
    const ProxyIndex other = pool.bind(lists[1]);
    CHECK(!pool.release(lists[0], other), "a must not release b's proxy");
    CHECK(lists[1].count == 1u && pool.free_count() == 7u,
          "the refused release must change nothing");

    // A fixed pseudo-random script of binds and releases across the lists.
    uint32_t state = 0x2545F491u;
    for (int step = 0; step < 2000; ++step) {
        state = state * 1664525u + 1013904223u;
        ProxyList& list = lists[(state >> 8) % 3u];
        if ((state >> 20) & 1u) {
            (void)pool.bind(list);
        } else if (list.head != kNoProxy) {
            CHECK(pool.release(list, list.head), "a list must release its own head");
        }

        uint16_t bound = 0u;
        uint16_t promised = 0u;
        for (const ProxyList& l : lists) {
            CHECK(l.count <= l.cap, "no list may exceed its cap");
            CHECK(members(pool, l).size() == l.count, "a list's count must match its links");
            bound = static_cast<uint16_t>(bound + l.count);
            promised =
                static_cast<uint16_t>(promised + (l.count < l.floor ? l.floor - l.count : 0u));
        }

        CHECK(pool.free_count() + bound == pool.capacity(), "every proxy is free or bound once");
        CHECK(pool.reserved() == promised, "reserved must equal the unused floors");
        CHECK(pool.free_count() >= pool.reserved(), "free >= reserved must hold");
    }

    return true;
}

}  // namespace

void append_proxy_pool_tests(std::vector<TestCase>& tests) {
    tests.push_back({"TC-POOL-8.4.1.1-001",
                     "§8.4.1.1/§8.4.7.1: a list below its floor binds even when every "
                     "unpromised proxy is borrowed; above it a list borrows only unpromised "
                     "proxies; no list exceeds its cap.",
                     tc_pool_8_4_1_1_001});
    tests.push_back({"TC-POOL-8.4.1.1-002",
                     "§8.4.1.4: a proxy released below its list's floor is promised back to "
                     "that floor; one released above it is free to any list.",
                     tc_pool_8_4_1_1_002});
    tests.push_back({"TC-POOL-8.4.1.1-003",
                     "reserve() refuses more proxies than are free and unpromised.",
                     tc_pool_8_4_1_1_003});
    tests.push_back({"TC-POOL-8.4.1.1-004",
                     "a list iterates its proxies in bind order and shares none with another "
                     "list; a freed proxy is the next one bound.",
                     tc_pool_8_4_1_1_004});
    tests.push_back({"TC-POOL-8.4.1.1-005",
                     "release() of a proxy not on the list changes nothing; free >= reserved "
                     "and reserved = unused floors hold through any sequence of binds and "
                     "releases.",
                     tc_pool_8_4_1_1_005});
}

}  // namespace spec_test
