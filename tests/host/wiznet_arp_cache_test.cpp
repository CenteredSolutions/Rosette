/**
 * @file wiznet_arp_cache_test.cpp
 * @brief The ArpCache state machine: first contact, negative caching with
 *        backoff, background refresh, expiry, stale entries and their
 *        revalidation, LRU replacement.
 *
 * The cache never touches a chip, so each case drives it with explicit
 * timestamps and plays the chip's part by calling answered() / unanswered()
 * for the address next_query() hands out.
 *
 * Build and run: `cmake -S tests/host -B build/tests-host && ctest --test-dir build/tests-host`.
 */

#include "test_util.hpp"

#include "ports/net/wiznet/arp_cache.hpp"

#include <array>
#include <cstdint>

namespace {

namespace wiz = rosette::net::wiznet;
using Cache = wiz::ArpCache;
using Lookup = Cache::Lookup;

const Cache::Ipv4 kHost{192, 168, 1, 10};
const Cache::Ipv4 kConsole{192, 168, 1, 11};
const Cache::Mac kHostMac{0x02, 0x00, 0x00, 0x00, 0x00, 0x10};
const Cache::Mac kNewHostMac{0x02, 0x00, 0x00, 0x00, 0x00, 0x99};

Lookup lookup(Cache& cache, const Cache::Ipv4& ip, uint32_t now, Cache::Mac* out = nullptr) {
    Cache::Mac mac{};
    const Lookup result = cache.lookup(ip, now, mac);
    if (out != nullptr) {
        *out = mac;
    }

    return result;
}

/// @brief next_query() hands out `ip` at `now`.
bool queries(Cache& cache, const Cache::Ipv4& ip, uint32_t now) {
    Cache::Ipv4 got{};
    return cache.next_query(now, got) && got == ip;
}

bool idle(Cache& cache, uint32_t now) {
    Cache::Ipv4 got{};
    return !cache.next_query(now, got);
}

// TC-NET-WIZ-ARP-001: a miss is dropped and queried at once; the answer is a hit.
void first_contact() {
    wiz::ArpCacheStorage<4> cache;
    check(lookup(cache, kHost, 0u) == Lookup::miss, "ARP-001: unknown next hop misses");
    check(queries(cache, kHost, 0u), "ARP-001: the miss is queried at once");
    check(idle(cache, 0u), "ARP-001: an entry in flight is not handed out twice");
    check(lookup(cache, kHost, 1u) == Lookup::miss, "ARP-001: still a miss while in flight");

    cache.answered(kHost, kHostMac, 5u);
    Cache::Mac mac{};
    check(lookup(cache, kHost, 6u, &mac) == Lookup::hit && mac == kHostMac,
          "ARP-001: the answer is a hit with the reply's MAC");
}

// TC-NET-WIZ-ARP-002: no answer makes the entry unreachable, retried with capped backoff.
void negative_backoff() {
    wiz::ArpCachePolicy policy{};
    policy.retry_ms = 1000u;
    policy.max_retry_ms = 4000u;
    wiz::ArpCacheStorage<4> cache(policy);
    (void)lookup(cache, kHost, 0u);
    (void)queries(cache, kHost, 0u);
    cache.unanswered(kHost, 100u);
    check(lookup(cache, kHost, 200u) == Lookup::unreachable, "ARP-002: unanswered is unreachable");
    check(idle(cache, 1099u), "ARP-002: no retry before retry_ms");
    check(queries(cache, kHost, 1100u), "ARP-002: first retry after retry_ms");

    cache.unanswered(kHost, 1100u);
    (void)lookup(cache, kHost, 1200u);
    check(idle(cache, 3099u) && queries(cache, kHost, 3100u),
          "ARP-002: second retry after 2 x retry_ms");

    cache.unanswered(kHost, 3100u);
    (void)lookup(cache, kHost, 3200u);
    (void)queries(cache, kHost, 7100u);
    cache.unanswered(kHost, 7100u);
    (void)lookup(cache, kHost, 7200u);
    check(idle(cache, 11099u) && queries(cache, kHost, 11100u),
          "ARP-002: backoff is capped at max_retry_ms");
}

// TC-NET-WIZ-ARP-003: an entry in use is refreshed in the background and follows a new MAC.
void refresh() {
    wiz::ArpCachePolicy policy{};
    policy.refresh_ms = 60000u;
    wiz::ArpCacheStorage<4> cache(policy);
    (void)lookup(cache, kHost, 0u);
    (void)queries(cache, kHost, 0u);
    cache.answered(kHost, kHostMac, 0u);
    (void)lookup(cache, kHost, 30000u);
    check(idle(cache, 59999u), "ARP-003: no refresh before refresh_ms");
    check(queries(cache, kHost, 60000u), "ARP-003: an entry in use is refreshed at refresh_ms");

    Cache::Mac mac{};
    check(lookup(cache, kHost, 60001u, &mac) == Lookup::hit && mac == kHostMac,
          "ARP-003: sends keep the MAC while the refresh is in flight");
    cache.answered(kHost, kNewHostMac, 60002u);
    check(lookup(cache, kHost, 60003u, &mac) == Lookup::hit && mac == kNewHostMac,
          "ARP-003: a refresh replaces a changed MAC");
}

// TC-NET-WIZ-ARP-004: an entry nobody sent to since its last answer turns stale at its due
// time, keeps its MAC, and costs no traffic; an unused negative entry leaves the table.
void idle_entries() {
    wiz::ArpCacheStorage<4> cache;
    (void)lookup(cache, kHost, 0u);
    (void)queries(cache, kHost, 0u);
    cache.answered(kHost, kHostMac, 0u);
    check(idle(cache, 60000u), "ARP-004: an unused entry is not refreshed");

    const Cache::Entry* entry = cache.find(kHost);
    check(entry != nullptr && entry->state == Cache::State::stale && entry->mac == kHostMac,
          "ARP-004: an unused resolved entry turns stale and keeps its MAC");
    check(idle(cache, 3600000u), "ARP-004: a stale entry is never queried on its own");

    (void)lookup(cache, kConsole, 0u);
    (void)queries(cache, kConsole, 0u);
    cache.unanswered(kConsole, 0u);
    check(idle(cache, 1000u) && cache.find(kConsole) == nullptr,
          "ARP-004: an unused unreachable entry is evicted when due");
}

// TC-NET-WIZ-ARP-005: unanswered refreshes back off and keep the MAC until expire_ms.
void refresh_failures() {
    wiz::ArpCachePolicy policy{};
    policy.refresh_ms = 60000u;
    policy.expire_ms = 120000u;
    policy.retry_ms = 1000u;
    policy.max_retry_ms = 32000u;
    wiz::ArpCacheStorage<4> cache(policy);
    (void)lookup(cache, kHost, 0u);
    (void)queries(cache, kHost, 0u);
    cache.answered(kHost, kHostMac, 0u);
    (void)lookup(cache, kHost, 30000u);  // the only use before the refresh

    // Refresh at 60 s, then retries 1, 2, 4, 8, 16 s apart; none needs a new use.
    const uint32_t attempts[] = {60000u, 61000u, 63000u, 67000u, 75000u, 91000u};
    for (uint32_t at : attempts) {
        check(idle(cache, at - 1u) && queries(cache, kHost, at),
              "ARP-005: an unanswered refresh is retried on the backoff");
        cache.unanswered(kHost, at);
    }

    (void)lookup(cache, kHost, 100000u);
    Cache::Mac mac{};
    check(lookup(cache, kHost, 119999u, &mac) == Lookup::hit && mac == kHostMac,
          "ARP-005: the MAC is used until expire_ms");
    check(lookup(cache, kHost, 120000u) == Lookup::unreachable,
          "ARP-005: the MAC expires at expire_ms without an answer");

    check(queries(cache, kHost, 123000u), "ARP-005: an expired entry keeps retrying");
    cache.answered(kHost, kHostMac, 123000u);
    check(lookup(cache, kHost, 123001u) == Lookup::hit, "ARP-005: a late answer restores the MAC");
    check(lookup(cache, kHost, 242999u) == Lookup::hit, "ARP-005: an answer restarts the expiry");
}

// TC-NET-WIZ-ARP-008: a MAC whose refreshes could never start turns stale at expire_ms, not
// unreachable: nothing contradicted it, so the next send still uses it and revalidates.
void expiry_without_queries() {
    wiz::ArpCachePolicy policy{};
    policy.refresh_ms = 60000u;
    policy.expire_ms = 90000u;
    policy.retry_ms = 1000u;
    wiz::ArpCacheStorage<4> cache(policy);
    (void)lookup(cache, kHost, 0u);
    (void)queries(cache, kHost, 0u);
    cache.answered(kHost, kHostMac, 0u);

    // The engine is busy at every refresh: each query is put back unstarted.
    for (uint32_t now = 60000u; now < 90000u; now += 10000u) {
        (void)lookup(cache, kHost, now);
        check(queries(cache, kHost, now), "ARP-008: the refresh is handed out");
        cache.requeue(kHost);
    }

    check(lookup(cache, kHost, 89999u) == Lookup::hit, "ARP-008: requeue keeps the MAC");
    Cache::Mac mac{};
    check(lookup(cache, kHost, 90000u, &mac) == Lookup::hit && mac == kHostMac,
          "ARP-008: past expire_ms with no unanswered query, the MAC is still used");
    check(queries(cache, kHost, 90000u), "ARP-008: that send revalidates the entry at once");
}

// TC-NET-WIZ-ARP-009: a send to a stale entry goes out with its MAC and revalidates it;
// an answer confirms it, and an unanswered revalidation ends it after expire_ms - refresh_ms.
void stale_revalidation() {
    wiz::ArpCachePolicy policy{};
    policy.refresh_ms = 60000u;
    policy.expire_ms = 120000u;
    policy.retry_ms = 1000u;
    policy.max_retry_ms = 32000u;
    wiz::ArpCacheStorage<4> cache(policy);

    // An SNTP server polled every 1024 s, with no other traffic to it in between.
    (void)lookup(cache, kHost, 0u);
    (void)queries(cache, kHost, 0u);
    cache.answered(kHost, kHostMac, 0u);
    check(idle(cache, 60000u), "ARP-009: the server's entry goes stale unused");

    Cache::Mac mac{};
    check(lookup(cache, kHost, 1024000u, &mac) == Lookup::hit && mac == kHostMac,
          "ARP-009: the next poll is sent with the stale MAC, not dropped");
    check(queries(cache, kHost, 1024000u), "ARP-009: that send queries the entry at once");
    cache.answered(kHost, kNewHostMac, 1024001u);
    check(lookup(cache, kHost, 1024002u, &mac) == Lookup::hit && mac == kNewHostMac,
          "ARP-009: the revalidation's answer replaces the MAC");

    // The peer is gone: the revalidation goes unanswered on the refresh backoff.
    (void)lookup(cache, kConsole, 0u);
    (void)queries(cache, kConsole, 0u);
    cache.answered(kConsole, kHostMac, 0u);
    (void)idle(cache, 60000u);
    check(lookup(cache, kConsole, 2000000u) == Lookup::hit,
          "ARP-009: a stale entry is a hit before its revalidation answers");
    const uint32_t attempts[] = {2000000u, 2001000u, 2003000u, 2007000u, 2015000u, 2031000u};
    for (uint32_t at : attempts) {
        check(queries(cache, kConsole, at), "ARP-009: the revalidation is retried on the backoff");
        cache.unanswered(kConsole, at);
    }

    check(lookup(cache, kConsole, 2059999u) == Lookup::hit,
          "ARP-009: the MAC is used for expire_ms - refresh_ms while unanswered");
    check(lookup(cache, kConsole, 2060000u) == Lookup::unreachable,
          "ARP-009: an unanswered revalidation makes the entry unreachable");

    wiz::ArpCacheStorage<1> single(policy);
    (void)lookup(single, kHost, 0u);
    (void)queries(single, kHost, 0u);
    single.answered(kHost, kHostMac, 0u);
    (void)idle(single, 60000u);
    check(lookup(single, kConsole, 60001u) == Lookup::miss && single.find(kHost) == nullptr,
          "ARP-009: a full table replaces a stale entry");
}

// TC-NET-WIZ-ARP-006: a full table replaces its least recently used entry, never one in flight.
void lru_replacement() {
    wiz::ArpCacheStorage<2> cache;
    const Cache::Ipv4 kThird{192, 168, 1, 12};
    (void)lookup(cache, kHost, 0u);
    (void)lookup(cache, kConsole, 1u);
    (void)lookup(cache, kHost, 2u);  // kConsole is now the least recently used
    (void)lookup(cache, kThird, 3u);
    check(cache.find(kConsole) == nullptr && cache.find(kHost) != nullptr
              && cache.find(kThird) != nullptr,
          "ARP-006: the least recently used entry is replaced");

    wiz::ArpCacheStorage<1> single;
    (void)lookup(single, kHost, 0u);
    (void)queries(single, kHost, 0u);
    check(lookup(single, kConsole, 1u) == Lookup::miss && single.find(kHost) != nullptr,
          "ARP-006: an entry in flight is never replaced");
}

// TC-NET-WIZ-ARP-007: hearing from a host with failed queries brings its retry forward.
void heard_from() {
    wiz::ArpCachePolicy policy{};
    policy.retry_ms = 1000u;
    policy.max_retry_ms = 32000u;
    wiz::ArpCacheStorage<4> cache(policy);
    (void)lookup(cache, kHost, 0u);
    uint32_t now = 0u;
    for (int i = 0; i < 5; ++i) {  // backoff grows to 16 s
        (void)lookup(cache, kHost, now);
        (void)queries(cache, kHost, now);
        cache.unanswered(kHost, now);
        now += 40000u;
    }

    now -= 40000u;
    cache.heard_from(kHost, now + 10u);
    check(queries(cache, kHost, now + 1010u),
          "ARP-007: retried within retry_ms of hearing from it");

    wiz::ArpCacheStorage<4> refreshing(policy);
    (void)lookup(refreshing, kConsole, 0u);
    (void)queries(refreshing, kConsole, 0u);
    refreshing.answered(kConsole, kHostMac, 0u);
    refreshing.heard_from(kConsole, 10u);
    check(idle(refreshing, 1010u), "ARP-007: a healthy resolved entry is not queried early");

    now = 60000u;
    for (int i = 0; i < 4; ++i) {  // refresh backoff grows to 8 s
        (void)lookup(refreshing, kConsole, now);
        (void)queries(refreshing, kConsole, now);
        refreshing.unanswered(kConsole, now);
        now += 10000u;
    }

    now -= 10000u;
    refreshing.heard_from(kConsole, now + 10u);
    check(queries(refreshing, kConsole, now + 1010u),
          "ARP-007: a failing refresh is retried within retry_ms of hearing from it");
}

}  // namespace

int main() {
    first_contact();
    negative_backoff();
    refresh();
    idle_entries();
    refresh_failures();
    expiry_without_queries();
    stale_revalidation();
    lru_replacement();
    heard_from();
    return test_summary();
}
