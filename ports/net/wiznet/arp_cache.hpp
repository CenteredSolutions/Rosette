#pragma once

/**
 * @file ports/net/wiznet/arp_cache.hpp
 * @brief A small IPv4-to-MAC table that lets a WIZnet chip send without ARP.
 *
 * A WIZnet socket remembers one destination MAC (Sn_DHAR), so a socket that
 * alternates between peers makes the chip ARP before nearly every SEND, and a
 * peer that does not answer stalls the socket's next send for the chip's whole
 * retry time (about 1.8 s at reset values). `ArpCache` keeps the answers for
 * the handful of next hops a board talks to (the host, an operator console, the other
 * participants), so WiznetArpResolver can hand each send its MAC directly
 * (SEND_MAC on the W5100S, Sn_MR2.DHAM on the W6100 / W6300).
 *
 * The table is a pure state machine: it never touches the chip and takes the
 * time as an argument, so it is tested on its own. Each entry is in one of
 * five states:
 *
 *   - `wanted`: a send missed and was dropped; the entry waits for its first query.
 *   - `resolved`: the MAC is known; sends use it until `expire_ms` after the
 *     last answer. From `refresh_ms` after that answer, the entry is queried
 *     again in the background while sends keep using the MAC (RFC 1122
 *     §2.3.2.1). An unanswered refresh is retried with exponential backoff
 *     from `retry_ms`, so the window between `refresh_ms` and `expire_ms`
 *     rides out a peer that misses several requests in a row (a Wi-Fi host
 *     in power save). Any answer restarts both deadlines; reaching
 *     `expire_ms` without one makes the entry `unreachable` when a query went
 *     unanswered, and `stale` when none did.
 *   - `stale`: the MAC was answered once and nothing has contradicted it since,
 *     but nothing has confirmed it lately either. The entry costs no traffic.
 *     The next send uses the MAC and makes the entry `resolved` again, due for
 *     a query at once, with `expire_ms - refresh_ms` (the window a failing
 *     refresh gets) to answer before it turns `unreachable`. This is the
 *     STALE state of IPv6 Neighbor Unreachability Detection (RFC 4861
 *     §7.3.2), which Linux applies to ARP as well: time alone is no evidence
 *     that a host moved, so a peer the board talks to rarely (an SNTP server
 *     polled every 17 minutes) is reached on the first send instead of losing
 *     that datagram to a fresh query.
 *   - `unreachable`: the next hop did not answer; sends are dropped at no cost,
 *     and the query is retried with exponential backoff from `retry_ms` up to
 *     `max_retry_ms` (a negative cache). Hearing from the host (heard_from())
 *     resets the backoff, so a peer that reboots is reached again within
 *     `retry_ms`.
 *   - `empty`.
 *
 * An entry is queried again only if a lookup used it since its last query
 * finished, or, for a `resolved` entry, since its last answer. Otherwise,
 * when it falls due, a `resolved` entry turns `stale` and any other entry is
 * evicted, so the background traffic is bounded by the peers the board is
 * actually sending to. A full table evicts its least recently used entry that
 * has no query in flight, which is usually a `stale` one.
 *
 * version() counts the changes that can end a hit: an entry settling,
 * leaving the table, or taking a new MAC. A caller that caches a hit
 * (WiznetArpResolver's per-socket CachedRoute) reuses it while the version
 * is unchanged and reports the uses with record_use(), so the cached
 * entry still counts as used for refresh and eviction.
 *
 * Access is not synchronized; WiznetArpResolver serializes it.
 *
 *     ArpCacheStorage<8> table;
 *     std::array<uint8_t, 6> mac{};
 *     if (table.lookup(ip, now_ms, mac) == ArpCache::Lookup::hit) { send_with(mac); }
 */

#include <array>
#include <cstddef>
#include <cstdint>

namespace rosette::net::wiznet {

/// @brief Timing of ArpCache refreshes and retries, in milliseconds.
struct ArpCachePolicy {
    /// Age of an answer at which an in-use entry is queried again.
    uint32_t refresh_ms = 60000u;
    /// Age of an answer at which its MAC is no longer used; greater than `refresh_ms`.
    uint32_t expire_ms = 120000u;
    /// Wait after the first unanswered query; doubles per failure.
    uint32_t retry_ms = 1000u;
    /// Ceiling of the retry backoff.
    uint32_t max_retry_ms = 32000u;
};

class ArpCache {
public:
    using Ipv4 = std::array<uint8_t, 4>;
    using Mac = std::array<uint8_t, 6>;

    enum class State : uint8_t { empty, wanted, resolved, stale, unreachable };

    /// @brief One next hop. Storage only; ArpCache owns every field.
    struct Entry {
        Ipv4 ip{};
        Mac mac{};
        State state = State::empty;
        bool in_flight = false;    ///< Handed out by next_query() and not yet completed.
        bool used = false;         ///< Looked up since its last query completed.
        uint8_t failures = 0u;     ///< Consecutive unanswered queries.
        uint32_t due_ms = 0u;      ///< When the entry is next queried (or, if unused, made
                                   ///< `stale` or evicted); not used while `stale`.
        uint32_t expires_ms = 0u;  ///< When a `resolved` MAC stops being confirmed.
        uint32_t last_use = 0u;    ///< Lookup counter at the last use, for LRU eviction.
    };

    /// @brief What a send to a next hop does.
    enum class Lookup : uint8_t {
        hit,          ///< MAC known; send with it. A `stale` MAC also starts its revalidation.
        miss,         ///< Not known yet; drop the datagram, a query is pending.
        unreachable,  ///< Did not answer; drop the datagram.
    };

    /**
     * @brief Binds the table to caller-owned entries.
     *
     * @param[in] entries  `capacity` entries, all `empty`; must outlive the table.
     * @param[in] capacity Number of next hops the table holds, at least 1.
     * @param[in] policy   Refresh and retry timing.
     */
    ArpCache(Entry* entries, std::size_t capacity, const ArpCachePolicy& policy = {}) noexcept
        : entries_(entries), capacity_(capacity), policy_(policy) {}

    ArpCache(const ArpCache&) = delete;
    ArpCache& operator=(const ArpCache&) = delete;

    /**
     * @brief Looks up the MAC for a next hop and records the use.
     *
     * Called on every send. A next hop that is not in the table is added as
     * `wanted`, due at once, taking an empty entry or else the least recently
     * used one that has no query in flight; when every entry is in flight the
     * miss is not recorded and the next send tries again. A `resolved` entry
     * whose `expire_ms` has passed is settled here first: `unreachable` if a
     * query went unanswered, `stale` if none did. A `stale` entry is a hit
     * that also becomes `resolved`, due for a query now, with
     * `expire_ms - refresh_ms` to answer.
     *
     * @param[in]  ip     Next-hop IPv4 address, network byte order.
     * @param[in]  now_ms Monotonic milliseconds.
     * @param[out] mac    The MAC, written only on `hit`.
     * @param[out] slot   When not null, the entry's index, written only on
     *   `hit`; record_use() takes it.
     * @return `hit`, `miss`, or `unreachable`; see Lookup.
     *
     * Verification:
     * - Correct means: a send uses only a MAC that some query answered and
     *   no unanswered query has outlived; a send to a `stale` entry is a hit
     *   that schedules its revalidation; a new next hop on a full table
     *   displaces the least recently used entry, never one in flight.
     * - Covered by: TC-NET-WIZ-ARP-001, TC-NET-WIZ-ARP-006, TC-NET-WIZ-ARP-008,
     *   TC-NET-WIZ-ARP-009.
     *
     * References:
     * - RFC 826 (ARP); RFC 1122 §2.3.2.1 (cache, flooding limits);
     *   RFC 4861 §7.3.2-7.3.3 (STALE, revalidation on use)
     */
    Lookup lookup(const Ipv4& ip, uint32_t now_ms, Mac& mac, std::size_t* slot = nullptr) noexcept;

    /**
     * @brief Counts a send that reused an earlier hit on entry `slot` as a use.
     *
     * The entry becomes the most recently used and is refreshed when due, as
     * if lookup() had been called for it. A slot that is empty is left alone.
     *
     * Uses are reported in batches, so recency within one batch is the order
     * of the calls, not of the sends. This is the use-bit approximation of
     * LRU that CLOCK replacement makes: it can misorder only next hops that
     * were all used since the last batch, and keeping exact order would put
     * an atomic read-modify-write on every send.
     *
     * @param[in] slot Index lookup() wrote on the hit; below capacity().
     *
     * Verification:
     * - Correct means: an entry used only through cached hits is not the
     *   one a new next hop displaces.
     * - Covered by: TC-NET-WIZ-ARP-020.
     *
     * References:
     * - F. J. Corbató, "A Paging Experiment with the Multics System", MIT
     *   Project MAC Report MAC-M-384, 1968 (CLOCK, use-bit approximation of LRU)
     */
    void record_use(std::size_t slot) noexcept;

    /**
     * @brief Settles every `resolved` entry whose answer is `expire_ms` old.
     *
     * lookup(), next_query(), and unanswered() settle the entries they look
     * at, but a caller that reuses hits without lookup() needs expiry
     * settled on a schedule of its own, including while a query is in flight
     * and next_query() is not called. An entry with failed queries becomes
     * `unreachable` and one without becomes `stale`, each advancing
     * version().
     *
     * @param[in] now_ms Monotonic milliseconds.
     *
     * Verification:
     * - Correct means: after the call no `resolved` entry is past
     *   `expire_ms`, and version() has advanced if any entry settled.
     * - Covered by: TC-NET-WIZ-ARP-021.
     *
     * References:
     * - RFC 1122 §2.3.2.1 (cache timeout of an unconfirmed entry)
     */
    void settle_expired(uint32_t now_ms) noexcept;

    /**
     * @brief Changes so far that can end a hit; a cached hit holds while this is unchanged.
     *
     * Advances when an entry settles to `stale` or `unreachable`, leaves the
     * table, is answered, or the table is cleared. A `resolved` entry passing
     * `expire_ms` advances it when settle_expired(), lookup(), next_query(),
     * or unanswered() next looks at that entry.
     */
    uint32_t version() const noexcept { return version_; }

    /// @brief True when some entry has unanswered queries, the only case heard_from() acts on.
    bool any_failures() const noexcept;

    /**
     * @brief Chooses the next hop the chip should ARP for now, if any.
     *
     * Called when the chip's socket-less engine is free. Due entries are
     * chosen in order of urgency: `wanted` (a peer is waiting for first
     * contact), then `resolved` refreshes, then `unreachable` retries.
     * Earlier-due entries go first within each group, and `stale` entries are
     * never chosen. While scanning, a `resolved` entry past `expire_ms` is
     * settled as in lookup(), and a due entry that no lookup used since its
     * last query is not queried: a `resolved` one turns `stale`, any other is
     * evicted.
     *
     * @param[in]  now_ms Monotonic milliseconds.
     * @param[out] ip     The next hop to query, written when the call returns true.
     * @return `true` with the entry marked in flight; `false` when nothing is due.
     *
     * Verification:
     * - Correct means: an entry is in flight at most once; unused entries
     *   cost no query at their due time, a `resolved` one keeping its MAC as
     *   `stale` and any other leaving the table.
     * - Covered by: TC-NET-WIZ-ARP-003, TC-NET-WIZ-ARP-004.
     */
    bool next_query(uint32_t now_ms, Ipv4& ip) noexcept;

    /**
     * @brief Records the chip's answer for a query from next_query().
     *
     * The entry becomes `resolved` with `mac`, which may differ from the MAC
     * it held (a host that changed NICs or DHCP leases), is due for refresh
     * `refresh_ms` later, and expires `expire_ms` later. An answer for a next hop that left the
     * table meanwhile (clear(), eviction) is ignored.
     *
     * @param[in] ip     The next hop queried.
     * @param[in] mac    Sender hardware address of the ARP reply.
     * @param[in] now_ms Monotonic milliseconds.
     */
    void answered(const Ipv4& ip, const Mac& mac, uint32_t now_ms) noexcept;

    /**
     * @brief Records that a query from next_query() went unanswered.
     *
     * A `resolved` entry keeps its MAC and is queried again after
     * `retry_ms * 2^(failures - 1)`, capped at `max_retry_ms`, for as long as
     * its answer is younger than `expire_ms`; it stays a candidate for those
     * retries without a further use. Past `expire_ms`, and for a `wanted` or
     * `stale` entry at its first failure, the entry becomes `unreachable` and
     * keeps retrying on the same backoff, which continues from the failures
     * already counted.
     *
     * @param[in] ip     The next hop queried.
     * @param[in] now_ms Monotonic milliseconds.
     *
     * Verification:
     * - Correct means: a dead peer costs one query per backoff interval and
     *   nothing per send; a peer that answers any query before `expire_ms`
     *   never loses its MAC.
     * - Covered by: TC-NET-WIZ-ARP-002, TC-NET-WIZ-ARP-005.
     *
     * References:
     * - RFC 1122 §2.3.2.1 (cache validation, at most one request per second
     *   per destination)
     */
    void unanswered(const Ipv4& ip, uint32_t now_ms) noexcept;

    /**
     * @brief Notes a datagram received from `ip`, evidence that the host is up.
     *
     * An entry with unanswered queries behind it, `unreachable` or a
     * `resolved` entry whose refresh has failed, restarts its backoff and
     * falls due no later than `retry_ms` from now. A peer that rebooted or
     * woke from Wi-Fi power save is thus reached again promptly, while a host
     * that sends without answering ARP is still queried at most once per
     * `retry_ms`. A datagram does not prove the MAC, so the expiry stands.
     *
     * @param[in] ip     Source address of the datagram.
     * @param[in] now_ms Monotonic milliseconds.
     *
     * Verification:
     * - Correct means: after heard_from(), an entry with failures is queried
     *   within `retry_ms`; other entries are unchanged.
     * - Covered by: TC-NET-WIZ-ARP-007.
     */
    void heard_from(const Ipv4& ip, uint32_t now_ms) noexcept;

    /**
     * @brief Puts back a query from next_query() that could not be started.
     *
     * Starting fails only when the chip's socket-less engine is busy, which
     * says nothing about the next hop, so its state, MAC, and failure count
     * are unchanged. The entry stays due, and next_query() hands it out again
     * once the engine is free.
     *
     * @param[in] ip The next hop next_query() returned.
     */
    void requeue(const Ipv4& ip) noexcept;

    /// @brief Empties the table, as when the board's own address or subnet changes.
    void clear() noexcept;

    /// @brief The entry for `ip`, or null; for diagnostics and tests.
    const Entry* find(const Ipv4& ip) const noexcept;

    /// @brief Number of non-empty entries.
    std::size_t size() const noexcept;

    /// @brief Number of next hops the table holds.
    std::size_t capacity() const noexcept { return capacity_; }

private:
    Entry* find_mutable(const Ipv4& ip) noexcept;
    Entry* allocate() noexcept;
    uint32_t backoff_ms(uint8_t failures) const noexcept;
    void     settle_expired(Entry& entry, uint32_t now_ms) noexcept;
    void revalidate(Entry& entry, uint32_t now_ms) const noexcept;
    void complete(Entry& entry) noexcept;

    Entry* entries_;
    std::size_t capacity_;
    ArpCachePolicy policy_;
    uint32_t use_counter_ = 0u;
    uint32_t       version_ = 0u;
};

/// @brief The entries an ArpCacheStorage owns, as a base so they exist before its ArpCache.
template <std::size_t N>
struct ArpCacheEntries {
    std::array<ArpCache::Entry, N> arp_entries{};
};

/**
 * @brief An ArpCache with room for `N` next hops.
 */
template <std::size_t N>
class ArpCacheStorage : private ArpCacheEntries<N>, public ArpCache {
    static_assert(N > 0u, "an ArpCache needs at least one entry");

public:
    explicit ArpCacheStorage(const ArpCachePolicy& policy = {}) noexcept
        : ArpCacheEntries<N>{}, ArpCache(this->arp_entries.data(), N, policy) {}
};

}  // namespace rosette::net::wiznet
