/**
 * @file ports/net/wiznet/arp_cache.cpp
 * @brief The ArpCache state machine; see arp_cache.hpp.
 */

#include "ports/net/wiznet/arp_cache.hpp"

namespace rosette::net::wiznet {
namespace {

bool due(uint32_t now_ms, uint32_t at_ms) noexcept {
    return static_cast<int32_t>(now_ms - at_ms) >= 0;
}

/// Order in which due entries are queried: first contact, refresh, retry.
int urgency(ArpCache::State state) noexcept {
    switch (state) {
        case ArpCache::State::wanted:
            return 0;
        case ArpCache::State::resolved:
            return 1;
        default:
            return 2;
    }
}

}  // namespace

ArpCache::Lookup ArpCache::lookup(const Ipv4& ip, uint32_t now_ms, Mac& mac) noexcept {
    Entry* entry = find_mutable(ip);
    if (entry == nullptr) {
        entry = allocate();
        if (entry == nullptr) {
            return Lookup::miss;
        }

        *entry = Entry{};
        entry->ip = ip;
        entry->state = State::wanted;
        entry->due_ms = now_ms;
    }

    settle_expired(*entry, now_ms);
    entry->used = true;
    entry->last_use = ++use_counter_;
    switch (entry->state) {
        case State::stale:
            revalidate(*entry, now_ms);
            mac = entry->mac;
            return Lookup::hit;
        case State::resolved:
            mac = entry->mac;
            return Lookup::hit;
        case State::unreachable:
            return Lookup::unreachable;
        default:
            return Lookup::miss;
    }
}

bool ArpCache::next_query(uint32_t now_ms, Ipv4& ip) noexcept {
    Entry* best = nullptr;
    for (std::size_t i = 0; i < capacity_; ++i) {
        Entry& entry = entries_[i];
        settle_expired(entry, now_ms);
        if (entry.state == State::empty || entry.state == State::stale || entry.in_flight
            || !due(now_ms, entry.due_ms)) {
            continue;
        }

        // Nobody sent to it since its last query: a confirmed MAC is kept for
        // the next send to revalidate, anything else leaves the table.
        if (!entry.used) {
            if (entry.state == State::resolved) {
                entry.state = State::stale;
            } else {
                entry = Entry{};
            }

            continue;
        }

        const bool more_urgent = best == nullptr || urgency(entry.state) < urgency(best->state)
                                 || (urgency(entry.state) == urgency(best->state)
                                     && static_cast<int32_t>(entry.due_ms - best->due_ms) < 0);
        if (more_urgent) {
            best = &entry;
        }
    }

    if (best == nullptr) {
        return false;
    }

    best->in_flight = true;
    ip = best->ip;
    return true;
}

void ArpCache::answered(const Ipv4& ip, const Mac& mac, uint32_t now_ms) noexcept {
    Entry* entry = find_mutable(ip);
    if (entry == nullptr) {
        return;
    }

    complete(*entry);
    entry->state = State::resolved;
    entry->mac = mac;
    entry->failures = 0u;
    entry->due_ms = now_ms + policy_.refresh_ms;
    entry->expires_ms = now_ms + policy_.expire_ms;
}

void ArpCache::unanswered(const Ipv4& ip, uint32_t now_ms) noexcept {
    Entry* entry = find_mutable(ip);
    if (entry == nullptr) {
        return;
    }

    if (entry->failures < UINT8_MAX) {
        ++entry->failures;
    }

    entry->due_ms = now_ms + backoff_ms(entry->failures);
    settle_expired(*entry, now_ms);

    // A refresh retry needs no fresh use: the use that made the entry worth
    // refreshing stands until the MAC expires.
    if (entry->state == State::resolved) {
        entry->in_flight = false;
        return;
    }

    complete(*entry);
    entry->state = State::unreachable;
}

void ArpCache::heard_from(const Ipv4& ip, uint32_t now_ms) noexcept {
    Entry* entry = find_mutable(ip);
    if (entry == nullptr || entry->failures == 0u) {
        return;
    }

    entry->failures = 1u;
    const uint32_t soon = now_ms + policy_.retry_ms;
    if (static_cast<int32_t>(entry->due_ms - soon) > 0) {
        entry->due_ms = soon;
    }

    // The datagram is a use: without one, an entry nobody sends to would be
    // evicted at its retry instead of queried.
    entry->used = true;
}

void ArpCache::requeue(const Ipv4& ip) noexcept {
    Entry* entry = find_mutable(ip);
    if (entry != nullptr) {
        entry->in_flight = false;
    }
}

void ArpCache::clear() noexcept {
    for (std::size_t i = 0; i < capacity_; ++i) {
        entries_[i] = Entry{};
    }
}

const ArpCache::Entry* ArpCache::find(const Ipv4& ip) const noexcept {
    for (std::size_t i = 0; i < capacity_; ++i) {
        if (entries_[i].state != State::empty && entries_[i].ip == ip) {
            return &entries_[i];
        }
    }

    return nullptr;
}

std::size_t ArpCache::size() const noexcept {
    std::size_t count = 0u;
    for (std::size_t i = 0; i < capacity_; ++i) {
        if (entries_[i].state != State::empty) {
            ++count;
        }
    }

    return count;
}

ArpCache::Entry* ArpCache::find_mutable(const Ipv4& ip) noexcept {
    return const_cast<Entry*>(find(ip));
}

/// @brief An empty entry, else the least recently used entry not in flight, else null.
ArpCache::Entry* ArpCache::allocate() noexcept {
    Entry* victim = nullptr;
    for (std::size_t i = 0; i < capacity_; ++i) {
        Entry& entry = entries_[i];
        if (entry.state == State::empty) {
            return &entry;
        }

        const bool older =
            victim == nullptr || static_cast<int32_t>(entry.last_use - victim->last_use) < 0;
        if (!entry.in_flight && older) {
            victim = &entry;
        }
    }

    return victim;
}

/// @brief `retry_ms * 2^(failures - 1)`, capped at `max_retry_ms`.
uint32_t ArpCache::backoff_ms(uint8_t failures) const noexcept {
    uint32_t wait = policy_.retry_ms;
    for (uint8_t i = 1u; i < failures && wait < policy_.max_retry_ms; ++i) {
        wait *= 2u;
    }

    return wait < policy_.max_retry_ms ? wait : policy_.max_retry_ms;
}

/**
 * @brief Settles a `resolved` entry whose answer is `expire_ms` old.
 *
 * An unanswered query is evidence against the MAC, so an entry with
 * failures becomes `unreachable`. Without one, the refreshes were never
 * started (the socket-less engine stayed busy) and the MAC stands, so the
 * entry becomes `stale`, to be revalidated by the next send.
 *
 * @param[in,out] entry  The entry; unchanged unless `resolved` and expired.
 * @param[in]     now_ms Monotonic milliseconds.
 */
void ArpCache::settle_expired(Entry& entry, uint32_t now_ms) const noexcept {
    if (entry.state != State::resolved || !due(now_ms, entry.expires_ms)) {
        return;
    }

    entry.state = entry.failures == 0u ? State::stale : State::unreachable;
}

/**
 * @brief Returns a `stale` entry to `resolved`, due for a query now.
 *
 * The send that called for it goes out with the MAC meanwhile. The entry
 * gets `expire_ms - refresh_ms`, the window a failing refresh gets, to
 * answer before settle_expired() makes it `unreachable`.
 *
 * @param[in,out] entry  A `stale` entry.
 * @param[in]     now_ms Monotonic milliseconds.
 *
 * References:
 * - RFC 4861 §7.3.3 (a packet sent to a STALE entry starts reachability
 *   confirmation)
 */
void ArpCache::revalidate(Entry& entry, uint32_t now_ms) const noexcept {
    entry.state = State::resolved;
    entry.failures = 0u;
    entry.due_ms = now_ms;
    entry.expires_ms = now_ms + (policy_.expire_ms - policy_.refresh_ms);
}

/// @brief Ends the entry's query; the next one needs a fresh use.
void ArpCache::complete(Entry& entry) noexcept {
    entry.in_flight = false;
    entry.used = false;
}

}  // namespace rosette::net::wiznet
