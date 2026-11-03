// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

#pragma once

/**
 * @file ports/net/wiznet/arp_resolver.hpp
 * @brief Gives each IPv4 send on a WIZnet chip its destination MAC, so the chip never ARPs inline.
 *
 * A WIZnet SEND to a unicast address ARPs whenever the destination differs
 * from the last one on that socket, and an unanswered ARP holds the socket's
 * next send for the chip's retry time. `WiznetArpResolver` moves address
 * resolution off the send path:
 *
 *   1. route() picks the MAC for each IPv4 datagram. A multicast group gets
 *      its group MAC (RFC 1112 §6.4) and a broadcast gets ff:ff:ff:ff:ff:ff.
 *      A unicast destination is mapped to its next hop: the destination when
 *      it is on the board's subnet (or the mask is 0.0.0.0, as for an IPv4
 *      link-local address), else the gateway. The next hop is then looked
 *      up in the ArpCache.
 *   2. A next hop the cache does not know yet, or that did not answer, has
 *      its datagram dropped instead of sent with the chip's inline ARP. UDP is
 *      best effort, and RTPS reliability repairs the loss as it repairs any
 *      other, while a fallback to plain SEND would bring the stall back.
 *   3. The chip's socket-less ARP command (WiznetDevice::start_arp_probe())
 *      resolves next hops in the background, one at a time: a miss starts a
 *      probe at once when the engine is free, and poll() collects answers and
 *      starts refreshes and retries.
 *
 * IPv6 sends are left to the chip's neighbor discovery (Route::chip).
 *
 * Every member function except cache() may be called from any thread; one
 * mutex guards the cache, the probe in flight, and the drop count, and is
 * always taken before the bus lock.
 *
 * The common send repeats its socket's last destination, so each socket
 * caches the route of its last unicast hit (CachedRoute), as a Linux socket
 * caches its destination entry in `sk_dst_cache` and revalidates it with
 * dst_check() against a route generation. route() reuses the cached route,
 * without the mutex, a table scan, or a clock read, while the cache's
 * version() and the board's IPv4 generation are those it was cached under;
 * both are published as atomics at the end of every locked call. The point
 * is a send whose cost does not vary: a hit never waits behind poll(),
 * another socket's lookup, or a probe being started, and does the same
 * work however full the table is. A reused
 * hit marks its slot used, and poll() hands those marks to the cache, so
 * refresh and eviction see the entry as in use. Expiry is settled by poll(),
 * so a cached MAC is used at most one poll period past `expire_ms`.
 * heard_from() likewise skips the mutex unless some entry has failures, the
 * only case in which it acts.
 *
 *     ArpCacheStorage<8> table;
 *     WiznetArpResolver<Chip, Bus> arp{device, table};
 *     sockets.attach_arp_resolver(&arp);    // before the first socket opens
 *     for (;;) { arp.poll(platform::monotonic_ms()); ... }
 */

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

#include "rosette/platform/platform.hpp"

#include "arp_cache.hpp"
#include "device.hpp"

namespace rosette::net::wiznet {

/// @brief Socket-less ARP timing (SLRTR, SLRCR) for WiznetArpResolver.
struct ArpProbeTiming {
    /// Wait for a reply to each request. A LAN host answers in well under a millisecond.
    uint16_t timeout_ms = 100u;
    /// Requests after the first one.
    uint8_t retries = 1u;
};

template <typename Chip, typename Bus>
class WiznetArpResolver {
public:
    using device_type = WiznetDevice<Chip, Bus>;
    using Ipv4 = ArpCache::Ipv4;
    using Mac = ArpCache::Mac;

    /// @brief How a datagram leaves the chip; see route().
    enum class Route : uint8_t {
        chip,    ///< Let the chip resolve the address (IPv6).
        to_mac,  ///< Send to the MAC route() wrote, without ARP.
        drop,    ///< Do not send: the next hop is unresolved, unreachable, or unroutable.
    };

    /**
     * @brief A socket's last unicast hit, which route() reuses while it still holds.
     *
     * Owned by one socket and passed to every route() call it makes; a
     * default-constructed CachedRoute holds nothing. It stays valid only
     * while the cache's version() and the board's IPv4 generation match the
     * values stamped into it, so any change that could end the hit forces
     * the next route() back to the locked lookup. Only route() reads or writes
     * the fields.
     */
    struct CachedRoute {
        Ipv4     dst{};
        Mac      mac{};
        uint32_t cache_version = 0u;
        uint32_t config_generation = 0u;
        uint8_t  slot = 0u;
        bool     valid = false;
    };

    /**
     * @brief Binds the resolver to a chip and a table.
     *
     * @param[in] device The chip; must outlive the resolver.
     * @param[in] cache  The table of next hops; must outlive the resolver.
     * @param[in] timing ARP request timeout and retries.
     *
     * Example:
     * @code{.cpp}
     * static ArpCacheStorage<8> table;
     * static WiznetArpResolver<BoardChip, BoardBus> arp{device, table};
     * sockets.attach_arp_resolver(&arp);
     * auto socket = sockets.open(SocketDomain::ipv4);    // sends now use the table
     * arp.poll(rosette::platform::monotonic_ms());        // from the network's poll step
     * @endcode
     * Full program: boards/common/rp2_wiznet/wiznet_board_network.cpp
     */
    WiznetArpResolver(device_type& device, ArpCache& cache,
                      const ArpProbeTiming& timing = {}) noexcept
        : device_(device), cache_(cache), timing_(timing) {}

    WiznetArpResolver(const WiznetArpResolver&) = delete;
    WiznetArpResolver& operator=(const WiznetArpResolver&) = delete;

    /**
     * @brief Chooses the destination MAC for one datagram, or that it is dropped.
     *
     * Runs on every send. A unicast destination equal to `cached`'s, while
     * the cache's version and the board's IPv4 generation are those it was
     * cached under, reuses its MAC with two atomic loads and no lock.
     * Otherwise the mutex is taken and the next hop looked up; a hit is
     * cached in `cached`, and a miss starts a probe (a few register writes,
     * no waiting) when the engine is free.
     *
     * @param[in]     peer Destination of the datagram.
     * @param[out]    mac  The destination MAC, written on Route::to_mac.
     * @param[in,out] cached The sending socket's cached route; the socket's
     *   sends must not run concurrently with each other.
     * @return Route::chip for IPv6; Route::to_mac with `mac` for multicast,
     *   broadcast, and a resolved next hop; Route::drop otherwise, counted by
     *   drops().
     *
     * Verification:
     * - Correct means: a unicast datagram is sent only to the MAC its next hop
     *   answered with; an off-subnet destination uses the gateway's MAC; no
     *   datagram waits for ARP; a reused MAC is abandoned as soon as the
     *   entry is answered with another MAC, settles, leaves the table, or the
     *   board's address changes.
     * - Covered by: TC-NET-WIZ-ARP-010 through TC-NET-WIZ-ARP-013,
     *   TC-NET-WIZ-ARP-019 through TC-NET-WIZ-ARP-021.
     *
     * References:
     * - RFC 1122 §3.3.1.1 (local/remote decision), §2.3.2.2 (dropping while
     *   unresolved); RFC 1112 §6.4 (group MAC); RFC 919 (broadcast)
     * - Linux net/core/dst.c and include/net/sock.h: the per-socket
     *   destination cache (`sk_dst_cache`, __sk_dst_check()) this mirrors
     */
    Route route(const UdpEndpoint& peer, Mac& mac, CachedRoute& cached) noexcept {
        if (peer.ipv6) {
            return Route::chip;
        }

        const Ipv4 dst = peer.ipv4();
        if (device_type::is_multicast_ipv4(dst)) {
            mac = device_type::multicast_mac_from_ipv4(dst);
            return Route::to_mac;
        }

        if (cached.valid && cached.dst == dst
            && cached.cache_version == cache_version_.load(std::memory_order_acquire)
            && cached.config_generation == device_.ipv4_generation()) {
            route_used_[cached.slot].store(true, std::memory_order_relaxed);
            mac = cached.mac;
            return Route::to_mac;
        }

        cached.valid = false;
        rosette::platform::LockGuard guard(lock_);
        sync_config_locked();
        collect_route_uses_locked();
        if (is_broadcast(dst)) {
            mac = kBroadcastMac;
            return Route::to_mac;
        }

        Ipv4 hop{};
        std::size_t    slot = 0u;
        const uint32_t now_ms = rosette::platform::monotonic_ms();
        Route          route = Route::drop;
        if (next_hop(dst, hop) && cache_.lookup(hop, now_ms, mac, &slot) == ArpCache::Lookup::hit) {
            route = Route::to_mac;
            cached = CachedRoute{dst,
                                 mac,
                                 cache_.version(),
                                 generation_,
                                 static_cast<uint8_t>(slot),
                                 slot < kCachedRouteSlots};
        } else {
            ++drops_;
            service_locked(now_ms);
        }

        publish_locked();
        return route;
    }

    /**
     * @brief Notes a datagram received from `source` against the next hop it came through.
     *
     * The cache is keyed by next hop, so the sender is mapped the same way
     * route() maps a destination: an on-link sender is its own next hop, and
     * an off-link one arrived through the gateway. Hearing from either
     * shortens that hop's backoff (ArpCache::heard_from()), so a gateway
     * whose refresh failed is retried soon after traffic from beyond it
     * arrives.
     *
     * Runs on every receive, so while no entry has failures it returns after
     * one atomic load.
     *
     * @param[in] source Sender of a received datagram; IPv6 senders are ignored.
     *
     * Verification:
     * - Correct means: a datagram from an on-link host affects only that
     *   host's entry; one from off-link affects only the gateway's entry.
     * - Covered by: TC-NET-WIZ-ARP-007 (cache rule), TC-NET-WIZ-ARP-017 (hop mapping).
     *
     * References:
     * - RFC 1122 §3.3.1.1 (local/remote decision); RFC 1122 §2.3.2.1
     */
    void heard_from(const UdpEndpoint& source) noexcept {
        if (source.ipv6 || !failing_.load(std::memory_order_acquire)) {
            return;
        }

        rosette::platform::LockGuard guard(lock_);
        sync_config_locked();
        Ipv4 hop{};
        if (next_hop(source.ipv4(), hop)) {
            cache_.heard_from(hop, rosette::platform::monotonic_ms());
        }

        publish_locked();
    }

    /**
     * @brief Collects a finished ARP probe and starts the next one that is due.
     *
     * Called from the board network's periodic step. Costs one SLIR read
     * while a probe is in flight and nothing otherwise, unless a probe is due.
     * It also hands the uses of cached routes to the cache and settles
     * expired entries, which ends the routes cached for them; the settling
     * runs on every call, also while a probe is pending, so a cached MAC
     * outlives its entry's expiry by at most one poll period.
     *
     * @param[in] now_ms Monotonic milliseconds.
     *
     * Verification:
     * - Correct means: an answer becomes a cache hit, a timeout a negative
     *   entry; refreshes and retries start when due; a change of the board's
     *   IPv4 configuration empties the cache.
     * - Covered by: TC-NET-WIZ-ARP-010, TC-NET-WIZ-ARP-016, TC-NET-WIZ-ARP-020,
     *   TC-NET-WIZ-ARP-021.
     */
    void poll(uint32_t now_ms) noexcept {
        rosette::platform::LockGuard guard(lock_);
        sync_config_locked();
        collect_route_uses_locked();
        cache_.settle_expired(now_ms);
        service_locked(now_ms);
        publish_locked();
    }

    /// @brief Datagrams dropped because their next hop was unresolved, unreachable, or unroutable.
    uint32_t drops() const noexcept {
        rosette::platform::LockGuard guard(lock_);
        return drops_;
    }

    /// @brief The table; read it only while no other thread uses the resolver.
    const ArpCache& cache() const noexcept { return cache_; }

private:
    static constexpr Mac kBroadcastMac{0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu, 0xFFu};

    /// Cache slots whose hits a CachedRoute may hold; hits on later slots always take the mutex.
    static constexpr std::size_t kCachedRouteSlots = 32u;

    /**
     * @brief Makes the cache's version and failure state visible to the lock-free paths.
     *
     * Called at the end of every locked call that can change the cache, so
     * a CachedRoute recorded under the mutex matches the published version
     * until the cache next changes.
     */
    void publish_locked() noexcept {
        cache_version_.store(cache_.version(), std::memory_order_release);
        failing_.store(cache_.any_failures(), std::memory_order_release);
    }

    /// @brief Hands the slots that cached routes used since the last call to
    /// ArpCache::record_use().
    void collect_route_uses_locked() noexcept {
        const std::size_t slots =
            cache_.capacity() < kCachedRouteSlots ? cache_.capacity() : kCachedRouteSlots;
        for (std::size_t slot = 0; slot < slots; ++slot) {
            if (route_used_[slot].load(std::memory_order_relaxed)) {
                route_used_[slot].store(false, std::memory_order_relaxed);
                cache_.record_use(slot);
            }
        }
    }

    /**
     * @brief Empties the cache when configure_ipv4() has run since the last look.
     *
     * The new gateway is entered as wanted at once: every off-subnet
     * destination needs it, and resolving it before the first such send
     * spares one-shot exchanges (an SNTP request to a remote server) their
     * dropped first datagram.
     *
     * Every send and receive runs this check, so it reads the device's
     * generation without the bus lock and copies the configuration under
     * the lock only when that changed; a send never waits behind another
     * socket's datagram transfer to learn that the address is unchanged.
     *
     * Verification:
     * - Correct means: the first route() or heard_from() after
     *   configure_ipv4() sees the new address, mask, and gateway, and finds
     *   the cache empty but for the new gateway.
     * - Covered by: TC-NET-WIZ-ARP-011, TC-NET-WIZ-ARP-012,
     *   TC-NET-WIZ-ARP-016.
     *
     * References:
     * - RFC 1122 §3.3.1.1 (next-hop choice from the board's own address and mask)
     */
    void sync_config_locked() noexcept {
        if (device_.ipv4_generation() == generation_) {
            return;
        }

        generation_ = device_.ipv4_route(config_);
        cache_.clear();
        if (config_.gateway != Ipv4{}) {
            Mac unused{};
            (void)cache_.lookup(config_.gateway, rosette::platform::monotonic_ms(), unused);
        }
    }

    /// @brief Limited broadcast, or the directed broadcast of the board's own subnet.
    bool is_broadcast(const Ipv4& dst) const noexcept {
        bool limited = true;
        bool mask_set = false;
        bool directed = true;
        for (std::size_t i = 0; i < dst.size(); ++i) {
            const uint8_t mask = config_.subnet[i];
            limited = limited && dst[i] == 0xFFu;
            mask_set = mask_set || mask != 0u;
            directed =
                directed && (dst[i] & mask) == (config_.ip[i] & mask) && (dst[i] | mask) == 0xFFu;
        }

        return limited || (mask_set && directed);
    }

    /**
     * @brief The on-link destination itself, else the gateway.
     *
     * A gateway is configured when GAR is anything but 0.0.0.0; individual
     * zero octets (192.168.0.1) are ordinary.
     *
     * @return `false` when `dst` is off-link and no gateway is configured.
     */
    bool next_hop(const Ipv4& dst, Ipv4& hop) const noexcept {
        bool on_link = true;
        for (std::size_t i = 0; i < dst.size(); ++i) {
            on_link =
                on_link && (dst[i] & config_.subnet[i]) == (config_.ip[i] & config_.subnet[i]);
        }

        if (on_link) {
            hop = dst;
            return true;
        }

        hop = config_.gateway;
        return config_.gateway != Ipv4{};
    }

    /**
     * @brief Moves a finished probe into the cache, then starts the next due one.
     *
     * A probe the device reports `idle` while one was outstanding is counted
     * as unanswered, so its entry is retried rather than left in flight.
     */
    void service_locked(uint32_t now_ms) noexcept {
        if (probing_) {
            Mac mac{};
            const auto outcome = device_.poll_arp_probe(mac);
            if (outcome == device_type::ArpProbe::pending) {
                return;
            }

            probing_ = false;
            if (outcome == device_type::ArpProbe::answered) {
                cache_.answered(probe_ip_, mac, now_ms);
            } else {
                cache_.unanswered(probe_ip_, now_ms);
            }
        }

        Ipv4 ip{};
        if (!cache_.next_query(now_ms, ip)) {
            return;
        }

        if (!device_.start_arp_probe(ip, timing_.timeout_ms, timing_.retries)) {
            cache_.requeue(ip);
            return;
        }

        probing_ = true;
        probe_ip_ = ip;
    }

    device_type& device_;
    ArpCache& cache_;
    ArpProbeTiming timing_;
    mutable rosette::platform::Mutex lock_;
    typename device_type::Ipv4Config config_{};
    uint32_t generation_ = 0u;
    bool probing_ = false;
    Ipv4 probe_ip_{};
    uint32_t drops_ = 0u;
    std::atomic<uint32_t>                            cache_version_{0u};
    std::atomic<bool>                                failing_{false};
    std::array<std::atomic<bool>, kCachedRouteSlots> route_used_{};
};

}  // namespace rosette::net::wiznet
