/**
 * @file rtps/proxy_pool.hpp
 * @brief A fixed pool of matched-endpoint proxies shared by a participant's endpoints.
 *
 * Why a pool
 * ----------
 * A writer keeps one ReaderProxy per matched reader (§8.4.1.1) and a reader
 * one WriterProxy per matched writer (§8.4.7.1). Sizing that storage inside
 * every endpoint reserves the most matches any endpoint could have, in every
 * endpoint at once, while a firmware's endpoints differ widely: a service
 * with one occasional client next to a status topic read by a dozen nodes.
 * A pool sized for the matches the participant has at once spends RAM only
 * on those, and lets the busy endpoint borrow what the quiet ones leave.
 *
 * Structure
 * ---------
 * The proxies are one array, sized at build time; a proxy never moves while
 * it is bound, so pointers held into it stay valid. Each proxy has a 16-bit
 * link. A bound proxy's link chains it into its endpoint's ProxyList (the
 * endpoint keeps the head, so iterating its matches touches only its own
 * proxies); a free proxy's link chains it into the pool's free list. A proxy
 * is on exactly one of those lists, so one link per proxy serves both, and
 * taking a free proxy is O(1).
 *
 *   ProxyPool                    links, free list, admission (proxy_pool.cpp)
 *   └── TypedProxyPool<Proxy>    typed access to the array and an endpoint's proxies
 *       └── ProxyPoolStorage<Proxy, N>   owns the N proxies and N links
 *
 * Admission
 * ---------
 * Every list has a floor (matches it is always granted) and a cap (matches
 * it may never exceed). The pool keeps `reserved`, the free proxies promised
 * to floors not yet used, and binds a proxy to a list when
 *
 *   count < cap   and   (count < floor  or  free > reserved)
 *
 * so a list below its floor always succeeds and a list at or above it only
 * borrows proxies nobody is promised. The pool's owner reserves each
 * endpoint's floor (reserve()) before the endpoint binds; the invariant
 * `free >= reserved` then holds after every bind and release. This is the
 * shared-buffer admission of Choudhury and Hahne: a guaranteed share per
 * queue and a bounded claim on the rest.
 *
 * Spec references (all OMG formal/2022-04-01)
 * -------------------------------------------
 *   ReaderProxy                  — §8.4.1.1
 *   matched_reader_add / remove  — §8.4.1.4
 *   WriterProxy                  — §8.4.7.1
 *   matched_writer_add / remove  — §8.4.7.4
 *
 * Design: docs/design/shared_proxy_pools.md.
 */

#pragma once

#include <stdint.h>

namespace rosette::rtps {

// ============================================================
// Configuration defaults
// ============================================================

/// Matches every user endpoint is always granted, reserved in its pool for
/// every endpoint slot whether or not an endpoint is created in it.
#ifndef RTPS_MIN_MATCHES_PER_ENDPOINT
#  define RTPS_MIN_MATCHES_PER_ENDPOINT 1
#endif

/// Index of a proxy in its pool.
using ProxyIndex = uint16_t;

/// The end of a list, and the link of no proxy.
constexpr ProxyIndex kNoProxy = 0xFFFFu;

/**
 * @brief One endpoint's bound proxies: the head of its list and its admission bounds.
 *
 * The endpoint owns the list; the pool links and unlinks its proxies and
 * keeps `count`. `floor` and `cap` are set once when the endpoint is
 * created, with floor <= cap.
 */
struct ProxyList {
    ProxyIndex head = kNoProxy;  ///< First bound proxy, in bind order.
    uint8_t count = 0u;          ///< Proxies bound.
    uint8_t floor = 0u;          ///< Matches always granted.
    uint8_t cap = 0u;            ///< Most matches at once.
};

/**
 * @brief The free list and admission of a proxy pool, by index.
 *
 * Holds no proxies: TypedProxyPool adds typed access to the array, and
 * ProxyPoolStorage owns it. Every operation is compiled once, in
 * proxy_pool.cpp, for pools of every proxy type and size.
 *
 * Thread safety: not thread-safe; the caller holds the participant lock
 * (ParticipantDriver::mutex()) for every call.
 */
class ProxyPool {
public:
    ProxyPool(const ProxyPool&) = delete;
    ProxyPool& operator=(const ProxyPool&) = delete;

    /**
     * @brief Promises `n` more free proxies to floors not yet used.
     *
     * The pool's owner calls this for each endpoint's floor before the
     * endpoint binds: for every slot when the pool is created, and for the
     * part of an endpoint's floor above the slot's when it is created.
     *
     * @param[in] n Proxies to set aside.
     * @return `false`, with nothing changed, when fewer than `n` free
     *   proxies are unpromised (free − reserved < n).
     *
     * Verification:
     * - Correct means: free >= reserved holds after the call.
     * - Covered by: TC-POOL-8.4.1.1-003.
     */
    bool reserve(uint16_t n) noexcept;

    /**
     * @brief Withdraws a promise reserve() made.
     *
     * @param[in] n Proxies no longer set aside; at most reserved().
     */
    void unreserve(uint16_t n) noexcept;

    /**
     * @brief True when bind() would bind a proxy to `list` now.
     *
     * @param[in] list The endpoint's list.
     * @return `true` when the list is below its cap and either below its
     *   floor or a free proxy exists beyond those reserved for floors.
     */
    bool may_bind(const ProxyList& list) const noexcept;

    /**
     * @brief Takes a free proxy and appends it to `list`.
     *
     * Appending keeps the list in bind order, so an endpoint serves its
     * readers or writers in the order they matched. Finding the tail walks
     * the list, which the duplicate check of matched_reader_add() or
     * matched_writer_add() has just walked too.
     *
     * @param[in,out] list The endpoint's list; its count is incremented.
     * @return The bound proxy's index, or kNoProxy when may_bind() is
     *   false; the list is then unchanged. The caller initializes the proxy.
     *
     * Verification:
     * - Correct means: a list below its floor always binds; above it, a list
     *   binds only while free > reserved; no list exceeds its cap; and
     *   free >= reserved holds afterwards.
     * - Covered by: TC-POOL-8.4.1.1-001, TC-POOL-8.4.1.1-002, TC-POOL-8.4.1.1-004.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.4.1.4 (matched_reader_add), §8.4.7.4 (matched_writer_add)
     */
    ProxyIndex bind(ProxyList& list) noexcept;

    /**
     * @brief Unlinks a proxy from `list` and returns it to the free list.
     *
     * The caller clears the proxy first. Below the list's floor the freed
     * proxy is promised to that floor again, so the endpoint can always
     * rebind it.
     *
     * @param[in,out] list  The endpoint's list.
     * @param[in]     index A proxy bound to `list`.
     * @return `false`, with nothing changed, when `index` is not on `list`.
     *
     * Verification:
     * - Correct means: the proxy is free, the list's other proxies keep their
     *   order, and free >= reserved holds afterwards.
     * - Covered by: TC-POOL-8.4.1.1-002, TC-POOL-8.4.1.1-005.
     *
     * References:
     * - OMG DDSI-RTPS 2.5 §8.4.1.4 (matched_reader_remove), §8.4.7.4 (matched_writer_remove)
     */
    bool release(ProxyList& list, ProxyIndex index) noexcept;

    /// @brief The proxy after `index` on its list, or kNoProxy.
    ProxyIndex next(ProxyIndex index) const noexcept { return links_[index]; }

    /// @brief Proxies in the pool.
    uint16_t capacity() const noexcept { return capacity_; }

    /// @brief Proxies bound to no list.
    uint16_t free_count() const noexcept { return free_count_; }

    /// @brief Free proxies promised to floors not yet used.
    uint16_t reserved() const noexcept { return reserved_; }

protected:
    /**
     * @brief Creates a pool with every proxy free and nothing reserved.
     *
     * @param[in] links    One link per proxy, owned by the derived storage,
     *   which outlives the pool.
     * @param[in] capacity Proxies in the pool, 1 to 65,534.
     */
    ProxyPool(ProxyIndex* links, uint16_t capacity) noexcept;

    ~ProxyPool() noexcept = default;

private:
    ProxyIndex* links_;
    uint16_t capacity_;
    ProxyIndex free_head_;
    uint16_t free_count_;
    uint16_t reserved_ = 0u;
};

/**
 * @brief A ProxyPool with typed access to its proxies.
 *
 * Endpoints hold one of these: StatefulWriter a TypedProxyPool<ReaderProxy>,
 * StatefulReader a TypedProxyPool<WriterProxy>.
 *
 * @tparam Proxy ReaderProxy or WriterProxy.
 */
template <class Proxy>
class TypedProxyPool : public ProxyPool {
public:
    /// @brief The proxy at `index`, which is less than capacity().
    Proxy& operator[](ProxyIndex index) noexcept { return proxies_[index]; }

    const Proxy& operator[](ProxyIndex index) const noexcept { return proxies_[index]; }

    /// @brief The index of `proxy`, which belongs to this pool.
    ProxyIndex index_of(const Proxy& proxy) const noexcept {
        return static_cast<ProxyIndex>(&proxy - proxies_);
    }

    /// @brief Iterates the proxies of one list, in bind order.
    template <class P>
    class Iterator {
    public:
        Iterator(P* proxies, const ProxyPool* pool, ProxyIndex index) noexcept
            : proxies_(proxies), pool_(pool), index_(index) {}

        P& operator*() const noexcept { return proxies_[index_]; }

        Iterator& operator++() noexcept {
            index_ = pool_->next(index_);
            return *this;
        }

        bool operator!=(const Iterator& other) const noexcept { return index_ != other.index_; }

    private:
        P* proxies_;
        const ProxyPool* pool_;
        ProxyIndex index_;
    };

    /// @brief The proxies bound to one list, for a range-for.
    template <class P>
    struct Range {
        P* proxies;
        const ProxyPool* pool;
        ProxyIndex head;

        Iterator<P> begin() const noexcept { return {proxies, pool, head}; }

        Iterator<P> end() const noexcept { return {proxies, pool, kNoProxy}; }
    };

    /**
     * @brief The proxies bound to `list`, in bind order.
     *
     * Releasing a proxy relinks it into the free list, so a loop must not
     * advance past a proxy it has released; the endpoints release only
     * outside these loops.
     */
    Range<Proxy> bound(const ProxyList& list) noexcept { return {proxies_, this, list.head}; }

    Range<const Proxy> bound(const ProxyList& list) const noexcept {
        return {proxies_, this, list.head};
    }

protected:
    /**
     * @brief Creates a pool over storage owned by the derived class.
     *
     * @param[in] proxies  `capacity` proxies, all unmatched.
     * @param[in] links    `capacity` links.
     * @param[in] capacity Proxies in the pool.
     */
    TypedProxyPool(Proxy* proxies, ProxyIndex* links, uint16_t capacity) noexcept
        : ProxyPool(links, capacity), proxies_(proxies) {}

    ~TypedProxyPool() noexcept = default;

private:
    Proxy* proxies_;
};

namespace detail {
/// @brief The storage of a ProxyPoolStorage, as a base class so it is
/// constructed before the pool that links it.
template <class Proxy, uint16_t N>
struct ProxyPoolSlots {
    Proxy proxies[N];
    ProxyIndex links[N];
};
}  // namespace detail

/**
 * @brief A TypedProxyPool that owns its N proxies.
 *
 * A participant holds one for its user writers' ReaderProxies and one for
 * its user readers' WriterProxies; StatefulWriterStorage and
 * BoundedStatefulReader each hold a private one for an endpoint whose
 * capacity is its own, such as an SEDP builtin endpoint.
 *
 * @tparam Proxy ReaderProxy or WriterProxy.
 * @tparam N     Proxies in the pool, 1 to 65,534.
 *
 * Example:
 * @code{.cpp}
 * rosette::rtps::ProxyPoolStorage<rosette::rtps::ReaderProxy, 16> pool;
 * pool.reserve(4);                       // one floor for each of 4 writers
 * rosette::rtps::ProxyList list{rosette::rtps::kNoProxy, 0, 1, 8};
 * const rosette::rtps::ProxyIndex i = pool.bind(list);
 * if (i != rosette::rtps::kNoProxy) {
 *     pool[i].init(reader_guid, locators, 1, false, first_sn, true);
 *     // ... later, on unmatch:
 *     pool[i].clear();
 *     pool.release(list, i);
 * }
 * @endcode
 * Full program: tests/spec/src/test_proxy_pool_admission.cpp
 */
template <class Proxy, uint16_t N>
class ProxyPoolStorage : private detail::ProxyPoolSlots<Proxy, N>, public TypedProxyPool<Proxy> {
    static_assert(N >= 1u && N < kNoProxy, "a pool holds 1 to 65,534 proxies");

public:
    ProxyPoolStorage() noexcept : TypedProxyPool<Proxy>(this->proxies, this->links, N) {}
};

}  // namespace rosette::rtps
