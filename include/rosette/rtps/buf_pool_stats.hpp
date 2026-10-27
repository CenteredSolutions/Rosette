/**
 * @file rtps/buf_pool_stats.hpp
 * @brief Buffer pool usage statistics, the measurements a firmware sizes its pool classes from.
 *
 * Built only with RTPS_BUFPOOL_STATS (CMake: ROSETTE_RTPS_POOL_STATS=ON). The
 * pool records two views of every class:
 *
 * - Demand: the requests whose best fit is this class, that is, whose size is
 *   above the next smaller class and at most this one. Their count, the
 *   smallest, largest, and mean size asked for, a histogram of those sizes in
 *   kBufPoolStatsBins equal bins across the class's range, how many were
 *   served by a larger class because this one was empty (spilled up), and how
 *   many got no block at all (refused).
 * - Supply: the blocks of this class. How many allocations it served
 *   (including requests spilled up into it from smaller classes), how many
 *   are live now, and the most that were live at once.
 *
 * Demand tells where to put class boundaries: a histogram crowded at the top
 * of a class's range says the class below is just too small; one crowded at
 * the bottom says most of each block is wasted. Supply tells counts: a peak
 * well below the count means the class can shrink, and any refusal means it
 * is too small.
 *
 * Sizes are the octets requested, headroom and tailroom included, which is
 * what picks the class. The pool never sees how many octets a caller then
 * writes; a caller that requests worst-case blocks (a receive buffer) shows
 * as requests at that worst case.
 *
 * A refusal is also logged as a warning, with the size and class, on the
 * 1st, 2nd, 4th, 8th, ... refusal of its class, so an exhausted class is
 * visible without flooding the log.
 *
 * References: tools for the same question in other allocators: Linux
 * /proc/slabinfo (active and total objects per cache), Zephyr's
 * k_mem_slab_runtime_stats (used and max_used blocks).
 */

#pragma once

#include <stdint.h>

namespace rosette::rtps {

/// Bins in each class's request-size histogram.
inline constexpr uint8_t kBufPoolStatsBins = 8u;

/// @brief The statistics of one pool class (see the file comment for the two views).
struct BufPoolClassStats {
    // Demand: requests whose best fit is this class.
    uint32_t requests = 0;        ///< Requests, served or not.
    uint32_t request_octets = 0;  ///< Their sizes summed; the mean is this / requests.
    uint16_t request_min = 0;     ///< Smallest size requested; 0 before the first.
    uint16_t request_max = 0;     ///< Largest size requested.
    uint32_t spilled_up = 0;      ///< Served by a larger class: this one was empty.
    uint32_t refused = 0;         ///< Got no block: every eligible class was empty.
    /// Requests per size bin: bin i covers sizes above
    /// lower + i * width up to lower + (i + 1) * width (BufPoolStats::bin_width()).
    uint32_t request_bins[kBufPoolStatsBins] = {};

    // Supply: this class's blocks.
    uint32_t served = 0;      ///< Allocations served, including spilled-up requests.
    uint16_t live = 0;        ///< Blocks allocated now.
    uint16_t high_water = 0;  ///< Most blocks allocated at once since the last reset.
};

/**
 * @brief Records a MultiBufPool's per-class demand and supply.
 *
 * The pool calls on_request() for every alloc(), on_served() when a block
 * is handed out, and on_freed() when one returns to its free list, each
 * under the pool's lock. The class size table is the pool's own.
 *
 * Verification:
 * - Correct means: every request is counted once, in its best-fit class,
 *   whether served, spilled, or refused; live counts equal the blocks
 *   allocated; high_water never falls below live.
 * - Covered by: TC-BUF-STATS-001, TC-BUF-STATS-002.
 */
template <uint8_t N>
class BufPoolStats {
public:
    explicit constexpr BufPoolStats(const uint16_t (&class_sizes)[N]) noexcept
        : sizes_(class_sizes) {}

    /// @brief The class a request of @p size fits best, or N when none holds it.
    uint8_t best_fit(uint32_t size) const noexcept {
        uint8_t c = 0;
        while (c < N && sizes_[c] < size) {
            ++c;
        }

        return c;
    }

    /// @brief Lower end of class @p c's histogram range: the next smaller class's size.
    uint32_t bin_lower(uint8_t c) const noexcept { return c == 0u ? 0u : sizes_[c - 1u]; }

    /// @brief Width of one histogram bin of class @p c, in octets.
    uint32_t bin_width(uint8_t c) const noexcept {
        return (sizes_[c] - bin_lower(c) + kBufPoolStatsBins - 1u) / kBufPoolStatsBins;
    }

    /**
     * @brief Counts one request in its best-fit class's demand.
     *
     * @param[in] size     Octets requested, headroom included.
     * @param[in] class_of The class that served it, or N when none did.
     * @return true when the request was refused and this refusal should be
     *   logged: the 1st, 2nd, 4th, 8th, ... of its class.
     */
    bool on_request(uint32_t size, uint8_t class_of) noexcept {
        const uint8_t home = best_fit(size);
        if (home >= N) {
            return class_of >= N;
        }

        BufPoolClassStats& s = stats_[home];
        ++s.requests;
        s.request_octets += size;
        const uint16_t sz = static_cast<uint16_t>(size);
        if (s.request_min == 0u || sz < s.request_min) {
            s.request_min = sz;
        }

        if (sz > s.request_max) {
            s.request_max = sz;
        }

        const uint32_t above = size > bin_lower(home) ? size - bin_lower(home) - 1u : 0u;
        const uint32_t bin = above / bin_width(home);
        ++s.request_bins[bin < kBufPoolStatsBins ? bin : kBufPoolStatsBins - 1u];
        if (class_of >= N) {
            ++s.refused;
            return (s.refused & (s.refused - 1u)) == 0u;
        }

        if (class_of != home) {
            ++s.spilled_up;
        }

        return false;
    }

    /// @brief Counts a block of class @p c handed out.
    void on_served(uint8_t c) noexcept {
        BufPoolClassStats& s = stats_[c];
        ++s.served;
        ++s.live;
        if (s.live > s.high_water) {
            s.high_water = s.live;
        }
    }

    /// @brief Counts a block of class @p c returned to its free list.
    void on_freed(uint8_t c) noexcept {
        if (stats_[c].live > 0u) {
            --stats_[c].live;
        }
    }

    /// @brief The statistics of class @p c; @p c must be below N.
    const BufPoolClassStats& operator[](uint8_t c) const noexcept { return stats_[c]; }

    /// @brief Starts a new measurement window: clears every count, and sets
    ///   each high-water mark to the blocks live now.
    void reset() noexcept {
        for (BufPoolClassStats& s : stats_) {
            const uint16_t live = s.live;
            s = BufPoolClassStats{};
            s.live = live;
            s.high_water = live;
        }
    }

private:
    const uint16_t (&sizes_)[N];
    BufPoolClassStats stats_[N] = {};
};

/**
 * @brief Logs every pool class's statistics, one line for supply and demand
 *   and one for the size histogram, then optionally starts a new window.
 *
 * For a firmware to call periodically, or once after a representative
 * workload, while sizing its pool. The lines go through rtps_log_write() at
 * INFO level whatever RTPS_LOG_LEVEL is, since the caller asked for them;
 * classes with no requests and no blocks served are skipped. Does nothing
 * unless built with RTPS_BUFPOOL_STATS.
 *
 * @param[in] reset Clear the counts afterwards (BufPoolStats::reset()).
 *
 * Verification:
 * - Correct means: each logged figure is the one MultiBufPool::stats()
 *   returns for that class at the time of the call.
 * - Covered by: TC-BUF-STATS-003.
 *
 * References:
 * - docs/mcu_memory_model.md, "Pool pressure"
 *
 * Example:
 * @code{.cpp}
 * // In the firmware's 10 s status report, built with ROSETTE_RTPS_POOL_STATS=ON:
 * rosette::rtps::buf_pool_stats_log(true);
 * // [I][pool] 512 x8: served 412 (12 spilled in), live 2, peak 7 | ...
 * @endcode
 * Full program: tests/spec/src/test_buf_pool_stats.cpp
 */
void buf_pool_stats_log(bool reset) noexcept;

}  // namespace rosette::rtps
