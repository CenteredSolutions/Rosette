/**
 * @file rtps/buf_pool_stats_log.cpp
 * @brief Logs the buffer pool's per-class usage statistics (rtps/buf_pool_stats.hpp).
 *
 * Two lines per class that saw any use:
 *
 *   pool 512 x8: served 412, live 2, peak 7 | asked 380 (300..508, mean 402), up 12, refused 0
 *   pool 512 sizes >256 by 32: 0 0 3 41 2 0 110 224
 *
 * The first is supply then demand; the second is the demand histogram, bin i
 * counting requests above 256 + 32 i octets and at most 256 + 32 (i + 1).
 */

#include "rosette/rtps/buf_pool.hpp"
#include "rosette/rtps/buf_pool_stats.hpp"

#include <stdio.h>

namespace rosette::rtps {

void buf_pool_stats_log([[maybe_unused]] bool reset) noexcept {
#if defined(RTPS_BUFPOOL_STATS) && RTPS_LOG_LEVEL > RTPS_LOG_LEVEL_NONE
    for (uint8_t c = 0; c < g_buf_pool.class_count(); ++c) {
        const MultiBufPool::ClassInfo info = g_buf_pool.class_info(c);
        const BufPoolClassStats       s = g_buf_pool.stats(c);
        if (s.requests == 0u && s.served == 0u) {
            continue;
        }

        const unsigned long mean =
            s.requests == 0u ? 0ul : static_cast<unsigned long>(s.request_octets / s.requests);
        rtps_log_write(RTPS_LOG_LEVEL_INFO, "pool",
                       "%u x%u: served %lu, live %u, peak %u | asked %lu (%u..%u, mean %lu), "
                       "up %lu, refused %lu",
                       info.block_size, info.total, static_cast<unsigned long>(s.served), s.live,
                       s.high_water, static_cast<unsigned long>(s.requests), s.request_min,
                       s.request_max, mean, static_cast<unsigned long>(s.spilled_up),
                       static_cast<unsigned long>(s.refused));

        char line[kBufPoolStatsBins * 11u + 1u] = {};
        int  used = 0;
        for (uint8_t b = 0; b < kBufPoolStatsBins; ++b) {
            used += snprintf(line + used, sizeof(line) - static_cast<size_t>(used), " %lu",
                             static_cast<unsigned long>(s.request_bins[b]));
        }

        rtps_log_write(RTPS_LOG_LEVEL_INFO, "pool", "%u sizes >%lu by %lu:%s", info.block_size,
                       static_cast<unsigned long>(g_buf_pool.stats_bin_lower(c)),
                       static_cast<unsigned long>(g_buf_pool.stats_bin_width(c)), line);
    }
#endif

#ifdef RTPS_BUFPOOL_STATS
    if (reset) {
        g_buf_pool.stats_reset();
    }
#endif
}

}  // namespace rosette::rtps
