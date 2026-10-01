/**
 * @file ports/chip/rp2/rp2_newlib_time.c
 * @brief newlib's clock_gettime() and _gettimeofday() on the RP2040 / RP2350 hardware timer.
 *
 * The Micro XRCE-DDS client's time source (uxr_millis / uxr_nanos) calls
 * clock_gettime(), which bare newlib does not implement. Both report the
 * microsecond timer that monotonic_us() reads, as time since boot; the
 * micro-ROS agent backend then aligns that base with the agent's wall clock
 * (rmw_uros_sync_session()). Rosette's own wall clock is WallClock.
 */

#include <stdint.h>
#include <sys/time.h>
#include <time.h>

#include "hardware/timer.h"

int clock_gettime(clockid_t clk_id, struct timespec* tp)
{
    (void)clk_id;
    if (tp == NULL) {
        return -1;
    }

    const uint64_t us = time_us_64();
    tp->tv_sec = (time_t)(us / 1000000ull);
    tp->tv_nsec = (long)((us % 1000000ull) * 1000ull);
    return 0;
}

int _gettimeofday(struct timeval* tv, void* tz)
{
    (void)tz;
    if (tv == NULL) {
        return -1;
    }

    const uint64_t us = time_us_64();
    tv->tv_sec = (time_t)(us / 1000000ull);
    tv->tv_usec = (suseconds_t)(us % 1000000ull);
    return 0;
}
