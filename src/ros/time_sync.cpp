/**
 * @file src/ros/time_sync.cpp
 * @brief ros/time_sync.hpp over platform::WallClock, the one clock every time source sets.
 *
 * The SNTP client (net/sntp_client.hpp, run by the board network) and the
 * micro-ROS agent's time exchange (`rmw_agent_link().sync_time`) both write
 * WallClock's offset, so the ROS view of time is the same on either backend
 * and survives sessions being torn down and rebuilt.
 */

#include "rosette/ros/time_sync.hpp"

#include "rosette/platform/wall_clock.hpp"

namespace rosette::ros {

bool time_synced() noexcept {
    return platform::WallClock::synchronized();
}

int64_t epoch_nanos() noexcept {
    int64_t unix_us = 0;
    return platform::WallClock::try_now_us(unix_us) ? unix_us * 1000 : 0;
}

int64_t epoch_millis() noexcept {
    int64_t unix_us = 0;
    return platform::WallClock::try_now_us(unix_us) ? unix_us / 1000 : 0;
}

Stamp time_stamp() noexcept {
    int64_t unix_us = 0;
    if (!platform::WallClock::try_now_us(unix_us) || unix_us < 0) {
        return Stamp{0, 0u};
    }

    // One __aeabi_uldivmod on a 32-bit core: it returns the quotient and the
    // remainder together, and the remainder fits 32 bits.
    const uint64_t us = static_cast<uint64_t>(unix_us);
    const uint32_t sub_us = static_cast<uint32_t>(us % 1000000u);
    return Stamp{static_cast<int32_t>(us / 1000000u), sub_us * 1000u};
}

}  // namespace rosette::ros
