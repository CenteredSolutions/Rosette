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
    return platform::WallClock::synchronized() ? platform::WallClock::now_us() * 1000 : 0;
}

}  // namespace rosette::ros
