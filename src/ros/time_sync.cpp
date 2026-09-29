/**
 * @file src/ros/time_sync.cpp
 * @brief ros/time_sync.hpp over platform::WallClock, the one clock every time source sets.
 *
 * Time sources write WallClock's offset (on the agent backend, the agent's
 * time exchange, `rmw_agent_link().sync_time`), so the ROS view of time is
 * the same on either backend and survives sessions being torn down and
 * rebuilt.
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
