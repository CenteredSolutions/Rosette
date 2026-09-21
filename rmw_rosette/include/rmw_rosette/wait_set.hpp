/**
 * @file rmw_rosette/wait_set.hpp
 * @brief rmw_wait_set_t backing store.
 *
 * rclc_executor calls `rmw_wait()` with a bounded timeout across an
 * arbitrary mix of subscriptions/services/clients/guard-conditions/events
 * and expects it to return as soon as any one of them is ready, or on
 * timeout. rosette::rtps has no blocking I/O model to hang this off of —
 * everything is driven by `RTPSParticipant::tick(now_ms)` plus
 * `parse_received()` fed from a socket recv loop.
 *
 * `rmw_wait()` (rmw_wait_set.cpp) bridges the two by polling: tick, drain
 * available socket reads, check each entity's readiness, repeat until
 * something is ready or the deadline passes. This burns CPU proportional
 * to poll granularity rather than blocking efficiently, a reasonable
 * trade on an MCU with nothing else to do, and the same pattern rclc's
 * own reference applications already use.
 */

#pragma once

#include "types.hpp"

#include <cstdint>

#ifndef RMW_ROSETTE_WAIT_POLL_PERIOD_MS
#  define RMW_ROSETTE_WAIT_POLL_PERIOD_MS 5u
#endif

namespace rmw_rosette {

/// Backing store for one rmw_wait_set_t. rmw_create_wait_set() allocates
/// one per rclc_executor; rmw_wait() is called repeatedly against it with
/// the same set of subscriptions/guard_conditions/services/clients/events
/// rcl passes in each time (rmw_wait mutates its input arrays in place to
/// report which are ready — see rmw/rmw.h). The context reference is all
/// a wait set needs to carry between calls: the entity arrays themselves
/// arrive fresh as rmw_wait() parameters on every call.
struct WaitSet {
    Context* context = nullptr;
};

}  // namespace rmw_rosette
