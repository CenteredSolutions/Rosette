/**
 * @file rmw_rosette/wait_set.hpp
 * @brief rmw_wait_set_t backing store and the tick-driven rmw_wait() loop.
 *
 * rclc_executor calls rmw_wait() with a bounded timeout across an
 * arbitrary mix of subscriptions/services/clients/guard-conditions/events
 * and expects it to return as soon as any one of them is ready, or on
 * timeout. rosette::rtps has no blocking I/O model to hang this off of —
 * everything is driven by RTPSParticipant::tick(now_ms) plus
 * parse_received() fed from a socket recv loop.
 *
 * That polling shape actually maps cleanly onto rmw_wait(): busy-poll
 * (tick + drain-available socket reads + check each entity's "ready"
 * flag) in a short sleep loop until something's ready or the deadline
 * passes. It burns CPU proportional to poll granularity rather than
 * blocking efficiently, which is a fine trade on an MCU with nothing else
 * to do, and matches how rclc's own reference apps already spin.
 *
 * None of the actual polling logic is written yet — this is the shape it
 * needs to fit into.
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
/// report which are ready — see rmw/rmw.h).
struct WaitSet {
    Context* context = nullptr;

    // TODO: nothing to store yet beyond the context reference — rmw_wait()
    // receives the actual entity arrays as parameters on every call. This
    // struct exists mainly so rmw_create_wait_set() has something to
    // allocate and rmw_destroy_wait_set() something to free.
};

/// One iteration of the poll loop rmw_wait() drives:
///   1. context->participant->tick(now_ms())
///   2. drain the transport's recv() into participant->parse_received()
///      until it would block
///   3. for each subscription/service/client in the wait set, check
///      whether its rtps::StatefulReader has a change available (see the
///      TODO on Subscription in types.hpp about buffering
///      OnChangeAvailable callbacks) and null out entries that aren't
///      ready, per rmw_wait()'s contract
///   4. for each guard_condition, check + clear its `triggered` flag
/// Returns true if anything was found ready.
inline bool poll_once(Context& /*context*/) noexcept {
    // TODO
    return false;
}

}  // namespace rmw_rosette
