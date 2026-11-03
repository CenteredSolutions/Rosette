// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file rmw_rosette/wait_set.hpp
 * @brief rmw_wait_set_t backing store.
 *
 * Executors call `rmw_wait()` with a mix of subscriptions, services,
 * clients, guard conditions, and events and expect it to return as soon as
 * any is ready, or on timeout. The wait itself is rosette::rtps::ParticipantDriver's
 * spin_until(): it runs the protocol and sleeps between passes on the
 * platform Event and the transport's sockets, so a wait set carries only the
 * context whose driver to use.
 */

#pragma once

#include "types.hpp"

#include <cstdint>

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
