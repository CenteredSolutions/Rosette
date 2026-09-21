/**
 * @file rmw_wait_set.cpp
 * @brief rmw_create_wait_set / rmw_wait — the executor's blocking entry point.
 *
 * rmw_wait() here evaluates readiness for subscriptions only. Guard
 * conditions, services, clients, and events are accepted (their arrays
 * may be non-null per the ABI) but are always reported not-ready:
 * services/clients have no `rmw_create_service()`/`rmw_create_client()`
 * implementation yet (rmw_service.cpp/rmw_client.cpp), and guard
 * conditions have no producer wired to them yet
 * (rmw_guard_condition.cpp). Extending readiness evaluation to those kinds
 * does not change this loop's shape — poll, check readiness, sleep,
 * repeat — only what it checks.
 */

#include "rmw_rosette/identifier.h"
#include "rmw_rosette/pool.hpp"
#include "rmw_rosette/types.hpp"
#include "rmw_rosette/wait_set.hpp"

#include "rosette/platform/host_runtime.hpp"

#include <rmw/rmw.h>
#include <rmw/error_handling.h>

#ifndef RMW_ROSETTE_MAX_WAIT_SETS
#  define RMW_ROSETTE_MAX_WAIT_SETS 4
#endif

namespace {

struct WaitSetSlot {
    rmw_wait_set_t ws{};
    rmw_rosette::WaitSet impl{};
};

rmw_rosette::Pool<WaitSetSlot, RMW_ROSETTE_MAX_WAIT_SETS>& wait_set_pool() {
    static rmw_rosette::Pool<WaitSetSlot, RMW_ROSETTE_MAX_WAIT_SETS> pool;
    return pool;
}

/// `rmw_wait_set_t` is always the first member of `WaitSetSlot`, so this
/// reinterpret is well-defined (standard-layout, same address as the
/// struct itself).
WaitSetSlot* slot_from(rmw_wait_set_t* ws) noexcept {
    return reinterpret_cast<WaitSetSlot*>(ws);
}

}  // namespace

extern "C" {

/**
 * @brief Creates a wait set bound to a context's participant/transport.
 *
 * @param[in] context Context whose participant/transport rmw_wait() will
 *   poll for this wait set.
 * @param[in] max_conditions Unused; wait_set_pool()'s capacity is fixed
 *   at compile time via `RMW_ROSETTE_MAX_WAIT_SETS` (pool.hpp).
 * @return The new wait set, or `nullptr` if `context`/its `impl` is null
 *   or the wait set pool is exhausted.
 */
rmw_wait_set_t* rmw_create_wait_set(rmw_context_t* context, size_t max_conditions) {
    (void)max_conditions;
    if (!context || !context->impl) {
        RMW_SET_ERROR_MSG("rmw_rosette: rmw_create_wait_set: null context");
        return nullptr;
    }

    WaitSetSlot* slot = wait_set_pool().acquire();
    if (!slot) {
        RMW_SET_ERROR_MSG("rmw_rosette: rmw_create_wait_set: wait set pool exhausted "
                          "(RMW_ROSETTE_MAX_WAIT_SETS)");
        return nullptr;
    }

    slot->impl.context = &context->impl->rosette_context;
    slot->ws.implementation_identifier = rmw_rosette_identifier;
    slot->ws.guard_conditions = nullptr;
    slot->ws.data = &slot->impl;
    return &slot->ws;
}

/**
 * @brief Destroys a wait set and returns its slot to the pool.
 *
 * @param[in] wait_set Wait set to destroy.
 * @return `RMW_RET_OK`, or `RMW_RET_INVALID_ARGUMENT` if `wait_set` is null.
 */
rmw_ret_t rmw_destroy_wait_set(rmw_wait_set_t* wait_set) {
    if (!wait_set) return RMW_RET_INVALID_ARGUMENT;
    wait_set_pool().release(slot_from(wait_set));
    return RMW_RET_OK;
}

/**
 * @brief Polls a participant's transport and readers until an entity in
 * the wait set is ready or the timeout elapses.
 *
 * The blocking primitive rclc_executor builds its spin loop on: it drives
 * `RTPSParticipant::tick()`/`SocketTransportRuntime::spin_some()` (the
 * only way this stack's RTPS state machine advances) between readiness
 * checks, since nothing else in the process does so otherwise.
 *
 * Per `rmw_wait()`'s contract (rmw/rmw.h), every entry in each input
 * array starts non-null and this function nulls out entries that are
 * *not* ready by the time it returns, leaving ready ones untouched.
 * Guard conditions, services, clients, and events are always reported
 * not-ready (see the file comment); only subscriptions are evaluated,
 * via each one's `Subscription::pending_count`.
 *
 * The deadline follows `wait_timeout`'s documented contract: `nullptr`
 * blocks indefinitely, `{0, 0}` checks once without sleeping, and any
 * other value bounds the wait to that duration. Between checks, the loop
 * sleeps `RMW_ROSETTE_WAIT_POLL_PERIOD_MS` via `rosette_platform_delay_ms()`.
 *
 * @param[in,out] subscriptions Subscriptions to wait on; entries not
 *   ready are set to `nullptr`. May be `nullptr`.
 * @param[in,out] guard_conditions Always fully nulled out (not evaluated).
 * @param[in,out] services Always fully nulled out (not evaluated).
 * @param[in,out] clients Always fully nulled out (not evaluated).
 * @param[in,out] events Always fully nulled out (not evaluated).
 * @param[in] wait_set Wait set providing the context to poll.
 * @param[in] wait_timeout Maximum time to wait; see above for `nullptr`/
 *   zero handling.
 * @return `RMW_RET_OK` if any entity became ready (or `wait_timeout` was
 *   `nullptr` and one eventually did); `RMW_RET_TIMEOUT` if the deadline
 *   passed with nothing ready; `RMW_RET_INVALID_ARGUMENT` if `wait_set`
 *   or its `data` is null.
 */
rmw_ret_t rmw_wait(rmw_subscriptions_t* subscriptions, rmw_guard_conditions_t* guard_conditions,
                   rmw_services_t* services, rmw_clients_t* clients, rmw_events_t* events,
                   rmw_wait_set_t* wait_set, const rmw_time_t* wait_timeout) {
    if (!wait_set || !wait_set->data) return RMW_RET_INVALID_ARGUMENT;

    auto* impl = static_cast<rmw_rosette::WaitSet*>(wait_set->data);
    rmw_rosette::Context& ctx = *impl->context;

    if (guard_conditions) {
        for (size_t i = 0; i < guard_conditions->guard_condition_count; ++i) {
            guard_conditions->guard_conditions[i] = nullptr;
        }
    }
    if (services) {
        for (size_t i = 0; i < services->service_count; ++i) services->services[i] = nullptr;
    }
    if (clients) {
        for (size_t i = 0; i < clients->client_count; ++i) clients->clients[i] = nullptr;
    }
    if (events) {
        for (size_t i = 0; i < events->event_count; ++i) events->events[i] = nullptr;
    }

    // rmw_wait()'s contract (rmw/rmw.h): NULL wait_timeout means block
    // indefinitely; {0,0} means check once and return immediately;
    // otherwise wait up to that long. Deadline math stays in plain
    // uint32_t milliseconds off rtps_platform_get_ms() — the same clock
    // tick()/parse_received() already use throughout rosette::rtps — not
    // std::chrono, which assumes a hosted C++ runtime this stack doesn't
    // require (see rosette/platform/host_runtime.hpp: the whole point of
    // that header is giving every target, bare-metal included, the same
    // three calls).
    uint32_t timeout_ms = 0;
    const bool has_deadline = (wait_timeout != nullptr);
    if (has_deadline) {
        timeout_ms = static_cast<uint32_t>(wait_timeout->sec * 1000u +
                                            wait_timeout->nsec / 1000000u);
    }
    const uint32_t start_ms = rtps_platform_get_ms();

    rtps::SocketTransportRuntime runtime(*ctx.transport);
    uint8_t rx_buffer[RTPS_BUF_BLOCK_SIZE];

    for (;;) {
        runtime.spin_some(rx_buffer, sizeof(rx_buffer), *ctx.participant);
        const uint32_t now_ms = rtps_platform_get_ms();
        ctx.participant->tick(now_ms);

        bool any_ready = false;
        if (subscriptions) {
            for (size_t i = 0; i < subscriptions->subscriber_count; ++i) {
                auto* sub_impl =
                    static_cast<rmw_rosette::Subscription*>(subscriptions->subscribers[i]);
                if (sub_impl->pending_count > 0) {
                    any_ready = true;
                } else {
                    subscriptions->subscribers[i] = nullptr;
                }
            }
        }

        if (any_ready) return RMW_RET_OK;
        // Unsigned subtraction wraps correctly across the ~49.7-day
        // rollover of rtps_platform_get_ms() as long as the actual
        // elapsed time never exceeds that span, same assumption the rest
        // of rosette::rtps already makes for its own deadlines.
        if (has_deadline && (now_ms - start_ms) >= timeout_ms) {
            return RMW_RET_TIMEOUT;  // Covers timeout_ms == 0 (check-once) too.
        }

        // A short, fixed poll granularity rather than a real blocking
        // wait — see wait_set.hpp's file comment for why this trade is
        // reasonable on an MCU with nothing else to block on.
        rosette_platform_delay_ms(RMW_ROSETTE_WAIT_POLL_PERIOD_MS);
    }
}

}  // extern "C"
