/**
 * @file rmw_wait_set.cpp
 * @brief rmw_create_wait_set / rmw_wait — the executor's blocking entry point.
 *
 * rmw_wait() evaluates readiness for subscriptions, services, clients, and
 * guard conditions: an entity is ready when its ChangeRing holds a change
 * (a sample, request, or correlated reply) or its guard condition is
 * triggered. QoS events are accepted, as the ABI requires, and always
 * reported not-ready, since event initialization is unsupported
 * (rmw_qos_event.cpp).
 */

#include "rmw_rosette/argument_checks.hpp"
#include "rmw_rosette/identifier.h"
#include "rmw_rosette/pool.hpp"
#include "rmw_rosette/types.hpp"
#include "rmw_rosette/wait_set.hpp"


#include <rmw/rmw.h>
#include <rmw/error_handling.h>

#ifndef RMW_ROSETTE_MAX_WAIT_SETS
#  define RMW_ROSETTE_MAX_WAIT_SETS 4
#endif

namespace {

/**
 * @brief Refuses one rmw_wait() condition array that holds a null entry.
 *
 * @param[in] count   The array's entry count.
 * @param[in] entries The array; may be null only when `count` is 0.
 * @param[in] name    The argument's name for the error message.
 * @return RMW_RET_OK when every entry is non-null; otherwise RMW_RET_INVALID_ARGUMENT
 *   with the error state set.
 */
rmw_ret_t check_wait_entries(size_t count, void* const* entries, const char* name) noexcept {
    if (count != 0u && entries == nullptr) {
        return rmw_rosette::refuse_argument("rmw_wait", name, rmw_rosette::ArgumentFault::null);
    }

    for (size_t i = 0; i < count; ++i) {
        if (entries[i] == nullptr) {
            return rmw_rosette::refuse_argument("rmw_wait", name, rmw_rosette::ArgumentFault::null);
        }
    }

    return RMW_RET_OK;
}

/**
 * @brief Refuses rmw_wait() condition arrays that hold a null entry.
 *
 * rmw_wait() treats a null entry as invalid before the wait and nulls the
 * entries that did not trigger after it, so a null entry on the way in is a
 * caller reusing an output array; it is refused before the participant runs,
 * where it would otherwise count as not ready and could leave the wait to
 * block until its timeout.
 *
 * @param[in] subscriptions    Subscriptions to wait on; may be null.
 * @param[in] guard_conditions Guard conditions to wait on; may be null.
 * @param[in] services         Services to wait on; may be null.
 * @param[in] clients          Clients to wait on; may be null.
 * @param[in] events           Events to wait on; may be null.
 * @return RMW_RET_OK when every supplied array is fully populated; otherwise
 *   RMW_RET_INVALID_ARGUMENT with the error state naming the array.
 *
 * Verification:
 * - Correct means: a null entry in any array is refused without waiting and
 *   leaves every array as given.
 * - Covered by: rmw_rosette/test/test_live_context_refusals.cpp (TC-RMW-ARG-008).
 *
 * References:
 * - rmw/rmw.h (Humble, Jazzy), `rmw_wait` ("NULL entries in arrays prior to
 *   wait are considered invalid")
 */
rmw_ret_t check_wait_arrays(const rmw_subscriptions_t*    subscriptions,
                            const rmw_guard_conditions_t* guard_conditions,
                            const rmw_services_t* services, const rmw_clients_t* clients,
                            const rmw_events_t* events) noexcept {
    rmw_ret_t ret = RMW_RET_OK;
    if (subscriptions != nullptr) {
        ret = check_wait_entries(subscriptions->subscriber_count, subscriptions->subscribers,
                                 "subscriptions entry");
    }

    if (ret == RMW_RET_OK && guard_conditions != nullptr) {
        ret = check_wait_entries(guard_conditions->guard_condition_count,
                                 guard_conditions->guard_conditions, "guard_conditions entry");
    }

    if (ret == RMW_RET_OK && services != nullptr) {
        ret = check_wait_entries(services->service_count, services->services, "services entry");
    }

    if (ret == RMW_RET_OK && clients != nullptr) {
        ret = check_wait_entries(clients->client_count, clients->clients, "clients entry");
    }

    if (ret == RMW_RET_OK && events != nullptr) {
        ret = check_wait_entries(events->event_count, events->events, "events entry");
    }

    return ret;
}

/**
 * @brief Reports whether any listed subscription or guard condition is ready.
 *
 * Read-only: guard-condition flags are consumed later, by finalize_ready(),
 * so a condition seen here is still reported when the wait returns.
 */
bool any_ready(const rmw_subscriptions_t* subscriptions,
               const rmw_guard_conditions_t* guard_conditions,
               const rmw_services_t* services, const rmw_clients_t* clients) noexcept {
    if (subscriptions) {
        for (size_t i = 0; i < subscriptions->subscriber_count; ++i) {
            const auto* sub = static_cast<const rmw_rosette::Subscription*>(
                subscriptions->subscribers[i]);
            if (sub && sub->received.ready()) return true;
        }
    }
    if (services) {
        for (size_t i = 0; i < services->service_count; ++i) {
            const auto* srv = static_cast<const rmw_rosette::Service*>(services->services[i]);
            if (srv && srv->requests.ready()) return true;
        }
    }
    if (clients) {
        for (size_t i = 0; i < clients->client_count; ++i) {
            const auto* cli = static_cast<const rmw_rosette::Client*>(clients->clients[i]);
            if (cli && cli->replies.ready()) return true;
        }
    }
    if (guard_conditions) {
        for (size_t i = 0; i < guard_conditions->guard_condition_count; ++i) {
            const auto* gc = static_cast<const rmw_rosette::GuardCondition*>(
                guard_conditions->guard_conditions[i]);
            if (gc && gc->triggered.load()) return true;
        }
    }
    return false;
}

/**
 * @brief Applies rmw_wait()'s output contract: nulls every entry not ready.
 *
 * Triggered guard conditions are consumed (flag cleared) as they are
 * reported, so each trigger wakes exactly one wait. Events are always
 * nulled (see the file comment).
 */
void finalize_ready(rmw_subscriptions_t* subscriptions, rmw_guard_conditions_t* guard_conditions,
                    rmw_services_t* services, rmw_clients_t* clients,
                    rmw_events_t* events) noexcept {
    if (subscriptions) {
        for (size_t i = 0; i < subscriptions->subscriber_count; ++i) {
            const auto* sub = static_cast<const rmw_rosette::Subscription*>(
                subscriptions->subscribers[i]);
            if (!sub || !sub->received.ready()) subscriptions->subscribers[i] = nullptr;
        }
    }
    if (guard_conditions) {
        for (size_t i = 0; i < guard_conditions->guard_condition_count; ++i) {
            auto* gc = static_cast<rmw_rosette::GuardCondition*>(
                guard_conditions->guard_conditions[i]);
            if (!gc || !gc->triggered.exchange(false)) guard_conditions->guard_conditions[i] = nullptr;
        }
    }
    if (services) {
        for (size_t i = 0; i < services->service_count; ++i) {
            const auto* srv = static_cast<const rmw_rosette::Service*>(services->services[i]);
            if (!srv || !srv->requests.ready()) services->services[i] = nullptr;
        }
    }
    if (clients) {
        for (size_t i = 0; i < clients->client_count; ++i) {
            const auto* cli = static_cast<const rmw_rosette::Client*>(clients->clients[i]);
            if (!cli || !cli->replies.ready()) clients->clients[i] = nullptr;
        }
    }
    if (events) {
        for (size_t i = 0; i < events->event_count; ++i) events->events[i] = nullptr;
    }
}


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
 * @return The new wait set, or `nullptr` if check_context() refuses
 *   `context` (null, not initialized, another rmw's, or shut down) or the
 *   wait set pool is exhausted.
 *
 * Verification:
 * - Correct means: no wait set is created in a context that was shut down.
 * - Covered by: rmw_rosette/test/test_live_context_refusals.cpp (TC-RMW-ARG-005).
 *
 * References:
 * - rmw/rmw.h (Humble), `rmw_create_wait_set`
 */
rmw_wait_set_t* rmw_create_wait_set(rmw_context_t* context, size_t max_conditions) {
    (void)max_conditions;
    if (rmw_rosette::check_context(context, __func__, true) != RMW_RET_OK) {
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
 * Refuses before releasing anything, so a wait set of another
 * implementation stays with its owner.
 *
 * @param[in] wait_set Wait set to destroy.
 * @return `RMW_RET_OK`; `RMW_RET_INVALID_ARGUMENT` if `wait_set` is null or
 *   not initialized; `RMW_RET_INCORRECT_RMW_IMPLEMENTATION` for another rmw's.
 *
 * Verification:
 * - Correct means: a refused destroy leaves the slot allocated, and the
 *   destroy that follows with the identifier restored succeeds.
 * - Covered by: test_rmw_implementation's test_wait_set (rmw_destroy_wait_set).
 *
 * References:
 * - rmw/rmw.h (Humble, Jazzy), `rmw_destroy_wait_set`
 */
rmw_ret_t rmw_destroy_wait_set(rmw_wait_set_t* wait_set) {
    const rmw_ret_t checked = rmw_rosette::check_handle(wait_set, __func__, "wait_set");
    if (checked != RMW_RET_OK) {
        return checked;
    }

    wait_set_pool().release(slot_from(wait_set));
    return RMW_RET_OK;
}

/**
 * @brief Runs the participant until an entity in the wait set is ready or
 * the timeout elapses.
 *
 * The blocking primitive every executor builds its spin loop on. On a host
 * the context's pump thread runs the participant (participant_pump.hpp), and
 * this sleeps until the pump finds an entity ready after a pass. On a board
 * this is where the RTPS state machine advances: rosette::rtps::ParticipantDriver
 * drains received datagrams, ticks, and checks readiness under the
 * participant lock, then sleeps until a datagram arrives, a guard condition
 * is triggered, or the participant's next timer is due.
 *
 * Per `rmw_wait()`'s contract (rmw/rmw.h), every entry in each input
 * array starts non-null and this function nulls out entries that are
 * *not* ready by the time it returns, leaving ready ones untouched.
 * Subscriptions, services, and clients are ready when their ChangeRing
 * holds a change; guard conditions when triggered (the trigger is consumed
 * as it is reported). Events are always reported not-ready (see the file
 * comment).
 *
 * The deadline follows `wait_timeout`'s documented contract: `nullptr`
 * blocks indefinitely, `{0, 0}` checks once without sleeping, and any
 * other value bounds the wait to that duration. There is no polling
 * interval: an idle wait costs no CPU until something needs it.
 *
 * @param[in,out] subscriptions Subscriptions to wait on; entries not
 *   ready are set to `nullptr`. May be `nullptr`.
 * @param[in,out] guard_conditions Guard conditions to wait on; entries not
 *   triggered are set to `nullptr`. May be `nullptr`.
 * @param[in,out] services Services to wait on; entries without a pending
 *   request are set to `nullptr`. May be `nullptr`.
 * @param[in,out] clients Clients to wait on; entries without a pending
 *   reply are set to `nullptr`. May be `nullptr`.
 * @param[in,out] events Always fully nulled out (not evaluated).
 * @param[in] wait_set Wait set providing the context to poll.
 * @param[in] wait_timeout Maximum time to wait; see above for `nullptr`/
 *   zero handling.
 * @return `RMW_RET_OK` if any entity became ready (or `wait_timeout` was
 *   `nullptr` and one eventually did); `RMW_RET_TIMEOUT` if the deadline
 *   passed with nothing ready; `RMW_RET_INVALID_ARGUMENT` if `wait_set`
 *   or its `data` is null, or a supplied array holds a null entry, without
 *   waiting.
 *
 * Verification:
 * - Correct means: returns as soon as a subscription has data or a guard
 *   condition is triggered, reports exactly those entries, and never
 *   returns RMW_RET_TIMEOUT with a ready entry left unreported.
 * - Covered by: rmw_rosette/test/test_guard_condition.cpp,
 *   rmw_rosette/test/test_pub_sub_loopback.cpp,
 *   rmw_rosette/test/test_live_context_refusals.cpp (TC-RMW-ARG-008),
 *   rmw_rosette/test/test_participant_pump.cpp.
 *
 * References:
 * - rmw/rmw.h (Humble), `rmw_wait`
 */
rmw_ret_t rmw_wait(rmw_subscriptions_t* subscriptions, rmw_guard_conditions_t* guard_conditions,
                   rmw_services_t* services, rmw_clients_t* clients, rmw_events_t* events,
                   rmw_wait_set_t* wait_set, const rmw_time_t* wait_timeout) {
    rmw_ret_t checked = rmw_rosette::check_handle(wait_set, __func__, "wait_set");
    if (checked == RMW_RET_OK) {
        checked = check_wait_arrays(subscriptions, guard_conditions, services, clients, events);
    }

    if (checked != RMW_RET_OK) {
        return checked;
    }

    auto* impl = static_cast<rmw_rosette::WaitSet*>(wait_set->data);
    rmw_rosette::Context& ctx = *impl->context;

    // rmw_wait()'s contract (rmw/rmw.h): NULL wait_timeout blocks until
    // something is ready; {0,0} checks once; otherwise wait up to that long.
    uint32_t timeout_ms = rosette::platform::kWaitForever;
    if (wait_timeout != nullptr) {
        const uint64_t ms = static_cast<uint64_t>(wait_timeout->sec) * 1000u +
                            wait_timeout->nsec / 1000000u;
        timeout_ms = ms >= rosette::platform::kWaitForever
            ? rosette::platform::kWaitForever - 1u : static_cast<uint32_t>(ms);
    }

    // Readiness is evaluated under the participant lock after every protocol
    // pass, whichever thread runs the passes.
    const bool ready = rmw_rosette::wait_until_ready(
        ctx,
        [&]() noexcept { return any_ready(subscriptions, guard_conditions, services, clients); },
        timeout_ms);

    rmw_rosette::ContextLock lock(ctx);
    finalize_ready(subscriptions, guard_conditions, services, clients, events);
    return ready ? RMW_RET_OK : RMW_RET_TIMEOUT;
}

}  // extern "C"
