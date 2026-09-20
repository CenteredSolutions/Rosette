/**
 * @file rmw_wait_set.cpp
 * @brief rmw_create_wait_set / rmw_wait — the executor's blocking entry point.
 *
 * See wait_set.hpp for the polling design this is meant to drive.
 */

#include "rmw_rosette/identifier.h"
#include "rmw_rosette/wait_set.hpp"

#include <rmw/rmw.h>
#include <rmw/error_handling.h>

extern "C" {

rmw_wait_set_t* rmw_create_wait_set(rmw_context_t* context, size_t max_conditions) {
    (void)context; (void)max_conditions;
    RMW_SET_ERROR_MSG("rmw_rosette: rmw_create_wait_set not implemented");
    return nullptr;
}

rmw_ret_t rmw_destroy_wait_set(rmw_wait_set_t* wait_set) {
    (void)wait_set;
    RMW_SET_ERROR_MSG("rmw_rosette: rmw_destroy_wait_set not implemented");
    return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_wait(rmw_subscriptions_t* subscriptions, rmw_guard_conditions_t* guard_conditions,
                   rmw_services_t* services, rmw_clients_t* clients, rmw_events_t* events,
                   rmw_wait_set_t* wait_set, const rmw_time_t* wait_timeout) {
    (void)subscriptions; (void)guard_conditions; (void)services;
    (void)clients; (void)events; (void)wait_set; (void)wait_timeout;
    // TODO: loop calling rmw_rosette::poll_once() every
    // RMW_ROSETTE_WAIT_POLL_PERIOD_MS until something's ready or
    // wait_timeout elapses, then null out the not-ready entries in each
    // of the *_t arrays per rmw_wait()'s in-place-filter contract.
    RMW_SET_ERROR_MSG("rmw_rosette: rmw_wait not implemented");
    return RMW_RET_UNSUPPORTED;
}

}  // extern "C"
