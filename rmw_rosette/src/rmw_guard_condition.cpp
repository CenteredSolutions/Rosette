/**
 * @file rmw_guard_condition.cpp
 * @brief rmw_create_guard_condition / rmw_trigger_guard_condition.
 *
 * Guard conditions are a pure rmw_wait_set signaling primitive with no
 * RTPS analogue — see types.hpp::GuardCondition. This is the smallest
 * file in the package; there's no protocol logic here, just a flag rmw_wait()
 * checks each poll.
 */

#include "rmw_rosette/identifier.h"
#include "rmw_rosette/types.hpp"

#include <rmw/rmw.h>
#include <rmw/error_handling.h>

extern "C" {

rmw_guard_condition_t* rmw_create_guard_condition(rmw_context_t* context) {
    (void)context;
    // TODO: allocate rmw_rosette::GuardCondition + rmw_guard_condition_t.
    RMW_SET_ERROR_MSG("rmw_rosette: rmw_create_guard_condition not implemented");
    return nullptr;
}

rmw_ret_t rmw_destroy_guard_condition(rmw_guard_condition_t* guard_condition) {
    (void)guard_condition;
    RMW_SET_ERROR_MSG("rmw_rosette: rmw_destroy_guard_condition not implemented");
    return RMW_RET_UNSUPPORTED;
}

rmw_ret_t rmw_trigger_guard_condition(const rmw_guard_condition_t* guard_condition) {
    (void)guard_condition;
    // TODO: set ->triggered = true; rmw_wait()'s poll loop clears it after
    // reporting ready.
    RMW_SET_ERROR_MSG("rmw_rosette: rmw_trigger_guard_condition not implemented");
    return RMW_RET_UNSUPPORTED;
}

}  // extern "C"
