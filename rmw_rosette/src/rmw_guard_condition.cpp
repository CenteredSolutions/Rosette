/**
 * @file rmw_guard_condition.cpp
 * @brief rmw_create_guard_condition / rmw_destroy_guard_condition /
 *        rmw_trigger_guard_condition.
 *
 * Guard conditions are the rmw wait-set primitive rcl uses to wake a
 * blocked rmw_wait() from another thread: executor interrupts, SIGINT
 * handling, and graph-change notifications all go through one. Each is a
 * pooled rmw_rosette::GuardCondition (types.hpp) holding an atomic flag;
 * rmw_trigger_guard_condition() sets it and rmw_wait() consumes it.
 */

#include "rmw_rosette/identifier.h"
#include "rmw_rosette/pool.hpp"
#include "rmw_rosette/types.hpp"

#include <rmw/rmw.h>
#include <rmw/error_handling.h>

#ifndef RMW_ROSETTE_MAX_GUARD_CONDITIONS
/// rclpy and rclcpp create several per context (shutdown, SIGINT, and one
/// per executor), plus one per waitable that needs manual wakeups.
#  define RMW_ROSETTE_MAX_GUARD_CONDITIONS 32
#endif

namespace {

/// The handle rcl sees and the flag it points at, in one pooled slot.
struct GuardConditionSlot {
    rmw_guard_condition_t handle{};
    rmw_rosette::GuardCondition impl{};
};

rmw_rosette::Pool<GuardConditionSlot, RMW_ROSETTE_MAX_GUARD_CONDITIONS>& guard_condition_pool() {
    static rmw_rosette::Pool<GuardConditionSlot, RMW_ROSETTE_MAX_GUARD_CONDITIONS> pool;
    return pool;
}

}  // namespace

extern "C" {

/**
 * @brief Creates an untriggered guard condition.
 *
 * @param[in] context Context the guard condition belongs to; recorded on
 *   the handle, as rcl expects.
 * @return The new guard condition, or `nullptr` if `context` is null or
 *   RMW_ROSETTE_MAX_GUARD_CONDITIONS are already in use.
 *
 * Verification:
 * - Correct means: a fresh guard condition is not reported ready by rmw_wait().
 * - Covered by: rmw_rosette/test/test_guard_condition.cpp.
 *
 * References:
 * - rmw/rmw.h (Humble), `rmw_create_guard_condition`
 */
rmw_guard_condition_t* rmw_create_guard_condition(rmw_context_t* context) {
    if (!context) {
        RMW_SET_ERROR_MSG("rmw_rosette: rmw_create_guard_condition: null context");
        return nullptr;
    }
    GuardConditionSlot* slot = guard_condition_pool().acquire();
    if (!slot) {
        RMW_SET_ERROR_MSG("rmw_rosette: rmw_create_guard_condition: pool exhausted "
                          "(RMW_ROSETTE_MAX_GUARD_CONDITIONS)");
        return nullptr;
    }
    slot->handle.implementation_identifier = rmw_rosette_identifier;
    slot->handle.data = &slot->impl;
    slot->handle.context = context;
    return &slot->handle;
}

/**
 * @brief Destroys a guard condition created by rmw_create_guard_condition().
 *
 * @param[in] guard_condition Guard condition to destroy. The caller must
 *   have removed it from every wait set first.
 * @return `RMW_RET_OK`, or `RMW_RET_INVALID_ARGUMENT` if null.
 *
 * References:
 * - rmw/rmw.h (Humble), `rmw_destroy_guard_condition`
 */
rmw_ret_t rmw_destroy_guard_condition(rmw_guard_condition_t* guard_condition) {
    if (!guard_condition) return RMW_RET_INVALID_ARGUMENT;
    guard_condition_pool().release(reinterpret_cast<GuardConditionSlot*>(guard_condition));
    return RMW_RET_OK;
}

/**
 * @brief Marks a guard condition triggered; safe to call from any thread.
 *
 * The flag stays set until an rmw_wait() that includes this guard condition
 * reports it ready, so a trigger that lands between two waits is not lost.
 *
 * @param[in] guard_condition Guard condition to trigger.
 * @return `RMW_RET_OK`, or `RMW_RET_INVALID_ARGUMENT` if null.
 *
 * Verification:
 * - Correct means: the next rmw_wait() containing the condition returns
 *   RMW_RET_OK with it left non-null, and the one after that does not.
 * - Covered by: rmw_rosette/test/test_guard_condition.cpp.
 *
 * References:
 * - rmw/rmw.h (Humble), `rmw_trigger_guard_condition`
 */
rmw_ret_t rmw_trigger_guard_condition(const rmw_guard_condition_t* guard_condition) {
    if (!guard_condition || !guard_condition->data) return RMW_RET_INVALID_ARGUMENT;
    static_cast<rmw_rosette::GuardCondition*>(guard_condition->data)->triggered.store(true);
    return RMW_RET_OK;
}

}  // extern "C"
