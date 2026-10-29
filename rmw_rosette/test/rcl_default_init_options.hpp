/**
 * @file rcl_default_init_options.hpp
 * @brief Init options as rcl_init() hands them to rmw_init() with no environment set.
 *
 * Tests that call rmw_init() directly skip rcl, which fills in what
 * rmw_init_options_init() leaves zero. Where the headers have discovery
 * options, that is the automatic discovery range: rmw_init_options_init()
 * leaves it NOT_SET, as the stock rmws do, rmw_init() rejects NOT_SET, and
 * rcl_init() sets SUBNET when ROS_AUTOMATIC_DISCOVERY_RANGE is unset.
 */

#pragma once

#include "rmw_rosette/ros_features.h"

#include <rcutils/allocator.h>
#include <rmw/init_options.h>

namespace rmw_rosette_test {

/**
 * @brief Initializes `options` with rcl_init()'s defaults for an empty environment.
 *
 * @param[out] options Zero-initialized options to initialize; the caller
 *   finalizes them with rmw_init_options_fini().
 * @return What rmw_init_options_init() returns.
 *
 * References:
 * - rcl/src/rcl/init.c (Jazzy), `rcl_init`
 * - rcl/src/rcl/discovery_options.c (Jazzy), `rcl_get_automatic_discovery_range`
 */
inline rmw_ret_t init_options_as_rcl(rmw_init_options_t* options) {
    const rmw_ret_t ret = rmw_init_options_init(options, rcutils_get_default_allocator());
#if RMW_ROSETTE_HAS_DISCOVERY_OPTIONS
    if (ret == RMW_RET_OK) {
        options->discovery_options.automatic_discovery_range = RMW_AUTOMATIC_DISCOVERY_RANGE_SUBNET;
    }
#endif
    return ret;
}

}  // namespace rmw_rosette_test
