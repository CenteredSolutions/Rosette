/**
 * @file rcl_default_init_options.hpp
 * @brief Init options as rcl_init() hands them to rmw_init() with no environment set.
 *
 * Tests that call rmw_init() directly skip rcl, which fills in two fields
 * after rmw_init_options_init(): the enclave, "/" when no --enclave argument
 * is given, which rmw_init() requires; and, where the headers have discovery
 * options, the automatic discovery range, SUBNET when
 * ROS_AUTOMATIC_DISCOVERY_RANGE is unset.
 */

#pragma once

#include "rmw_rosette/ros_features.h"

#include <rcutils/allocator.h>
#include <rcutils/strdup.h>
#include <rmw/init_options.h>

namespace rmw_rosette_test {

/**
 * @brief Initializes `options` with rcl_init()'s defaults for an empty environment.
 *
 * @param[out] options Zero-initialized options to initialize; the caller
 *   finalizes them with rmw_init_options_fini(), which frees the enclave.
 * @return What rmw_init_options_init() returns, or RMW_RET_BAD_ALLOC when
 *   the enclave cannot be duplicated.
 *
 * References:
 * - rcl/src/rcl/init.c (Jazzy), `rcl_init` (the enclave and the discovery range)
 * - rcl/src/rcl/discovery_options.c (Jazzy), `rcl_get_automatic_discovery_range`
 */
inline rmw_ret_t init_options_as_rcl(rmw_init_options_t* options) {
    const rmw_ret_t ret = rmw_init_options_init(options, rcutils_get_default_allocator());
    if (ret != RMW_RET_OK) {
        return ret;
    }

    options->enclave = rcutils_strdup("/", options->allocator);
    if (options->enclave == nullptr) {
        return RMW_RET_BAD_ALLOC;
    }

#if RMW_ROSETTE_HAS(DISCOVERY_OPTIONS)
    options->discovery_options.automatic_discovery_range = RMW_AUTOMATIC_DISCOVERY_RANGE_SUBNET;
#endif
    return RMW_RET_OK;
}

}  // namespace rmw_rosette_test
