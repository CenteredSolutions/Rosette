/**
 * @file rmw_qos.cpp
 * @brief rmw_qos_profile_check_compatible.
 */

#include "rmw_rosette/identifier.h"
#include "rmw_rosette/qos.hpp"

#include <rmw/rmw.h>
#include <rmw/error_handling.h>

#include <cstdio>

extern "C" {

rmw_ret_t rmw_qos_profile_check_compatible(const rmw_qos_profile_t publisher_qos,
                                            const rmw_qos_profile_t subscription_qos,
                                            rmw_qos_compatibility_type_t* compatibility,
                                            char* reason, size_t reason_size) {
    if (!compatibility) return RMW_RET_INVALID_ARGUMENT;

    if (rmw_rosette::qos_profiles_compatible(publisher_qos, subscription_qos)) {
        *compatibility = RMW_QOS_COMPATIBILITY_OK;
        return RMW_RET_OK;
    }

    *compatibility = RMW_QOS_COMPATIBILITY_ERROR;
    if (reason && reason_size > 0) {
        // TODO: distinguish reliability vs. durability mismatch in the
        // message once qos_profiles_compatible() reports which rule failed
        // instead of just true/false.
        std::snprintf(reason, reason_size, "reliability or durability mismatch");
    }
    return RMW_RET_OK;
}

}  // extern "C"
