// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file rmw_qos.cpp
 * @brief rmw_qos_profile_check_compatible.
 */

#include "rmw_rosette/argument_checks.hpp"
#include "rmw_rosette/identifier.h"
#include "rmw_rosette/qos.hpp"

#include <rmw/rmw.h>
#include <rmw/error_handling.h>

#include <cstdio>

extern "C" {

/**
 * @brief Reports whether a publisher and a subscription with these profiles would match.
 *
 * rcl and the ros2 CLI ask this before creating an endpoint to warn about a
 * pairing that will never communicate. The answer comes from the same rules
 * the RTPS endpoint matcher applies (qos_profiles_compatible()), so the
 * prediction and the wire behaviour agree.
 *
 * @param[in]  publisher_qos    The publisher's profile.
 * @param[in]  subscription_qos The subscription's profile.
 * @param[out] compatibility    Receives OK or ERROR; must be non-null.
 * @param[out] reason           Receives a description of an incompatibility;
 *   may be null only when `reason_size` is 0.
 * @param[in]  reason_size      Capacity of `reason` in bytes.
 * @return `RMW_RET_OK` with `compatibility` set; `RMW_RET_INVALID_ARGUMENT` for a
 *   null `compatibility`, or a null `reason` with a non-zero `reason_size`.
 *
 * Verification:
 * - Correct means: a reliable subscription to a best-effort publisher, or a
 *   transient-local subscription to a volatile publisher, reports ERROR, and
 *   the invalid inputs are refused before anything is written.
 * - Covered by: test_rmw_implementation's test_qos_profile_check_compatible.
 *
 * References:
 * - rmw/rmw.h (Humble, Jazzy), `rmw_qos_profile_check_compatible`
 * - OMG DDS 1.4 §2.2.3 (requested/offered QoS compatibility)
 */
rmw_ret_t rmw_qos_profile_check_compatible(const rmw_qos_profile_t publisher_qos,
                                            const rmw_qos_profile_t subscription_qos,
                                            rmw_qos_compatibility_type_t* compatibility,
                                            char* reason, size_t reason_size) {
    if (rmw_rosette::check_not_null(compatibility, __func__, "compatibility") != RMW_RET_OK) {
        return RMW_RET_INVALID_ARGUMENT;
    }

    if (reason_size > 0 && rmw_rosette::check_not_null(reason, __func__, "reason") != RMW_RET_OK) {
        return RMW_RET_INVALID_ARGUMENT;
    }

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
