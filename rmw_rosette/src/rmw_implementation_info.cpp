/**
 * @file rmw_implementation_info.cpp
 * @brief Implementation-wide queries: optional features, log severity, GID equality.
 */

#include "rmw_rosette/identifier.h"

#include <rmw/error_handling.h>
#include <rmw/features.h>
#include <rmw/rmw.h>

#include <cstring>

extern "C" {

/**
 * @brief Reports optional rmw features this implementation provides.
 *
 * Humble defines two, both about populating `rmw_message_info_t`'s
 * `publication_sequence_number`/`reception_sequence_number`.
 * rmw_take_with_info() leaves them zero, so neither is claimed.
 *
 * @param[in] feature Feature to query.
 * @return `false` for every feature.
 *
 * References:
 * - rmw/features.h (Humble)
 */
bool rmw_feature_supported(rmw_feature_t feature) {
    (void)feature;
    return false;
}

/**
 * @brief Accepts rcl's middleware log-severity request.
 *
 * rosette::rtps fixes its log level at compile time (`RTPS_LOG_LEVEL`), so
 * the request is acknowledged without changing output.
 *
 * @param[in] severity Requested severity.
 * @return `RMW_RET_OK`.
 *
 * References:
 * - rmw/rmw.h (Humble), `rmw_set_log_severity`
 */
rmw_ret_t rmw_set_log_severity(rmw_log_severity_t severity) {
    (void)severity;
    return RMW_RET_OK;
}

/**
 * @brief Compares two rmw_rosette GIDs byte for byte.
 *
 * @param[in]  gid1   First GID; must come from rmw_rosette.
 * @param[in]  gid2   Second GID; must come from rmw_rosette.
 * @param[out] result `true` when all `RMW_GID_STORAGE_SIZE` bytes match.
 * @return `RMW_RET_OK`; `RMW_RET_INVALID_ARGUMENT` for null arguments;
 *   `RMW_RET_INCORRECT_RMW_IMPLEMENTATION` when either GID belongs to
 *   another implementation.
 *
 * Verification:
 * - Correct means: a GID equals itself and differs from any other entity's.
 *
 * References:
 * - rmw/rmw.h (Humble), `rmw_compare_gids_equal`
 */
rmw_ret_t rmw_compare_gids_equal(const rmw_gid_t* gid1, const rmw_gid_t* gid2, bool* result) {
    if (!gid1 || !gid2 || !result) return RMW_RET_INVALID_ARGUMENT;
    if (gid1->implementation_identifier != rmw_rosette_identifier ||
        gid2->implementation_identifier != rmw_rosette_identifier) {
        RMW_SET_ERROR_MSG("rmw_rosette: rmw_compare_gids_equal: foreign GID");
        return RMW_RET_INCORRECT_RMW_IMPLEMENTATION;
    }
    *result = std::memcmp(gid1->data, gid2->data, RMW_GID_STORAGE_SIZE) == 0;
    return RMW_RET_OK;
}

}  // extern "C"
