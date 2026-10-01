/**
 * @file rmw_rosette/identifier.h
 * @brief Implementation identifier rcl/rmw use to confirm handle ownership.
 *
 * Every rmw_*_t handle rmw_rosette hands back carries this string in its
 * `implementation_identifier` field. rcl checks it before dereferencing
 * `data` as one of ours (RMW_CHECK_TYPE_IDENTIFIERS_MATCH) — get this wrong
 * and every call silently operates on garbage.
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#define RMW_ROSETTE_ID "rmw_rosette"

extern const char * const rmw_rosette_identifier;

#ifdef __cplusplus
}
#endif
