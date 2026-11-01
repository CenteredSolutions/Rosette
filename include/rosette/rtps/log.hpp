// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file rtps/log.hpp
 * @brief Compile-time leveled logging with a platform HAL backend.
 *
 * Set RTPS_LOG_LEVEL at compile time (e.g. -DRTPS_LOG_LEVEL=4) to
 * control verbosity.  All macros below the configured level compile
 * to empty statements — no format strings are emitted into the firmware.
 *
 * Five levels, in increasing verbosity:
 *   RTPS_LOG_LEVEL_ERROR  1  Hardware/protocol faults; pool exhaustion.
 *   RTPS_LOG_LEVEL_WARN   2  Unexpected but recoverable conditions.
 *   RTPS_LOG_LEVEL_INFO   3  Lifecycle: participant join/leave, endpoint match.
 *   RTPS_LOG_LEVEL_DEBUG  4  Per-message TX/RX events; HEARTBEAT/ACKNACK.
 *   RTPS_LOG_LEVEL_TRACE  5  Every submessage, SN update, state change.
 *
 * Default level is WARN (only errors and warnings are logged).
 *
 * Platform backend
 * ----------------
 * Provide a definition of rtps_log_write() suited to the target:
 *   platform/pico_log.c  — Pico SDK stdout (USB CDC or UART).
 *
 * For builds with RTPS_LOG_LEVEL=0 the symbol is never referenced and
 * no backend is needed.
 *
 * Usage in source
 * ---------------
 *   RTPS_LOG_E("tag", "allocation failed, size=%u", sz);
 *   RTPS_LOG_I("spdp", "new peer %02x%02x%02x%02x", b0, b1, b2, b3);
 *   RTPS_LOG_D("writer", "DATA SN=%d:%lu to %u.%u.%u.%u:%u", ...);
 *   RTPS_LOG_T("parser", "submsg kind=0x%02x extent=%u", id, ext);
 */

#pragma once

// ============================================================
// Level constants
// ============================================================

#define RTPS_LOG_LEVEL_NONE   0
#define RTPS_LOG_LEVEL_ERROR  1   ///< Unrecoverable or pool-exhaustion faults.
#define RTPS_LOG_LEVEL_WARN   2   ///< Unexpected but recoverable conditions.
#define RTPS_LOG_LEVEL_INFO   3   ///< Lifecycle events (discovery, match, open).
#define RTPS_LOG_LEVEL_DEBUG  4   ///< Per-message TX/RX; HEARTBEAT/ACKNACK.
#define RTPS_LOG_LEVEL_TRACE  5   ///< Every submessage, SN, proxy state change.

#ifndef RTPS_LOG_LEVEL
#  define RTPS_LOG_LEVEL RTPS_LOG_LEVEL_WARN
#endif

// ============================================================
// Backend function declaration
//
// Implemented in platform/pico_log.c (or your platform equivalent).
// Guard with the level check so translation units that compile with
// RTPS_LOG_LEVEL=0 never reference the symbol.
// ============================================================

#if RTPS_LOG_LEVEL > RTPS_LOG_LEVEL_NONE

#  ifdef __cplusplus
extern "C" {
#  endif

void rtps_log_write(int level, const char* tag, const char* fmt, ...)
    __attribute__((format(printf, 3, 4)));

#  ifdef __cplusplus
}
#  endif

#endif /* RTPS_LOG_LEVEL > RTPS_LOG_LEVEL_NONE */

// ============================================================
// Logging macros
//
// Each macro compiles away entirely when the configured level is
// below the message's level — the condition is evaluated by the
// C preprocessor, not at runtime.
// ============================================================

#if RTPS_LOG_LEVEL >= RTPS_LOG_LEVEL_ERROR
#  define RTPS_LOG_E(tag, ...) \
     rtps_log_write(RTPS_LOG_LEVEL_ERROR, (tag), __VA_ARGS__)
#else
#  define RTPS_LOG_E(tag, ...) do {} while(0)
#endif

#if RTPS_LOG_LEVEL >= RTPS_LOG_LEVEL_WARN
#  define RTPS_LOG_W(tag, ...) \
     rtps_log_write(RTPS_LOG_LEVEL_WARN,  (tag), __VA_ARGS__)
#else
#  define RTPS_LOG_W(tag, ...) do {} while(0)
#endif

#if RTPS_LOG_LEVEL >= RTPS_LOG_LEVEL_INFO
#  define RTPS_LOG_I(tag, ...) \
     rtps_log_write(RTPS_LOG_LEVEL_INFO,  (tag), __VA_ARGS__)
#else
#  define RTPS_LOG_I(tag, ...) do {} while(0)
#endif

#if RTPS_LOG_LEVEL >= RTPS_LOG_LEVEL_DEBUG
#  define RTPS_LOG_D(tag, ...) \
     rtps_log_write(RTPS_LOG_LEVEL_DEBUG, (tag), __VA_ARGS__)
#else
#  define RTPS_LOG_D(tag, ...) do {} while(0)
#endif

#if RTPS_LOG_LEVEL >= RTPS_LOG_LEVEL_TRACE
#  define RTPS_LOG_T(tag, ...) \
     rtps_log_write(RTPS_LOG_LEVEL_TRACE, (tag), __VA_ARGS__)
#else
#  define RTPS_LOG_T(tag, ...) do {} while(0)
#endif
