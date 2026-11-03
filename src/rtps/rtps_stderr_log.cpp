// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file rtps_stderr_log.cpp
 * @brief rtps_log_write() sink on stderr, for hosts (rmw_rosette, host tools).
 *
 * One line per record: "[L][tag] message", the format of
 * rtps_stdio_log.cpp, which firmware uses to write the same records to stdout.
 */

#include <cstdarg>
#include <cstdio>

extern "C" void rtps_log_write(int level, const char* tag, const char* fmt, ...) {
    // Indexed by RTPS_LOG_LEVEL_* (rosette/rtps/log.hpp): 1 error .. 5 trace.
    static const char* kLevelName[] = {"-", "E", "W", "I", "D", "T"};
    const char* level_name = "?";
    if ((level >= 0) && (level <= 5)) {
        level_name = kLevelName[level];
    }

    std::fprintf(stderr, "[%s][%s] ", level_name, (tag != nullptr) ? tag : "rtps");

    va_list args;
    va_start(args, fmt);
    std::vfprintf(stderr, (fmt != nullptr) ? fmt : "", args);
    va_end(args);

    std::fputc('\n', stderr);
}
