/**
 * @file rtps_stdio_log.cpp
 * @brief rtps_log_write() sink on stdout, for firmware (USB CDC or UART stdio).
 *
 * One line per record: "[L][tag] message". Hosts use
 * src/platform/posix/rtps_posix_log.cpp, which writes to stderr instead.
 */

#include <cstdarg>
#include <cstdio>

extern "C" void rtps_log_write(int level, const char* tag, const char* fmt, ...)
{
    // Indexed by RTPS_LOG_LEVEL_* (rosette/rtps/log.hpp): 1 error .. 5 trace.
    static const char* kLevelName[] = {"-", "E", "W", "I", "D", "T"};
    const char* level_name = (level >= 0 && level <= 5) ? kLevelName[level] : "?";
    std::printf("[%s][%s] ", level_name, (tag != nullptr) ? tag : "rtps");
    va_list args;
    va_start(args, fmt);
    std::vprintf((fmt != nullptr) ? fmt : "", args);
    va_end(args);
    std::putchar('\n');
}
