/**
 * @file platform/pico_log.c
 * @brief rtps_log_write() implementation for Raspberry Pi Pico / Pico 2.
 *
 * Routes all RTPS log output through the Pico SDK's stdio layer, which
 * can target USB CDC or UART depending on CMakeLists.txt:
 *   pico_enable_stdio_usb(target 1)    -- USB CDC (default)
 *   pico_enable_stdio_uart(target 1)   -- hardware UART
 *
 * Output format per line:
 *   [   1234] E transport: unicast send failed: 64 bytes to 192.168.1.2:7400
 *   ^^^^^^^   ^  ^^^^^^^^  ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
 *   ms tick   level  tag   caller-supplied printf message
 *
 * This file is compiled only for Pico-family targets.  Other platforms
 * should provide their own rtps_log_write() implementation; builds that
 * set -DRTPS_LOG_LEVEL=0 do not reference the symbol at all.
 */

#include "log.hpp"

#if RTPS_LOG_LEVEL > RTPS_LOG_LEVEL_NONE

#include <stdio.h>
#include <stdarg.h>

/* Timestamp source: FreeRTOS tick counter converted to milliseconds.
 * Falls back to 0 for RTPS_NO_FREERTOS builds (unit-test or bare-metal). */
#ifndef RTPS_NO_FREERTOS
#  include "FreeRTOS.h"
#  include "task.h"
static unsigned long log_now_ms(void)
{
    return (unsigned long)(xTaskGetTickCount() *
                           (unsigned long)portTICK_PERIOD_MS);
}
#else
static unsigned long log_now_ms(void) { return 0UL; }
#endif

/* Single-character level prefix — index matches RTPS_LOG_LEVEL_* values. */
static const char level_char[] = { '?', 'E', 'W', 'I', 'D', 'T' };

void rtps_log_write(int level, const char* tag, const char* fmt, ...)
{
    if (level < 1) level = 1;
    if (level > 5) level = 5;

    printf("[%7lu] %c %s: ", log_now_ms(), level_char[level], tag);

    va_list ap;
    va_start(ap, fmt);
    vprintf(fmt, ap);
    va_end(ap);

    putchar('\n');
}

#endif /* RTPS_LOG_LEVEL > RTPS_LOG_LEVEL_NONE */
