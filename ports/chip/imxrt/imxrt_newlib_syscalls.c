// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file ports/chip/imxrt/imxrt_newlib_syscalls.c
 * @brief newlib's console output and clock on the i.MX RT: _write() to the board's consoles,
 * clock_gettime().
 *
 * The C library's stdout and stderr go to every console the board started
 * (imxrt_console.h): the LPUART named with rosette_imxrt_console_init(), as
 * blocking writes, and the USB serial port, when the board has one, through
 * its backlog. Each '\n' is sent as "\r\n", so printf() lines read correctly
 * on any serial terminal. stdout is unbuffered, which keeps a line whole on
 * the wire even if the program stops right after printing it and spares
 * newlib a heap buffer. Output written before any console starts is
 * discarded.
 *
 * clock_gettime() reports monotonic_us() as time since boot, for every
 * clock id: the Micro XRCE-DDS client's time source (uxr_millis / uxr_nanos)
 * and rcutils need it, and bare newlib does not implement it; the micro-ROS
 * agent backend then aligns that base with the agent's wall clock. Rosette's
 * own wall clock is WallClock. Nothing in an image calls gettimeofday() or
 * time(), which keep newlib's nosys stubs.
 *
 * References: newlib "System Calls" (_write); POSIX clock_gettime(); i.MX RT1060
 * Reference Manual §49 (LPUART); MCUXpresso SDK fsl_lpuart.
 */

#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <time.h>

#include "fsl_lpuart.h"
#include "imxrt_console.h"

static LPUART_Type* g_console;

/// rosette::platform::monotonic_us() with C linkage (chip_imxrt.cpp).
extern uint64_t rosette_imxrt_monotonic_us(void);

bool rosette_imxrt_console_init(LPUART_Type* uart, uint32_t baud, uint32_t source_clock_hz) {
    lpuart_config_t config;
    LPUART_GetDefaultConfig(&config);
    config.baudRate_Bps = baud;
    config.enableTx = true;
    config.enableRx = true;
    if (LPUART_Init(uart, &config, source_clock_hz) != kStatus_Success) {
        return false;
    }

    g_console = uart;
    setvbuf(stdout, NULL, _IONBF, 0);
    return true;
}

/// @brief Sends `length` octets to every console the board started.
static void console_send(const char* data, size_t length) {
    if (g_console != NULL) {
        (void)LPUART_WriteBlocking(g_console, (const uint8_t*)data, length);
    }

#if ROSETTE_IMXRT_USB_CONSOLE
    rosette_imxrt_usb_console_write(data, length);
#endif
}

int _write(int fd, const char* data, int length) {
    if (fd != 1 && fd != 2) {
        errno = EBADF;
        return -1;
    }

    int start = 0;
    for (int i = 0; i < length; ++i) {
        if (data[i] != '\n') {
            continue;
        }

        console_send(data + start, (size_t)(i - start));
        console_send("\r\n", 2u);
        start = i + 1;
    }

    console_send(data + start, (size_t)(length - start));
    return length;
}

int clock_gettime(clockid_t clk_id, struct timespec* tp) {
    (void)clk_id;
    if (tp == NULL) {
        return -1;
    }

    const uint64_t us = rosette_imxrt_monotonic_us();
    tp->tv_sec = (time_t)(us / 1000000ull);
    tp->tv_nsec = (long)((us % 1000000ull) * 1000ull);
    return 0;
}
