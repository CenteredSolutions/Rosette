/**
 * @file platform/host_runtime.hpp
 * @brief Platform-provided monotonic clock and local-address accessors.
 *
 * Rosette's transport-neutral code needs a millisecond time base and a way to
 * ask "is the network up yet" without pulling in a specific RTOS or IP stack.
 * Each board or host target implements these three `extern "C"` functions:
 *
 *  - `src/platform/pico_freertos.c` for RP2040 / RP2350 + FreeRTOS+TCP
 *  - `src/platform/posix/host_runtime.cpp` for host / POSIX builds
 *
 * The functions are intentionally C-linkage so firmware written in either C or
 * C++ can provide or consume them.
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Returns a monotonic millisecond counter.
 *
 * The epoch is arbitrary but fixed for the life of the process/boot. Callers
 * use differences, never the absolute value. Wraps after ~49.7 days.
 *
 * @return Milliseconds since an unspecified fixed start point.
 */
uint32_t rosette_platform_get_ms(void);

/**
 * @brief Returns the current local IPv4 address in host byte order.
 *
 * @return The address, or 0 when no IPv4 address is configured yet.
 */
uint32_t rosette_platform_get_ipv4_addr(void);

/**
 * @brief Copies the current local IPv6 address into @p out.
 *
 * @param[out] out 16-byte buffer that receives the address on success; zeroed
 * when no IPv6 address is available.
 * @return `true` when a non-empty IPv6 address was written.
 */
bool rosette_platform_get_ipv6_addr(uint8_t out[16]);

#ifdef __cplusplus
}  // extern "C"
#endif
