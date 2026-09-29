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

/**
 * @brief Blocks the caller for approximately @p ms milliseconds.
 *
 * Paces a firmware superloop between polls. On FreeRTOS this yields to
 * other tasks (`vTaskDelay`); on the host it sleeps the thread; on a bare
 * superloop it busy-waits.
 *
 * @param[in] ms Milliseconds to wait.
 */
void rosette_platform_delay_ms(uint32_t ms);

/**
 * @brief Sleeps until an interrupt wakes the CPU or @p max_wait_ms elapses.
 *
 * Unlike `rosette_platform_delay_ms()`, this returns early when *any* enabled
 * interrupt fires. The WIZnet poll-mode socket path uses it so a receive wakes
 * on the chip's INT line (a GPIO IRQ wired in `board_init`) rather than on a
 * fixed polling tick.
 *
 * On the bare Pico this is `WFE` with a hardware-alarm timeout; on the host a
 * short thread sleep; on FreeRTOS a `vTaskDelay` (the RTOS socket path is
 * event-driven and does not call this).
 *
 * @param[in] max_wait_ms Upper bound on the wait; the call may return sooner.
 */
void rosette_platform_wait_for_event(uint32_t max_wait_ms);

/**
 * @brief Records the board's current local IPv4 address.
 *
 * The network bring-up layer calls this once DHCP completes (or a static
 * address is applied) so `rosette_platform_get_ipv4_addr()` and code that gates
 * on "network up" see the address. Host builds derive the address themselves
 * and ignore this.
 *
 * @param[in] addr_host_order The address in host byte order, or 0 to clear it.
 */
void rosette_platform_set_ipv4_addr(uint32_t addr_host_order);

/**
 * @brief Records the board's current local IPv6 address.
 *
 * The network bring-up layer calls this when an IPv6 address is assigned (SLAAC
 * or link-local) or lost so `rosette_platform_get_ipv6_addr()` reflects it.
 * On FreeRTOS+TCP builds the stack owns the endpoint and this is a no-op; the
 * no-RTOS WIZnet path (future hardware IPv6) uses it.
 *
 * @param[in] addr 16-byte address, or all-zero to clear it.
 */
void rosette_platform_set_ipv6_addr(const uint8_t addr[16]);

#ifdef __cplusplus
}  // extern "C"
#endif
