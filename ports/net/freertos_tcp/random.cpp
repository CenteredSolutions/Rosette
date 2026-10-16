/**
 * @file ports/net/freertos_tcp/random.cpp
 * @brief xApplicationGetRandomNumber(): the random numbers FreeRTOS+TCP asks the application for.
 *
 * The stack draws them for DHCP transaction IDs, link-local address
 * candidates and probe delays, DNS query IDs, IPv6 interface identifiers,
 * and ephemeral UDP ports. None of these is a secret: they must differ from
 * one boot and one board to the next, so that two boards started together do
 * not pick the same link-local address or answer each other's DHCP offers.
 * The generator is xorshift32 seeded from the platform's boot nonce (a
 * hardware TRNG draw on chips that have one) mixed with the monotonic clock.
 * It is not a cryptographic generator; ipconfigUSE_TCP is off, so no TCP
 * initial sequence number depends on it.
 *
 * Concurrency: the IP task and a task binding a socket to port 0 can both
 * call it. A race between them may hand both the same value, which costs a
 * retried bind or DHCP exchange and nothing else.
 */

#include <cstdint>

extern "C" {
#include "FreeRTOS.h"
#include "FreeRTOS_IP.h"
}

#include "rosette/platform/platform.hpp"

namespace {

uint32_t g_state = 0u;

}  // namespace

/**
 * @brief Writes the next 32 random bits.
 *
 * @param[out] pulNumber Receives the value.
 * @return pdTRUE: the generator cannot fail.
 *
 * Verification:
 * - Correct means: two EVKs reset together request DHCP with different
 *   transaction IDs and, without a server, settle on different 169.254/16
 *   addresses.
 * - Covered by: no automated test.
 *
 * References: G. Marsaglia, "Xorshift RNGs", J. Stat. Software 8(14), 2003;
 * FreeRTOS+TCP FreeRTOS_IP.h (xApplicationGetRandomNumber).
 */
extern "C" BaseType_t xApplicationGetRandomNumber(uint32_t* pulNumber) {
    if (g_state == 0u) {
        const uint32_t seed = rosette::platform::boot_nonce()
                              ^ static_cast<uint32_t>(rosette::platform::monotonic_us());
        g_state = seed != 0u ? seed : 0x9E3779B9u;
    }

    uint32_t x = g_state;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    g_state = x;
    *pulNumber = x;
    return pdTRUE;
}
