/**
 * @file chip_rp2.cpp
 * @brief RP2040 / RP2350 chip layer of the platform contract.
 *
 * The monotonic clock is the 64-bit microsecond hardware timer, which runs
 * from reset and is the same whether or not an RTOS is present, so FreeRTOS
 * and bare-metal builds share one time base. The unique ID is the flash
 * chip's 64-bit ID (pico_unique_id). The chip has no wall-clock source; time
 * arrives later through NTP or an agent (WallClock).
 */

#include "rosette/platform/platform.hpp"

#include "hardware/timer.h"
#include "pico/unique_id.h"

namespace rosette::platform {

uint64_t monotonic_us() noexcept {
    return time_us_64();
}

void device_unique_id(uint8_t out[8]) noexcept {
    pico_unique_board_id_t id;
    pico_get_unique_board_id(&id);
    for (int i = 0; i < 8; ++i) out[i] = id.id[i];
}

bool realtime_hint_us(int64_t& unix_us) noexcept {
    (void)unix_us;
    return false;
}

}  // namespace rosette::platform
