// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file ports/chip/rp2/rosette_chip_port.hpp
 * @brief RP2040 / RP2350 code placement for the platform contract.
 *
 * A flash image executes in place from QSPI flash through the XIP cache. A
 * cache miss costs a QSPI fetch, and flash is unreadable while it is being
 * erased or programmed. ROSETTE_RAM_FUNCTION() puts a function in the SDK's
 * `.time_critical.<name>` section, which crt0 copies to SRAM at boot, so an
 * interrupt path runs with neither cost.
 *
 * References: Pico SDK pico_platform_sections __not_in_flash_func();
 * RP2040 datasheet §4.4 (XIP), RP2350 datasheet §4.4.
 */

#pragma once

#include "pico.h"

#define ROSETTE_RAM_FUNCTION(name) __not_in_flash_func(name)
