// SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
// SPDX-License-Identifier: Apache-2.0

/**
 * @file ports/chip/stub/rosette_chip_port.hpp
 * @brief Stub code placement: the default section.
 *
 * A chip that executes from flash defines ROSETTE_RAM_FUNCTION(name) to put
 * the function in a section its startup code copies to RAM, one section per
 * name so the linker can still discard each unused function (the rp2 port
 * shows the form).
 */

#pragma once

#define ROSETTE_RAM_FUNCTION(name) name
