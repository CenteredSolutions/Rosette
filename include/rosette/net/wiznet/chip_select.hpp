/**
 * @file net/wiznet/chip_select.hpp
 * @brief Resolves the board's WIZnet part into a `BoardChip` trait alias.
 *
 * The board owns the choice. `board_config.h` names the fitted part:
 *
 *     #define ROSETTE_WIZNET_CHIP  W5100S
 *
 * The token is the trait struct's name in `rosette::net::wiznet` (see
 * `w5100s.hpp`). This header includes the trait header for every part it knows
 * about — each is `#pragma once` and self-contained — then aliases the selected
 * one as `BoardChip`.
 *
 * Adding a part:
 *   1. add `wXXXX.hpp` with a `struct WXXXX : ChipBase { ... }` register trait
 *   2. add its `#include` below (guard with `__has_include` so boards that
 *      never select it still build without the header present)
 *   3. set `#define ROSETTE_WIZNET_CHIP WXXXX` in that board's board_config.h
 */

#pragma once

#include "board_config.h"

#include "w5100s.hpp"

#if defined(__has_include)
#  if __has_include("w6100.hpp")
#    include "w6100.hpp"
#  endif
#  if __has_include("w5500.hpp")
#    include "w5500.hpp"
#  endif
#endif

#ifndef ROSETTE_WIZNET_CHIP
#  error "board_config.h must define ROSETTE_WIZNET_CHIP (e.g. W5100S)"
#endif

namespace rosette::net::wiznet {

/// The register trait for the part this board is built for.
using BoardChip = ROSETTE_WIZNET_CHIP;

}  // namespace rosette::net::wiznet
