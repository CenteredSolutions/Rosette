# Capacity configuration header

Rosette reserves every table and pool at compile time. A firmware designer sets
those sizes in one C header of their own, `rosette_config.h`, next to their
application source, the way FreeRTOS reads `FreeRTOSConfig.h` and lwIP reads
`lwipopts.h`. The build system only says where that header is. Every limit,
its default, its unit, and what it costs is documented in one file the designer
reads and copies from: `include/rosette/config/rosette_config_defaults.h`.

## Files

| File | Owner | Contents |
|---|---|---|
| `rosette_config.h` | application | only the limits it changes, as `#define`s |
| `include/rosette/config/rosette_config_defaults.h` | Rosette | every user-facing limit under `#ifndef`, documented, in two profiles |
| `include/rosette/config/rosette_capacity_derived.h` | Rosette | every internal `RTPS_*` table size computed from those limits, and `#error` range checks |

The defaults header is the only file a designer needs to read. The derived
header holds what `rmw_rosette_capacity.cmake` computes today (writers from
publishers, services and clients; proxy pools from matched endpoints; buffer
classes from the datagram and sample sizes), written once in the preprocessor
so a non-CMake build (Arduino, PlatformIO, Make) gets the same tables.

## Lookup and precedence

```c
#if defined(ROSETTE_CONFIG_FILE)
#  include ROSETTE_CONFIG_FILE           /* path given by the build */
#elif __has_include(<rosette_config.h>)
#  include <rosette_config.h>            /* found on the include path */
#endif
/* then every limit: #ifndef X / #define X <default> / #endif */
```

A value is taken from the first of:

1. a compiler `-D` (CI matrices, bench scripts, the spec tests' fixed sets);
2. the application's `rosette_config.h`;
3. the profile default in `rosette_config_defaults.h`.

## Profiles

`ROSETTE_PROFILE` picks the default column; a designer overrides single
values on top of it.

| Limit | `ROSETTE_PROFILE_FIRMWARE` | `ROSETTE_PROFILE_HOST` |
|---|---|---|
| `RMW_ROSETTE_MAX_NODES` | 1 | 4 |
| `RMW_ROSETTE_MAX_PUBLISHERS` / `_SUBSCRIPTIONS` | 4 / 4 | 16 / 16 |
| `RMW_ROSETTE_MAX_SERVICES` / `_CLIENTS` | 4 / 2 | 4 / 4 |
| `RMW_ROSETTE_PARAMETER_SERVICES_PER_NODE` | 0 | 6 |
| `RMW_ROSETTE_MAX_REMOTE_PARTICIPANTS` / `_KNOWN_` | 8 / 96 | 32 / 32 |
| `RMW_ROSETTE_MAX_MATCHED_ENDPOINTS` | 8 | 16 |
| `RMW_ROSETTE_MAX_DATAGRAM_SIZE` | 1472 | 65500 |
| `RMW_ROSETTE_MAX_SAMPLE_SIZE` / `LARGE_BUFFERS` | 4096 / 2 | 1 MiB / 48 |

The values are today's `rmw_rosette_firmware.cmake` and
`rmw_rosette_capacity.cmake` defaults, so a build that sets nothing keeps its
current sizes. CMake passes `ROSETTE_PROFILE` (a fact about the platform, not a
capacity choice); a build that passes nothing gets the firmware profile.

## One configuration per image

The tables are sized in headers shared by Rosette's libraries and the
application, so every translation unit must see the same configuration (the
one-definition rule). `ROSETTE_CONFIG_FILE` is therefore a PUBLIC compile
definition of the Rosette targets, never set per source file. The derived
header also defines a link-time guard: the library defines a symbol whose name
encodes the main sizes, and the rmw entry points reference the name computed
in the caller's translation unit, so a library and an application built with
different configurations fail to link instead of corrupting memory.

## CMake after the change

- `rmw_rosette_capacity.cmake` keeps its cache variables as optional
  overrides: empty by default, and passed as `-D` only when set. Existing
  `-DRMW_ROSETTE_MAX_*=` command lines keep working. All arithmetic and range
  checks move to the derived header.
- `ROSETTE_CONFIG_FILE` names the application's header.
- An existing build directory keeps old non-empty cache values, which then win
  over `rosette_config.h`; reconfigure it fresh once.

## Steps

1. Add both headers and a probe test that compiles them with no `-D` for each
   profile and checks every derived `RTPS_*` value equals what
   `rmw_rosette_capacity.cmake` produces today. Nothing switches over yet.
2. Make the rmw build use the header: cache variables become optional
   overrides, the CMake math is deleted, and the firmware examples carry a
   `rosette_config.h` instead of CMake `set()` lines.
3. Move bare-RTPS builds (`cmake/rosette_rtps.cmake`) and the board BSP's ARP
   cache size onto the same header.

## Arduino

The Arduino IDE does not put the sketch folder on a library's include path, so
`__has_include(<rosette_config.h>)` finds nothing there and the firmware
defaults apply. PlatformIO builds can pass `-DROSETTE_CONFIG_FILE=...` in
`build_flags`. Sizing for IDE users is decided in the Arduino library design.
