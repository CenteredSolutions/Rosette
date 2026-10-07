# Porting Rosette

Rosette's portable core (RTPS, `rmw_rosette`, address management, DHCP, SNTP, storage) reaches hardware and
operating systems through three narrow contracts: the **chip**, the **OS**, and the **network stack**. Code that
implements a contract for one chip, OS, or network stack is a **port** and lives under `ports/`. A **board** chooses
one port on each axis and supplies its wiring. This document describes the layout, the rules that keep portable code
portable, the contracts a port implements, and how to add one.

`tools/style/check_port_boundaries.py` enforces the rules. It runs in `tools/style/run_style_checks.sh` (and so in
the `style` workflow) against `tools/style/port_boundary_baseline.txt`, which lists the violations the tree still
carries. New code must pass, and the baseline may only shrink.

## Layout

```
ports/
  chip/<name>/          monotonic clock, unique id, flash, SPI/QSPI bus, GPIO edge interrupt, newlib shims
    rp2/  host/  stub/
  os/<name>/            Mutex, Event, sleep_ms: rosette_os_port.hpp (fixed name, see below)
    baremetal/  freertos/  posix/  stub/
  net/<name>/           adapters from an IP stack to rosette::net::SocketProvider
    freertos_tcp/  posix/  wiznet/ (hardware sockets: adds DHCP, no adapter)
  <axis>/<name>/port.cmake   adds the port to its CMake target; the directory's existence makes the name valid
  chip/<name>/prelude.cmake  optional: runs before project(), to set up a cross toolchain
boards/
  <board>/              board.cmake (ROSETTE_CHIP / ROSETTE_OS / ROSETTE_NET_STACK, ROSETTE_BOARD_COMMON, pins),
                        board_config.h, board_init.c
  common/wiznet/        composition shared by the WIZnet boards: driver types over the chip port's bus, board network
  common/freertos/      FreeRTOSConfig.h
include/rosette/, src/  portable only
```

Three choices follow from this layout:

- **The network chip is not under the MCU.** The WIZnet driver (`include/rosette/net/wiznet/`) talks to registers
  over a `Bus` template parameter and is portable; a W5500 on an STM32 reuses it unchanged. Only the bus and the
  interrupt line are chip-specific, and those come from `ports/chip/<name>`.
- **The build selects the OS port through a fixed-name header.** `platform.hpp` includes `"rosette_os_port.hpp"`,
  and only the selected `ports/os/<name>/` is on the include path, the same way each FreeRTOS port supplies its own
  `portmacro.h`. Portable code carries no `#if` chain over operating systems.
- **A port name is a directory, not a hard-coded list.** `cmake/rosette_ports.cmake` accepts `ROSETTE_CHIP=foo`
  when `ports/chip/foo/port.cmake` exists (also searched in `ROSETTE_PORT_DIRS`), and the same for `ROSETTE_OS`
  and `ROSETTE_NET_STACK`, so an out-of-tree port needs no edit to Rosette. A chip port's `port.cmake` may define
  `rosette_chip_add_image_outputs(target)`, which `rosette_add_image_outputs(target)` calls (the rp2 port: .uf2
  files and USB stdio), so the root CMakeLists and the examples name no chip SDK function.

## Rules

`check_port_boundaries.py` enforces three rules, with `_ZONES` and `_FAMILIES` as the table:

| Zone | May include |
|---|---|
| `ports/chip/rp2/` | Pico SDK, `board_config.h` |
| `ports/chip/imxrt/` | MCUXpresso SDK (with CMSIS), `board_config.h` |
| `ports/chip/host/`, `ports/os/posix/`, `ports/net/posix/` | POSIX |
| `ports/net/wiznet/` | nothing third-party (it only has a `port.cmake`) |
| `ports/os/baremetal/` | nothing third-party: it is built on chip-layer hooks |
| `ports/os/freertos/` | FreeRTOS kernel |
| `ports/net/freertos_tcp/` | FreeRTOS kernel, FreeRTOS+TCP |
| `boards/`, `tests/`, `examples/`, `tools/` | anything |
| everything else | the C/C++ standard libraries, littlefs, ROS 2, Rosette's portable headers |
| every zone | never the WIZnet ioLibrary |

1. **header**: a file includes only the third-party families its zone allows. The chip port doesn't include
   FreeRTOS: a bus that needs a lock takes `platform::Mutex`, which works on every OS.
2. **port-include**: portable code includes no header that lives under `ports/`, and a port includes no other
   port's headers. Only `boards/` combines ports.
3. **port-macro**: portable code doesn't branch on `ROSETTE_OS_*`, `ROSETTE_CHIP_*`, `ROSETTE_FREERTOS_TCP`,
   `PICO_*`, `__linux__`, and similar macros. The build picks the port by linking it.

## Contracts

| Axis | Contract | Notes |
|---|---|---|
| chip | `platform.hpp`: `monotonic_us`, `device_unique_id`, `realtime_hint_us` | |
| chip | `chip_sync.hpp`: `chip::critical_enter/exit`, `chip::wait_for_event_until(deadline_us)`, `chip::send_event` | the bare-metal OS layer is built on these alone; rp2 uses spin lock `PICO_SPINLOCK_ID_OS1` and WFE/SEV |
| chip | `flash.hpp`: `rosette_platform_flash_*` | storage region for littlefs |
| chip | `gpio_interrupt.hpp`: `attach_falling_edge(pin, void (*)(void*), void*)` | the host and stub chips return false |
| chip | bus concept used by `WiznetDevice<Chip, Bus>` | written down in `ports/chip/stub/stub_bus.hpp`; `Rp2SpiBus` / `Rp2QspiBus` implement it. `acquire()` nests, see "A bus shared between threads" |
| chip | `__atomic_*` helpers (`fetch_add`, `exchange`, `compare_exchange`, ...) on a core without atomic instructions | the RTPS buffer pool's `std::atomic` counts call them on Cortex-M0+; the Pico SDK's `pico_atomic` supplies them on the RP2040, `ports/chip/stub/stub_atomic.c` shows the set |
| os | `rosette_os_port.hpp`: `Mutex`, `Event` (with `signal_from_isr`), `sleep_ms` | one per `ports/os/<name>/` |
| net | `rosette::net::SocketProvider`, `AddressProvider`, `Interface` | |
| net | `rosette/net/board_network.hpp`: `bring_up_board_network`, `poll_board_network`, `board_socket_provider`, `board_ipv4_address`, `board_network_watch`, ... | each network port (`ports/net/posix/`) or board composition (`boards/common/wiznet/`) implements it; `rmw_rosette/src/network_port.cpp` forwards to it |

### What an application includes

Applications include portable headers only: `rosette/net/socket.hpp` (the `SocketProvider` / `Socket` contract),
`rosette/net/network_config.hpp`, `rosette/net/board_network.hpp`, `rosette/platform/platform.hpp`, and the
`rosette/ros/`, `rosette/storage/` headers. Backend-specific headers (`ports/net/posix/posix_socket.hpp`'s
`PosixSocketProvider`, the rp2 bus classes) are private to their ports: nothing outside a port or a board
composition constructs a backend. The board network does that, and hands out `board_socket_provider()` as a plain
`SocketProvider&`. Host test drivers in `tests/` may name port headers directly.

### The WIZnet ioLibrary

Rosette drives WIZnet chips only through its own register-level driver, `WiznetDevice<Chip, Bus>`; the ioLibrary is
neither fetched nor linked. The comments in `device.hpp` and `w6x00.hpp` that cite the io(6)Library's `socket.c` and
`wizchip_conf.c` are references for chip behavior. The check bans ioLibrary headers in every zone, boards included.

## Adding a port

1. Create `ports/<axis>/<name>/port.cmake` (in the tree, or in a directory listed in `ROSETTE_PORT_DIRS`). It adds
   the port's sources and include directory to the axis's CMake target. The stub ports (`ports/chip/stub/`,
   `ports/os/stub/`) are the smallest complete examples: every hook, each a trivial body.
2. Implement the contracts of its axis from the table above.
3. Pass the conformance tests: `rosette_platform_contract` (`tests/platform/platform_contract_test.cpp`) for a chip
   and OS pair, built as an image for the target, and the host suites under `tests/host` for a network stack.
4. Add a board under `boards/<board>/` (or in `ROSETTE_BOARD_DIRS`) whose `board.cmake` selects the new port.
5. Run `tools/style/run_style_checks.sh`: the boundary check proves the port includes only what its zone allows.

`tests/port_stub` and the `port-stub` workflow build the whole portable core, the WIZnet driver for all three chips
included, on a bare Cortex-M0+ with newlib only and the stub chip. A portable file that leaks a platform dependency
fails there as a link error, not only as an include finding.

## Port status

- **A bus shared between threads.** `WiznetDevice` nests `Bus::acquire()` (`open_udp_socket()` calls
  `close_socket()`, and each register access acquires again), so a lock there must be re-entrant. Every board drives
  its chip from one thread, so the rp2 buses hold nothing. Before a second thread uses the chip, either the platform
  contract gains a recursive mutex or the device takes the lock once per public call and the bus never does.
- **FreeRTOS+TCP** (`ports/net/freertos_tcp/`) configures and compiles off-target against FreeRTOS+TCP V4.2.2, but
  no image links yet: nothing implements `board_network.hpp` over it (`freertos_tcp_board_network.cpp` would call
  `add_board_network_interface()`, add endpoints, start the IP task, and hand out a `FreeRtosSocketProvider`), and
  nothing starts the scheduler. On the RP2350, the kernel's `ARM_CM33_NTZ` port also needs `configENABLE_FPU`,
  `configENABLE_MPU` and `configENABLE_TRUSTZONE` in `FreeRTOSConfig.h`; the Raspberry Pi kernel fork's RP2350 port
  is the likely base. Once it links, a WIZnet board in MACRAW mode with `ROSETTE_OS=freertos` and
  `ROSETTE_NET_STACK=freertos_tcp` gets a CI build.
- **lwIP** will be `ports/net/lwip/`. The boundary check already has its zone and header family.
