# Ports: layout, rules, and move list

Rosette's portable core (RTPS, rmw, address management, DHCP, SNTP, storage) reaches hardware and operating
systems through three narrow contracts. Code that implements a contract for one chip, OS, or network stack is a
**port** and lives under `ports/`. A **board** chooses one port on each axis and supplies its wiring. This
document states the rules, lists every file that moves or changes, and gives the order to do it in. PORTING.md,
written once the seams below exist, will explain how to add a port.

`tools/style/check_port_boundaries.py` enforces the rules. It runs in `tools/style/run_style_checks.sh`, against
`tools/style/port_boundary_baseline.txt` (100 entries today). New code must pass, and each step below deletes
baseline entries. When the baseline is empty, this plan is done and this file is replaced by PORTING.md.

## Layout

```
ports/
  chip/<name>/          monotonic clock, unique id, flash, SPI/QSPI bus, GPIO edge interrupt, newlib shims
    rp2/  host/
  os/<name>/            Mutex, Event, sleep_ms: rosette_os_port.hpp (fixed name, see below)
    baremetal/  freertos/  posix/
  net/<name>/           adapters from a third-party IP stack to rosette::net::SocketProvider
    freertos_tcp/  posix/  (lwip/ planned)
  <axis>/<name>/port.cmake   defines the port's CMake target; the directory's existence makes the name valid
boards/
  <board>/              board.cmake (ROSETTE_CHIP / ROSETTE_OS / ROSETTE_NET_STACK + pins), board_init.c
  common/rp2_wiznet/    composition shared by the WIZnet EVB-Pico boards: bus choice, board network bring-up
include/rosette/, src/  portable only
```

Three decisions follow from this layout:

- **The network chip is not under the MCU.** The WIZnet driver (`include/rosette/net/wiznet/`) talks to registers
  over a `Bus` template parameter and is portable; a W5500 on an STM32 reuses it unchanged. Only the bus and the
  interrupt line are chip-specific, and those come from `ports/chip/<name>`. MicroPython does the same: its
  WIZnet driver is in the shared `drivers/` and `extmod/`, not in `ports/rp2`.
- **The build selects the OS port through a fixed-name header.** `platform.hpp` includes `"rosette_os_port.hpp"`,
  and only the selected `ports/os/<name>/` is on the include path. That replaces the `#if ROSETTE_OS_*` chain,
  the same way each FreeRTOS port supplies its own `portmacro.h`.
- **A port name is a directory, not a hard-coded list.** `rosette_platform.cmake` accepts `ROSETTE_CHIP=foo` when
  `ports/chip/foo/port.cmake` exists (also searched in `ROSETTE_PORT_DIRS`), so an out-of-tree port needs no
  edit to Rosette.

## Rules

`check_port_boundaries.py` enforces these three rules, with `_ZONES` and `_FAMILIES` as the table:

| Zone | May include |
|---|---|
| `ports/chip/rp2/` | Pico SDK, `board_config.h` |
| `ports/chip/host/`, `ports/os/posix/`, `ports/net/posix/` | POSIX |
| `ports/os/baremetal/` | nothing third-party: it is built on chip-layer hooks |
| `ports/os/freertos/` | FreeRTOS kernel |
| `ports/net/freertos_tcp/` | FreeRTOS kernel, FreeRTOS+TCP |
| `boards/`, `tests/`, `examples/`, `tools/` | anything |
| everything else | the C/C++ standard libraries, littlefs, ROS 2, MicroPython, Rosette's portable headers |
| every zone | never the WIZnet ioLibrary |

1. **header**: a file includes only the third-party families its zone allows. The chip port doesn't include
   FreeRTOS: a bus that needs a lock takes `platform::Mutex`, which works on every OS.
2. **port-include**: portable code includes no header that lives under `ports/`, and a port includes no other
   port's headers. Only `boards/` combines ports.
3. **port-macro**: portable code doesn't branch on `ROSETTE_OS_*`, `ROSETTE_CHIP_*`, `ROSETTE_FREERTOS_TCP`,
   `PICO_*`, `__linux__`, and similar macros. The build picks the port by linking it.

## Contracts (the seams PORTING.md will document)

| Axis | Contract | Change |
|---|---|---|
| chip | `platform.hpp`: `monotonic_us`, `device_unique_id`, `realtime_hint_us` | add the hooks bare metal needs: `chip_critical_enter/exit`, `chip_wait_for_interrupt_until(deadline_us)` |
| chip | `flash.hpp`: `rosette_platform_flash_*` | unchanged |
| chip | *new* `gpio_interrupt.hpp`: `attach_falling_edge(pin, void (*)(void*), void*)` | replaces the Pico calls in `rx_interrupt_pico.hpp` |
| chip | bus concept used by `WiznetDevice<Chip, Bus>` | written down as a documented concept; `Rp2SpiBus` / `Rp2QspiBus` implement it |
| os | `rosette_os_port.hpp`: `Mutex`, `Event` (with `signal_from_isr`), `sleep_ms` | unchanged API; moves out of `include/` |
| net | `rosette::net::SocketProvider`, `AddressProvider`, `Interface` | unchanged |
| net | *promoted* `rosette/net/board_network.hpp`: `bring_up_board_network`, `poll_board_network`, `board_socket_provider`, `board_ipv4_address`, `board_network_watch`, ... | the WIZnet module's API, moved to namespace `rosette::net`; each network port or board composition implements it |
| net | `rmw_rosette/network_port.hpp` | one portable `network_port.cpp` over `board_network.hpp` replaces `network_port_{wiznet,posix}.cpp` |

### What an application includes

Applications include portable headers only: `rosette/net/socket.hpp` (the `SocketProvider` / `Socket` contract),
`rosette/net/network_config.hpp`, `rosette/net/board_network.hpp`, `rosette/platform/platform.hpp`, and the
`rosette/ros/`, `rosette/storage/` headers. Backend-specific headers (`rosette/net/posix/socket.hpp`'s
`PosixSocketProvider`, the rp2 bus classes, `rosette/net/wiznet/board_network.hpp` as it stands) become private to
their ports: nothing outside a port constructs a backend. The board network does that, and hands out
`board_socket_provider()` as a plain `SocketProvider&`. Today every example includes
`rosette/net/wiznet/board_network.hpp`, which is why none of them builds on a non-WIZnet network. Once the header
is promoted, `rclc_chatter` and `rtps_chatter` are portable as written.

`host_runtime.hpp` (`rosette_platform_get_ms`, `delay_ms`, `wait_for_event`, `get/set_ipv4_addr`,
`get/set_ipv6_addr`) is a second platform API that overlaps the first and also holds network state. It gets
removed: the clock functions become `platform::monotonic_ms`, `sleep_ms` and `Event::wait`, and the address
functions become reads of the `AddressState` the network layer already owns. Removing it also fixes a bug:
the FreeRTOS `wait_for_event` is `vTaskDelay(max_wait_ms)` and never wakes early.

## Move list

Actions: **move** (`git mv`, then fix includes), **split** (the contents go to several places), **refactor**
(stays, but loses its port dependency), **stays**, **delete**.

### Platform

| Current | Action | Destination / change |
|---|---|---|
| `include/rosette/platform/platform.hpp` | refactor | `#include "rosette_os_port.hpp"` replaces the `#if ROSETTE_OS_*` chain |
| `include/rosette/platform/os_posix.hpp` | move | `ports/os/posix/rosette_os_port.hpp` |
| `include/rosette/platform/os_freertos.hpp` | move | `ports/os/freertos/rosette_os_port.hpp` |
| `include/rosette/platform/os_baremetal.hpp` | move + refactor | `ports/os/baremetal/rosette_os_port.hpp`, built on the new chip hooks instead of `pico/critical_section.h`, `pico/mutex.h`, WFE |
| `include/rosette/platform/bus_pico_spi.hpp` | move + refactor | `ports/chip/rp2/rp2_spi_bus.hpp`; FreeRTOS semaphore becomes `platform::Mutex` |
| `include/rosette/platform/bus_pico_qspi.hpp` | move + refactor | `ports/chip/rp2/rp2_qspi_bus.hpp`; same |
| `include/rosette/platform/host_runtime.hpp` | delete | see "Contracts"; callers: `net/address.hpp`, `wiznet/{device,executor,link,slaac_provider,socket_provider}.hpp`, `net/freertos/address_provider.hpp`, four `board_init.c`, `tests/host/fake_runtime.hpp` |
| `include/rosette/platform/flash.hpp`, `littlefs_platform.hpp`, `wall_clock.hpp` | stays | contracts |
| `src/platform/wall_clock.cpp` | stays | portable |
| `src/platform/chip/rp2/chip_rp2.cpp` | move | `ports/chip/rp2/chip_rp2.cpp` |
| `src/platform/chip/host/chip_host.cpp` | move | `ports/chip/host/chip_host.cpp` |
| `src/platform/pico_bare.c` | split | flash to `ports/chip/rp2/rp2_flash.c`; `clock_gettime`/`_gettimeofday` to `ports/chip/rp2/rp2_newlib_time.c`; the rest is deleted with `host_runtime.hpp` |
| `src/platform/pico_freertos.c` | split | flash merges into the same `rp2_flash.c` (one version for both OSes, e.g. `flash_safe_execute`; check it on RP2350 with FreeRTOS SMP); endpoint address readout to `ports/net/freertos_tcp/freertos_tcp_address.cpp`; the rest is deleted |
| `src/platform/pico_littlefs.cpp` | move | `ports/chip/rp2/rp2_littlefs_region.cpp` |
| `src/platform/posix/host_littlefs.cpp` | move | `ports/chip/host/host_flash_file.cpp` |
| `src/platform/posix/host_runtime.cpp` | split | local-IPv4 detection to `ports/net/posix/posix_local_address.cpp`; the rest is deleted |
| `src/platform/posix/rtps_posix_log.cpp` | move | `ports/os/posix/rtps_stderr_log.cpp` |

### Network

| Current | Action | Destination / change |
|---|---|---|
| `include/rosette/net/posix/socket.hpp`, `src/net/posix/socket.cpp` | move | `ports/net/posix/posix_socket.{hpp,cpp}` (its `__linux__` branch is fine inside a port) |
| `include/rosette/net/freertos/{address_provider,igmp,interface,socket}.hpp` | move | `ports/net/freertos_tcp/freertos_tcp_*.hpp` |
| `src/net/freertos/{igmp,interface,socket}.cpp` | move | `ports/net/freertos_tcp/freertos_tcp_*.cpp` (see "Open decisions": `igmp.cpp` includes a header that doesn't exist) |
| `include/rosette/net/wiznet/macraw.{h,hpp}`, `src/net/wiznet/macraw.cpp` | move | `ports/net/freertos_tcp/wiznet_macraw_interface.*`: a FreeRTOS+TCP NetworkInterface over the portable WIZnet driver |
| `include/rosette/net/wiznet/rx_interrupt_pico.hpp` | split | handler logic to `include/rosette/net/wiznet/rx_interrupt.hpp` over `gpio_interrupt.hpp`; GPIO calls to `ports/chip/rp2/rp2_gpio_interrupt.cpp` |
| `include/rosette/net/wiznet/device.hpp` | refactor | per-socket FreeRTOS `TaskHandle_t` wake-ups become `platform::Event*` (the `SocketProvider::set_receive_event` path already exists) |
| `include/rosette/net/wiznet/executor.hpp` | refactor | one executor over `platform::Event` / `sleep_ms` replaces `PollingExecutor` + `FreeRtosExecutor` |
| `include/rosette/net/wiznet/board.hpp` | move | `boards/common/rp2_wiznet/wiznet_board_types.hpp` (chooses `Rp2SpiBus`/`Rp2QspiBus` from `board_config.h`) |
| `include/rosette/net/wiznet/chip_select.hpp` | refactor | chip model comes from a compile definition set by `board.cmake`, not `board_config.h`; drop the missing `w5500.hpp` branch or add the part |
| `include/rosette/net/wiznet/board_network.hpp` | move + refactor | `include/rosette/net/board_network.hpp`, namespace `rosette::net`: the public contract (see "What an application includes") |
| `src/net/wiznet/board_network.cpp` | move | `boards/common/rp2_wiznet/wiznet_board_network.cpp`, the WIZnet implementation of that contract |
| *new* | add | `ports/net/posix/posix_board_network.cpp` and `ports/net/freertos_tcp/freertos_tcp_board_network.cpp`: the same contract over host sockets and FreeRTOS+TCP |
| `examples/*/main.cpp` | refactor | include `rosette/net/board_network.hpp`, call `rosette::net::bring_up_board_network()` |
| `include/rosette/net/wiznet/dhcp_{client,provider,wire}.hpp`, `src/net/wiznet/dhcp_*.cpp` | move | `include/rosette/net/dhcp_*.hpp`, `src/net/dhcp_*.cpp`, namespace `rosette::net`: they depend only on the socket and provider interfaces |
| `include/rosette/net/wiznet/{device,socket_provider,link,interface,slaac_provider,w5100s,w6x00}.hpp` | stays | portable WIZnet driver |
| everything else in `include/rosette/net/`, `src/net/` | stays | portable |
| `rmw_rosette/src/network_port_wiznet.cpp`, `network_port_posix.cpp` | merge | `rmw_rosette/src/network_port.cpp`, forwarding to `rosette/net/board_network.hpp`; rmw no longer knows which network it runs on |
| `tests/agent/agent_live_probe.cpp`, `tests/net/sntp_live_probe.cpp` | stays | host test drivers may name `PosixSocketProvider` (the `tests/` zone allows port headers) |

### Build, config, boards

| Current | Action | Destination / change |
|---|---|---|
| `cmake/pico_sdk.cmake` | move | `ports/chip/rp2/port.cmake`, plus `rosette_port_add_image_outputs(target)` so the root CMakeLists and the contract test stop calling `pico_*` |
| `cmake/freertos_tcp.cmake` | move | `ports/net/freertos_tcp/port.cmake` |
| `cmake/rosette_platform.cmake` | refactor | sources and targets from `ports/chip/${ROSETTE_CHIP}/port.cmake` + `ports/os/${ROSETTE_OS}/port.cmake`; validity is "the directory exists" |
| `cmake/rosette_net.cmake` | refactor | stack from `ports/net/${ROSETTE_NET_STACK}/port.cmake`; the portable source list is written once, not per stack |
| `CMakeLists.txt` | refactor | no `pico_*`, `hardware_*` or `src/platform/pico_*` references; `rosette_bsp` comes from the board and its ports |
| `config/FreeRTOSConfig.h` | move | `boards/common/freertos/FreeRTOSConfig.h` (per-board values, so it's board composition) |
| `config/FreeRTOSIPConfig.h` | move | `ports/net/freertos_tcp/FreeRTOSIPConfig.h` (default a board may override) |
| `boards/bc-gen2/` | deleted | an RP2040 + W5100S board with a debug port; when a debug target is needed again it returns as a board over `boards/common/rp2_wiznet/`: the W5100S-EVB-Pico wiring plus two LEDs on GPIO 22 and 23 |
| `boards/*/board.cmake` | refactor | `ROSETTE_CHIP`, `ROSETTE_OS`, `ROSETTE_NET_STACK` (`wiznet`, `freertos_tcp`, `posix`, later `lwip`) + pins; the `ROSETTE_BOARD_ENABLE_PICO_SDK/FREERTOS/WIZNET` flags are derived, not set |
| `bindings/micropython/modrosette.c` | refactor | `sys/time.h` replaced by `mp_hal_ticks_*` (MicroPython is the port there) |

## The WIZnet ioLibrary

Rosette drives WIZnet chips only through its own register-level driver, `WiznetDevice<Chip, Bus>`; the ioLibrary is
neither fetched nor linked. The comments in `device.hpp` and `w6x00.hpp` that cite the io(6)Library's `socket.c` and
`wizchip_conf.c` are references for chip behavior. The check bans ioLibrary headers in every zone, boards included.

## Order

1. Commit the open work on `feature/rmw-rtps` so the moves start from a clean tree.
2. Add the check and baseline (done).
3. Remove the WIZnet ioLibrary (done).
4. Promote `board_network.hpp` to `rosette::net`, merge the rmw network ports, and switch the examples to it.
5. Add a **stub chip port** (`ports/chip/stub/`, compile-only, every hook a trivial stub) and a CI job that builds the
   portable core against it with no Pico SDK. It's the first chip that isn't Pico or host, so leaks show up as
   link errors, not just include findings.
6. Remove `host_runtime.hpp` (the largest seam; touches the WIZnet driver, address code and boards).
7. Refactor the WIZnet driver onto `platform::Event` and the GPIO-interrupt contract; move DHCP to `src/net/`.
8. Add the fixed-name OS header and move the OS ports; move the chip ports; split the pico runtime files.
9. Move the network ports and `boards/common/rp2_wiznet/`; convert `cmake/` to per-port `port.cmake`.
10. Write PORTING.md: the contracts table, the zone rules, "pass `rosette_platform_contract` and the socket-provider
   contract test", and a worked example (the stub port).

Moves (`git mv`) and edits go in separate commits, so `git log --follow` keeps each file's history.

## Open decisions

- **FreeRTOS+TCP stays until lwIP exists.** lwIP is the long-term IP stack (it runs on bare metal and on any RTOS);
  FreeRTOS+TCP stays because it fits FreeRTOS best. With bc-gen2 gone, no board uses FreeRTOS, and the port
  already doesn't build (`src/net/freertos/igmp.cpp` includes `igmp_freertos.hpp`, which doesn't exist). A port
  nobody builds decays, so it gets a CI build: `w5100s-evb-pico2` with `ROSETTE_OS=freertos` and
  `ROSETTE_NET_STACK=freertos_tcp` (MACRAW). The same board with a different OS and stack also shows the
  composition model working. Fix the missing header while moving the port.
- **lwIP.** It will be `ports/net/lwip/`. The check already has its zone and header family, so lwIP headers can't
  leak into portable code when the port lands.
- **MicroPython binding.** `rosette_mpy_backend_runtime.cpp` and `rosette_mpy_transport.cpp` include
  `rosette/ros/micro_ros/*`, which no longer exists. The binding needs its own repair before it can be checked.
- **Public include paths.** Decided: see "What an application includes". The only include applications lose is
  `rosette/net/wiznet/board_network.hpp`; its replacement, `rosette/net/board_network.hpp`, has the same functions
  in a portable namespace.
