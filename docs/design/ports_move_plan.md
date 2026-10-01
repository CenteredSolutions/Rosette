# Ports: layout, rules, and move list

Rosette's portable core (RTPS, rmw, address management, DHCP, SNTP, storage) reaches hardware and operating
systems through three narrow contracts. Code that implements a contract for one chip, OS, or network stack is a
**port** and lives under `ports/`. A **board** chooses one port on each axis and supplies its wiring. This
document states the rules, lists every file that moves or changes, and gives the order to do it in. PORTING.md,
written once the seams below exist, will explain how to add a port.

`tools/style/check_port_boundaries.py` enforces the rules. It runs in `tools/style/run_style_checks.sh`, against
`tools/style/port_boundary_baseline.txt` (48 entries today). New code must pass, and each step below deletes
baseline entries. When the baseline is empty, this plan is done and this file is replaced by PORTING.md.

## Layout

```
ports/
  chip/<name>/          monotonic clock, unique id, flash, SPI/QSPI bus, GPIO edge interrupt, newlib shims
    rp2/  host/  stub/
  os/<name>/            Mutex, Event, sleep_ms: rosette_os_port.hpp (fixed name, see below)
    baremetal/  freertos/  posix/  stub/
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
| chip | `platform.hpp`: `monotonic_us`, `device_unique_id`, `realtime_hint_us` | unchanged |
| chip | `chip_sync.hpp`: `chip::critical_enter/exit`, `chip::wait_for_event_until(deadline_us)`, `chip::send_event` | added in step 8: the bare-metal OS layer is built on these alone; rp2 uses spin lock `PICO_SPINLOCK_ID_OS1` and WFE/SEV |
| chip | `flash.hpp`: `rosette_platform_flash_*` | unchanged |
| chip | `gpio_interrupt.hpp`: `attach_falling_edge(pin, void (*)(void*), void*)` | added in step 7; replaced the Pico calls in `rx_interrupt_pico.hpp`; the host and stub chips return false |
| chip | bus concept used by `WiznetDevice<Chip, Bus>` | written down in `ports/chip/stub/stub_bus.hpp`; `Rp2SpiBus` / `Rp2QspiBus` implement it. `acquire()` holds nothing: the driver nests it, so a bus shared between threads needs a re-entrant lock (see "Open decisions") |
| chip | `__atomic_*` helpers (`fetch_add`, `exchange`, `compare_exchange`, ...) on a core without atomic instructions | new to the contract: the RTPS buffer pool's `std::atomic` counts call them on Cortex-M0+; the Pico SDK's `pico_atomic` supplies them on the RP2040, `ports/chip/stub/stub_atomic.c` shows the set |
| os | `rosette_os_port.hpp`: `Mutex`, `Event` (with `signal_from_isr`), `sleep_ms` | unchanged API; moved to `ports/os/<name>/` in step 8 |
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
`get/set_ipv6_addr`) was a second platform API that overlapped the first and also held network state. Step 6
removed it:

- the clock functions became `platform::monotonic_ms()` and `sleep_ms()`;
- `wait_for_event` (WFE, woken by the INT pin's otherwise empty GPIO callback) became a `platform::Event`
  that a blocking `recv_from()` sleeps on, signaled by the INT interrupt. (Step 6 put it in the WIZnet
  `PollingExecutor`; step 7 removed the executors and gave each socket its own event.) The FreeRTOS
  `wait_for_event` was `vTaskDelay(max_wait_ms)` and never woke early; it is gone with the rest;
- the address setters had no reader on the WIZnet path (`NetworkAddress` published into them, nothing read
  back), so they went. The two getters with a real source moved next to their one caller: the FreeRTOS+TCP
  endpoint readout into `FreeRtosObserver` (which also fixes its byte order: the stack keeps `ulIPAddress` in
  network order, and the old getter handed it on as host order), and the host's local-IPv4 detection into
  `src/net/posix/board_network.cpp`.

## Move list

Actions: **move** (`git mv`, then fix includes), **split** (the contents go to several places), **refactor**
(stays, but loses its port dependency), **stays**, **delete**.

### Platform

Done in step 8. Each port is one directory with a `port.cmake`; `cmake/rosette_platform.cmake` lists no port names.

| Former path | Now |
|---|---|
| `include/rosette/platform/platform.hpp` | stays; includes `"rosette_os_port.hpp"` with no `#if` chain |
| `include/rosette/platform/os_{posix,freertos}.hpp` | `ports/os/{posix,freertos}/rosette_os_port.hpp` |
| `include/rosette/platform/os_baremetal.hpp` | `ports/os/baremetal/rosette_os_port.hpp`, rewritten over `chip_sync.hpp` (no Pico SDK); CI also builds it on the stub chip |
| `include/rosette/platform/bus_pico_{spi,qspi}.hpp` | `ports/chip/rp2/rp2_{spi,qspi}_bus.hpp`, classes `Rp2SpiBus` / `Rp2QspiBus`; the FreeRTOS semaphore and critical section are gone (see the contracts table) |
| `src/platform/chip/{rp2,host}/chip_*.cpp` | `ports/chip/{rp2,host}/chip_*.cpp` |
| `src/platform/pico_bare.c`, `pico_freertos.c` | `ports/chip/rp2/rp2_flash.c`, one version for every OS over `flash_safe_execute()`, falling back to interrupts off when the other core has not opted in; and `ports/chip/rp2/rp2_newlib_time.c` |
| `src/platform/pico_littlefs.cpp` | `ports/chip/rp2/rp2_littlefs_region.cpp`, via `ROSETTE_CHIP_STORAGE_SOURCES` |
| `src/platform/posix/host_littlefs.cpp` | `ports/chip/host/host_flash_file.cpp`, via `ROSETTE_CHIP_STORAGE_SOURCES` |
| `src/platform/posix/rtps_posix_log.cpp` | `src/rtps/rtps_stderr_log.cpp`: it uses only `<cstdio>`, so it is portable |
| `include/rosette/platform/host_runtime.hpp`, `src/platform/posix/host_runtime.cpp` | deleted in step 6 |
| `include/rosette/platform/flash.hpp`, `littlefs_platform.hpp`, `wall_clock.hpp`, `gpio_interrupt.hpp`, `chip_sync.hpp`; `src/platform/wall_clock.cpp` | stay: contracts and portable code |

### Network

| Current | Action | Destination / change |
|---|---|---|
| `include/rosette/net/posix/socket.hpp`, `src/net/posix/socket.cpp` | move | `ports/net/posix/posix_socket.{hpp,cpp}` (its `__linux__` branch is fine inside a port) |
| `include/rosette/net/freertos/{address_provider,igmp,interface,socket}.hpp` | move | `ports/net/freertos_tcp/freertos_tcp_*.hpp` |
| `src/net/freertos/{igmp,interface,socket}.cpp` | move | `ports/net/freertos_tcp/freertos_tcp_*.cpp` (see "Open decisions": `igmp.cpp` includes a header that doesn't exist) |
| `include/rosette/net/wiznet/macraw.{h,hpp}`, `src/net/wiznet/macraw.cpp` | move | `ports/net/freertos_tcp/wiznet_macraw_interface.*`: a FreeRTOS+TCP NetworkInterface over the portable WIZnet driver (since step 7 it notifies its own RX task from the interrupt; the driver has no FreeRTOS code) |
| `include/rosette/net/wiznet/rx_interrupt_pico.hpp` | done | deleted: `WiznetSocketProvider::attach_interrupt()` attaches through `gpio_interrupt.hpp`; the Pico GPIO calls are in `src/platform/chip/rp2/chip_rp2.cpp`, which moves to `ports/chip/rp2/` |
| `include/rosette/net/wiznet/device.hpp` | done | the FreeRTOS task wake-ups and interrupt-dispatch task are gone; `clear_rx_interrupts()` clears every flagged socket's RECV so INT can raise a fresh edge |
| `include/rosette/net/wiznet/executor.hpp` | done | deleted rather than merged: `WiznetSocketProvider<Chip, Bus>` waits on `platform::Event` itself (a per-socket event for `recv_from()`, the caller's for `wait_readable()`), signals them all from the INT interrupt, and guards its socket table with `platform::Mutex` |
| `include/rosette/net/wiznet/board.hpp` | move | `boards/common/rp2_wiznet/wiznet_board_types.hpp` (chooses `Rp2SpiBus`/`Rp2QspiBus` from `board_config.h`) |
| `include/rosette/net/wiznet/chip_select.hpp` | refactor | chip model comes from a compile definition set by `board.cmake`, not `board_config.h`; drop the missing `w5500.hpp` branch or add the part |
| `include/rosette/net/wiznet/board_network.hpp` | done | `include/rosette/net/board_network.hpp`, namespace `rosette::net`: the public contract (see "What an application includes") |
| `src/net/wiznet/board_network.cpp` | move | `boards/common/rp2_wiznet/wiznet_board_network.cpp`, the WIZnet implementation of that contract |
| *new* | add | the same contract over host sockets (done: `src/net/posix/board_network.cpp`, moves to `ports/net/posix/` with the POSIX port) and FreeRTOS+TCP (`ports/net/freertos_tcp/freertos_tcp_board_network.cpp`) |
| `examples/*/main.cpp` | done | include `rosette/net/board_network.hpp`, call `rosette::net::bring_up_board_network()` |
| `include/rosette/net/wiznet/dhcp_{client,provider,wire}.hpp`, `src/net/wiznet/dhcp_*.cpp` | stays | portable code (only the socket and provider interfaces), but only a socket-offload chip needs it: a MAC-level stack (FreeRTOS+TCP, lwIP) and the host OS run their own DHCP |
| `include/rosette/net/wiznet/{device,socket_provider,link,interface,slaac_provider,w5100s,w6x00}.hpp` | stays | portable WIZnet driver |
| everything else in `include/rosette/net/`, `src/net/` | stays | portable |
| `rmw_rosette/src/network_port_wiznet.cpp`, `network_port_posix.cpp` | done | `rmw_rosette/src/network_port.cpp`, forwarding to `rosette/net/board_network.hpp`; rmw no longer knows which network it runs on |
| `tests/agent/agent_live_probe.cpp`, `tests/net/sntp_live_probe.cpp` | stays | host test drivers may name `PosixSocketProvider` (the `tests/` zone allows port headers) |

### Build, config, boards

| Current | Action | Destination / change |
|---|---|---|
| `cmake/pico_sdk.cmake` | move | `ports/chip/rp2/port.cmake`, plus `rosette_port_add_image_outputs(target)` so the root CMakeLists and the contract test stop calling `pico_*` |
| `cmake/freertos_tcp.cmake` | move | `ports/net/freertos_tcp/port.cmake` |
| `cmake/rosette_platform.cmake` | done | sources and targets from `ports/chip/${ROSETTE_CHIP}/port.cmake` + `ports/os/${ROSETTE_OS}/port.cmake`; validity is "the directory exists" |
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
4. Promote `board_network.hpp` to `rosette::net`, merge the rmw network ports, and switch the examples to it (done).
5. Add a **stub chip port** (`ports/chip/stub/`, compile-only, every hook a trivial stub) and a CI job that builds the
   portable core against it with no Pico SDK. It's the first chip that isn't Pico or host, so leaks show up as
   link errors, not just include findings. (done: `ports/chip/stub/`, `ports/os/stub/`, `tests/port_stub/`, and
   `.github/workflows/port-stub.yml`. The image links every object of every portable library and the WIZnet
   driver for all three chips, on Cortex-M0+ with newlib only. `rosette_platform.cmake` already takes a port from
   `ports/<axis>/<name>/port.cmake` or `ROSETTE_PORT_DIRS`, and `platform.hpp` falls back to
   `rosette_os_port.hpp`; the existing ports switch over in step 8.)
6. Remove `host_runtime.hpp` (the largest seam; touches the WIZnet driver, address code and boards). (done: see
   "What an application includes". The boards' empty INT callbacks went with it; `attach_rx_interrupt()` owns
   the interrupt. `NetworkAddress::wait()` had no caller and went too.)
7. Refactor the WIZnet driver onto `platform::Event` and the GPIO-interrupt contract. (done: see the
   `device.hpp`, `executor.hpp` and `rx_interrupt_pico.hpp` rows. DHCP stays with the WIZnet driver, since
   every other network stack brings its own.)
8. Add the fixed-name OS header and move the OS ports; move the chip ports; split the pico runtime files. (done:
   see "Platform". Still in shared files until step 9: `pico_sdk_init()` and the `pico_*` / `hardware_*` links in
   the root `CMakeLists.txt` and `cmake/rosette_net.cmake`, and `wiznet/board.hpp`'s include of the rp2 buses.)
9. Move the network ports and `boards/common/rp2_wiznet/`; convert `cmake/` to per-port `port.cmake`.
10. Write PORTING.md: the contracts table, the zone rules, "pass `rosette_platform_contract` and the socket-provider
   contract test", and a worked example (the stub port).

Moves (`git mv`) and edits go in separate commits, so `git log --follow` keeps each file's history.

## Open decisions

- **A bus shared between threads.** `WiznetDevice` nests `Bus::acquire()` (`open_udp_socket()` calls
  `close_socket()`, and each register access acquires again), so a lock there must be re-entrant. The old FreeRTOS
  bus used a plain mutex and would have deadlocked on the first nested call. Every board drives its chip from
  one thread, so the rp2 buses hold nothing. Before a second thread uses the chip, either the platform contract
  gains a recursive mutex or the device takes the lock once per public call and the bus never does.
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
