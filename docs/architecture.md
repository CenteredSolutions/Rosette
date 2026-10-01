# Rosette Architecture

Rosette puts a microcontroller on a ROS 2 network as an ordinary DDS
participant. It provides two things:

- **`rosette::rtps`**: a DDSI-RTPS 2.5 protocol stack (discovery, reliability,
  fragmentation) written for fixed memory (`include/rosette/rtps`,
  `src/rtps`).
- **`rmw_rosette`**: a ROS 2 middleware (`rmw`) implementation over that stack
  (`rmw_rosette/`).

Applications use the stock ROS 2 client libraries (`rcl`, `rclc`, `rclcpp`,
`rclpy`) with `rmw_rosette` as their middleware. On a host that means
`RMW_IMPLEMENTATION=rmw_rosette`; on a microcontroller, `rcl` and `rclc` are
linked statically and call `rmw_rosette` directly. There is no agent, no
bridge, and no Rosette-specific client API: a firmware is an `rclc` program.
Interoperability is proven against stock middleware, never against Rosette
itself: the `tests/ros2` harness pairs `rmw_rosette` with `rmw_fastrtps_cpp`
by default and accepts any stock RMW (`ROSETTE_PEER_RMW`).

## Layers

```
 ┌────────────────────────────────────────────────────────────────────┐
 │ Application: an rclc program (firmware) or rclcpp/rclpy (host)     │
 ├────────────────────────────────────────────────────────────────────┤
 │ Stock rcl, rclc, rcutils, rosidl type supports                     │  ROS 2 Humble
 ├────────────────────────────────────────────────────────────────────┤
 │ rmw_rosette                        rmw_rosette/                    │
 │   entity pools, QoS mapping, CDR codec, services over rq/rr        │
 │   topics, graph guard, ros_discovery_info, ChangeRing take history │
 ├────────────────────────────────────────────────────────────────────┤
 │ rosette::rtps                      include/rosette/rtps, src/rtps  │
 │   RTPSParticipant ─ SPDP, SEDP, EndpointMatchTable                 │
 │   StatefulWriter / StatefulReader ─ reliability, DATA_FRAG         │
 │   ParticipantDriver ─ receive, tick, sleep; the one lock           │
 │   SocketUdpTransport ─ the three RTPS ports of §9.6.1.1            │
 │   MultiBufPool ─ every payload octet                               │
 ├────────────────────────────────────────────────────────────────────┤
 │ rosette::net                       include/rosette/net             │
 │   SocketProvider: POSIX │ WIZnet hardware UDP │ FreeRTOS+TCP       │
 │   address providers: DHCP, IPv4 link-local, static, IPv6 SLAAC     │
 ├────────────────────────────────────────────────────────────────────┤
 │ rosette::platform                  include/rosette/platform        │
 │   OS layer (posix │ freertos │ baremetal) × chip layer (host │ rp2) │
 ├────────────────────────────────────────────────────────────────────┤
 │ Board: boards/<name>/  pins, chip, Pico SDK board header           │
 └────────────────────────────────────────────────────────────────────┘
```

Each layer reaches the one below through a narrow, build-time seam:

| Seam | Header | Chosen by |
| --- | --- | --- |
| Platform port: clock, mutex, wake event, unique ID | `rosette/platform/platform.hpp` | `ROSETTE_OS` and `ROSETTE_CHIP` in the board's `board.cmake` (`cmake/rosette_platform.cmake`) |
| Sockets | `rosette/net/socket.hpp` (`SocketProvider`) | the board: WIZnet boards use the chip's hardware sockets, hosts use BSD sockets |
| Board network: bring-up, addresses, the image's `SocketProvider` | `rosette/net/board_network.hpp` | one implementation per image: `src/net/posix/board_network.cpp` on hosts, `src/net/wiznet/board_network.cpp` on WIZnet boards |
| Network port for `rmw_rosette` | `rmw_rosette/network_port.hpp` | `network_port.cpp`, which forwards to the board network |
| Capacity | `rmw_rosette/capacity.hpp`, `RTPS_*` macros | `rmw_rosette/cmake/rmw_rosette_capacity.cmake` from a few application limits |

A seam is resolved at build time, so an image contains exactly one platform
port, one socket backend, and one network port, with no virtual dispatch
between layers beyond `SocketProvider`.

## Two ways to build on it

| | `rclc` over `rmw_rosette` | RTPS core directly |
| --- | --- | --- |
| Example | `examples/rclc_chatter` | `examples/rtps_chatter` |
| Application code | stock `rclc`: nodes, publishers, subscriptions, services, timers, executor | `RTPSParticipant::create_writer()/create_reader()`, hand-written CDR |
| Graph visibility | `ros2 node list`, `ros2 node info` show the node | topics only (`ros2 topic list`); no node |
| Serialization | generated type supports, any message | the application encodes XCDR1 itself |
| Flash / static RAM (RP2350) | 238 KB / 59–124 KB depending on capacity | 157 KB / 74 KB |
| Use when | always, unless flash is the binding constraint | a minimal image with a handful of fixed topics |

Both paths run the same RTPS core with the same guarantees; the rmw path adds
the ROS 2 conventions (topic name mangling, services, graph information) that
`rtps_chatter` reproduces by hand for two topics.

## Runtime model

### One lock, one driver

`ParticipantDriver` (`rosette/rtps/participant_driver.hpp`) is the only thing
that runs a participant. Its `spin_until(ready, timeout)` drains every socket,
ticks the participant's timers (heartbeats, repairs, SPDP announcements, lease
expiry), and evaluates `ready`, all under the driver's mutex; then it releases
the mutex and sleeps in the transport until a datagram arrives, `wake()` is
called, or the next timer is due. RTPS objects take no locks of their own:
whoever touches the participant or its endpoints holds `driver.mutex()`.

`rmw_wait()` and `rmw_publisher_wait_for_all_acked()` are both
`spin_until()` with different predicates. An idle node therefore sleeps (WFE on
bare metal, blocked in `poll()` on a host) until data or a deadline needs it.
On WIZnet boards the chip's INT pin signals the driver's wake event from the
interrupt handler, so a datagram ends the sleep within interrupt latency.

### Create entities before the first spin

Because only the driver runs the participant, the first SPDP announcement goes
out in the first `rmw_wait()`. A firmware that creates all its nodes and
entities before the executor first spins (the natural `rclc` order) is
discovered with its complete endpoint set, and every SEDP announcement a peer
sends back arrives after its local endpoints exist. That order lets a small
remote-endpoint table keep exactly the endpoints the firmware matches, even on
a network far larger than the table (see
[mcu_memory_model.md](mcu_memory_model.md#remote-endpoints-are-kept-small-and-displaced-when-full)).

### Life of a sample

Publishing (`rcl_publish` on a firmware):

1. `rmw_publish()` serializes the message into one buffer-pool block using the
   type's generated CDR callbacks (micro-ROS type support) or introspection
   metadata.
2. The writer adds the change to its KEEP_LAST history and sends it to every
   matched reader: one DATA submessage, or DATA_FRAG pieces when the sample
   exceeds the datagram limit (1472 octets on WIZnet hardware UDP).
3. Reliable readers acknowledge with ACKNACK; a missing sequence number is
   repaired from the history, or answered with GAP when the history no longer
   has it. The block is released when the change leaves the history.

Receiving:

1. The transport reads one datagram into the receive buffer; the parser walks
   its submessages.
2. The matched reader copies a DATA payload into a pool block (or assembles
   DATA_FRAG pieces into one), records the sequence number, and requests
   missing ones.
3. The reader callback puts a reference to the block in the subscription's
   `ChangeRing`; `rmw_wait()` reports the subscription ready.
4. `rmw_take()` deserializes from the block into the application's message and
   releases the reference.

### Discovery and the ROS 2 graph

| Mechanism | What it carries | Spec |
| --- | --- | --- |
| SPDP | participants, their locators and lease | DDSI-RTPS 2.5 §8.5.3 |
| SEDP | every writer and reader: topic, type, QoS, locators | §8.5.4 |
| Matching | topic and type names equal; durability and reliability compatible | §8.5.5.1 |
| `ros_discovery_info` | which nodes own which endpoints (`rmw_dds_common/msg/ParticipantEntitiesInfo`) | ROS 2 `rmw_dds_common` |

ROS 2 names map to DDS names the way stock middleware maps them: topic
`/rosette/chatter` becomes `rt/rosette/chatter`, type `std_msgs/msg/String`
becomes `std_msgs::msg::dds_::String_`, and a service `/x` becomes the topics
`rq/xRequest` and `rr/xReply`, correlated by the related sample identity.

## Networking on WIZnet boards

The W5100S, W6100, and W6300 run UDP/IP in silicon, so a WIZnet board needs no
software IP stack and no RTOS. The board network
(`rosette/net/board_network.hpp`, implemented for WIZnet boards in
`src/net/wiznet/board_network.cpp`) owns the chip for the whole image: `bring_up_board_network()` initializes the chip,
waits for link, and acquires the addresses the firmware's `NetworkConfig`
names (`rosette/net/network_config.hpp`; by default DHCP, falling back to
RFC 3927 link-local); `poll_board_network()` renews the lease, and retries
DHCP while link-local holds, from the main loop. On
the W6100 and W6300, which also run IPv6 in silicon, the same module runs
stateless address autoconfiguration beside IPv4
(`rosette/net/wiznet/slaac_provider.hpp`): a link-local address, and a global
address and default router from the router's advertisements. The RTPS
transport and `rmw_rosette`'s network port both reach the chip through
`board_socket_provider()`, which hands out IPv6 UDP sockets on those parts;
RTPS itself announces IPv4 locators. `examples/wiznet_udp_echo` with
`tools/wiznet_udp_echo_probe.py` checks the socket path and every address the
board holds, below any ROS layer. What the network must provide (subnet,
multicast, ports, domain) is in [network_configuration.md](network_configuration.md).

Hardware UDP cannot reassemble IP fragments, so every datagram must fit 1472
octets. Rosette fragments its own samples to fit, and its readers request one
repair per ACKNACK so a stock peer's repair burst never exceeds the limit.
Samples larger than 1472 octets *from* a stock peer need the peer to cap its
datagrams (Fast DDS `maxMessageSize`); see `rmw_rosette/README.md`.

Address acquisition is described in
[design/network_address_bringup.md](design/network_address_bringup.md) and the
W6x00 driver in [design/wiznet_w61xx_w63xx.md](design/wiznet_w61xx_w63xx.md).

## Memory

Nothing in the RTPS core or `rmw_rosette` allocates at runtime; every table and
pool is sized at build time. [mcu_memory_model.md](mcu_memory_model.md)
describes the object model, the measured cost of each object, and the knobs.

## Persistent storage

`rosette::storage` (`include/rosette/storage`) mounts a LittleFS volume on the
board's flash (or a file on a host): `Volume` (mount, atomic whole-file
writes), `FileStore` (a keyed blob store that never interprets content), and
`DeviceConfig` (Rosette's own boot record). File formats belong to the
application. `examples/storage_demo` exercises it on a host.

## Source map

| Path | Contents |
| --- | --- |
| `include/rosette/rtps`, `src/rtps` | RTPS protocol: `participant`, `spdp`, `sedp`, `endpoint_match_table`, `stateful_writer`, `stateful_reader`, `reader_proxy`, `writer_proxy`, `history_cache`, `sn_window`, `fragment_assembler`, `message_parser`, `message_builder`, `buf_pool`, `transport`, `participant_driver` |
| `rmw_rosette/` | the `rmw` implementation (a colcon package); `cmake/rmw_rosette_capacity.cmake` is the capacity model |
| `include/rosette/net`, `src/net` | `SocketProvider` backends (POSIX, WIZnet, FreeRTOS+TCP), the board network, address providers, mDNS and NTP helpers |
| `include/rosette/net/wiznet` | register-level WIZnet driver templated on chip and bus |
| `include/rosette/platform`, `src/platform` | the platform contract (`platform.hpp`, `flash.hpp`, `gpio_interrupt.hpp`, `chip_sync.hpp`) and WallClock |
| `ports/os/<name>`, `ports/chip/<name>` | platform ports: OS layers (`rosette_os_port.hpp`), chip layers (clock, flash, GPIO, buses, LittleFS volume); each has a `port.cmake`, and `stub` is the template |
| `include/rosette/storage`, `src/storage` | LittleFS volume and file store |
| `boards/<name>` | `board.cmake` (OS, chip, features), `board_config.h` (pins, chip), `board_init.c` |
| `cmake/` | board selection (`rosette_prelude.cmake`, `boards.cmake`), `rosette_rtps.cmake`, `rmw_rosette_firmware.cmake`, dependency fetchers |
| `examples/` | firmware and host examples, per-board status, planned examples ([examples/README.md](../examples/README.md)) |
| `tests/spec` | DDSI-RTPS conformance tests with `TC-<AREA>-<section>-<seq>` IDs and spec-coverage reports |
| `tests/loopback`, `tests/host`, `tests/platform` | two in-process participants; network and driver unit tests; platform contract |
| `tests/ros2` | interop harness against stock ROS 2 middleware |
| `tools/` | `pico_uf2_flash.py` (flash and monitor a board), `gen_compile_commands.py` |

## How correctness is established

| Level | Evidence |
| --- | --- |
| Protocol statements | `tests/spec`: each test cites the DDSI-RTPS 2.5 statement it checks; `*_spec_report` targets compute coverage from `spec/ddsi_rtps_2_5/*.matrix.json`; fuzz targets for the parsers |
| Two participants | `tests/loopback`: discovery and reliable delivery in one process, no sockets |
| Platform port | `tests/platform/platform_contract_test.cpp`, run on the host and flashed to each board |
| rmw semantics | `rmw_rosette/test`: codec golden bytes, guard conditions, `ChangeRing`, pub/sub over UDP |
| Interoperability | `tests/ros2`: stock `rclpy` nodes on `rmw_rosette` and on `rmw_fastrtps_cpp` exchange topics and services, large messages through a 1472-octet link, reliable bursts, graph events |
| Hardware | `rtps_chatter` and `rclc_chatter` on the W5100S-EVB-Pico2 against stock Fast DDS |

The capability table and the open gaps of `rmw_rosette` are in
[`rmw_rosette/README.md`](../rmw_rosette/README.md).
