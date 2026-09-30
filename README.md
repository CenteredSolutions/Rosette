# Rosette

Rosette puts microcontrollers on a ROS 2 network as first-class DDS
participants, with no agent and no bridge. It provides:

- **`rosette::rtps`**: a DDSI-RTPS 2.5 stack (discovery, reliability,
  fragmentation) that allocates nothing at runtime.
- **`rmw_rosette`**: a ROS 2 middleware implementation over it.

Applications are ordinary ROS 2 programs. A firmware is a stock `rclc`
program linked with `rmw_rosette`; on a host, any `rclcpp` or `rclpy` program
runs on it with `RMW_IMPLEMENTATION=rmw_rosette`. Stock tools on the rest of
the network (`ros2 topic echo`, `ros2 node list`, Fast DDS nodes) see the board
as just another node.

```c
// examples/rclc_chatter — the whole ROS side of a firmware is stock rclc
wiz::bring_up_board_network("rosette-rclc");                  // DHCP / link-local
rclc_support_init(&support, 0, nullptr, &allocator);          // rmw_rosette underneath
rclc_node_init_default(&node, "rclc_chatter", "rosette", &support);
rclc_publisher_init_default(&pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String), "chatter");
...
for (;;) rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
```

```console
$ ros2 topic echo /rosette/chatter std_msgs/msg/String      # stock Fast DDS, no configuration
data: hello from rclc #42
$ ros2 node list
/rosette/rclc_chatter
```

## Features

**ROS 2 on the board, no agent**
- Stock `rcl` / `rclc` and generated message type supports, linked statically;
  `rmw_rosette` is the middleware.
- Publishers and subscriptions, services and clients, timers, and parameter
  services. Stock `ros2 node list` / `ros2 node info` show board nodes
  (`ros_discovery_info`); on the board, topic counts and the graph guard
  condition follow discovery.
- QoS: reliable and best-effort, volatile and transient-local, keep-last
  depths, with reliable delivery repaired by HEARTBEAT / ACKNACK.
- Any `.msg` / `.srv` package: `rmw_rosette` serializes through the generated
  CDR callbacks, byte for byte as Fast DDS does.

**DDSI-RTPS 2.5 stack**
- SPDP / SEDP discovery, stateful reliable writers and readers, DATA_FRAG
  fragmentation and reassembly for messages up to 64 KB over 1472-octet
  datagrams.
- Fixed memory: every table and pool is sized at build time from a few
  application limits, and nothing is allocated per message.
- Conformance tests keyed to spec sections (`TC-<AREA>-<section>-<seq>`).

**Networking**
- WIZnet W5100S, W6100, and W6300 hardware UDP sockets over SPI, with the
  chip's INT pin waking the firmware when a datagram arrives.
- IPv4: DHCP with RFC 3927 link-local fallback, duplicate-address reaction,
  lease renewal, and re-acquisition when the cable returns.
- IPv6 on the W6100 / W6300: link-local and SLAAC global addresses with
  duplicate address detection, and UDP6 sockets.

**Boards and tools**
- RP2040 and RP2350 boards, bare metal; POSIX hosts.
- A LittleFS volume and file store for on-board configuration.
- `tools/pico_uf2_flash.py`: flash over USB (or trigger BOOTSEL over the
  network) and follow the console.

## Status

| Proven | How |
| --- | --- |
| Topics and services both ways with stock `rmw_fastrtps_cpp` (ROS 2 Humble), default QoS, parameter services | `tests/ros2/run_rmw_interop_loopback.sh` |
| Messages up to 64 KB through a link that drops datagrams over 1472 octets | `tests/ros2/run_rmw_frag_loopback.sh` |
| Stock `ros2 node list` / `node info` show Rosette nodes | `ros_discovery_info`, same harness |
| A stock `rclc` node on an RP2350 + W5100S board, no RTOS, talking to unconfigured Fast DDS; rcl heap steady at ~4 KB | `examples/rclc_chatter` on the W5100S-EVB-Pico2 |
| Dual-stack networking on an RP2040 + W6100 board: DHCP, IPv6 link-local and SLAAC global, ping and UDP echo over each | `examples/wiznet_udp_echo` on the W6100-EVB-Pico |
| DDSI-RTPS protocol statements, by spec section | `tests/spec` (`TC-<AREA>-<section>-<seq>`) |

Open gaps (graph queries for remote node names, QoS events, a FreeRTOS+TCP
network port) are listed in [`rmw_rosette/README.md`](rmw_rosette/README.md#known-gaps).

## Architecture in one picture

```
 rclc firmware / rclcpp / rclpy
 stock rcl, rclc, type supports
 ─────────────────────────────── rmw ABI
 rmw_rosette          entity pools, CDR, services, graph, take rings
 rosette::rtps        SPDP/SEDP, StatefulWriter/Reader, DATA_FRAG, buffer pool
 rosette::net         SocketProvider: POSIX │ WIZnet hardware UDP │ FreeRTOS+TCP
 rosette::platform    OS (posix │ freertos │ baremetal) × chip (host │ rp2)
 boards/<name>        pins, chip, Pico SDK board
```

Every table and pool is sized at build time from a few limits
(`RMW_ROSETTE_MAX_PUBLISHERS`, ...). See [docs/architecture.md](docs/architecture.md)
and [docs/mcu_memory_model.md](docs/mcu_memory_model.md).

## Boards

Select a board at configure time with `-DROSETTE_BOARD=<name>`.

| Board | MCU | Network | OS | `rclc_chatter` | `rtps_chatter` |
| --- | --- | --- | --- | --- | --- |
| `posix` | host | BSD sockets | POSIX | host: `RMW_IMPLEMENTATION=rmw_rosette` | – |
| `w5100s-evb-pico2` | RP2350 | W5100S hardware UDP | none | ✔ runs | ✔ runs |
| `w6100-evb-pico` | RP2040 | W6100 hardware UDP, IPv4 + IPv6, SPI | none | builds (RP2040 C client stack); not run on hardware | builds; `wiznet_udp_echo` runs (IPv4, IPv6 link-local and SLAAC global) |
| `w6100-evb-pico2` | RP2350 | W6100 hardware UDP, IPv4 + IPv6, SPI | none | builds; not run on hardware | builds; not run on hardware |
| `w6300-evb-pico2` | RP2350 | W6300 hardware UDP, single-lane QSPI | none | builds; not run on hardware | builds; not run on hardware |
| `w5100s-evb-pico` | RP2040 | W5100S hardware UDP | none | builds (RP2040 C client stack); not run on hardware | builds |

`posix` is the default board on macOS; `w5100s-evb-pico2` elsewhere. "Runs"
means verified on the board against stock ROS 2; "builds" means the image
links and has not been run. The W6100 and W6300 have no `SEND_MAC` command, so
whether they address RTPS discovery multicast correctly is checked only when a
ROS example first runs on one.

## Quick start

Host tests, no hardware:

```bash
cmake -S tests/spec -B build/tests-spec && cmake --build build/tests-spec -j
ctest --test-dir build/tests-spec --output-on-failure
```

`rmw_rosette` on a host, talking to stock Fast DDS:

```bash
mkdir -p ~/rosette_ws/src && ln -s "$PWD/rmw_rosette" ~/rosette_ws/src/
cd ~/rosette_ws && source /opt/ros/humble/setup.bash
colcon build --packages-select rmw_rosette && source install/setup.bash
export ROS_DOMAIN_ID=42
RMW_IMPLEMENTATION=rmw_rosette ros2 topic pub /hello std_msgs/msg/String "{data: hi}" &
RMW_IMPLEMENTATION=rmw_fastrtps_cpp ros2 topic echo /hello std_msgs/msg/String
```

An `rclc` firmware for the W5100S-EVB-Pico2 (RP2350), with the C client
stack built for Cortex-M33:

```bash
cmake -S . -B build-hw/pico2 -DROSETTE_BOARD=w5100s-evb-pico2 \
      -DROSETTE_ROS2_C_STACK_DIR=$PWD/build-micro-ros-pico2/firmware/mcu_ws/install
cmake --build build-hw/pico2 --target rclc_chatter
tools/pico_uf2_flash.py build-hw/pico2/examples/rclc_chatter/rclc_chatter.uf2 --monitor 60
```

The W6100-EVB-Pico (RP2040) takes its own Cortex-M0+ stack; check its network
first with the echo example:

```bash
cmake -S . -B build-hw/w6100-pico -DROSETTE_BOARD=w6100-evb-pico \
      -DROSETTE_ROS2_C_STACK_DIR=$PWD/build-micro-ros-pico/firmware/mcu_ws/install
cmake --build build-hw/w6100-pico --target wiznet_udp_echo rclc_chatter
tools/pico_uf2_flash.py build-hw/w6100-pico/examples/wiznet_udp_echo/wiznet_udp_echo.uf2 --monitor 30
```

[GETTING_STARTED.md](GETTING_STARTED.md) walks through each step: installing
ROS 2 and the toolchain, building the C client stack for each CPU, and
starting your own firmware project.

## Repository map

| Path | Contents |
| --- | --- |
| `include/rosette/rtps`, `src/rtps` | DDSI-RTPS protocol stack |
| `rmw_rosette/` | ROS 2 `rmw` implementation (colcon package) |
| `include/rosette/net`, `src/net` | socket backends, WIZnet driver, address providers |
| `include/rosette/platform`, `src/platform` | platform ports (OS × chip) |
| `include/rosette/storage`, `src/storage` | LittleFS volume and file store |
| `boards/` | board definitions |
| `cmake/` | board selection, RTPS and firmware build modules, dependency fetchers |
| `examples/` | firmware and host examples, current and planned: [examples/README.md](examples/README.md) |
| `tests/` | `spec` (conformance), `loopback`, `host`, `platform`, `ros2` (interop) |
| `tools/` | `pico_uf2_flash.py` (flash a board, follow its console), `wiznet_udp_echo_probe.py` (ping and echo every board address) |

## Documentation

| Document | Read it for |
| --- | --- |
| [GETTING_STARTED.md](GETTING_STARTED.md) | installing, building, flashing, and writing a firmware |
| [examples/README.md](examples/README.md) | what each example shows, per-board status, planned examples |
| [docs/network_configuration.md](docs/network_configuration.md) | addresses, ROS domain, ports and multicast, datagram size, checking the network |
| [docs/architecture.md](docs/architecture.md) | layers, runtime model, life of a sample, source map |
| [docs/mcu_memory_model.md](docs/mcu_memory_model.md) | how objects are held, measured costs, the capacity knobs |
| [rmw_rosette/README.md](rmw_rosette/README.md) | rmw capabilities, serialization, large messages, known gaps |
| [tests/spec/README.md](tests/spec/README.md) | conformance coverage reports and fuzzing |
| [tests/ros2/README.md](tests/ros2/README.md) | the interop harness |
| [docs/design/network_address_bringup.md](docs/design/network_address_bringup.md) | how the address layer is built: providers, DHCP, link-local, SLAAC, conflicts |
| [docs/design/wiznet_w61xx_w63xx.md](docs/design/wiznet_w61xx_w63xx.md) | the W6100/W6300 driver and its hardware IPv6 |
| [docs/micro_ros_agent_client.md](docs/micro_ros_agent_client.md) | the agent-based micro-ROS examples |
