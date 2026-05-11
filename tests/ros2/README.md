# ROS 2 POSIX Interop Smoke

This directory provides a host-side smoke publisher for validating Rosette
interop against a stock ROS 2 Docker node.

The smoke target generates `std_msgs__String.h` at build time via
`tools/msg_gen/msg_gen.py` and uses the generated type directly.

## Build

```bash
cmake -S tests/ros2 -B /tmp/rosette-ros2-build
cmake --build /tmp/rosette-ros2-build -j
```

## Run publisher directly

```bash
/tmp/rosette-ros2-build/ros2_posix_chatter_smoke \
  --domain 0 \
  --participant 1 \
  --duration-ms 15000 \
  --publish-ms 500 \
  --topic chatter
```

If auto-detection picks loopback, set a routable IPv4 explicitly:

```bash
ROSETTE_LOCAL_IP=172.18.0.10 /tmp/rosette-ros2-build/ros2_posix_chatter_smoke
```

## Packet-capture probe project

This directory also contains a tiny bidirectional probe pair for diagnosing
Rosette discovery and transport issues:

- `ros2_posix_packet_probe`: Rosette-side POSIX node built from this repo
- `ros2_packet_probe_peer.py`: stock ROS 2 peer using `rclpy`
- `run_ros2_packet_capture.sh`: convenience runner that starts both nodes and a
  `tcpdump` capture filtered to the selected DDS domain's RTPS port block

Why this exists:

- Discovery failures often show up before user-data ever flows.
- One-way publish tests are useful, but they do not exercise the full
  reliable writer/reader handshake.
- A symmetric probe gives you SPDP, SEDP, HEARTBEAT, ACKNACK, and payload
  traffic in one repeatable capture.

### Build the probe

```bash
cmake -S tests/ros2 -B /tmp/rosette-ros2-build
cmake --build /tmp/rosette-ros2-build -j --target ros2_posix_packet_probe
```

### Run the Rosette probe by hand

```bash
/tmp/rosette-ros2-build/ros2_posix_packet_probe \
  --domain 0 \
  --participant 31 \
  --duration-ms 20000 \
  --publish-ms 1000 \
  --tx-topic rosette_probe_tx \
  --rx-topic rosette_probe_rx \
  --expect-rx 1
```

### Run the stock ROS 2 peer by hand

Source your ROS 2 environment first, then run:

```bash
ROS_DOMAIN_ID=0 python3 tests/ros2/ros2_packet_probe_peer.py \
  --domain 0 \
  --duration-sec 20 \
  --publish-period-sec 1.0 \
  --tx-topic /rosette_probe_rx \
  --rx-topic /rosette_probe_tx \
  --expect-rx 1
```

### Capture packets automatically

This is the easiest local workflow when `tcpdump` is available and your ROS 2
Python environment is already sourced:

```bash
ROSETTE_ROS2_CAPTURE_IFACE=en0 \
tests/ros2/run_ros2_packet_capture.sh /tmp/rosette-ros2-build/ros2_posix_packet_probe
```

Useful environment overrides:

- `ROSETTE_ROS2_DOMAIN` (default: `0`)
- `ROSETTE_ROS2_PARTICIPANT` (default: `31`)
- `ROSETTE_ROS2_TX_TOPIC` (default: `/rosette_probe_tx`)
- `ROSETTE_ROS2_RX_TOPIC` (default: `/rosette_probe_rx`)
- `ROSETTE_ROS2_DURATION_MS` (default: `20000`)
- `ROSETTE_ROS2_PUBLISH_MS` (default: `1000`)
- `ROSETTE_ROS2_EXPECT_RX` (default: `1`)
- `ROSETTE_ROS2_CAPTURE_IFACE` (recommended on macOS)
- `ROSETTE_ROS2_PCAP_FILE` (default: `/tmp/rosette-rtps-domain<id>-<timestamp>.pcap`)
- `ROSETTE_LOCAL_IP` (optional Rosette-side IPv4 override)

### Two-machine use

If you want a cleaner network capture, run the Rosette probe on one machine and
the Python peer on another machine on the same L2 segment:

1. Start `tcpdump` on one or both machines with a domain-specific RTPS filter.
2. Launch `ros2_packet_probe_peer.py` on the stock ROS 2 host.
3. Launch `ros2_posix_packet_probe` on the Rosette host.
4. Stop capture after both sides report at least one RX sample.

For domain `N`, the probe runner captures `udp portrange PB-(PB+99)` where
`PB = 7400 + 250 * N`. That is intentionally wider than the exact participant
ports so you catch discovery multicast, metatraffic unicast, and user traffic
even when the peer's participant ID is not known ahead of time.

## Native ROS 2 baseline script

If your goal is to validate the network and DDS layer without Rosette in the
loop yet, use [`native_ros2_packet_probe.py`](./native_ros2_packet_probe.py) on
two native ROS 2 machines. This is the simplest "are these two hosts actually
talking?" tool in the folder.

Run on host A:

```bash
ROS_DOMAIN_ID=0 python3 tests/ros2/native_ros2_packet_probe.py \
  --role alpha \
  --domain 0 \
  --session rosette_baseline \
  --duration-sec 20 \
  --publish-period-sec 1.0 \
  --expect-rx 1 \
  --capture \
  --capture-iface en0
```

Run on host B:

```bash
ROS_DOMAIN_ID=0 python3 tests/ros2/native_ros2_packet_probe.py \
  --role beta \
  --domain 0 \
  --session rosette_baseline \
  --duration-sec 20 \
  --publish-period-sec 1.0 \
  --expect-rx 1 \
  --capture \
  --capture-iface en0
```

Notes:

- Use the same `--domain` and `--session` on both machines.
- Swap only the `--role` value: one side must be `alpha`, the other `beta`.
- `--capture` is optional, but very handy. If `tcpdump` needs privileges on
  your machine, run the script with the appropriate elevation.
- The script uses RELIABLE + VOLATILE QoS so you capture not just discovery,
  but also the HEARTBEAT/ACKNACK traffic that helps explain communication
  stalls.

## Native ROS 2 smoke script (recommended on macOS)

Runs `ros2 topic echo --once` on the host and publishes from
`ros2_posix_chatter_smoke`.

```bash
tests/ros2/run_ros2_native_smoke.sh /tmp/rosette-ros2-build/ros2_posix_chatter_smoke
```

## Docker interop smoke script (recommended on Linux)

```bash
tests/ros2/run_ros2_docker_smoke.sh /tmp/rosette-ros2-build/ros2_posix_chatter_smoke
```

## Auto smoke script (best default for CI and local)

Chooses native mode on macOS and Docker mode on Linux when available:

```bash
tests/ros2/run_ros2_smoke_auto.sh /tmp/rosette-ros2-build/ros2_posix_chatter_smoke
```

Optional environment:

- `ROSETTE_ROS2_IMAGE` (default: `osrf/ros:humble-desktop`)
- `ROSETTE_ROS2_TOPIC` (default: `/chatter`)
- `ROSETTE_ROS2_DOMAIN` (default: `0`)
- `ROSETTE_ROS2_SMOKE_MODE` (`auto|native|docker`, default: `auto`)
- `ROSETTE_LOCAL_IP` (optional override used by the Rosette publisher)

## Platform shims

POSIX platform hooks used by this smoke target live in:

- `platform/posix/rtps_posix_platform.cpp`
- `platform/posix/rtps_posix_log.cpp`

## CTest integration (optional)

```bash
cmake -S tests/ros2 -B /tmp/rosette-ros2-build -DROSETTE_ENABLE_ROS2_SMOKE_TESTS=ON
cmake --build /tmp/rosette-ros2-build -j
ctest --test-dir /tmp/rosette-ros2-build --output-on-failure
```

## Discovery/logging troubleshooting

- Ensure `RTPS_LOG_LEVEL` is at least `3` (`INFO`) for discovery events.
- In this test CMake, smoke builds use `RTPS_LOG_LEVEL=4` (`DEBUG`) by default.
- If auto-detection picks loopback, set `ROSETTE_LOCAL_IP` to a routable interface.
