# ROS 2 POSIX Interop Smoke

This directory provides a host-side smoke publisher for validating milkDDS
interop against a stock ROS 2 Docker node.

The smoke target generates `std_msgs__String.h` at build time via
`tools/msg_gen/msg_gen.py` and uses the generated type directly.

## Build

```bash
cmake -S lib/milkDDS/tests/ros2 -B /tmp/milkdds-ros2-build
cmake --build /tmp/milkdds-ros2-build -j
```

## Run publisher directly

```bash
/tmp/milkdds-ros2-build/ros2_posix_chatter_smoke \
  --domain 0 \
  --participant 1 \
  --duration-ms 15000 \
  --publish-ms 500 \
  --topic chatter
```

If auto-detection picks loopback, set a routable IPv4 explicitly:

```bash
MILKDDS_LOCAL_IP=172.18.0.10 /tmp/milkdds-ros2-build/ros2_posix_chatter_smoke
```

## Docker interop smoke script

```bash
lib/milkDDS/tests/ros2/run_ros2_docker_smoke.sh /tmp/milkdds-ros2-build/ros2_posix_chatter_smoke
```

Optional environment:

- `MILKDDS_ROS2_IMAGE` (default: `osrf/ros:humble-desktop`)
- `MILKDDS_ROS2_TOPIC` (default: `/chatter`)
- `MILKDDS_ROS2_DOMAIN` (default: `0`)
- `MILKDDS_LOCAL_IP` (optional override used by the milkDDS publisher)

## Platform shims

POSIX platform hooks used by this smoke target live in:

- `lib/milkDDS/platform/posix/rtps_posix_platform.cpp`
- `lib/milkDDS/platform/posix/rtps_posix_log.cpp`

## CTest integration (optional)

```bash
cmake -S lib/milkDDS/tests/ros2 -B /tmp/milkdds-ros2-build -DMILKDDS_ENABLE_ROS2_DOCKER_TESTS=ON
cmake --build /tmp/milkdds-ros2-build -j
ctest --test-dir /tmp/milkdds-ros2-build --output-on-failure
```
