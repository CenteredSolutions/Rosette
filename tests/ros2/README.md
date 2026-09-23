# rmw Interop Harness

This directory proves that `rmw_rosette` interoperates with stock ROS 2
middleware. Nothing here is Rosette-specific code: the same stock `rclpy` node
runs on both sides of a session, and only `RMW_IMPLEMENTATION` differs. If a
session passes, a stock ROS 2 application on `rmw_fastrtps_cpp` or
`rmw_cyclonedds_cpp` can talk to one on `rmw_rosette`.

| File | Role |
| --- | --- |
| `rmw_interop_probe.py` | Bidirectional `rclpy` node. Role `alpha` publishes `/<session>/alpha_to_beta` and subscribes `/<session>/beta_to_alpha`; `beta` mirrors it. Exits 0 only if it loaded the expected RMW and received `--expect-rx` samples. |
| `rtps_packet_capture.py` | Optional `tcpdump` capture of one DDS domain's RTPS port block (DDSI-RTPS 2.5 §9.6.1.1). |
| `run_rmw_interop_loopback.sh` | Runs `alpha` on the RMW under test and `beta` on a stock RMW on one host. |

## What a pass proves

Both probes use RELIABLE + VOLATILE QoS, so a pass requires, in both directions:

1. SPDP participant discovery (DDSI-RTPS 2.5 §8.5.3).
2. SEDP endpoint matching, including topic-name and type-name mangling that
   matches stock ROS 2 (`rt/` prefix, `std_msgs::msg::dds_::String_`).
3. Reliable delivery: DATA, HEARTBEAT, and ACKNACK exchange (§8.4.7, §8.4.12).
4. CDR payload compatibility for `std_msgs/msg/String`.

Each probe checks `rclpy.get_rmw_implementation_identifier()` against
`RMW_IMPLEMENTATION`, so a misconfigured environment fails instead of
silently testing a stock RMW against itself.

## Prerequisites

- A ROS 2 installation (Humble is the project's default distro), sourced.
- `rmw_rosette` built in a colcon workspace and that overlay sourced:

  ```bash
  mkdir -p ~/rosette_ws/src && ln -s "$PWD/rmw_rosette" ~/rosette_ws/src/
  cd ~/rosette_ws && colcon build --packages-select rmw_rosette
  source install/setup.bash
  ```

## Single-host session

```bash
tests/ros2/run_rmw_interop_loopback.sh
```

| Variable | Default | Meaning |
| --- | --- | --- |
| `ROSETTE_RMW` | `rmw_rosette` | RMW under test (role `alpha`) |
| `ROSETTE_PEER_RMW` | `rmw_fastrtps_cpp` | Stock reference RMW (role `beta`) |
| `ROSETTE_DOMAIN` | `42` | DDS domain id |
| `ROSETTE_DURATION_SEC` | `20` | Session length |
| `ROSETTE_EXPECT_RX` | `3` | Minimum samples each side must receive |
| `ROSETTE_CAPTURE` | `0` | `1` records a pcap from `alpha` (needs `tcpdump` privileges) |
| `ROSETTE_CAPTURE_IFACE` | `any` on Linux | Capture interface |

Validate the harness itself first by running two stock middlewares against each
other; this must pass before a Rosette failure means anything:

```bash
ROSETTE_RMW=rmw_cyclonedds_cpp tests/ros2/run_rmw_interop_loopback.sh
```

## Two-host session

To put a real network between the peers, run one role per host with the same
`--session` and `--domain`:

```bash
# host A (RMW under test)
RMW_IMPLEMENTATION=rmw_rosette python3 tests/ros2/rmw_interop_probe.py \
  --role alpha --domain 42 --session lab1 --capture

# host B (stock reference)
RMW_IMPLEMENTATION=rmw_fastrtps_cpp python3 tests/ros2/rmw_interop_probe.py \
  --role beta --domain 42 --session lab1
```

## Reading a capture

Open the pcap in Wireshark; its RTPS dissector decodes SPDP/SEDP, DATA,
HEARTBEAT, and ACKNACK. For domain `N` the capture spans UDP ports
`7400 + 250*N` through `7400 + 250*N + 99`: discovery multicast, user
multicast, and unicast ports for participant ids 0 through 44.
