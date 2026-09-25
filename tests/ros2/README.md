# rmw Interop Harness

This directory proves that `rmw_rosette` interoperates with stock ROS 2
middleware. Nothing here is Rosette-specific code: the same stock `rclpy` node
runs on both sides of a session, and only `RMW_IMPLEMENTATION` differs. If a
session passes, a stock ROS 2 application on `rmw_fastrtps_cpp` or
`rmw_cyclonedds_cpp` can talk to one on `rmw_rosette`.

| File | Role |
| --- | --- |
| `rmw_interop_probe.py` | Bidirectional default `rclpy` node. Role `alpha` publishes `/<session>/alpha_to_beta` and subscribes `/<session>/beta_to_alpha`; `beta` mirrors it. Each serves `std_srvs/SetBool` on `/<session>/<role>_echo` and calls the peer's echo and `get_parameters` services. Exits 0 only if it loaded the expected RMW, received `--expect-rx` samples, and completed `--expect-calls` correctly correlated calls per peer service. With `--expect-peer-graph` it must also list the peer's node, with the peer's echo service under it, which only the peer's `ros_discovery_info` messages can provide. |
| `rmw_serialization_parity.py` | Serializes a corpus of populated messages (strings, arrays, nested sequences, mixed alignment) through `rmw_serialize()` on `rmw_rosette` and on a stock RMW, then checks that the CDR layouts match and that each side decodes the other's bytes. |
| `rmw_frag_probe.py` / `run_rmw_frag_loopback.sh` | Large-message test at a small datagram limit (default 1472): images from 16 B to 60 KB and 20 KB `get_parameters` replies both ways, every octet verified; passes only when rosette's transport counters show no datagram above the limit and none truncated. |
| `rmw_reliable_burst_probe.py` / `run_rmw_reliable_burst_loopback.sh` | Reader-history test: a stock publisher sends a reliable burst while the rosette subscription is not taking (the shape of a late-match repair burst); the subscription must then take exactly the newest min(count, depth) samples. The publisher role with `--topic` drives a board's subscription. |
| `rcl_graph_wait_probe.cpp` / `rmw_graph_peer.py` / `run_rmw_graph_loopback.sh` | Graph test: an rcl program on the RMW under test blocks on its node's graph guard condition (as rclcpp's GraphListener does) until a stock peer's late publisher and subscription appear, checks both counts are 1, then sees them fall to 0 when the peer destroys them. Each wait must end well inside its 10 s timeout, which only a triggered graph guard allows. The script compiles the probe against the sourced rcl. |
| `rtps_packet_capture.py` | Optional `tcpdump` capture of one DDS domain's RTPS port block (DDSI-RTPS 2.5 §9.6.1.1). |
| `run_rmw_interop_loopback.sh` | Runs `alpha` on the RMW under test and `beta` on a stock RMW on one host; `beta` runs with `--expect-peer-graph`. |

## What a pass proves

Topics use RELIABLE + VOLATILE QoS, so a pass requires, in both directions:

1. SPDP participant discovery (DDSI-RTPS 2.5 §8.5.3).
2. SEDP endpoint matching, including topic-name and type-name mangling that
   matches stock ROS 2 (`rt/` prefix, `std_msgs::msg::dds_::String_`).
3. Reliable delivery: DATA, HEARTBEAT, and ACKNACK exchange (§8.4.7, §8.4.12).
4. CDR payload compatibility for `std_msgs/msg/String`, `std_srvs/srv/SetBool`,
   and `rcl_interfaces/srv/GetParameters`.
5. Service request/reply correlation (`PID_RELATED_SAMPLE_IDENTITY`): each echo
   reply must carry the value its own request sent.

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

## Serialization parity

```bash
tests/ros2/rmw_serialization_parity.py --reference rmw_fastrtps_cpp
```

Padding octets are not compared (CDR leaves their content unspecified), and
octets a stock RMW reports past the last field are ignored (Fast DDS reports
its size estimate as the serialized length). Every data octet must match.

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
| `ROSETTE_EXPECT_CALLS` | `3` | Minimum correlated calls per peer service, per side |
| `ROSETTE_CAPTURE` | `0` | `1` records a pcap from `alpha` (needs `tcpdump` privileges) |
| `ROSETTE_CAPTURE_IFACE` | `any` on Linux | Capture interface |

Validate the harness itself first by running the stock middleware against
itself; this must pass before a Rosette failure means anything:

```bash
ROSETTE_RMW=rmw_fastrtps_cpp tests/ros2/run_rmw_interop_loopback.sh
```

Service correlation is not standardized across DDS vendors: Fast DDS carries
it in inline QoS (the convention `rmw_rosette` follows), while
`rmw_cyclonedds_cpp` prefixes a request header to the payload, so stock Cyclone
and Fast DDS services do not interoperate either. Against a Cyclone peer, check
topics only:

```bash
ROSETTE_PEER_RMW=rmw_cyclonedds_cpp ROSETTE_EXPECT_CALLS=0 tests/ros2/run_rmw_interop_loopback.sh
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
