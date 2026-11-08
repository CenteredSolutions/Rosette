# rmw_rosette

An `rmw` implementation over `rosette::rtps` (`include/rosette/rtps`,
`src/rtps` at the repo root). Stock `rcl`, `rclc`, and `rclpy` load it
with `RMW_IMPLEMENTATION=rmw_rosette` and join the ROS 2 graph directly as an
RTPS participant, with no `micro_ros_agent` in the loop.

## Status

Verified against stock ROS 2 Humble (`rmw_fastrtps_cpp`) on one host:

| Capability | Evidence |
| --- | --- |
| `rmw_implementation` loads the library (full Humble rmw ABI exported) | stock `rclpy.init()` reports `rmw_rosette` |
| Default stock nodes, including their parameter services | `tests/ros2/run_rmw_interop_loopback.sh` |
| Reliable pub/sub with Fast DDS, both directions | same |
| Services and clients with Fast DDS, both directions, correlated per call | same (`std_srvs/SetBool` echo and `get_parameters`) |
| Stock `ros2 node list`, `ros2 node info`, and `*_by_node` queries show rosette nodes with their publishers, subscriptions, services, and clients, updated as they are created and destroyed | `tests/ros2/run_rmw_interop_loopback.sh` (stock side lists alpha's node and its echo service), `test/test_local_node_entities.cpp` |
| Endpoints created after discovery are announced; destroyed ones are withdrawn | TC-SEDP-8.5.4-007/008/009, TC-PART-8.5.1-005, live check against Fast DDS |
| CDR for any installed message type, byte-compatible with Fast DDS | `tests/ros2/rmw_serialization_parity.py`, `test/test_introspection_c_codec.cpp` |
| Guard conditions, wait sets, matched counts, actual QoS | `test/test_guard_condition.cpp`, `test/test_pub_sub_loopback.cpp` |
| Unconfigured stock Fast DDS through a link that drops datagrams over 1472 octets: discovery, topics and services both ways; Rosette's messages up to 64 KB | `tests/ros2/run_rmw_frag_loopback.sh` (default `stock` mode) |
| Messages up to 64 KB both ways when the peer caps `maxMessageSize` at the link limit | `ROSETTE_PEER_MODE=capped tests/ros2/run_rmw_frag_loopback.sh` |
| A stock `rclc` node on an RP2350 WIZnet board, topics both ways with stock Fast DDS; rcl heap steady at about 4 KB | `examples/rclc_chatter` on the W5100S-EVB-Pico2 |

### Serialization

`message_codec.hpp` resolves each type's serializer once per endpoint:
`rosidl_typesupport_microxrcedds_c` callbacks when a micro-ROS workspace
provides them, otherwise `rosidl_typesupport_introspection_c` or `_cpp` metadata walked
by `introspection_c_codec.cpp`, which every ROS 2 installation generates (the
approach `rmw_cyclonedds_cpp` takes). Both emit XCDR1 identical to Fast DDS.
The microxrcedds path compiles only in a micro-ROS workspace and has not been
built on this branch.

### Services

A service is a reader on `rq/<service>Request` and a writer on
`rr/<service>Reply` (`service_endpoints.hpp`). Requests and replies are
correlated through `PID_RELATED_SAMPLE_IDENTITY`, written under both the
DDS-RPC PID (0x0083) and the eProsima PID (0x800F) that Fast DDS 2.6 reads. A
server's reply writer is TRANSIENT_LOCAL so a reply written before the calling
client's reader has matched still arrives; clients keep only replies that name
their request writer or reply reader.

### Choosing the network interface

A participant joins and sends SPDP multicast on one interface and advertises
that interface's address as its unicast locators, so a peer that hears it can
answer. `ROSETTE_INTERFACE` names the interface, by name or by an address it
holds:

```sh
ROSETTE_INTERFACE=eth0 RMW_IMPLEMENTATION=rmw_rosette \
  ros2 topic pub /rosette/command std_msgs/msg/String "{data: hi}"
ROSETTE_INTERFACE=10.42.0.1 RMW_IMPLEMENTATION=rmw_rosette \
  ros2 topic pub /rosette/command std_msgs/msg/String "{data: hi}"
```

Unset, it is the interface of the default route, and on a host with more
than one network the choice is printed once on stderr with the alternatives.
A bench host whose default route is Wi-Fi while the boards are on Ethernet
needs the setting. A name that matches no interface, an address no interface
holds, an interface that is down, or a list of several is reported on stderr
and `rmw_init()` fails; the process never falls back to another interface.

### Large messages and small links

Samples are serialized once into a buffer-pool block and never copied
again. A sample whose DATA would exceed the transport's datagram limit
(`RMW_ROSETTE_MAX_DATAGRAM_SIZE`, lowered at runtime by the environment
variable of the same name) goes out as DATA_FRAG, one fragment per datagram,
with inline QoS in every fragment. Received fragments reassemble into a pool
block. `RMW_ROSETTE_TRANSPORT_STATS=1` prints the datagram counters
(largest sent/received, oversize, truncated, dropped) at shutdown.

A link without IP reassembly (WIZnet hardware UDP, 1472 octets) silently
loses any datagram above its limit. Stock peers, which send up to 64 KB per
datagram, work over it unmodified:

- Discovery and repairs: a stock writer answers an ACKNACK by packing every
  requested change into one datagram, which such a link would lose on every
  retry. On a limited transport every Rosette reader, SEDP included, requests
  one missing change per ACKNACK and asks for the next as soon as it lands
  (`repairs_must_fit_datagram()`), so each repair is a single DATA. Readers
  track holes per writer (`SequenceNumberWindow`) and honor GAP and HEARTBEAT
  firstSN, so lost or reordered announcements are always recovered.
- Samples: Rosette fragments to the limit, so it can send anything. A stock
  peer sends a sample larger than the link limit as one IP-fragmented
  datagram, and RTPS has no parameter by which a reader can ask for smaller
  ones, so such samples cannot reach a board over this link. Either the peer
  caps its datagrams (Fast DDS `maxMessageSize`), or the board uses an IP
  stack that reassembles (lwIP over MACRAW).

`RMW_ROSETTE_STRICT_LINK=1` makes a host transport drop every received
datagram over the limit, the way such a link does, so the host harness faces
a board's losses against real stock traffic.

### Capacity

Nothing is allocated at runtime. `cmake/rmw_rosette_capacity.cmake` derives
every pool, RTPS table, SEDP history, and buffer-pool size from a few limits
(`RMW_ROSETTE_MAX_NODES`, `_PUBLISHERS`, `_SUBSCRIPTIONS`, `_SERVICES`,
`_CLIENTS`, `_REMOTE_PARTICIPANTS`, `_REMOTE_ENDPOINTS`, `_MAX_DATAGRAM_SIZE`,
`_MAX_SAMPLE_SIZE`, `_LARGE_BUFFERS`)
plus the entities client libraries create per node
(`RMW_ROSETTE_PARAMETER_SERVICES_PER_NODE`, default 6; set 0 for `rclc`
without a parameter server). `include/rmw_rosette/capacity.hpp` static_asserts
the same relationships.

## Known gaps

1. **Storage.** Flash access still lives in the Pico runtime files; it
   becomes a `storage::BlockDevice` (host file, RP2 flash via
   `flash_safe_execute()`) that LittleFS takes by reference.
2. **Graph introspection on rosette nodes.** Each context publishes its nodes
   on `ros_discovery_info` (`rmw_dds_common/msg/ParticipantEntitiesInfo`,
   `ros_discovery_info_writer.hpp`), so stock peers see them. Rosette nodes
   see the graph through `rmw_count_publishers`/`rmw_count_subscribers`
   (answered by the participant's `EndpointMatchTable`) and the context's one
   graph guard condition, which discovery triggers. `rmw_get_node_names*` and
   the `*_by_node` queries need other participants' `ros_discovery_info`, which
   nothing reads yet, so they report `RMW_RET_UNSUPPORTED`.
   `rmw_get_topic_names_and_types` and `rmw_get_service_names_and_types` need
   remote endpoint names, which the match table keeps only as keys: a hosted
   build keeps them on the heap in a `GraphCache` (`RMW_ROSETTE_GRAPH_CACHE`,
   on for POSIX), and firmware leaves it out and reports `RMW_RET_UNSUPPORTED`.
3. **QoS events and listeners.** Event initialization reports
   `RMW_RET_UNSUPPORTED`, which both client libraries accept.
4. **C++ type support on boards.** `rclcpp`'s C++ message structs are encoded
   from `rosidl_typesupport_introspection_cpp` metadata on hosted builds
   (`RMW_ROSETTE_CXX_TYPESUPPORT`, on by default for POSIX). A firmware build
   leaves that codec out, because C++ message containers allocate from the heap
   on every decoded sample; firmware runs stock `rclc` over C message structs.
5. **Sample size.** A sample larger than `RMW_ROSETTE_MAX_SAMPLE_SIZE` is refused
   on publish, and evicted on receipt before a reader takes it. On a host,
   samples above the largest pool block (65,532 bytes) live on the heap
   (`RMW_ROSETTE_HEAP_SAMPLES`), bounded by that limit and
   `RMW_ROSETTE_HEAP_BUDGET`. A sample that would need more fragments than a
   receiver can track (256) is refused on publish. Firmware has no heap path.
6. **Reported QoS.** The actual-QoS queries report the requested policies:
   system defaults are not resolved, depth is capped by the history pool, and
   durability is reported as requested. `ignore_local_publications` is not
   applied.
7. **FreeRTOS+TCP board network.** The network port forwards to the board
   network (`rosette/net/board_network.hpp`), which hosts and WIZnet boards
   implement; FreeRTOS+TCP boards do not yet.

## Verification

| Test | Scope |
| --- | --- |
| `test/test_introspection_c_codec.cpp` | type resolution, DDS type names, golden bytes, round trips, malformed-input rejection |
| `test/test_local_node_entities.cpp` | `ParticipantEntitiesInfo` encoding, decoded through `rmw_dds_common`'s own type support; node and endpoint removal |
| `test/test_guard_condition.cpp` | trigger semantics and cross-thread wakeup of `rmw_wait()` |
| `test/test_init_options.cpp` | init options own their enclave and security root: copy duplicates them, fini frees them (no leak per `rcl_init()`), fini refuses another rmw's options, copy survives allocation failure |
| `test/test_pub_sub_loopback.cpp` | publish → wait → take through one participant over UDP |
| `tests/spec` (`TC-SEDP-*`, `TC-PART-8.5.1-005`, `TC-SPDP-8.5.3-005/006`, `TC-SWR-8.4.3-002`) | endpoint lifecycle, disposals, SPDP numbering, preemptive ACKNACK |
| `tests/ros2/rmw_serialization_parity.py` | CDR layout and cross-decoding against a live stock RMW |
| `tests/ros2/run_rmw_interop_loopback.sh` | acceptance: stock `rclpy` default nodes on both sides, topics and services, and the stock side's view of rosette's node |
| `tests/spec`, `tests/loopback` | the RTPS core this package is built on |

Build and test in a colcon workspace:

```bash
mkdir -p ~/rosette_ws/src && ln -s "$PWD/rmw_rosette" ~/rosette_ws/src/
cd ~/rosette_ws && source /opt/ros/humble/setup.bash
colcon build --packages-select rmw_rosette
colcon test --packages-select rmw_rosette && colcon test-result --verbose
```

`-DRMW_ROSETTE_RTPS_LOG_LEVEL=4` prints per-message RTPS traffic (0 none,
1 error, 2 warning, 3 info, 4 debug, 5 trace) to stderr, which is the fastest
way to see discovery and reliability exchanges without packet-capture rights.

## Build integration

The package compiles the RTPS core and its POSIX platform glue from the
repository root (`ROSETTE_ROOT`, defaulting to `..`), so `librmw_rosette.so` is
self-contained.

### Firmware

A firmware image links stock `rcl`, `rclc`, and the message type supports as
static libraries, cross-compiled once by micro_ros_setup's `generate_lib` flow,
and compiles `rmw_rosette` itself with the board's toolchain
(`cmake/rmw_rosette_firmware.cmake` at the repository root). Only the
per-package libraries from `mcu_ws/install/lib` are used; the micro-ROS
middleware (`rmw_microxrcedds`, `microxrcedds_client`) is never linked, and
rcl calls `rmw_rosette` directly. Rebuilding after an `rmw_rosette` or RTPS
change compiles only those sources.

The network port (`include/rmw_rosette/network_port.hpp`) is the socket
provider and IPv4 address `rmw_init()` uses. `src/network_port.cpp` forwards
both to the board network (`rosette/net/board_network.hpp`): BSD sockets and
the host's address on a host, the chip's sockets and its assigned address on a
WIZnet board, where the firmware calls `bring_up_board_network()` before
`rcl_init()`.
Firmware capacity defaults to one rclc node with no parameter server and
`RMW_ROSETTE_POOL_PROFILE=firmware`.

```bash
cmake -S . -B build-hw/pico2 -DROSETTE_BOARD=w5100s-evb-pico2 \
      -DROSETTE_MICRO_ROS_BOOTSTRAP_ON_BUILD=OFF \
      -DROSETTE_ROS2_C_STACK_DIR=<micro-ROS workspace>/firmware/mcu_ws/install
cmake --build build-hw/pico2 --target rclc_chatter
```
