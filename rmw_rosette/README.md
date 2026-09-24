# rmw_rosette

An `rmw` implementation over `rosette::rtps` (`include/rosette/rtps`,
`src/rtps` at the repo root). Stock `rcl`, `rclc`, `rclcpp`, and `rclpy` load it
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
| Endpoints created after discovery are announced; destroyed ones are withdrawn | TC-SEDP-8.5.4-007/008/009, TC-PART-8.5.1-005, live check against Fast DDS |
| CDR for any installed message type, byte-compatible with Fast DDS | `tests/ros2/rmw_serialization_parity.py`, `test/test_introspection_c_codec.cpp` |
| Guard conditions, wait sets, matched counts, actual QoS | `test/test_guard_condition.cpp`, `test/test_pub_sub_loopback.cpp` |

### Serialization

`message_codec.hpp` resolves each type's serializer once per endpoint:
`rosidl_typesupport_microxrcedds_c` callbacks when a micro-ROS workspace
provides them, otherwise `rosidl_typesupport_introspection_c` metadata walked
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

### Capacity

Nothing is allocated at runtime. `cmake/rmw_rosette_capacity.cmake` derives
every pool, RTPS table, SEDP history, and buffer-pool size from a few limits
(`RMW_ROSETTE_MAX_NODES`, `_PUBLISHERS`, `_SUBSCRIPTIONS`, `_SERVICES`,
`_CLIENTS`, `_REMOTE_PARTICIPANTS`, `_REMOTE_ENDPOINTS`, `_MAX_DATAGRAM_SIZE`)
plus the entities client libraries create per node
(`RMW_ROSETTE_PARAMETER_SERVICES_PER_NODE`, default 6; set 0 for `rclc`
without a parameter server). `include/rmw_rosette/capacity.hpp` static_asserts
the same relationships.

## Known gaps

1. **Concurrency and platform layer.** `rmw_wait()` drives the participant,
   and `rmw_publish()` from another thread (`rclcpp` `MultiThreadedExecutor`,
   a second core) races it. This lands with the build-time platform ports
   (one contract header; OS layer bare-metal/FreeRTOS/POSIX; chip layer
   RP2xxx/host), which also provide the mutex and the single monotonic clock
   that RTPS timers and time sync share.
2. **Large messages.** Publishing serializes into one `RTPS_BUF_BLOCK_SIZE`
   (2 KiB) buffer, so larger messages (images, point clouds) fail to publish;
   sending needs DATA_FRAG, and received payloads larger than one block need
   `fragment_assembler.hpp` wired into the reader path.
3. **Graph introspection.** `rmw_count_*`, `rmw_get_*_names_and_types*`, and
   `rmw_get_node_names*` need a `GraphCache` fed by SEDP and the
   `ros_discovery_info` topic (`rmw_dds_common/msg/ParticipantEntitiesInfo`)
   that carries node identity; `ros2 node list` and `count_publishers()` on a
   rosette node report unsupported until then.
4. **QoS events and listeners.** Event initialization reports
   `RMW_RET_UNSUPPORTED`, which both client libraries accept.
5. **Board transports.** `rmw_init.cpp` uses POSIX sockets; FreeRTOS+TCP and
   WIZnet targets take the socket provider from board configuration.

## Verification

| Test | Scope |
| --- | --- |
| `test/test_introspection_c_codec.cpp` | type resolution, DDS type names, golden bytes, round trips, malformed-input rejection |
| `test/test_guard_condition.cpp` | trigger semantics and cross-thread wakeup of `rmw_wait()` |
| `test/test_pub_sub_loopback.cpp` | publish → wait → take through one participant over UDP |
| `tests/spec` (`TC-SEDP-*`, `TC-PART-8.5.1-005`, `TC-SPDP-8.5.3-005/006`, `TC-SWR-8.4.3-002`) | endpoint lifecycle, disposals, SPDP numbering, preemptive ACKNACK |
| `tests/ros2/rmw_serialization_parity.py` | CDR layout and cross-decoding against a live stock RMW |
| `tests/ros2/run_rmw_interop_loopback.sh` | acceptance: stock `rclpy` default nodes on both sides, topics and services |
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
self-contained. For micro-ROS firmware, add this package to the workspace
`.repos` alongside `rcl`/`rclc` and select it as the RMW; the board supplies its
socket provider (gap 5), and the capacity limits are set for the board.
