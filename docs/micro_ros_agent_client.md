# The micro-ROS Agent Backend

Rosette supports two ROS 2 backends on the same networking and storage layers
(see [architecture.md](architecture.md)); a firmware designer picks one per
project. This document covers the micro-ROS agent backend: `rmw_microxrcedds`
speaks XRCE-DDS to a `micro_ros_agent` on the network, and the agent joins the
DDS graph on the board's behalf.

| | `rmw_rosette` | micro-ROS agent |
| --- | --- | --- |
| On the wire | DDSI-RTPS, directly to every peer | XRCE-DDS to one agent; the agent speaks RTPS |
| Extra process | none | `micro_ros_agent` on the network |
| Application API | stock `rclc` | stock `rclc` |
| Middleware in the image | `rmw_rosette` + `rosette::rtps` | `rmw_microxrcedds` + `microxrcedds_client` (`libmicroros.a`) |
| Session setup | `SessionSupervisor` | `SessionSupervisor` + `AgentSessionGate` |
| Build switch | `ROSETTE_ROS2_C_STACK_DIR` | `ROSETTE_ENABLE_MICRO_ROS=ON` |

Application code is the same on both: an rclc `create_session()` that builds
support, node, entities and executor with default options, and a
`destroy_session()` that finalizes them. Only session setup differs.

## What Rosette adds

| Piece | Where | Does |
| --- | --- | --- |
| `UdpSocketTransport` | `rosette/ros/socket_transport.hpp` | the XRCE transport, over Rosette's `SocketProvider` (the WIZnet chip's hardware sockets on a board) |
| `AgentLocator`s | `rosette/ros/agent_locator*.hpp` | where the agent is: a fixed address, mDNS/DNS-SD, or the XRCE multicast probe |
| `AgentSessionGate` | `rosette/ros/agent_session_gate.hpp` | locates and pings the agent before a session is created; pings it while the session runs (three misses end it); syncs the epoch clock |
| `SessionSupervisor` | `rosette/ros/session_supervisor.hpp` | creates the session once the network is ready and the gate passes; rebuilds it when the address moves or the agent goes away |
| wall clock | `rosette/ros/time_sync.hpp` | `time_synced()`, `epoch_nanos()`, `time_stamp()` for message headers, read from `platform::WallClock`, which the agent's time exchange sets |

The XRCE client always uses the custom transport over Rosette sockets, on every
platform, so agent location and the socket path are the code the board runs.
The gate installs the transport as rmw_microxrcedds's default, which is why
`rclc_support_init()` needs no micro-ROS-specific options. `rosette_agent` is
the library to link.

```cpp
static rosette::UdpSocketTransport transport(sockets, any_v4, any_v4, 50u);
static rosette::MdnsAgentLocator locator(sockets);
static rosette::ros::AgentSessionGate gate(transport, locator, rosette::ros::rmw_agent_link());
rosette::ros::SessionSupervisor session(wiz::board_network_watch(),
                                        {create_session, destroy_session}, &gate);
for (;;) {
    if (session.step(platform::monotonic_ms()) == rosette::ros::SessionState::running) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(50));
    }
}
```

The gate's timing is `AgentGatePolicy`: a ping every 2 s with a 200 ms timeout,
three consecutive misses to declare the agent gone, and a clock resync every
60 s. The wall clock keeps its last offset across reconnects.

## Testing on a host against a real agent

The same stack runs on a host with only the socket backend swapped:
`tests/agent/agent_live_probe.cpp` is a stock rclc node on the agent backend
over Rosette's POSIX sockets, with the gate and the supervisor.
`tests/agent/run_agent_live.sh` starts a stock `micro_ros_agent` in Docker
and a stock rclpy echo peer, then checks three things: traffic and service
calls round-trip and the clock syncs to the agent host
(TC-ROS-AGT-LIVE-001); stopping the agent is detected within the ping budget
(TC-ROS-AGT-LIVE-002); and restarting it rebuilds the session
(TC-ROS-AGT-LIVE-003).

```bash
cmake -S . -B build-host-agent -DROSETTE_BOARD=posix -DROSETTE_ENABLE_MICRO_ROS=ON
cmake --build build-host-agent -j --target agent_live_probe
source /opt/ros/humble/setup.bash && tests/agent/run_agent_live.sh
```

## Example

`examples/led_files` runs on bare-metal WIZnet Pico boards: five rclc services
(an LED, and put/get/list/erase on a `FileStore`) over its own `rosette_msgs`
package.

```bash
cmake -S . -B build-hw/w6100-pico-agent -DROSETTE_BOARD=w6100-evb-pico \
      -DROSETTE_ENABLE_MICRO_ROS=ON [-DROSETTE_MICRO_ROS_AGENT_IP=192.168.1.10]
cmake --build build-hw/w6100-pico-agent -j --target led_files
tools/pico_uf2_flash.py build-hw/w6100-pico-agent/examples/led_files/led_files.uf2 --monitor 30

# the agent
docker run --rm --net=host microros/micro-ros-agent:humble udp4 --port 8888
```

The first build runs micro_ros_setup's `generate_lib` flow to produce
`libmicroros.a` (20–30 minutes, once per workspace). To reuse a built
workspace, pass `-DROSETTE_MICRO_ROS_WORKSPACE_DIR=<dir>
-DROSETTE_MICRO_ROS_BOOTSTRAP_ON_BUILD=OFF`.

Then, from a host with the example's `rosette_msgs` package built in an
overlay:

```bash
ros2 service call /led_files/led/set rosette_msgs/srv/SetLed "{led: 0, state: true}"
ros2 service call /led_files/files/put rosette_msgs/srv/PutFile "{key: k1, body: hello}"
ros2 service call /led_files/files/list rosette_msgs/srv/ListFiles "{}"
```

Interface packages under `examples/*/` are compiled into `libmicroros`
automatically; `ROSETTE_MICRO_ROS_EXTRA_INTERFACE_PACKAGES` adds others.

## Finding the agent

The gate asks its `AgentLocator` whenever it holds no answering endpoint, so
an agent that moves is found again. In `led_files`, `ROSETTE_MICRO_ROS_AGENT_IP`
selects `StaticAgentLocator`; without it, the board discovers the agent by
the mechanism `ROSETTE_MICRO_ROS_DISCOVERY` selects:

**`mdns`** (default): `MdnsAgentLocator` (`rosette/ros/agent_locator_mdns.hpp`)
browses the DNS-SD service `_microrosagent._udp.local`, or resolves the A
record of `ROSETTE_MICRO_ROS_MDNS_HOSTNAME` when set. It connects to the
**source address of the mDNS reply**, which is routable from the board by
construction, so an agent inside a Docker bridge is still reachable. Loopback,
link-local, and `0.0.0.0` answers are rejected. The resolver sends one-shot
queries from an ephemeral port with the RFC 6762 unicast-response bit set, so
it coexists with a host mDNS daemon. Advertise the agent with:

```bash
tools/micro_ros/advertise_agent_mdns.sh 8888
```

**`xrce`**: `MulticastAgentLocator` (`rosette/ros/agent_locator_xrce.hpp`)
sends the upstream Micro XRCE-DDS `GET_INFO` probe on `239.255.0.2:7400`. The
reply reports whatever address the agent was configured with, which may not be
routable from the board. A host-side responder is in
`tools/micro_ros/agent_multicast_responder.py`.

| Setting | Default |
| --- | --- |
| `ROSETTE_MICRO_ROS_AGENT_IP` / `_AGENT_PORT` | empty / `8888` |
| `ROSETTE_MICRO_ROS_DISCOVERY` | `mdns` |
| `ROSETTE_MICRO_ROS_MDNS_SERVICE` | `_microrosagent._udp.local` |
| `ROSETTE_MICRO_ROS_MDNS_HOSTNAME` | empty (browse the service) |
| `ROSETTE_MICRO_ROS_MDNS_TIMEOUT_MS` / `_ATTEMPTS` | `1000` / `3` |
| `ROSETTE_MICRO_ROS_DISCOVERY_GROUP` / `_PORT` | `239.255.0.2` / `7400` |
| `ROSETTE_MICRO_ROS_DISCOVERY_TIMEOUT_MS` / `_ATTEMPTS` | `750` / `3` |

## micro-ROS library limits

`libmicroros.a` has its own static limits, set at bootstrap
(`cmake/micro_ros.cmake`): `ROSETTE_MICRO_ROS_MAX_NODES` (2),
`_MAX_PUBLISHERS` (10), `_MAX_SUBSCRIPTIONS` (10), `_MAX_SERVICES` (8),
`_MAX_CLIENTS` (6), `_MAX_HISTORY` (8), and the XRCE buffers
`_MAX_INPUT_BUFFER_SIZE` / `_MAX_OUTPUT_BUFFER_SIZE` (8192),
`_MAX_TRANSPORT_MTU` (1400), `_STREAM_HISTORY` (8). Changing any of them
rebuilds the library.

## References

- micro-ROS: [Micro XRCE-DDS](https://micro-xrce-dds.docs.eprosima.com/),
  [micro_ros_setup](https://github.com/micro-ROS/micro_ros_setup) `generate_lib`,
  the "Handle reconnections and liveliness" and "Custom transports" tutorials
- OMG DDS-XRCE 1.0 (the agent protocol)
- RFC 6762 (mDNS), RFC 6763 (DNS-SD)
