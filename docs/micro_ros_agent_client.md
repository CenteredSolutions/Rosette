# micro-ROS Agent Client Examples

Rosette's primary path puts a board on the ROS 2 network directly, as a DDS
participant running `rmw_rosette` (see [architecture.md](architecture.md)).
The repository also carries a second, agent-based path: C++ wrappers over
`rclc` and `rmw_microxrcedds` (`include/rosette/ros/`), where the board speaks
XRCE-DDS to a `micro_ros_agent` on a host and the agent joins the DDS graph on
its behalf. This document covers that path and its two examples.

| | `rmw_rosette` (primary) | micro-ROS agent client |
| --- | --- | --- |
| On the wire | DDSI-RTPS, directly to every peer | XRCE-DDS to one agent; the agent speaks RTPS |
| Extra process | none | `micro_ros_agent` on a host |
| Application API | stock `rclc` | `rosette::App` / `rosette::Module` wrappers over `rclc` |
| Middleware in the image | `rmw_rosette` + `rosette::rtps` | `rmw_microxrcedds` + `microxrcedds_client` (`libmicroros.a`) |
| Build switch | `ROSETTE_ROS2_C_STACK_DIR` | `ROSETTE_ENABLE_MICRO_ROS=ON` |

## Examples

| Example | Runs on | Shows |
| --- | --- | --- |
| `examples/rosette_smoke` | host (`ROSETTE_BOARD=posix`) | the smallest client: publish `std_msgs/Float32` on `/mcu_temperature_c` over the stock XRCE UDP transport |
| `examples/led_files` | host (verified against a live agent); builds for the W5100S-EVB-Pico2 | `rosette::App` with two `rosette::Module`s: an LED service and a `FileStore`-backed file service over its own `rosette_msgs` package |

`examples/board_smoke` is not part of the build (see its README).

### Host

```bash
cmake -S . -B build/micro-ros-host -DROSETTE_BOARD=posix -DROSETTE_ENABLE_MICRO_ROS=ON
cmake --build build/micro-ros-host -j --target rosette_smoke led_files

# terminal 1
source /opt/ros/humble/setup.bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888

# terminal 2
ROSETTE_AGENT_IP=127.0.0.1 build/micro-ros-host/examples/rosette_smoke/rosette_smoke

# terminal 3
source /opt/ros/humble/setup.bash
ros2 topic echo /mcu_temperature_c std_msgs/msg/Float32
```

The first build runs micro_ros_setup's `generate_lib` flow to produce
`libmicroros.a` (20–30 minutes, once per workspace). To reuse a built
workspace, pass `-DROSETTE_MICRO_ROS_WORKSPACE_DIR=<dir>
-DROSETTE_MICRO_ROS_BOOTSTRAP_ON_BUILD=OFF`.

### Board

```bash
cmake -S . -B build/micro-ros-pico2 -DROSETTE_BOARD=w5100s-evb-pico2 \
      -DROSETTE_ENABLE_MICRO_ROS=ON -DROSETTE_MICRO_ROS_AGENT_IP=192.168.1.10
cmake --build build/micro-ros-pico2 -j --target led_files
tools/pico_uf2_flash.py build/micro-ros-pico2/examples/led_files/led_files.uf2 --monitor 30
```

Then, from a host with the example's `rosette_msgs` package built in an
overlay:

```bash
ros2 service call /led_files/led/set rosette_msgs/srv/SetLed "{led: 0, state: true}"
ros2 service call /led_files/files/list rosette_msgs/srv/ListFiles "{}"
```

Interface packages under `examples/*/` are compiled into `libmicroros`
automatically; `ROSETTE_MICRO_ROS_EXTRA_INTERFACE_PACKAGES` adds others.

## The wrapper API

`rosette::App` owns the agent session. It resolves and pings the agent, builds
the context and node, asks every registered `Module` to create its entities,
spins the executor, detects agent loss by ping, tears everything down, and
rebuilds when the agent returns. The executor is sized from the sum of each
module's `executor_handles()`.

```cpp
class LedModule final : public rosette::Module {
public:
    using Request = rosette::service_traits<rosette::SetLedSrv>::Request;
    using Response = rosette::service_traits<rosette::SetLedSrv>::Response;

    const char* name() const noexcept override { return "led"; }
    uint8_t executor_handles() const noexcept override { return 1u; }

    void register_(rosette::Node& node) noexcept override {
        node.create_service<rosette::SetLedSrv>(
            service_, "led/set",
            [](const Request& req, Response& resp) noexcept {
                gpio_put(22, req.state);
                resp.success = true;
            });
    }

    void on_disconnect() noexcept override { service_.reset(); }

private:
    std::optional<rosette::Service<rosette::SetLedSrv>> service_;  // rebuilt on reconnect
};

int main() {
    rosette::AppOptions options{};
    options.node_name = "board";
    options.node_namespace = "led_files";   // entities land under /led_files/...

    rosette::App app(options);
    static LedModule led;
    app.add(led);
    app.spin();   // never returns; or call app.spin_once(now_ms) from your own loop
}
```

Each generated type is bound to the wrappers through a
`rosette::message_traits<T>` or `rosette::service_traits<T>` specialization
that supplies its `rosidl` type support and `init`/`fini` hooks; see
`examples/led_files/rosette_msgs_traits.hpp`.

Actions (`rosette::ActionServer`) run cooperatively from `App`'s spin: one
active goal per server, advanced on each spin. Action entities must live as
long as their `App`, because `rclc` has no executor remove call for them.

## Finding the agent

With `ROSETTE_MICRO_ROS_AGENT_IP` set, the board uses that address. Without
it, the board discovers the agent, by the mechanism
`ROSETTE_MICRO_ROS_DISCOVERY` selects:

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
  [micro_ros_setup](https://github.com/micro-ROS/micro_ros_setup) `generate_lib`
- OMG DDS-XRCE 1.0 (the agent protocol)
- RFC 6762 (mDNS), RFC 6763 (DNS-SD)
