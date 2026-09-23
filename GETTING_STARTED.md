# Getting Started With Rosette

This guide gets you from a clean checkout to three useful outcomes:

1. Compile an example for a selected board.
2. Start a project that brings up networking, creates a node, and publishes a
   metric.
3. Run the host test suites.

## 1. Choose A Board

Rosette uses one CMake option for board selection:

```bash
-DRTPS_BOARD=<board>
```

Valid board names are:

- `posix`: host/POSIX build, useful for tests and host experiments.
- `w5100s-evb-pico`: WIZnet W5100S-EVB-Pico, RP2040.
- `w5100s-evb-pico2`: WIZnet W5100S-EVB-Pico2, RP2350.
- `bc-gen2`: custom RP2040 + W5100S board.

The board file controls the processor, Pico SDK board name, WIZnet wiring, BSP
sources, and whether the firmware example is built.

## 2. Compile The Minimal Firmware Example

For the Pico2 EVB:

```bash
cmake -S . -B build/w5100s-evb-pico2 -DRTPS_BOARD=w5100s-evb-pico2
cmake --build build/w5100s-evb-pico2 -j --target rtps_example
```

For the RP2040 EVB:

```bash
cmake -S . -B build/w5100s-evb-pico -DRTPS_BOARD=w5100s-evb-pico
cmake --build build/w5100s-evb-pico -j --target rtps_example
```

For the custom `bc-gen2` board:

```bash
cmake -S . -B build/bc-gen2 -DRTPS_BOARD=bc-gen2
cmake --build build/bc-gen2 -j --target rtps_example
```

The `rtps_example` target is embedded-only. A `posix` top-level configure builds
the reusable stack library path, but does not create `rtps_example`.

### Dependency Inputs

CMake can fetch dependencies through `FetchContent`. For locked-down networks,
CI, or reproducible local development, point CMake at local checkouts:

```bash
PICO_SDK_PATH=/path/to/pico-sdk \
FREERTOS_KERNEL_PATH=/path/to/FreeRTOS-Kernel \
FREERTOS_TCP_PATH=/path/to/FreeRTOS-Plus-TCP \
WIZNET_IOLIB_PATH=/path/to/ioLibrary_Driver \
LITTLEFS_PATH=/path/to/littlefs \
cmake -S . -B build/w5100s-evb-pico2 -DRTPS_BOARD=w5100s-evb-pico2
```

## 3. Start A New Embedded Project

The fastest starting point is `examples/main_rp2350.cpp`. Despite the filename,
the board selection determines whether the build targets RP2040 or RP2350.

### Project Skeleton

A minimal native Rosette firmware follows this control flow:

1. Initialize board hardware with `board_init()`.
2. Start FreeRTOS+TCP and wait for an IPv4 or IPv6 address.
3. Create `rosette::net::freertos::FreeRtosSocketProvider`.
4. Create `rtps::SocketUdpTransport` with the RTPS metatraffic and user ports.
5. Create `rtps::SocketTransportRuntime`.
6. Create a `milkros::Context`, start the runtime, then start the participant.
7. Create a node and publisher.
8. Call `context.tick(now_ms)` regularly.
9. Publish your metric on a timer.

The example does exactly that in `service_task()`.

### Bring Up Networking

For WIZnet boards, copy the network setup pattern from
`examples/main_rp2350.cpp`:

```cpp
uint8_t mac[6]{};
board_get_mac_address(mac);
wiznet_macraw_set_mac_address(mac);

FreeRTOS_FillEndPoint(
    &g_network_interface,
    &g_ipv4_endpoint,
    ip_address.data(),
    netmask.data(),
    gateway.data(),
    dns.data(),
    mac);

g_ipv4_endpoint.bits.bWantDHCP = pdTRUE_UNSIGNED;

if (FreeRTOS_IPInit_Multi() != pdPASS) {
    // Stop startup and report a fatal network error.
}
```

Then wait until `rtps::rtps_platform_get_ipv4_addr()` returns nonzero, or until
`rtps::rtps_platform_get_ipv6_addr()` reports an address.

Use source-level compile definitions when you want static or zeroconf-style
addressing. Today these are consumed by the example source, so define them in
your project target, board profile, or local application CMake:

```cmake
target_compile_definitions(your_firmware_target PRIVATE
    RTPS_NET_MODE=0
    RTPS_NET_STATIC_IP={192u,168u,1u,200u}
    RTPS_NET_STATIC_MASK={255u,255u,255u,0u}
    RTPS_NET_STATIC_GW={192u,168u,1u,1u}
    RTPS_NET_STATIC_DNS={8u,8u,8u,8u}
)
```

The network mode constants used by the example are:

- `0`: static IPv4.
- `1`: DHCP.
- `2`: IPv6 SLAAC.
- `3`: zeroconf-style IPv4 fallback.

### Start The RTPS Node

After the network is ready, create the transport and context:

```cpp
static rosette::net::freertos::FreeRtosSocketProvider socket_provider;

const uint16_t meta_port =
    static_cast<uint16_t>(rtps::spdp_multicast_port(kDomainId));
const uint16_t user_port =
    static_cast<uint16_t>(rtps::spdp_unicast_port(kDomainId, kParticipantId));

static rtps::SocketUdpTransport transport(
    socket_provider,
    meta_port,
    user_port,
    rtps::spdp_multicast_locator(kDomainId));

static rtps::SocketTransportRuntime runtime(transport);
static DemoContext context(transport, runtime);

if (!context.start_runtime()) {
    // Transport could not bind or join discovery multicast.
}

update_participant_addresses(context.participant());

if (!context.start()) {
    // Participant startup failed.
}
```

`DemoContext` in the example subclasses `milkros::Context` so every tick spins
the transport runtime, RTPS participant, and timers together.

### Publish A Metric

Define or generate a message type, provide `milkros::message_traits`, then
publish from your service loop.

For a compact string metric:

```cpp
struct DeviceMetricMsg {
    char data[96];
};

auto node = context.create_node("rosette_device", "");
auto metric_pub = node.create_publisher<DeviceMetricMsg>("device_metric");

uint32_t sequence = 0;
uint32_t last_publish_ms = 0;

for (;;) {
    const uint32_t now_ms = rtps_platform_get_ms();
    context.tick(now_ms);

    if ((now_ms - last_publish_ms) >= 1000u) {
        DeviceMetricMsg msg{};
        std::snprintf(msg.data,
                      sizeof(msg.data),
                      "uptime_ms=%lu seq=%lu",
                      static_cast<unsigned long>(now_ms),
                      static_cast<unsigned long>(sequence++));
        (void)metric_pub.publish(msg);
        last_publish_ms = now_ms;
    }

    vTaskDelay(pdMS_TO_TICKS(10u));
}
```

Use `examples/main_rp2350.cpp` as the reference for the required
`message_traits<DeviceMetricMsg>` specialization. That specialization gives the
DDS type name and CDR encode/decode procedures for your message.

## 4. Start A micro-ROS Project

Use the micro-ROS path when you want to connect through a micro-ROS Agent rather
than the native RTPS participant flow.

Build the board smoke firmware:

```bash
cmake -S . -B build/micro-ros-pico2 \
  -DRTPS_BOARD=w5100s-evb-pico2 \
  -DROSETTE_ENABLE_MICRO_ROS=ON
cmake --build build/micro-ros-pico2 -j --target rtps_micro_ros_board_smoke
```

Run the host Agent in one terminal:

```bash
source /opt/ros/humble/setup.bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

By default the board finds the Agent over mDNS / DNS-SD. Advertise the Agent on
the same host (or its Docker host) in a second terminal and leave it running:

```bash
tools/micro_ros/advertise_agent_mdns.sh 8888
```

This registers the `_microrosagent._udp.local` service with Avahi. The board
browses that service and connects to the **source address of the mDNS reply**,
so the Agent is reachable even when it runs behind a Docker bridge and would
otherwise advertise an unreachable address.

To find the Agent by host name instead of by service, build with
`-DROSETTE_MICRO_ROS_MDNS_HOSTNAME=myhost` (the board resolves `myhost.local`)
and skip the script — any mDNS responder for that host, including a plain Linux
box running Avahi, is enough.

The legacy XRCE `GET_INFO` locator is still available with
`-DROSETTE_MICRO_ROS_DISCOVERY=xrce`, paired with its responder:

```bash
python3 tools/micro_ros/agent_multicast_responder.py --agent-port 8888
```

After flashing the board, observe the smoke topic from a ROS 2 terminal:

```bash
source /opt/ros/humble/setup.bash
ros2 topic echo /mcu_temperature_c std_msgs/msg/Float32
```

With a known agent IP:

```bash
cmake -S . -B build/micro-ros-pico2 \
  -DRTPS_BOARD=w5100s-evb-pico2 \
  -DROSETTE_ENABLE_MICRO_ROS=ON \
  -DROSETTE_MICRO_ROS_AGENT_IP=192.168.1.50 \
  -DROSETTE_MICRO_ROS_AGENT_PORT=8888
```

Without an agent IP, the smoke target autodiscovers the Agent. The mechanism is
chosen by `ROSETTE_MICRO_ROS_DISCOVERY` (default `mdns`, or `xrce`).

mDNS locator settings (`ROSETTE_MICRO_ROS_DISCOVERY=mdns`):

- `ROSETTE_MICRO_ROS_MDNS_SERVICE`, default `_microrosagent._udp.local`.
- `ROSETTE_MICRO_ROS_MDNS_HOSTNAME`, default empty; when set, the board
  resolves this host's A record instead of browsing the service.
- `ROSETTE_MICRO_ROS_MDNS_TIMEOUT_MS`, default `1000`.
- `ROSETTE_MICRO_ROS_MDNS_ATTEMPTS`, default `3`.

XRCE locator settings (`ROSETTE_MICRO_ROS_DISCOVERY=xrce`):

- `ROSETTE_MICRO_ROS_DISCOVERY_GROUP`, default `239.255.0.2`.
- `ROSETTE_MICRO_ROS_DISCOVERY_PORT`, default `7400`.
- `ROSETTE_MICRO_ROS_DISCOVERY_TIMEOUT_MS`, default `750`.
- `ROSETTE_MICRO_ROS_DISCOVERY_ATTEMPTS`, default `3`.

The smoke app creates a node named `rosette_micro_ros_smoke` and publishes
`std_msgs/msg/Float32` on `mcu_temperature_c`.

`Context::start_result()` returns `ContextStartResult`, a
`Result<void, ContextStartError>` that lets production firmware distinguish
network setup, Agent discovery, and RCL initialization failures. The smoke app
retries context startup so boot can succeed even when the Agent appears after
the MCU network comes up.

## 5. Run Tests

### DDSI-RTPS Spec Tests

```bash
cmake -S tests/spec -B build/tests-spec
cmake --build build/tests-spec -j
ctest --test-dir build/tests-spec --output-on-failure
```

Generate conformance reports:

```bash
cmake --build build/tests-spec --target message_parser_spec_report
cmake --build build/tests-spec --target message_builder_spec_report
cmake --build build/tests-spec --target fragment_assembler_spec_report
cmake --build build/tests-spec --target spdp_spec_report
cmake --build build/tests-spec --target sedp_spec_report
cmake --build build/tests-spec --target history_cache_spec_report
cmake --build build/tests-spec --target stateful_writer_spec_report
cmake --build build/tests-spec --target stateful_reader_spec_report
cmake --build build/tests-spec --target participant_spec_report
```

### ROS 2 Interop

`tests/ros2/` proves `rmw_rosette` against stock ROS 2 middleware. With a
sourced ROS 2 environment and a colcon overlay containing `rmw_rosette`:

```bash
tests/ros2/run_rmw_interop_loopback.sh
```

See `tests/ros2/README.md` for two-host sessions and packet capture.

## 6. Common Build Patterns

Configure a host build explicitly:

```bash
cmake -S . -B build/posix -DRTPS_BOARD=posix
cmake --build build/posix -j
```

Build only the default embedded firmware example:

```bash
cmake --build build/w5100s-evb-pico2 -j --target rtps_example
```

Build only the micro-ROS static library bootstrap:

```bash
cmake --build build/micro-ros-pico2 -j --target rtps_micro_ros_bootstrap
```

When in doubt, inspect the configured board summary that CMake prints:

```text
RTPS Stack build configuration
  Board       : ...
  Family      : ...
  Board dir   : ...
  Processor   : ...
```
