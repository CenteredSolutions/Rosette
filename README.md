# Rosette

Rosette is a small embedded ROS/RTPS stack for constrained boards, with host-side
test and interop tooling. The repository contains:

- `rtps/`: DDSI-RTPS message parsing, discovery, reliability, transport, and
  participant support.
- `ros/`: lightweight ROS-style context, node, publisher, subscription, service,
  timer, and generated interface support.
- `ros/micro_ros/`: micro-ROS client integration and transport helpers.
- `net/`: socket abstraction layers for POSIX, FreeRTOS+TCP, and WIZnet MACRAW.
- `boards/`: board definitions selected with `-DRTPS_BOARD=<name>`.
- `examples/`: firmware and smoke examples.
- `rmw_rosette/`: ROS 2 middleware (rmw) implementation over the RTPS stack, used
  by stock `rcl`/`rclc`/`rclcpp`/`rclpy` via `RMW_IMPLEMENTATION=rmw_rosette`.
- `tests/spec/`: host-side DDSI-RTPS conformance tests.
- `tests/loopback/`: two in-process RTPS participants completing discovery and
  reliable delivery with no sockets.
- `tests/ros2/`: rmw interop harness proving `rmw_rosette` against stock ROS 2
  middleware.

## Supported Boards

Select the board at CMake configure time with `RTPS_BOARD`.

| Board | Target | Notes |
| --- | --- | --- |
| `posix` | Host/POSIX | Builds the host library path; top-level firmware example is disabled. |
| `w5100s-evb-pico` | RP2040 + W5100S | Builds `rtps_example` for WIZnet W5100S-EVB-Pico. |
| `w5100s-evb-pico2` | RP2350 + W5100S | Builds `rtps_example` for WIZnet W5100S-EVB-Pico2. This is the default on non-macOS hosts. |
| `bc-gen2` | RP2040 + W5100S | Builds `rtps_example` with the custom `bc-gen2` pin map. |

On macOS the default board is `posix`; elsewhere it is `w5100s-evb-pico2`.
For repeatable builds, pass `-DRTPS_BOARD=...` explicitly.

## Minimal Example Build

Embedded boards build the `rtps_example` firmware from `examples/main_rp2350.cpp`.
The output includes the normal Pico SDK artifacts, including a `.uf2`.

```bash
cmake -S . -B build/w5100s-evb-pico2 -DRTPS_BOARD=w5100s-evb-pico2
cmake --build build/w5100s-evb-pico2 -j --target rtps_example
```

Swap the board name for another variant:

```bash
cmake -S . -B build/w5100s-evb-pico -DRTPS_BOARD=w5100s-evb-pico
cmake --build build/w5100s-evb-pico -j --target rtps_example

cmake -S . -B build/bc-gen2 -DRTPS_BOARD=bc-gen2
cmake --build build/bc-gen2 -j --target rtps_example
```

CMake can fetch the Pico SDK, FreeRTOS, FreeRTOS+TCP, WIZnet ioLibrary, and
LittleFS when network access is available. For offline or pinned builds, provide
local checkouts with:

- `PICO_SDK_PATH`
- `FREERTOS_KERNEL_PATH`
- `FREERTOS_TCP_PATH`
- `WIZNET_IOLIB_PATH`
- `LITTLEFS_PATH`

## Start A New Project

Use `GETTING_STARTED.md` for the full walkthrough. The shortest path is:

1. Pick a board and copy or replace `examples/main_rp2350.cpp`.
2. Bring up the board network with the board BSP, WIZnet MACRAW driver, and
   FreeRTOS+TCP endpoints.
3. Create a `milkros::Context`, start the transport runtime, and start the RTPS
   participant.
4. Create a node and publisher.
5. Tick the context regularly and publish your metric on a fixed interval.

For micro-ROS agent-backed projects, enable the smoke target:

```bash
cmake -S . -B build/micro-ros-pico2 \
  -DRTPS_BOARD=w5100s-evb-pico2 \
  -DROSETTE_ENABLE_MICRO_ROS=ON
cmake --build build/micro-ros-pico2 -j --target rtps_micro_ros_board_smoke
```

The micro-ROS smoke publishes `std_msgs/msg/Float32` on
`mcu_temperature_c`. Set `ROSETTE_MICRO_ROS_AGENT_IP` if you want a fixed agent
address; leave it empty to use the built-in multicast agent locator.

## Tests

Run the standards-facing host conformance suite:

```bash
cmake -S tests/spec -B build/tests-spec
cmake --build build/tests-spec -j
ctest --test-dir build/tests-spec --output-on-failure
```

Run the in-process RTPS loopback proof:

```bash
cmake -S tests/loopback -B build/tests-loopback
cmake --build build/tests-loopback -j
ctest --test-dir build/tests-loopback --output-on-failure
```

Prove `rmw_rosette` against a stock ROS 2 middleware (requires a sourced ROS 2
environment and a colcon overlay containing `rmw_rosette`):

```bash
tests/ros2/run_rmw_interop_loopback.sh
```

## Further Reading

- `GETTING_STARTED.md`: practical build, board, project, and test workflow.
- `tests/spec/README.md`: conformance coverage and fuzzing commands.
- `tests/ros2/README.md`: rmw interop harness details.
- `rmw_rosette/README.md`: rmw implementation status and known gaps.
- `ros/micro_ros/README.md`: micro-ROS wrapper details.
