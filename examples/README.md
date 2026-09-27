# Examples

Every example builds from the repository root: configure once per board, then
build the example's target. Board firmware lands at
`<build>/examples/<name>/<name>.uf2`; flash it with
`tools/pico_uf2_flash.py <uf2> --monitor 60`.
[GETTING_STARTED.md](../GETTING_STARTED.md) walks through the setup.

## Current

| Example | What it shows | Needs | Hardware-verified on |
| --- | --- | --- | --- |
| [`wiznet_udp_echo`](wiznet_udp_echo/main.cpp) | The network below ROS: DHCP / link-local IPv4, IPv6 link-local and SLAAC on the W6100 / W6300, UDP echo (port 7) and a status port (port 8) per family. The first thing to flash on a new board. | a WIZnet board | W6100-EVB-Pico (IPv4, IPv6 link-local and global) |
| [`rclc_chatter`](rclc_chatter/main.cpp) | A stock `rclc` node on `rmw_rosette`: publishes `/rosette/chatter` every second, prints `/rosette/command`, reports the rcl heap. The template for your own firmware. | a WIZnet board, the C client stack for the board's CPU | W5100S-EVB-Pico2 |
| [`rtps_chatter`](rtps_chatter/main.cpp) | The same topics on the RTPS core directly, with no `rcl`: participant, writer, reader, and hand-written CDR. The smallest ROS-visible image. | a WIZnet board | W5100S-EVB-Pico2 |
| [`storage_demo`](storage_demo/main.cpp) | The LittleFS volume, a `DeviceConfig` record, and the `FileStore` blob API. | host (`ROSETTE_BOARD=posix`) | host |
| [`rosette_smoke`](rosette_smoke/main.cpp) | The smallest micro-ROS agent client: publishes `std_msgs/Float32` to a `micro_ros_agent`. | `ROSETTE_ENABLE_MICRO_ROS=ON`, an agent | host |
| [`led_files`](led_files/main.cpp) | An agent client with two modules: an LED service and a file service over its own `rosette_msgs` package. | `ROSETTE_ENABLE_MICRO_ROS=ON`, an agent | host |

`board_smoke/` is the original FreeRTOS bring-up smoke. It is not part of the
build; its [README](board_smoke/README.md) says why.

The agent-client examples are described in
[docs/micro_ros_agent_client.md](../docs/micro_ros_agent_client.md). Network
settings that every board example shares (addresses, domain, ports) are in
[docs/network_configuration.md](../docs/network_configuration.md).

## Board support

| Board | `wiznet_udp_echo` | `rtps_chatter` | `rclc_chatter` |
| --- | --- | --- | --- |
| `w5100s-evb-pico2` | builds (IPv4 only) | runs | runs |
| `w5100s-evb-pico` | builds (IPv4 only) | builds | builds with the RP2040 C client stack |
| `w6100-evb-pico` | runs | builds | builds with the RP2040 C client stack |
| `w6100-evb-pico2` | builds | builds | builds |
| `w6300-evb-pico2` | builds | builds | builds |

"Runs" means verified on the board against stock ROS 2 (or, for the echo, a
Linux host); "builds" means the image links and has not been run.

## Planned

| Example | What it will show |
| --- | --- |
| `rclc_service` | An `rclc` service server (`std_srvs/SetBool`) on a board, called with `ros2 service call`. |
| `rclc_action` | An `rclc` action server on a board, sized by an action-server capacity input. |
| `rclc_chatter` over the agent | The same `rclc` application built against `rmw_microxrcedds` and a `micro_ros_agent`, showing that an application moves between the two backends without changes. |
