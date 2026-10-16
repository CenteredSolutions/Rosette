# Getting Started With Rosette

This guide goes from a clean checkout to a board on your ROS 2 network, then
to a firmware project of your own:

1. [Install ROS 2 and the toolchain](#1-install-ros-2-and-the-toolchain)
2. [Run the host tests](#2-run-the-host-tests) (no hardware, no ROS 2)
3. [Run `rmw_rosette` on a host](#3-run-rmw_rosette-on-a-host) against stock Fast DDS
4. [Build the ROS 2 C client stack](#4-build-the-ros-2-c-client-stack-once-per-cpu) for your MCU, once per CPU
5. [Build, flash, and talk to a board](#5-build-flash-and-talk-to-a-board): the W5100S-EVB-Pico2 and the W6100-EVB-Pico
6. [Start your own firmware](#6-start-your-own-firmware)
7. [Use your own message types](#7-use-your-own-message-types)
8. [Use the RTPS core directly](#8-use-the-rtps-core-directly)
9. [Debugging](#9-debugging)

Read [docs/architecture.md](docs/architecture.md) for how the pieces fit,
[docs/network_configuration.md](docs/network_configuration.md) before putting
a board on a shared LAN, and [docs/mcu_memory_model.md](docs/mcu_memory_model.md)
before sizing a real firmware. [examples/README.md](examples/README.md) lists
every example and which boards it runs on.

## Prerequisites

| For | You need |
| --- | --- |
| Host tests | CMake ≥ 3.20, a C++17 compiler |
| `rmw_rosette` on a host, interop tests | ROS 2 Humble (`/opt/ros/humble`), `colcon` |
| Firmware | `arm-none-eabi-gcc`, Python 3; the Pico SDK and LittleFS are fetched at configure time |
| The C client stack (step 4) | ROS 2 Humble, network access for micro_ros_setup, 20–30 minutes per CPU |
| A board | a WIZnet EVB-Pico board (this guide uses the W5100S-EVB-Pico2 and the W6100-EVB-Pico) wired to the same IPv4 subnet as your ROS 2 host, with a DHCP server or link-local addressing |

Step 1 installs all of it on Ubuntu 22.04.

To build offline or against pinned checkouts, point CMake at them:
`PICO_SDK_PATH`, `LITTLEFS_PATH`, and for FreeRTOS boards
`FREERTOS_KERNEL_PATH` and `FREERTOS_TCP_PATH`.

## 1. Install ROS 2 and the toolchain

Rosette targets ROS 2 Humble on Ubuntu 22.04 (Jammy); the commands below are
the ROS 2 project's Debian install plus the Arm cross compiler.

ROS 2 Humble, with `colcon`, `rosdep`, and `vcstool` (`ros-dev-tools`):

```bash
sudo apt update && sudo apt install -y software-properties-common curl
sudo add-apt-repository -y universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
     -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install -y ros-humble-ros-base ros-dev-tools
sudo rosdep init && rosdep update
```

`ros-humble-ros-base` brings `rclcpp`, `rclpy`, the `ros2` command line,
`rmw_fastrtps_cpp` (Humble's default middleware), and the common message
packages. Add `ros-humble-rmw-cyclonedds-cpp` to also test against Cyclone
DDS (`ROSETTE_PEER_RMW=rmw_cyclonedds_cpp` in `tests/ros2`). See the
[ROS 2 Humble installation guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
for other platforms.

The firmware toolchain (Ubuntu's `gcc-arm-none-eabi` 10.3 is sufficient):

```bash
sudo apt install -y build-essential cmake ninja-build git python3 python3-pip \
     gcc-arm-none-eabi libnewlib-arm-none-eabi libstdc++-arm-none-eabi-newlib
```

To flash over USB without root, your user must be in the group that owns
`/dev/ttyACM*` (`sudo usermod -aG dialout $USER`, then log in again).

## 2. Run the host tests

Each suite is its own CMake project:

```bash
for suite in spec loopback host platform; do
  cmake -S tests/$suite -B build/tests-$suite
  cmake --build build/tests-$suite -j
  ctest --test-dir build/tests-$suite --output-on-failure
done
```

| Suite | Proves |
| --- | --- |
| `tests/spec` | DDSI-RTPS 2.5 statements, one test ID per statement (`TC-SEDP-8.5.5.1-008`, ...) |
| `tests/loopback` | two participants in one process discover each other and deliver reliably, no sockets |
| `tests/host` | address providers (DHCP, link-local, manager, snapshot), WIZnet frame encoding, mDNS wire format, buffer-pool classes |
| `tests/platform` | the platform contract on the host port (the same test runs on each board) |

Spec-coverage reports: `cmake --build build/tests-spec --target sedp_spec_report`
(and `spdp_`, `stateful_writer_`, `stateful_reader_`, `participant_`, ...);
see [tests/spec/README.md](tests/spec/README.md).

## 3. Run `rmw_rosette` on a host

`rmw_rosette` is a colcon package. Build it in a workspace:

```bash
mkdir -p ~/rosette_ws/src && ln -s "$PWD/rmw_rosette" ~/rosette_ws/src/
cd ~/rosette_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select rmw_rosette
source install/setup.bash
```

Any stock ROS 2 program now runs on it. Publish from `rmw_rosette` and echo
from stock Fast DDS:

```bash
export ROS_DOMAIN_ID=42
RMW_IMPLEMENTATION=rmw_rosette ros2 topic pub /hello std_msgs/msg/String "{data: hello from rosette}" &
RMW_IMPLEMENTATION=rmw_fastrtps_cpp ros2 topic echo /hello std_msgs/msg/String
```

The examples use domain 42 to stay clear of whatever else runs on domain 0 of
your LAN. The interop harness proves topics, services, parameter services, and
the graph in both directions:

```bash
tests/ros2/run_rmw_interop_loopback.sh          # run under bash, with the workspace sourced
tests/ros2/run_rmw_frag_loopback.sh             # 16 B – 60 KB messages over a 1472-octet link
```

`colcon test --packages-select rmw_rosette` runs the package's unit tests.

## 4. Build the ROS 2 C client stack once per CPU

A firmware links stock `rcl`, `rclc`, `rcutils`, and the message type supports
as static libraries. micro_ros_setup's `generate_lib` flow cross-compiles them;
Rosette uses its per-package libraries and never links the micro-ROS
middleware, so `rcl` calls `rmw_rosette` directly. The bootstrap fetches
micro_ros_setup and sources `/opt/ros/humble/setup.bash` itself.

The stack is compiled for one CPU. Build one per CPU you target, each into a
workspace you keep, configured with any board that has that CPU:

| CPU | Boards | Build it with |
| --- | --- | --- |
| RP2350 (Cortex-M33) | `w5100s-evb-pico2`, `w6100-evb-pico2`, `w6300-evb-pico2` | the commands below |
| RP2040 (Cortex-M0+) | `w5100s-evb-pico`, `w6100-evb-pico` | the same, with `-DROSETTE_BOARD=w6100-evb-pico` and `build-micro-ros-pico` |

```bash
# RP2350
cmake -S . -B build/c-stack-pico2 -DROSETTE_BOARD=w5100s-evb-pico2 \
      -DROSETTE_ENABLE_MICRO_ROS=ON \
      -DROSETTE_MICRO_ROS_WORKSPACE_DIR=$PWD/build-micro-ros-pico2
cmake --build build/c-stack-pico2 --target rosette_micro_ros_bootstrap

# RP2040
cmake -S . -B build/c-stack-pico -DROSETTE_BOARD=w6100-evb-pico \
      -DROSETTE_ENABLE_MICRO_ROS=ON \
      -DROSETTE_MICRO_ROS_WORKSPACE_DIR=$PWD/build-micro-ros-pico
cmake --build build/c-stack-pico --target rosette_micro_ros_bootstrap
```

Each result is `<workspace>/firmware/mcu_ws/install`, which every later
firmware build for that CPU names as `ROSETTE_ROS2_C_STACK_DIR`. Rebuild a
stack only when you add message packages (step 7) or change ROS distro.

Keep the pairs straight: the firmware link does not check the stack's CPU, so
an RP2040 image linked against the RP2350 stack builds and then faults. To
check a stack, `arm-none-eabi-readelf -A <install>/lib/librcl.a | grep -m1
Tag_CPU_arch` prints `v8-M.mainline` for RP2350 and `v6S-M` for RP2040.

## 5. Build, flash, and talk to a board

`examples/rclc_chatter` is a stock `rclc` node: it publishes
`hello from rclc #N` on `/rosette/chatter` every second and prints every string
received on `/rosette/command`. `examples/udp_echo` checks the network
underneath it with plain UDP. Configure one build directory per board:

| | W5100S-EVB-Pico2 | W6100-EVB-Pico |
| --- | --- | --- |
| MCU | RP2350 | RP2040 |
| Ethernet chip | W5100S: IPv4 | W6100: IPv4 and IPv6 |
| `ROSETTE_BOARD` | `w5100s-evb-pico2` | `w6100-evb-pico` |
| C client stack | `build-micro-ros-pico2` | `build-micro-ros-pico` |

W5100S-EVB-Pico2:

```bash
cmake -S . -B build-hw/pico2 -DROSETTE_BOARD=w5100s-evb-pico2 \
      -DROSETTE_ROS2_C_STACK_DIR=$PWD/build-micro-ros-pico2/firmware/mcu_ws/install
cmake --build build-hw/pico2 --target udp_echo rclc_chatter
```

W6100-EVB-Pico:

```bash
cmake -S . -B build-hw/w6100-pico -DROSETTE_BOARD=w6100-evb-pico \
      -DROSETTE_ROS2_C_STACK_DIR=$PWD/build-micro-ros-pico/firmware/mcu_ws/install
cmake --build build-hw/w6100-pico --target udp_echo rclc_chatter
```

Both boards are bare metal, powered and flashed over the Pico's USB port, and
take their Ethernet from the on-board chip.

### Check the network first

Flash the echo example and read the addresses it prints:

```bash
tools/pico_uf2_flash.py build-hw/w6100-pico/examples/udp_echo/udp_echo.uf2 --monitor 30
```

```text
[net] PHY link up
[net] address 192.168.1.57
[net] IPv6 address fe80:0:0:0:208:dcff:fe12:3456
[echo] udp/7 ipv4 open, ipv6 open; status on udp/8
```

Then, from the ROS 2 host, `tools/udp_echo_probe.py 192.168.1.57`
pings and echoes every address the board holds. The W5100S has no IPv6, so it
prints an IPv4 address only. If this step fails, fix the network before going
further; [docs/network_configuration.md](docs/network_configuration.md) lists
what the board needs.

### Flash and talk to `rclc_chatter`

Flash it over USB. `tools/pico_uf2_flash.py` opens the board's console at 1200
baud, which reboots a running Pico SDK image into BOOTSEL, copies the UF2 to
the drive that appears, and streams the new image's console:

```bash
tools/pico_uf2_flash.py build-hw/pico2/examples/rclc_chatter/rclc_chatter.uf2 --monitor 60
```

The first flash of a blank board needs BOOTSEL held at power-up. If your
desktop does not mount the drive automatically, mount it
(`udisksctl mount -b /dev/sdX1`) and the tool continues. When the USB console
is unreliable (a VM's USB passthrough), a board running `udp_echo`
can be sent to BOOTSEL over the network instead:
`tools/pico_uf2_flash.py <uf2> --bootsel-udp 192.168.1.57`.

The console shows the network coming up, then a report every five seconds:

```text
[net] PHY link up
[net] address 192.168.1.57
[rclc] node /rosette/rclc_chatter up: /rosette/chatter, /rosette/command
[rclc] published=5 commands=0 heap_in_use=4040 arp_timeouts=0
```

From any ROS 2 host on the subnet, on domain 0 with stock middleware and no
configuration ([docs/network_configuration.md](docs/network_configuration.md#ros-domain)
shows how to put a firmware on another domain):

```bash
ros2 topic echo /rosette/chatter std_msgs/msg/String
ros2 topic pub /rosette/command std_msgs/msg/String "{data: hi}"
ros2 node info /rosette/rclc_chatter
```

## 6. Start your own firmware

A firmware project includes Rosette's prelude before `project()` (it selects
the board and sets up the cross toolchain), adds the Rosette tree, and links
`rmw_rosette_firmware`:

```cmake
# CMakeLists.txt
cmake_minimum_required(VERSION 3.20)

set(ROSETTE_DIR "${CMAKE_CURRENT_LIST_DIR}/rosette" CACHE PATH "Rosette checkout")
include(${ROSETTE_DIR}/cmake/rosette_prelude.cmake)   # board + toolchain; before project()

project(my_firmware C CXX ASM)
set(CMAKE_CXX_STANDARD 17)

add_subdirectory(${ROSETTE_SOURCE_DIR} ${CMAKE_BINARY_DIR}/rosette)

add_executable(my_firmware main.cpp)
target_link_libraries(my_firmware PRIVATE rmw_rosette_firmware rosette_bsp)
rosette_add_image_outputs(my_firmware)   # the chip port's images: .uf2 and USB stdio on rp2
```

Rosette usually lives in the project as a git submodule at `rosette/`. Declare
the firmware's capacity when you configure: exactly the entities it creates.
Everything else is derived.

```bash
cmake -S . -B build -DROSETTE_BOARD=w5100s-evb-pico2 \
      -DROSETTE_ROS2_C_STACK_DIR=<path>/build-micro-ros-pico2/firmware/mcu_ws/install \
      -DRMW_ROSETTE_MAX_PUBLISHERS=1 -DRMW_ROSETTE_MAX_SUBSCRIPTIONS=1 \
      -DRMW_ROSETTE_MAX_SERVICES=0 -DRMW_ROSETTE_MAX_CLIENTS=0
cmake --build build        # build/my_firmware.uf2
```

For `rclc_chatter`'s shape, those four limits cut static RAM from 123.7 KB to
59.2 KB; [docs/mcu_memory_model.md](docs/mcu_memory_model.md) explains every
knob.

`main.cpp` follows one order: board, network, then `rcl`, and every entity
created before the executor first spins.

```cpp
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/string.h>

#include "board.h"
#include "rosette/net/board_network.hpp"
#include "rosette/platform/platform.hpp"
#include "pico/stdlib.h"

namespace net = rosette::net;

int main() {
    board_init();
    net::bring_up_board_network("my-board");   // blocks until an IPv4 address is held

    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rclc_support_init(&support, 0, nullptr, &allocator);   // rmw_init() opens the RTPS transport

    rcl_node_t node = rcl_get_zero_initialized_node();
    rclc_node_init_default(&node, "my_node", "my_ns", &support);

    // Create every publisher, subscription, service, and timer here, then
    // add the callbacks to one executor. Entities that exist before the
    // first spin are announced together and matched from the first
    // discovery datagram.

    rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
    rclc_executor_init(&executor, &support.context, /*handles=*/1, &allocator);
    // rclc_executor_add_subscription(...), rclc_executor_add_timer(...)

    for (;;) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));   // sleeps until data or a deadline
        net::poll_board_network(rosette::platform::monotonic_ms());   // DHCP renewal and retry, SLAAC, conflicts
    }
}
```

Check every `rcl_ret_t` in real code; `examples/rclc_chatter/main.cpp` shows a
compact pattern, plus fixed message buffers so nothing is allocated per
message.

## 7. Use your own message types

Message types come from the C client stack. Add your interface packages to it
and rebuild the stack for each CPU you target (step 4):

```bash
cmake -S . -B build/c-stack-pico2 ... \
      -DROSETTE_MICRO_ROS_EXTRA_INTERFACE_PACKAGES="$PWD/../my_msgs;$PWD/../other_msgs"
cmake --build build/c-stack-pico2 --target rosette_micro_ros_bootstrap
```

(Interface packages under `examples/*/` are included automatically.) Then use
the generated C types as usual:

```cpp
#include <my_msgs/msg/motor_state.h>
const rosidl_message_type_support_t* ts = ROSIDL_GET_MSG_TYPE_SUPPORT(my_msgs, msg, MotorState);
```

`rmw_rosette` serializes any type through its generated CDR callbacks, byte
for byte as Fast DDS does, so the same `.msg` package built normally on the
host is all the other side needs.

Strings and sequences in generated C messages are pointer, size, and capacity.
Point them at buffers you own (static arrays, as `rclc_chatter` does, or
`micro_ros_utilities` to allocate once at startup) so receiving never
allocates.

## 8. Use the RTPS core directly

`examples/rtps_chatter` talks to the same topics as `rclc_chatter` without
`rcl`: it creates an `RTPSParticipant`, a writer and a reader with ROS 2 names
(`rt/rosette/chatter`, type `std_msgs::msg::dds_::String_`), and encodes the
CDR itself. It needs no C client stack and is the smallest image (157 KB
flash, 74 KB RAM):

```bash
cmake -S . -B build-hw/pico2 -DROSETTE_BOARD=w5100s-evb-pico2
cmake --build build-hw/pico2 --target rtps_chatter
tools/pico_uf2_flash.py build-hw/pico2/examples/rtps_chatter/rtps_chatter.uf2 --monitor 60
```

The core's sizes are board inputs in `cmake/rosette_rtps.cmake`
(`ROSETTE_RTPS_MAX_SAMPLE_SIZE`, `ROSETTE_RTPS_MAX_PEERS`, ...). Every call
into the participant or its endpoints holds `driver.mutex()`.

## 9. Debugging

| Symptom | Look at |
| --- | --- |
| No address, or no echo | the board's console `[net]` lines; `tools/udp_echo_probe.py`; [docs/network_configuration.md](docs/network_configuration.md#checking-the-network-below-ros) |
| Nothing discovered | Same `ROS_DOMAIN_ID` on both sides (firmware uses 0)? An `rmw_rosette` host on several networks uses its default route's interface unless `ROSETTE_INTERFACE=eth0` names the boards' one. Same IPv4 subnet, multicast delivered, UDP 7400+ open on the host's firewall ([ports](docs/network_configuration.md#what-ros-2-traffic-needs))? A stale `ros2` daemon (`ros2 daemon stop`) |
| RP2040 firmware faults at boot | it was linked against the RP2350 C client stack (step 4) |
| Discovery works, data does not | RTPS log level 4 shows every HEARTBEAT, ACKNACK, and DATA (`-DROSETTE_RMW_LOG_LEVEL=4` for firmware `rmw_rosette`, `-DROSETTE_RTPS_LOG_LEVEL=4` for `rosette_rtps`, `-DRMW_ROSETTE_RTPS_LOG_LEVEL=4` for the colcon package) |
| Large messages lost to a board | the peer's datagrams exceed 1472 octets; cap them (Fast DDS `maxMessageSize`) — see `rmw_rosette/README.md` |
| Reproduce a board's link on a host | `RMW_ROSETTE_STRICT_LINK=1 RMW_ROSETTE_MAX_DATAGRAM_SIZE=1472` |
| Datagram sizes, truncation | `RMW_ROSETTE_TRANSPORT_STATS=1` (host, printed at shutdown); `rtps_chatter` prints them every five seconds |
| Entity creation fails | a capacity limit is too small; the rcl error string names the pool |
| RAM | [docs/mcu_memory_model.md](docs/mcu_memory_model.md#measuring) |

When CMake configures, it prints the selected board, CPU, and pins, and the
`rmw_rosette` capacity it derived:

```text
Rosette build configuration
  Board       : w5100s-evb-pico2
  Processor   : RP2350
rmw_rosette capacity: 4 publishers, 4 subscriptions, 4 services, 2 clients -> 11 RTPS writers, 10 RTPS readers
```
