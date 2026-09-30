cmake_minimum_required(VERSION 3.20)

# led_files — stock rclc services on a WIZnet board through a micro-ROS agent:
# an LED control service and an on-device file store, over this example's own
# rosette_msgs package (rosette_msgs/ is compiled into libmicroros
# automatically — cmake/micro_ros.cmake scans examples/ for interface packages).
# The agent is located by -DROSETTE_MICRO_ROS_AGENT_IP=<address>, or else by
# -DROSETTE_MICRO_ROS_DISCOVERY=mdns|xrce.
#
# Build it either way:
#
#   * from the repo root:
#       cmake -S . -B build -DROSETTE_ENABLE_MICRO_ROS=ON [-DROSETTE_BOARD=...]
#       cmake --build build --target led_files
#
#   * standalone, for a bare-metal WIZnet Pico board:
#       cd examples/led_files
#       cmake -S . -B build -DROSETTE_BOARD=w5100s-evb-pico2
#       cmake --build build
#     This pulls in the whole Rosette tree and runs the libmicroros bootstrap
#     (which cross-builds libmicroros for a Pico board — arm-none-eabi-gcc must
#     be on PATH).

if(NOT TARGET rosette_net)
    include(${CMAKE_CURRENT_LIST_DIR}/../../cmake/rosette_prelude.cmake)
    project(rosette_example_led_files C CXX ASM)
    set(ROSETTE_ENABLE_MICRO_ROS ON CACHE BOOL "led_files requires micro-ROS" FORCE)
    add_subdirectory("${ROSETTE_SOURCE_DIR}" "${CMAKE_BINARY_DIR}/rosette")
    message(STATUS "led_files: firmware will be at ${CMAKE_BINARY_DIR}/rosette/examples/led_files/led_files.uf2")
    return()
endif()

if(NOT ROSETTE_ENABLE_MICRO_ROS)
    message(WARNING "led_files skipped: configure with -DROSETTE_ENABLE_MICRO_ROS=ON")
    return()
endif()

# The board network (net::bring_up_board_network) is the bare-metal WIZnet port.
if(NOT ROSETTE_BOARD_ENABLE_WIZNET OR NOT ROSETTE_BOARD_ENABLE_PICO_SDK OR ROSETTE_BOARD_ENABLE_FREERTOS)
    return()
endif()

add_executable(led_files main.cpp)
target_link_libraries(led_files PRIVATE rosette_agent rosette_storage rosette_bsp pico_stdlib)
pico_add_extra_outputs(led_files)
pico_enable_stdio_usb(led_files 1)
pico_enable_stdio_uart(led_files 0)
