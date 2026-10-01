cmake_minimum_required(VERSION 3.20)

# led_files — stock rclc services on a WIZnet board: an LED control service
# and an on-device file store, over this example's own rosette_msgs package.
# The same main.cpp builds against either ROS 2 backend; the build
# configuration picks one:
#
#   * micro-ROS agent, when ROSETTE_ENABLE_MICRO_ROS=ON. rosette_msgs/ is
#     compiled into libmicroros automatically (cmake/micro_ros.cmake scans
#     examples/ for interface packages). The agent is located by
#     -DROSETTE_MICRO_ROS_AGENT_IP=<address>, or else by
#     -DROSETTE_MICRO_ROS_DISCOVERY=mdns|xrce.
#       cmake -S . -B build -DROSETTE_ENABLE_MICRO_ROS=ON [-DROSETTE_BOARD=...]
#       cmake --build build --target led_files
#
#   * rmw_rosette, when ROSETTE_ROS2_C_STACK_DIR names the cross-built C
#     client stack (its micro-ROS workspace build compiles rosette_msgs the
#     same way). The five services need RMW_ROSETTE_MAX_SERVICES >= 5:
#       cmake -S . -B build-hw/pico -DROSETTE_BOARD=w6100-evb-pico \
#             -DROSETTE_MICRO_ROS_BOOTSTRAP_ON_BUILD=OFF -DRMW_ROSETTE_MAX_SERVICES=5 \
#             -DROSETTE_ROS2_C_STACK_DIR=$PWD/build-micro-ros-pico/firmware/mcu_ws/install
#       cmake --build build-hw/pico --target led_files
#
#   * standalone agent build, for a bare-metal WIZnet Pico board:
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

# The board network (net::bring_up_board_network) is the bare-metal WIZnet port.
if(NOT ROSETTE_BOARD_ENABLE_WIZNET OR NOT ROSETTE_BOARD_ENABLE_PICO_SDK OR ROSETTE_BOARD_ENABLE_FREERTOS)
    return()
endif()

if(ROSETTE_ENABLE_MICRO_ROS)
    set(_led_files_backend rosette_agent)
    set(_led_files_agent 1)
else()
    include(${ROSETTE_ROOT}/cmake/rmw_rosette_firmware.cmake)
    if(NOT TARGET rmw_rosette_firmware)
        message(STATUS "led_files skipped: enable ROSETTE_ENABLE_MICRO_ROS or set ROSETTE_ROS2_C_STACK_DIR")
        return()
    endif()
    if(RMW_ROSETTE_MAX_SERVICES LESS 5)
        message(WARNING "led_files skipped: its five services need -DRMW_ROSETTE_MAX_SERVICES=5 or more")
        return()
    endif()
    set(_led_files_backend rmw_rosette_firmware rosette_session)
    set(_led_files_agent 0)
endif()

add_executable(led_files main.cpp)
target_include_directories(led_files PRIVATE ${ROSETTE_ROOT}/include)
target_compile_definitions(led_files PRIVATE ROSETTE_LED_FILES_AGENT_BACKEND=${_led_files_agent})
target_link_libraries(led_files PRIVATE ${_led_files_backend} rosette_storage rosette_bsp pico_stdlib)
pico_add_extra_outputs(led_files)
pico_enable_stdio_usb(led_files 1)
pico_enable_stdio_uart(led_files 0)
