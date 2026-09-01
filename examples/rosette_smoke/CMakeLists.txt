cmake_minimum_required(VERSION 3.20)

# rosette_smoke — smallest end-to-end micro-ROS client. Stock XRCE-DDS UDP
# transport, no BSP; publishes std_msgs/msg/Float32.
#
#   from the repo root:  cmake -S . -B build -DROSETTE_ENABLE_MICRO_ROS=ON
#   standalone:          cd examples/rosette_smoke
#                        cmake -S . -B build -DROSETTE_BOARD=<board>

if(NOT TARGET rosette_net)
    include(${CMAKE_CURRENT_LIST_DIR}/../../cmake/rosette_prelude.cmake)
    project(rosette_example_smoke C CXX ASM)
    set(ROSETTE_ENABLE_MICRO_ROS ON CACHE BOOL "rosette_smoke requires micro-ROS" FORCE)
    add_subdirectory("${ROSETTE_SOURCE_DIR}" "${CMAKE_BINARY_DIR}/rosette")
    return()
endif()

if(NOT ROSETTE_ENABLE_MICRO_ROS)
    return()
endif()

add_executable(rosette_smoke main.cpp)
target_link_libraries(rosette_smoke PRIVATE rosette_ros)

if(ROSETTE_BOARD_ENABLE_PICO_SDK)
    pico_add_extra_outputs(rosette_smoke)
    pico_enable_stdio_usb(rosette_smoke 1)
    pico_enable_stdio_uart(rosette_smoke 0)
endif()
