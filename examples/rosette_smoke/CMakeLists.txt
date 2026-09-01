# rosette_smoke — smallest end-to-end micro-ROS client.
# Stock XRCE-DDS UDP transport, no BSP; publishes std_msgs/msg/Float32.

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
