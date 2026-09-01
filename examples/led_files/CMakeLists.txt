# led_files — the App + Module reference application: an LED control service
# and an on-device file store, over this example's own rosette_msgs package
# (rosette_msgs/ is compiled into libmicroros automatically by
# cmake/micro_ros.cmake, which scans examples/ for interface packages).

if(NOT ROSETTE_ENABLE_MICRO_ROS)
    return()
endif()

add_executable(led_files main.cpp)
target_include_directories(led_files PRIVATE "${CMAKE_CURRENT_LIST_DIR}")
target_link_libraries(led_files PRIVATE rosette_ros rosette_storage)

if(ROSETTE_BOARD_ENABLE_PICO_SDK)
    target_compile_definitions(led_files PRIVATE ROSETTE_BOARD_PICO)
    target_link_libraries(led_files PRIVATE rosette_bsp hardware_gpio)
    pico_add_extra_outputs(led_files)
    pico_enable_stdio_usb(led_files 1)
    pico_enable_stdio_uart(led_files 0)
endif()
