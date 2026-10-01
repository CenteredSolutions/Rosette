# rclc_chatter — a stock rclc application on a WIZnet board, with rmw_rosette
# as its middleware (no agent). Built from the repo root with the cross-built
# ROS 2 C client stack:
#
#   cmake -S . -B build-hw/pico2 -DROSETTE_BOARD=w5100s-evb-pico2 \
#         -DROSETTE_MICRO_ROS_BOOTSTRAP_ON_BUILD=OFF \
#         -DROSETTE_ROS2_C_STACK_DIR=$PWD/build-micro-ros-pico2/firmware/mcu_ws/install
#   cmake --build build-hw/pico2 --target rclc_chatter
#   tools/pico_uf2_flash.py build-hw/pico2/examples/rclc_chatter/rclc_chatter.uf2 --monitor 60

if(NOT ROSETTE_BOARD_ENABLE_WIZNET OR NOT ROSETTE_BOARD_ENABLE_PICO_SDK OR ROSETTE_BOARD_ENABLE_FREERTOS)
    return()
endif()

include(${ROSETTE_ROOT}/cmake/rmw_rosette_firmware.cmake)
if(NOT TARGET rmw_rosette_firmware)
    return()
endif()

add_executable(rclc_chatter main.cpp)
target_include_directories(rclc_chatter PRIVATE ${ROSETTE_ROOT}/include)
target_link_libraries(rclc_chatter PRIVATE rmw_rosette_firmware rosette_session rosette_bsp)
rosette_add_image_outputs(rclc_chatter)
