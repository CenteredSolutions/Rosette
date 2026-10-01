# rtps_chatter — agentless ROS 2 talker/listener on the Rosette RTPS core over
# WIZnet hardware UDP sockets (no micro-ROS, no agent). Built from the repo
# root for a WIZnet Pico board:
#
#   cmake -S . -B build-hw/pico2 -DROSETTE_BOARD=w5100s-evb-pico2 \
#         -DROSETTE_MICRO_ROS_BOOTSTRAP_ON_BUILD=OFF
#   cmake --build build-hw/pico2 --target rtps_chatter
#   tools/pico_uf2_flash.py build-hw/pico2/examples/rtps_chatter/rtps_chatter.uf2 --monitor 60

if(NOT ROSETTE_BOARD_ENABLE_WIZNET OR NOT ROSETTE_BOARD_ENABLE_PICO_SDK)
    return()
endif()

include(${ROSETTE_ROOT}/cmake/rosette_rtps.cmake)

add_executable(rtps_chatter main.cpp)
target_link_libraries(rtps_chatter PRIVATE rosette_rtps rosette_bsp)
rosette_add_image_outputs(rtps_chatter)
