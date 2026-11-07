# SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
# SPDX-License-Identifier: Apache-2.0

# rtps_chatter — agentless ROS 2 talker/listener on the Rosette RTPS core over
# the board's socket provider (no micro-ROS, no agent). It builds for any
# board with a network (a rosette_bsp): the WIZnet boards' hardware sockets,
# or FreeRTOS+TCP over the MIMXRT1020-EVK's ENET.
#
# W5100S-EVB-Pico2:
#
#   cmake -S . -B build-hw/pico2 -DROSETTE_BOARD=w5100s-evb-pico2 \
#         -DROSETTE_MICRO_ROS_BOOTSTRAP_ON_BUILD=OFF
#   cmake --build build-hw/pico2 --target rtps_chatter
#   tools/pico_uf2_flash.py build-hw/pico2/examples/rtps_chatter/rtps_chatter.uf2 --monitor 60
#
# MIMXRT1020-EVK, flashed through its on-board DAPLink:
#
#   cmake -S . -B build-hw/rt1020-evk -DROSETTE_BOARD=mimxrt1020-evk \
#         -DROSETTE_ENABLE_MICRO_ROS=OFF
#   cmake --build build-hw/rt1020-evk --target rtps_chatter
#   pyocd flash -t mimxrt1020 build-hw/rt1020-evk/examples/rtps_chatter/rtps_chatter.hex
if(NOT TARGET rosette_bsp)
    return()
endif()

# What main.cpp creates: one writer of depth 4 (rt/rosette/chatter) and one
# reader (rt/rosette/command), whose names take 67 octets with the shared
# std_msgs String type stored once. These are facts of the code, so they are
# set here rather than cached; the network-sized inputs (peers, remote
# endpoints, matches) keep their board or command-line values.
set(ROSETTE_RTPS_MAX_WRITERS 1)
set(ROSETTE_RTPS_MAX_READERS 1)
set(ROSETTE_RTPS_MAX_WRITER_DEPTH 4)
set(ROSETTE_RTPS_LOCAL_NAME_BYTES 128)
include(${ROSETTE_ROOT}/cmake/rosette_rtps.cmake)

add_executable(rtps_chatter main.cpp)
target_link_libraries(rtps_chatter PRIVATE rosette_rtps rosette_bsp)
rosette_add_image_outputs(rtps_chatter)
