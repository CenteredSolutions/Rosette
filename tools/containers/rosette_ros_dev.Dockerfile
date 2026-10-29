# Development and interop image for one ROS 2 distribution.
#
# The image carries everything the host-side rmw_rosette build and the
# tests/ros2 stock-peer sessions need: the distro's ros-base (rclcpp, rclpy,
# rmw_fastrtps_cpp), rmw_cyclonedds_cpp as the second stock peer, colcon, and
# the arm-none-eabi toolchain for firmware builds. Build one image per distro:
#
#   docker build --build-arg ROS_DISTRO=humble -t rosette-humble \
#       -f tools/containers/rosette_ros_dev.Dockerfile tools/containers
#   docker build --build-arg ROS_DISTRO=jazzy -t rosette-jazzy \
#       -f tools/containers/rosette_ros_dev.Dockerfile tools/containers
#
# Run with the checkout bind-mounted at the same path and host networking so
# DDS discovery reaches the bench:
#
#   docker run --rm --net=host -v $PWD:$PWD -w $PWD rosette-jazzy bash
ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}-ros-base

ARG ROS_DISTRO

RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
      gcc-arm-none-eabi libnewlib-arm-none-eabi libstdc++-arm-none-eabi-newlib \
      build-essential git ninja-build python3-pip python3-colcon-common-extensions \
      flex bison python3-vcstool iproute2 sudo clang-tidy libusb-1.0-0-dev pkg-config \
      usbutils tshark \
      ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
      ros-${ROS_DISTRO}-rmw-fastrtps-cpp \
    && rm -rf /var/lib/apt/lists/*

# Bind-mounted checkouts belong to uid 1000 on the bench. Noble-based images
# ship an `ubuntu` user at that uid, so it gives way to `dev`.
RUN if id -u ubuntu >/dev/null 2>&1; then userdel -r ubuntu; fi \
    && groupadd -g 1000 dev && useradd -m -u 1000 -g 1000 dev \
    && echo 'dev ALL=(ALL) NOPASSWD:ALL' > /etc/sudoers.d/dev

USER dev
RUN rosdep update --rosdistro ${ROS_DISTRO}
