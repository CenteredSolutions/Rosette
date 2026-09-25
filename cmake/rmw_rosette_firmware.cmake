# rmw_rosette_firmware — stock ROS 2 C client libraries over rmw_rosette,
# linked into a firmware image.
#
# A firmware ROS stack has two halves that change at very different rates:
#
#   ros2_c_client_stack  rcl, rcl_action, rclc, rcutils, rosidl and the message
#                        packages, cross-compiled once as static libraries by
#                        micro_ros_setup's generate_lib flow. Rosette uses its
#                        per-package libraries from mcu_ws/install/lib and never
#                        its merged libmicroros.a, so the micro-ROS middleware
#                        (rmw_microxrcedds, microxrcedds_client) is not linked.
#   rmw_rosette_firmware rmw_rosette and the RTPS core, compiled here with the
#                        board's toolchain, platform port, and capacity, so a
#                        change to either rebuilds only these sources.
#
# rcl calls the rmw_* entry points directly (a static image has no
# rmw_implementation loader), so rmw_rosette_firmware is the image's only rmw.
# The firmware defines rmw_rosette's network port
# (rmw_rosette/include/rmw_rosette/network_port.hpp) next to its network
# bring-up.
#
# Inputs:
#   ROSETTE_ROS2_C_STACK_DIR   the cross-built colcon install prefix, e.g.
#                              <micro-ROS workspace>/firmware/mcu_ws/install
#   RMW_ROSETTE_*              capacity limits (rmw_rosette/cmake/
#                              rmw_rosette_capacity.cmake); the defaults below
#                              suit one rclc node on a WIZnet board.
#
# Output: INTERFACE target `rmw_rosette_firmware`, the one target a firmware
# links. It carries the stack's headers, the STATIC library
# `rmw_rosette_static`, and one linker group holding that library together
# with every stack archive: rcl calls rmw_rosette, which calls rcutils and the
# type supports, so the archives reference each other in both directions and
# a group resolves them regardless of order.

if(TARGET rmw_rosette_firmware)
  return()
endif()

set(ROSETTE_ROS2_C_STACK_DIR "" CACHE PATH
    "Cross-built ROS 2 C client stack install prefix (micro-ROS workspace firmware/mcu_ws/install)")
if(NOT ROSETTE_ROS2_C_STACK_DIR)
  message(STATUS "rmw_rosette_firmware: ROSETTE_ROS2_C_STACK_DIR not set; rclc firmware targets skipped")
  return()
endif()
if(NOT EXISTS "${ROSETTE_ROS2_C_STACK_DIR}/lib/librcl.a")
  message(FATAL_ERROR "ROSETTE_ROS2_C_STACK_DIR='${ROSETTE_ROS2_C_STACK_DIR}' has no lib/librcl.a")
endif()

# ------------------------------------------------------------
# ros2_c_client_stack
# ------------------------------------------------------------
# Humble installs most headers under include/<package>/<package>/..., so every
# package directory is an include root; a few packages (rclc) install
# directly under include/.
file(GLOB _r2_include_roots LIST_DIRECTORIES true "${ROSETTE_ROS2_C_STACK_DIR}/include/*")
list(PREPEND _r2_include_roots "${ROSETTE_ROS2_C_STACK_DIR}/include")

# Every static library except the micro-ROS middleware. Message packages are
# included whole; the linker keeps only the type supports the firmware names.
file(GLOB _r2_libs "${ROSETTE_ROS2_C_STACK_DIR}/lib/lib*.a")
list(FILTER _r2_libs EXCLUDE REGEX "/lib(rmw_microxrcedds|microxrcedds_client)\\.a$")

add_library(ros2_c_client_stack_headers INTERFACE)
target_include_directories(ros2_c_client_stack_headers SYSTEM INTERFACE ${_r2_include_roots})

# ------------------------------------------------------------
# Capacity: one rclc node, no parameter server, no rosout (rcl is built with
# RCL_LOGGING_ENABLED=OFF), over a 1472-octet link. Cache defaults, so a board
# or the command line can override any of them.
# ------------------------------------------------------------
set(RMW_ROSETTE_MAX_NODES 1 CACHE STRING "Nodes per process")
set(RMW_ROSETTE_MAX_PUBLISHERS 4 CACHE STRING "Application publishers")
set(RMW_ROSETTE_MAX_SUBSCRIPTIONS 4 CACHE STRING "Application subscriptions")
set(RMW_ROSETTE_MAX_SERVICES 4 CACHE STRING "Application services")
set(RMW_ROSETTE_MAX_CLIENTS 2 CACHE STRING "Application service clients")
set(RMW_ROSETTE_PARAMETER_SERVICES_PER_NODE 0 CACHE STRING "Parameter services per node")
set(RMW_ROSETTE_BUILTIN_PUBLISHERS_PER_NODE 0 CACHE STRING "Publishers each node creates itself")
set(RMW_ROSETTE_MAX_REMOTE_PARTICIPANTS 8 CACHE STRING "Remote DDS participants tracked")
set(RMW_ROSETTE_MAX_MATCHED_ENDPOINTS 8 CACHE STRING "Remote matches per local endpoint")
set(RMW_ROSETTE_MAX_REMOTE_ENDPOINTS 64 CACHE STRING "Remote writers (and readers) known from SEDP")
set(RMW_ROSETTE_MAX_DATAGRAM_SIZE ${ROSETTE_RTPS_MAX_DATAGRAM_SIZE} CACHE STRING "Largest UDP datagram")
set(RMW_ROSETTE_MAX_SAMPLE_SIZE 4096 CACHE STRING "Largest serialized message")
set(RMW_ROSETTE_LARGE_BUFFERS 2 CACHE STRING "Blocks of RMW_ROSETTE_MAX_SAMPLE_SIZE")
set(RMW_ROSETTE_POOL_PROFILE firmware CACHE STRING "Buffer-pool layout")
include(${ROSETTE_ROOT}/rmw_rosette/cmake/rmw_rosette_capacity.cmake)

# ------------------------------------------------------------
# rmw_rosette_static
# ------------------------------------------------------------
set(_rr_dir ${ROSETTE_ROOT}/rmw_rosette)
add_library(rmw_rosette_static STATIC
  ${_rr_dir}/src/introspection_c_codec.cpp
  ${_rr_dir}/src/message_codec.cpp
  ${_rr_dir}/src/rmw_init.cpp
  ${_rr_dir}/src/rmw_node.cpp
  ${_rr_dir}/src/rmw_publisher.cpp
  ${_rr_dir}/src/rmw_subscription.cpp
  ${_rr_dir}/src/rmw_service.cpp
  ${_rr_dir}/src/rmw_client.cpp
  ${_rr_dir}/src/rmw_guard_condition.cpp
  ${_rr_dir}/src/rmw_wait_set.cpp
  ${_rr_dir}/src/rmw_graph.cpp
  ${_rr_dir}/src/rmw_implementation_info.cpp
  ${_rr_dir}/src/rmw_loaned_message.cpp
  ${_rr_dir}/src/rmw_qos.cpp
  ${_rr_dir}/src/rmw_qos_event.cpp
  ${_rr_dir}/src/rmw_serialize.cpp
  ${_rr_dir}/src/service_endpoints.cpp
  ${ROSETTE_ROOT}/src/rtps/buf_pool.cpp
  ${ROSETTE_ROOT}/src/rtps/message_parser.cpp
  ${ROSETTE_ROOT}/src/rtps/participant.cpp
  ${ROSETTE_ROOT}/src/rtps/participant_driver.cpp
  ${ROSETTE_ROOT}/src/rtps/sedp.cpp
  ${ROSETTE_ROOT}/src/rtps/endpoint_match_table.cpp
  ${ROSETTE_ROOT}/src/rtps/spdp.cpp
  ${ROSETTE_ROOT}/src/rtps/stateful_reader.cpp
  ${ROSETTE_ROOT}/src/rtps/sn_window.cpp
  ${ROSETTE_ROOT}/src/rtps/writer_proxy.cpp
  ${ROSETTE_ROOT}/src/rtps/transport.cpp
  ${ROSETTE_ROOT}/src/rtps/rtps_stdio_log.cpp
)
target_include_directories(rmw_rosette_static
  PUBLIC  ${_rr_dir}/include
  PRIVATE ${ROSETTE_ROOT}/include ${ROSETTE_ROOT}/include/rosette/rtps
)
set(ROSETTE_RMW_LOG_LEVEL 2 CACHE STRING "rosette::rtps log level inside rmw_rosette_static (0-5)")
# micro-ROS generates CDR callbacks for every message; rmw_rosette prefers
# them (rmw_rosette/include/rmw_rosette/message_codec.hpp), and rcl built with
# ROSIDL_TYPESUPPORT_SINGLE_TYPESUPPORT hands out only those handles.
target_compile_definitions(rmw_rosette_static PRIVATE
  RTPS_LOG_LEVEL=${ROSETTE_RMW_LOG_LEVEL}
  RMW_ROSETTE_HAVE_MICROXRCEDDS_TYPESUPPORT
)
rmw_rosette_apply_capacity(rmw_rosette_static)
target_link_libraries(rmw_rosette_static PUBLIC ros2_c_client_stack_headers rosette_platform rosette_net)

# ------------------------------------------------------------
# rmw_rosette_firmware
# ------------------------------------------------------------
add_library(rmw_rosette_firmware INTERFACE)
target_link_libraries(rmw_rosette_firmware INTERFACE
  rmw_rosette_static
  "-Wl,--start-group" $<TARGET_FILE:rmw_rosette_static> ${_r2_libs} "-Wl,--end-group")
