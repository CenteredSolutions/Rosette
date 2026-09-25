cmake_minimum_required(VERSION 3.20)
project(rmw_rosette CXX C)

# rmw_rosette: an rmw implementation over Rosette's DDSI-RTPS stack.
#
# The RTPS core (include/rosette/rtps, src/rtps) and its POSIX platform and
# socket glue are compiled directly into librmw_rosette, so the package has no
# build dependency on the rest of the Rosette tree and the shared library is
# self-contained when rmw_implementation dlopens it.
#
# Message serialization uses rosidl_typesupport_introspection_c, which every
# ROS 2 installation provides. When rosidl_typesupport_microxrcedds_c is also
# available (micro-ROS firmware workspaces), its generated callbacks are
# compiled in and preferred; see include/rmw_rosette/message_codec.hpp.

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rcutils REQUIRED)
find_package(rmw REQUIRED)
find_package(rosidl_runtime_c REQUIRED)
find_package(rosidl_typesupport_introspection_c REQUIRED)
find_package(rosidl_typesupport_microxrcedds_c QUIET)

include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/rmw_rosette_capacity.cmake)

# Root of the Rosette repository, which provides the RTPS core and platform glue.
set(ROSETTE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/.." CACHE PATH
    "Rosette repository root (provides include/rosette/rtps and src/rtps)")
get_filename_component(ROSETTE_ROOT "${ROSETTE_ROOT}" REALPATH)

set(ROSETTE_RTPS_SOURCES
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
  ${ROSETTE_ROOT}/src/platform/posix/host_runtime.cpp
  ${ROSETTE_ROOT}/src/platform/posix/rtps_posix_log.cpp
  ${ROSETTE_ROOT}/src/net/posix/socket.cpp
)

add_library(rmw_rosette SHARED
  src/introspection_c_codec.cpp
  src/message_codec.cpp
  src/rmw_init.cpp
  src/rmw_node.cpp
  src/rmw_publisher.cpp
  src/rmw_subscription.cpp
  src/rmw_service.cpp
  src/rmw_client.cpp
  src/rmw_guard_condition.cpp
  src/rmw_wait_set.cpp
  src/rmw_graph.cpp
  src/rmw_implementation_info.cpp
  src/rmw_loaned_message.cpp
  src/rmw_qos.cpp
  src/rmw_qos_event.cpp
  src/rmw_serialize.cpp
  src/service_endpoints.cpp
  ${ROSETTE_RTPS_SOURCES}
)

target_include_directories(rmw_rosette
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${ROSETTE_ROOT}/include
    ${ROSETTE_ROOT}/include/rosette/rtps
)

# Platform port (include/rosette/platform/platform.hpp). A host colcon build
# runs on POSIX; a micro-ROS firmware workspace sets its board's OS and chip.
set(ROSETTE_OS posix CACHE STRING "Platform OS layer: posix, freertos, baremetal")
set(ROSETTE_CHIP host CACHE STRING "Platform chip layer: host, rp2")
include(${ROSETTE_ROOT}/cmake/rosette_platform.cmake)
target_link_libraries(rmw_rosette PRIVATE rosette_platform)

# RTPS log levels (rosette/rtps/log.hpp): 0 none, 1 error, 2 warning,
# 3 info, 4 debug, 5 trace; output goes to stderr.
set(RMW_ROSETTE_RTPS_LOG_LEVEL 2 CACHE STRING "rosette::rtps log level (0-5)")
target_compile_definitions(rmw_rosette PRIVATE
  RTPS_LOG_LEVEL=${RMW_ROSETTE_RTPS_LOG_LEVEL}
)

rmw_rosette_apply_capacity(rmw_rosette)

target_link_libraries(rmw_rosette PUBLIC
  rcutils::rcutils
  rmw::rmw
  rosidl_runtime_c::rosidl_runtime_c
  rosidl_typesupport_introspection_c::rosidl_typesupport_introspection_c
)

if(rosidl_typesupport_microxrcedds_c_FOUND)
  target_compile_definitions(rmw_rosette PRIVATE RMW_ROSETTE_HAVE_MICROXRCEDDS_TYPESUPPORT)
  target_link_libraries(rmw_rosette PUBLIC
    rosidl_typesupport_microxrcedds_c::rosidl_typesupport_microxrcedds_c)
endif()

# Registers the package in the ament index so RMW_IMPLEMENTATION=rmw_rosette
# resolves; rmw_implementation then loads librmw_rosette.so by name.
ament_index_register_resource("rmw_implementation" CONTENT_FILE
  "${CMAKE_CURRENT_SOURCE_DIR}/resource/rmw_rosette")

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS rmw_rosette
  EXPORT export_rmw_rosette
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_rmw_rosette HAS_LIBRARY_TARGET)
ament_export_dependencies(rcutils rmw rosidl_runtime_c rosidl_typesupport_introspection_c)
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(std_msgs REQUIRED)

  # Each test binary links librmw_rosette directly, so it exercises this
  # implementation without going through rmw_implementation's loader.
  foreach(test_name test_introspection_c_codec test_guard_condition test_pub_sub_loopback)
    ament_add_gtest(${test_name} test/${test_name}.cpp TIMEOUT 60)
    target_link_libraries(${test_name} rmw_rosette)
    # Tests include rmw_rosette headers whose layouts depend on the capacity model.
    rmw_rosette_apply_capacity(${test_name})
    target_include_directories(${test_name} PRIVATE ${ROSETTE_ROOT}/include)
    # Same platform selection as the library, without a second copy of its state.
    target_compile_definitions(${test_name} PRIVATE
      $<TARGET_PROPERTY:rosette_platform,INTERFACE_COMPILE_DEFINITIONS>)
    ament_target_dependencies(${test_name} geometry_msgs sensor_msgs std_msgs)
  endforeach()
endif()

ament_package()
