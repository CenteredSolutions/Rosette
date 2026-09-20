cmake_minimum_required(VERSION 3.20)
project(rmw_rosette CXX C)

# ------------------------------------------------------------
# NOTE — build-integration status
#
# This CMakeLists.txt scaffolds an ament_cmake package structure so the
# file layout matches what `micro_ros_setup` / colcon expect from an rmw
# implementation. It has NOT been exercised against a real colcon
# workspace yet. Before this builds for real, expect to:
#
#   1. Point ROSETTE_RTPS_DIR at the repo root's include/rosette/rtps +
#      src/rtps (currently built as the `rtps_stack` target in the
#      top-level CMakeLists.txt) — either via a relative path for
#      in-tree builds, or a proper find_package/vendor step for the
#      standalone colcon build micro_ros_setup performs.
#   2. Confirm the ament_index resource registration below against
#      whatever `rmw_implementation_cmake` currently expects — the
#      `register_rmw_implementation()`-equivalent macro name/args may
#      have moved between ROS 2 distros.
#   3. Fill in the full rmw C ABI surface (see src/CMakeLists sources
#      list below) — only the functions scoped in the gap analysis are
#      stubbed so far; anything rcl/rclc calls that isn't listed here
#      will fail to link.
# ------------------------------------------------------------

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rcutils REQUIRED)
find_package(rmw REQUIRED)
find_package(rosidl_runtime_c REQUIRED)
find_package(rosidl_typesupport_microxrcedds_c REQUIRED)

# rosette::rtps — the embedded RTPS core (participant, discovery, stateful
# reader/writer, history cache). Built in-tree today via the top-level
# CMakeLists.txt's `rtps_stack` target; referenced here by relative path
# until it's packaged for standalone colcon consumption.
set(ROSETTE_RTPS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/.." CACHE PATH
    "Path to the rosette repo root (provides include/rosette/rtps, src/rtps)")

add_library(rmw_rosette
  src/rmw_init.cpp
  src/rmw_node.cpp
  src/rmw_publisher.cpp
  src/rmw_subscription.cpp
  src/rmw_service.cpp
  src/rmw_client.cpp
  src/rmw_guard_condition.cpp
  src/rmw_wait_set.cpp
  src/rmw_graph.cpp
  src/rmw_serialize.cpp
  src/rmw_qos.cpp
)

target_include_directories(rmw_rosette PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${ROSETTE_RTPS_DIR}/include
)

target_link_libraries(rmw_rosette PUBLIC
  rcutils::rcutils
  rmw::rmw
  rosidl_runtime_c::rosidl_runtime_c
  rosidl_typesupport_microxrcedds_c::rosidl_typesupport_microxrcedds_c
)

# TODO: link the actual rtps_stack sources/target once the vendoring path
# above is resolved (see NOTE at top of file).

# ------------------------------------------------------------
# ament_index registration — what makes `RMW_IMPLEMENTATION=rmw_rosette`
# resolvable at rcl init time. Verify this against the current
# rmw_implementation_cmake macro before relying on it.
# ------------------------------------------------------------
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
ament_export_dependencies(rcutils rmw rosidl_runtime_c rosidl_typesupport_microxrcedds_c)
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_pub_sub_loopback test/test_pub_sub_loopback.cpp)
  target_link_libraries(test_pub_sub_loopback rmw_rosette)
endif()

ament_package()
