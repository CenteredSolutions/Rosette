cmake_minimum_required(VERSION 3.20)

project(milkdds_ros2_tests CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Python3 COMPONENTS Interpreter REQUIRED)

option(MILKDDS_ENABLE_ROS2_DOCKER_TESTS
       "Enable ROS 2 Docker interop tests (requires docker + ROS image)"
       OFF)

set(ROS2_GEN_DIR ${CMAKE_CURRENT_BINARY_DIR}/generated/interfaces)
set(ROS2_STRING_MSG ${CMAKE_CURRENT_SOURCE_DIR}/../../ros/interfaces/msg/std_msgs__String.msg)
set(ROS2_STRING_HDR ${ROS2_GEN_DIR}/std_msgs__String.h)
set(MSG_GEN_ENTRY ${CMAKE_CURRENT_SOURCE_DIR}/../../tools/msg_gen/msg_gen.py)

add_custom_command(
    OUTPUT ${ROS2_STRING_HDR}
    COMMAND ${CMAKE_COMMAND} -E make_directory ${ROS2_GEN_DIR}
    COMMAND ${Python3_EXECUTABLE}
            ${MSG_GEN_ENTRY}
            ${ROS2_STRING_MSG}
            --output-dir ${ROS2_GEN_DIR}
    DEPENDS ${ROS2_STRING_MSG} ${MSG_GEN_ENTRY}
    COMMENT "Generating std_msgs__String.h with msg_gen"
    VERBATIM
)

add_custom_target(ros2_generated_interfaces DEPENDS ${ROS2_STRING_HDR})

add_executable(ros2_posix_chatter_smoke
    posix_chatter_smoke.cpp
    ../../platform/posix/rtps_posix_platform.cpp
    ../../platform/posix/rtps_posix_log.cpp
    ../../rtps/message_parser.cpp
    ../../rtps/stateful_reader.cpp
    ../../rtps/spdp.cpp
    ../../rtps/sedp.cpp
    ../../rtps/participant.cpp
    ../../rtps/buf_pool.cpp
    ../../rtps/transport.cpp
    ../../net/socket.cpp
    ../../net/posix/socket.cpp
)

add_dependencies(ros2_posix_chatter_smoke ros2_generated_interfaces)

target_include_directories(ros2_posix_chatter_smoke PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../..
    ${ROS2_GEN_DIR}
)

target_compile_definitions(ros2_posix_chatter_smoke PRIVATE
    RTPS_NO_FREERTOS
    RTPS_LOG_LEVEL=4
    RTPS_BUFPOOL_DEBUG=1
)

if (MILKDDS_ENABLE_ROS2_DOCKER_TESTS)
    enable_testing()
    add_test(
        NAME ros2_docker_chatter_smoke
        COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/run_ros2_docker_smoke.sh
                $<TARGET_FILE:ros2_posix_chatter_smoke>
    )
endif()
