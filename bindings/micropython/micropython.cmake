# MicroPython user module integration for Rosette.
#
# Expected usage:
#   set(USER_C_MODULES /abs/path/to/Rosette/bindings/micropython)
#   include(${MICROPY_DIR}/py/usermod.cmake)

set(ROSETTE_BINDINGS_DIR ${CMAKE_CURRENT_LIST_DIR})
get_filename_component(ROSETTE_REPO_DIR ${ROSETTE_BINDINGS_DIR}/../.. ABSOLUTE)
set(ROSETTE_MICRO_ROS_TRANSPORT "udp" CACHE STRING
    "Rosette MicroPython backend transport mode: udp or custom")
set_property(CACHE ROSETTE_MICRO_ROS_TRANSPORT PROPERTY STRINGS udp custom)

set(ROSETTE_MICRO_ROS_INCLUDE_DIR "" CACHE PATH
    "Include directory for a prebuilt micro-ROS generate_lib output")
set(ROSETTE_MICRO_ROS_LIBRARY "" CACHE FILEPATH
    "Path to a prebuilt libmicroros.a for the MicroPython Rosette binding")

set(ROSETTE_MICRO_ROS_CACHE_FILE ${ROSETTE_REPO_DIR}/build-micro-ros/CMakeCache.txt)
if(EXISTS ${ROSETTE_MICRO_ROS_CACHE_FILE})
    if(ROSETTE_MICRO_ROS_INCLUDE_DIR STREQUAL "")
        file(STRINGS ${ROSETTE_MICRO_ROS_CACHE_FILE}
            _rosette_micro_ros_include_line
            REGEX "^ROSETTE_MICRO_ROS_INCLUDE_DIR:PATH=")
        if(_rosette_micro_ros_include_line)
            string(REGEX REPLACE "^ROSETTE_MICRO_ROS_INCLUDE_DIR:PATH=" ""
                ROSETTE_MICRO_ROS_INCLUDE_DIR "${_rosette_micro_ros_include_line}")
        endif()
    endif()
    if(ROSETTE_MICRO_ROS_LIBRARY STREQUAL "")
        file(STRINGS ${ROSETTE_MICRO_ROS_CACHE_FILE}
            _rosette_micro_ros_library_line
            REGEX "^ROSETTE_MICRO_ROS_LIBRARY:FILEPATH=")
        if(_rosette_micro_ros_library_line)
            string(REGEX REPLACE "^ROSETTE_MICRO_ROS_LIBRARY:FILEPATH=" ""
                ROSETTE_MICRO_ROS_LIBRARY "${_rosette_micro_ros_library_line}")
        endif()
    endif()
endif()

set(ROSETTE_MICROPY_HAS_MICRO_ROS 0)
if(EXISTS ${ROSETTE_MICRO_ROS_INCLUDE_DIR}/rcl/rcl.h AND
   EXISTS ${ROSETTE_MICRO_ROS_LIBRARY})
    set(ROSETTE_MICROPY_HAS_MICRO_ROS 1)
endif()

set(ROSETTE_MICROPY_BACKEND_TRANSPORT_CUSTOM 0)
if(ROSETTE_MICRO_ROS_TRANSPORT STREQUAL "custom")
    set(ROSETTE_MICROPY_BACKEND_TRANSPORT_CUSTOM 1)
endif()

add_library(usermod_rosette INTERFACE)
target_sources(usermod_rosette INTERFACE
    ${ROSETTE_BINDINGS_DIR}/modrosette.c
    ${ROSETTE_BINDINGS_DIR}/rosette_mpy_backend_runtime.cpp
    ${ROSETTE_BINDINGS_DIR}/rosette_mpy_context.cpp
    ${ROSETTE_BINDINGS_DIR}/rosette_mpy_node.cpp
    ${ROSETTE_BINDINGS_DIR}/rosette_mpy_pubsub.cpp
    ${ROSETTE_BINDINGS_DIR}/rosette_mpy_schema.cpp
    ${ROSETTE_BINDINGS_DIR}/rosette_mpy_transport.cpp
    ${ROSETTE_BINDINGS_DIR}/rosette_mpy_stream.cpp)

target_include_directories(usermod_rosette INTERFACE
    ${ROSETTE_BINDINGS_DIR}
    ${ROSETTE_REPO_DIR}
    ${ROSETTE_REPO_DIR}/include)

if(ROSETTE_MICROPY_HAS_MICRO_ROS)
    target_include_directories(usermod_rosette INTERFACE
        ${ROSETTE_MICRO_ROS_INCLUDE_DIR})
    target_link_libraries(usermod_rosette INTERFACE
        ${ROSETTE_MICRO_ROS_LIBRARY})
endif()

target_compile_options(usermod_rosette INTERFACE
    $<$<COMPILE_LANGUAGE:CXX>:-std=gnu++17>)
target_compile_definitions(usermod_rosette INTERFACE
    ROSETTE_MICROPY_HAS_MICRO_ROS=${ROSETTE_MICROPY_HAS_MICRO_ROS}
    ROSETTE_MICROPY_BACKEND_TRANSPORT_CUSTOM=${ROSETTE_MICROPY_BACKEND_TRANSPORT_CUSTOM})
target_link_libraries(usermod INTERFACE usermod_rosette)
