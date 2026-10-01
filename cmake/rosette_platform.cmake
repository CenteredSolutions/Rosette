# rosette_platform — the build-time platform port (include/rosette/platform/platform.hpp).
#
# Inputs (normally set by the board's board.cmake):
#   ROSETTE_OS        an OS port: baremetal | freertos | posix | stub | ...
#   ROSETTE_CHIP      a chip port: host | rp2 | stub | ...
#   ROSETTE_ROOT      repository root (defaults to this file's parent directory)
#   ROSETTE_PORT_DIRS further directories laid out like ports/, searched after
#                     it, for ports kept outside this repository
#
# A name is a port when <dir>/<axis>/<name>/port.cmake exists (axis `os` or
# `chip`); nothing here lists the names. That file adds the port's sources,
# include directories, and libraries to `rosette_platform`, and a chip port
# sets ROSETTE_CHIP_STORAGE_SOURCES to its LittleFS block device;
# ROSETTE_PORT_DIR names the port's directory while it runs. The OS port's
# include directory holds `rosette_os_port.hpp`, which platform.hpp includes
# by that fixed name.
#
# Output: STATIC library target `rosette_platform`. It compiles the chip layer
# and WallClock, and carries PUBLIC ROSETTE_OS_<OS> / ROSETTE_CHIP_<CHIP>
# definitions, so a test can report which port it ran on. Linking two
# different ports into one image is impossible by construction: there is one
# target of this name per build.

if(TARGET rosette_platform)
  return()
endif()

if(NOT DEFINED ROSETTE_ROOT)
  get_filename_component(ROSETTE_ROOT "${CMAKE_CURRENT_LIST_DIR}/.." REALPATH)
endif()

# Sets <out> to the directory of port <name> on <axis>, or "" when there is none.
function(_rosette_find_port out axis name)
  foreach(_dir IN LISTS ROSETTE_ROOT_PORTS ROSETTE_PORT_DIRS)
    if(NOT name STREQUAL "" AND EXISTS "${_dir}/${axis}/${name}/port.cmake")
      set(${out} "${_dir}/${axis}/${name}" PARENT_SCOPE)
      return()
    endif()
  endforeach()
  set(${out} "" PARENT_SCOPE)
endfunction()

set(ROSETTE_ROOT_PORTS "${ROSETTE_ROOT}/ports")
_rosette_find_port(_rp_os_port os "${ROSETTE_OS}")
_rosette_find_port(_rp_chip_port chip "${ROSETTE_CHIP}")

if(NOT _rp_os_port)
  message(FATAL_ERROR "ROSETTE_OS='${ROSETTE_OS}' names no port: no os/${ROSETTE_OS}/port.cmake "
                      "under ${ROSETTE_ROOT_PORTS} or ROSETTE_PORT_DIRS")
endif()
if(NOT _rp_chip_port)
  message(FATAL_ERROR "ROSETTE_CHIP='${ROSETTE_CHIP}' names no port: no chip/${ROSETTE_CHIP}/port.cmake "
                      "under ${ROSETTE_ROOT_PORTS} or ROSETTE_PORT_DIRS")
endif()

add_library(rosette_platform STATIC ${ROSETTE_ROOT}/src/platform/wall_clock.cpp)
target_include_directories(rosette_platform PUBLIC ${ROSETTE_ROOT}/include)
target_compile_features(rosette_platform PUBLIC cxx_std_17)

set(ROSETTE_PORT_DIR "${_rp_chip_port}")
include("${_rp_chip_port}/port.cmake")
set(ROSETTE_PORT_DIR "${_rp_os_port}")
include("${_rp_os_port}/port.cmake")
unset(ROSETTE_PORT_DIR)

string(TOUPPER "${ROSETTE_OS}" _rp_os_upper)
string(TOUPPER "${ROSETTE_CHIP}" _rp_chip_upper)
target_compile_definitions(rosette_platform PUBLIC
  ROSETTE_OS_${_rp_os_upper}
  ROSETTE_CHIP_${_rp_chip_upper}
)

if(CMAKE_POSITION_INDEPENDENT_CODE)
  set_target_properties(rosette_platform PROPERTIES POSITION_INDEPENDENT_CODE ON)
endif()
