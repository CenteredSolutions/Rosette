# rosette_platform — the build-time platform port (include/rosette/platform/platform.hpp).
#
# Inputs (normally set by the board's board.cmake):
#   ROSETTE_OS        posix | freertos | baremetal, or a port directory name
#   ROSETTE_CHIP      host  | rp2, or a port directory name
#   ROSETTE_ROOT      repository root (defaults to this file's parent directory)
#   ROSETTE_PORT_DIRS further directories laid out like ports/, searched after
#                     it, for ports kept outside this repository
#
# A name is a port when <dir>/<axis>/<name>/port.cmake exists (axis `os` or
# `chip`). That file adds the port's sources and include directories to
# `rosette_platform`; ROSETTE_PORT_DIR names its directory while it runs.
# The OS port's include directory holds `rosette_os_port.hpp`, which
# platform.hpp includes by that fixed name. The names listed above are the
# ports that have not moved under ports/ yet (docs/design/ports_move_plan.md).
#
# Output: STATIC library target `rosette_platform`. It compiles the chip layer
# and WallClock, and carries PUBLIC ROSETTE_OS_<OS> / ROSETTE_CHIP_<CHIP>
# definitions, so every target that links it (the RTPS core, rmw_rosette, net
# backends) sees the same port. Linking two different ports into one image is
# impossible by construction: there is one target of this name per build.

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

set(_rp_valid_os posix freertos baremetal)
set(_rp_valid_chip host rp2)
if(NOT _rp_os_port AND NOT ROSETTE_OS IN_LIST _rp_valid_os)
  message(FATAL_ERROR "ROSETTE_OS='${ROSETTE_OS}' is not one of: ${_rp_valid_os}, "
                      "nor a port with os/${ROSETTE_OS}/port.cmake under ports/ or ROSETTE_PORT_DIRS")
endif()
if(NOT _rp_chip_port AND NOT ROSETTE_CHIP IN_LIST _rp_valid_chip)
  message(FATAL_ERROR "ROSETTE_CHIP='${ROSETTE_CHIP}' is not one of: ${_rp_valid_chip}, "
                      "nor a port with chip/${ROSETTE_CHIP}/port.cmake under ports/ or ROSETTE_PORT_DIRS")
endif()
if(ROSETTE_OS STREQUAL "baremetal" AND NOT ROSETTE_CHIP STREQUAL "rp2")
  message(FATAL_ERROR "ROSETTE_OS=baremetal is implemented on the Pico SDK; use ROSETTE_CHIP=rp2")
endif()

add_library(rosette_platform STATIC ${ROSETTE_ROOT}/src/platform/wall_clock.cpp)
target_include_directories(rosette_platform PUBLIC ${ROSETTE_ROOT}/include)
target_compile_features(rosette_platform PUBLIC cxx_std_17)

if(_rp_chip_port)
  set(ROSETTE_PORT_DIR "${_rp_chip_port}")
  include("${_rp_chip_port}/port.cmake")
else()
  target_sources(rosette_platform PRIVATE
    ${ROSETTE_ROOT}/src/platform/chip/${ROSETTE_CHIP}/chip_${ROSETTE_CHIP}.cpp)
endif()
if(_rp_os_port)
  set(ROSETTE_PORT_DIR "${_rp_os_port}")
  include("${_rp_os_port}/port.cmake")
endif()
unset(ROSETTE_PORT_DIR)

string(TOUPPER "${ROSETTE_OS}" _rp_os_upper)
string(TOUPPER "${ROSETTE_CHIP}" _rp_chip_upper)
target_compile_definitions(rosette_platform PUBLIC
  ROSETTE_OS_${_rp_os_upper}
  ROSETTE_CHIP_${_rp_chip_upper}
)

if(ROSETTE_OS STREQUAL "posix")
  find_package(Threads REQUIRED)
  target_link_libraries(rosette_platform PUBLIC Threads::Threads)
elseif(ROSETTE_OS STREQUAL "freertos")
  target_link_libraries(rosette_platform PUBLIC freertos_kernel)
endif()

if(ROSETTE_CHIP STREQUAL "rp2")
  target_link_libraries(rosette_platform PUBLIC
    pico_stdlib pico_sync pico_time pico_unique_id hardware_gpio hardware_irq hardware_sync
    hardware_timer)
endif()

if(CMAKE_POSITION_INDEPENDENT_CODE OR ROSETTE_CHIP STREQUAL "host")
  set_target_properties(rosette_platform PROPERTIES POSITION_INDEPENDENT_CODE ON)
endif()
