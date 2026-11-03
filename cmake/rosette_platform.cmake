# SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
# SPDX-License-Identifier: Apache-2.0

# rosette_platform — the build-time platform port (include/rosette/platform/platform.hpp).
#
# Inputs (normally set by the board's board.cmake):
#   ROSETTE_OS        an OS port: baremetal | freertos | posix | stub | ...
#   ROSETTE_CHIP      a chip port: host | rp2 | stub | ...
#   ROSETTE_ROOT      repository root (defaults to this file's parent directory)
#   ROSETTE_PORT_DIRS further directories laid out like ports/, searched after
#                     it, for ports kept outside this repository
#
# A name is a port when <dir>/<axis>/<name>/port.cmake exists
# (cmake/rosette_ports.cmake); nothing here lists the names. That file adds the port's sources,
# include directories, and libraries to `rosette_platform`, and a chip port
# sets ROSETTE_CHIP_STORAGE_SOURCES to its LittleFS block device;
# ROSETTE_PORT_DIR names the port's directory while it runs. The OS port's
# include directory holds `rosette_os_port.hpp`, which platform.hpp includes
# by that fixed name. A chip port's optional prelude.cmake runs earlier,
# before project() (cmake/rosette_prelude.cmake), to set up a cross toolchain.
#
# A chip port may also define rosette_chip_add_image_outputs(<target>), which
# rosette_add_image_outputs(<target>) calls to make a firmware image's
# flashable files and console (the rp2 port: .uf2 and USB stdio); on a port
# without one it does nothing. ROSETTE_CHIP_TEST_LIBRARIES names what the
# platform contract test needs to start a second core, if the chip has one.
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

include(${CMAKE_CURRENT_LIST_DIR}/rosette_ports.cmake)
rosette_require_port(_rp_os_port os "${ROSETTE_OS}" ROSETTE_OS)
rosette_require_port(_rp_chip_port chip "${ROSETTE_CHIP}" ROSETTE_CHIP)

add_library(rosette_platform STATIC
  ${ROSETTE_ROOT}/src/platform/wall_clock.cpp
  ${ROSETTE_ROOT}/src/support/fnv1a.cpp)
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

# Makes <target>'s flashable outputs and console, as its chip port defines them.
function(rosette_add_image_outputs target)
  if(COMMAND rosette_chip_add_image_outputs)
    rosette_chip_add_image_outputs(${target})
  endif()
endfunction()
