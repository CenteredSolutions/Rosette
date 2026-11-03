# SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
# SPDX-License-Identifier: Apache-2.0

# cmake/rosette_ports.cmake
#
# rosette_find_port(<out> <axis> <name>) sets <out> to the directory of the
# port <name> on <axis> (chip, os, or net), or to "" when there is none. A name
# is a port when <dir>/<axis>/<name>/port.cmake exists, for <dir> this
# repository's ports/ and then each entry of ROSETTE_PORT_DIRS (directories
# laid out like ports/, for ports kept outside this repository). Nothing here
# lists the names.
#
# rosette_require_port(<out> <axis> <name> <variable>) does the same, and stops
# the configure with an error naming <variable> when there is no such port.

include_guard(GLOBAL)

function(rosette_find_port out axis name)
  get_filename_component(_root_ports "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/../ports" REALPATH)
  foreach(_dir IN LISTS _root_ports ROSETTE_PORT_DIRS)
    if(NOT name STREQUAL "" AND EXISTS "${_dir}/${axis}/${name}/port.cmake")
      set(${out} "${_dir}/${axis}/${name}" PARENT_SCOPE)
      return()
    endif()
  endforeach()
  set(${out} "" PARENT_SCOPE)
endfunction()

function(rosette_require_port out axis name variable)
  rosette_find_port(_found ${axis} "${name}")
  if(NOT _found)
    message(FATAL_ERROR "${variable}='${name}' names no port: no ${axis}/${name}/port.cmake "
                        "under ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/../ports or ROSETTE_PORT_DIRS")
  endif()
  set(${out} "${_found}" PARENT_SCOPE)
endfunction()
