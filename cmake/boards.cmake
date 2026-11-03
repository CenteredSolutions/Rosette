# SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
# SPDX-License-Identifier: Apache-2.0

# cmake/boards.cmake
#
# Defines rosette_select_board(BOARD_NAME), which loads boards/<name>/board.cmake
# before project(). A board chooses one port on each axis and supplies its
# wiring; board.cmake sets, in the caller's scope:
#
#   ROSETTE_CHIP, ROSETTE_OS, ROSETTE_NET_STACK
#                         the ports (ports/chip/, ports/os/, ports/net/)
#   ROSETTE_BOARD_COMMON  optional: a composition under boards/common/ that the
#                         board shares with others; its bsp.cmake adds to the
#                         rosette_bsp target
#   ROSETTE_PROCESSOR, pins, and whatever its chip port's prelude reads
#                         (PICO_BOARD for rp2)
#
# A board with a board_init.c gets a rosette_bsp library (the root
# CMakeLists). ROSETTE_BOARD_DIR names the board's directory.
#
# A name is a board when <dir>/<name>/board.cmake exists, for <dir> this
# repository's boards/ and then each entry of ROSETTE_BOARD_DIRS (directories
# laid out like boards/, for a firmware project's own boards), the same way
# ROSETTE_PORT_DIRS extends ports/.
#
# ROSETTE_BOARD_ENABLE_PICO_SDK / _FREERTOS / _WIZNET are not board settings:
# the rp2 chip port, the freertos OS port, and the wiznet network port set them
# ON, for examples that use one of those directly.

function(rosette_select_board BOARD_NAME)
    get_filename_component(_root_boards "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/../boards" REALPATH)
    set(board_dir "")
    foreach(_dir IN LISTS _root_boards ROSETTE_BOARD_DIRS)
        if(EXISTS "${_dir}/${BOARD_NAME}/board.cmake")
            set(board_dir "${_dir}/${BOARD_NAME}")
            break()
        endif()
    endforeach()

    if(NOT board_dir)
        message(FATAL_ERROR
            "Unknown Rosette board '${BOARD_NAME}': no ${BOARD_NAME}/board.cmake under "
            "${_root_boards} or ROSETTE_BOARD_DIRS")
    endif()

    set(board_cmake "${board_dir}/board.cmake")

    set(ROSETTE_BOARD_DIR "${board_dir}" PARENT_SCOPE)
    include("${board_cmake}")
endfunction()
