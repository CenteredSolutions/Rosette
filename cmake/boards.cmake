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
# ROSETTE_BOARD_ENABLE_PICO_SDK / _FREERTOS / _WIZNET are not board settings:
# the rp2 chip port, the freertos OS port, and the wiznet network port set them
# ON, for examples that use one of those directly.

function(rosette_select_board BOARD_NAME)
    set(board_dir "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/../boards/${BOARD_NAME}")
    set(board_cmake "${board_dir}/board.cmake")

    if(NOT EXISTS "${board_cmake}")
        message(FATAL_ERROR
            "Unknown Rosette board '${BOARD_NAME}': expected ${board_cmake}")
    endif()

    set(ROSETTE_BOARD_DIR "${board_dir}" PARENT_SCOPE)
    include("${board_cmake}")
endfunction()
