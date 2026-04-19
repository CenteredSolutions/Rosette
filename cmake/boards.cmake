# cmake/boards.cmake
#
# Defines rtps_select_board(BOARD_NAME) which loads board-specific build
# settings from boards/<name>/board.cmake.

function(rtps_select_board BOARD_NAME)
    set(board_dir "${CMAKE_CURRENT_FUNCTION_LIST_DIR}/../boards/${BOARD_NAME}")
    set(board_cmake "${board_dir}/board.cmake")

    if(NOT EXISTS "${board_cmake}")
        message(FATAL_ERROR
            "Unknown RTPS board '${BOARD_NAME}': expected ${board_cmake}")
    endif()

    set(RTPS_BOARD_DIR "${board_dir}" PARENT_SCOPE)
    include("${board_cmake}")
endfunction()
