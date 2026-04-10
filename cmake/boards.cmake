# cmake/boards.cmake
#
# Defines rtps_select_board(BOARD_NAME) which:
#   1. Sets PICO_BOARD (in cache) so the Pico SDK targets the right chip.
#   2. Sets RTPS_PROCESSOR, RTPS_SPI_* and RTPS_W5100S_* variables
#      for use by the rest of CMakeLists.txt and by preprocessor defines
#      compiled into board_config.h.
#
# Board-specific C defines are NOT generated here — they live in each
# board's board_config.h so that the values are visible to the compiler
# without needing generator expressions.  CMake variables here are used
# only for messages and to select source files.

function(rtps_select_board BOARD_NAME)
    include(${CMAKE_CURRENT_LIST_DIR}/boards/${BOARD_NAME}/board.cmake)
endfunction()
