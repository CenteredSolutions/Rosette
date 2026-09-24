# rosette_platform — the build-time platform port (include/rosette/platform/platform.hpp).
#
# Inputs (normally set by the board's board.cmake):
#   ROSETTE_OS    posix | freertos | baremetal
#   ROSETTE_CHIP  host  | rp2
#   ROSETTE_ROOT  repository root (defaults to this file's parent directory)
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

set(_rp_valid_os posix freertos baremetal)
set(_rp_valid_chip host rp2)
if(NOT ROSETTE_OS IN_LIST _rp_valid_os)
  message(FATAL_ERROR "ROSETTE_OS='${ROSETTE_OS}' is not one of: ${_rp_valid_os}")
endif()
if(NOT ROSETTE_CHIP IN_LIST _rp_valid_chip)
  message(FATAL_ERROR "ROSETTE_CHIP='${ROSETTE_CHIP}' is not one of: ${_rp_valid_chip}")
endif()
if(ROSETTE_OS STREQUAL "baremetal" AND NOT ROSETTE_CHIP STREQUAL "rp2")
  message(FATAL_ERROR "ROSETTE_OS=baremetal is implemented on the Pico SDK; use ROSETTE_CHIP=rp2")
endif()

add_library(rosette_platform STATIC
  ${ROSETTE_ROOT}/src/platform/chip/${ROSETTE_CHIP}/chip_${ROSETTE_CHIP}.cpp
  ${ROSETTE_ROOT}/src/platform/wall_clock.cpp
)
target_include_directories(rosette_platform PUBLIC ${ROSETTE_ROOT}/include)
target_compile_features(rosette_platform PUBLIC cxx_std_17)

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
    pico_stdlib pico_sync pico_time pico_unique_id hardware_sync hardware_timer)
endif()

if(CMAKE_POSITION_INDEPENDENT_CODE OR ROSETTE_CHIP STREQUAL "host")
  set_target_properties(rosette_platform PROPERTIES POSITION_INDEPENDENT_CODE ON)
endif()
