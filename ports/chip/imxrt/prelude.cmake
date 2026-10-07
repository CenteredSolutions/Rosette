# ports/chip/imxrt/prelude.cmake — the i.MX RT port's setup before project().
#
# Included by cmake/rosette_prelude.cmake, before the first project() call.
# Installs the Cortex-M7 cross toolchain (arm_gcc_cortex_m7.cmake) and makes
# two vendor trees available to port.cmake:
#
#   MCUX_SDK_PATH  NXP MCUXpresso SDK (github.com/nxp-mcuxpresso/mcux-sdk):
#                  the device headers, startup, system init, linker scripts,
#                  XIP boot header, and the peripheral drivers the port uses
#   CMSIS_PATH     Arm CMSIS 5 (github.com/ARM-software/CMSIS_5): the Core
#                  headers the SDK's device header includes
#
# Each is a local checkout when the variable (CMake or environment) names one;
# otherwise FetchContent fetches a pinned release. The fetch is a sparse,
# blob-less clone (sparse_git_fetch.cmake) of exactly the paths this port
# compiles, about 16 MB for both and a few seconds on a first configure,
# instead of the SDK's full gigabyte. The pins are cache variables
# (ROSETTE_MCUX_SDK_GIT_*, ROSETTE_CMSIS_GIT_*): a tag and the commit it must
# resolve to, so every machine builds the same sources. A tree laid out like
# mcux-sdk below a subdirectory, such as github.com/micropython/nxp_driver
# (sdk/), is selected with ROSETTE_MCUX_SDK_GIT_REPOSITORY, _TAG, _COMMIT,
# and ROSETTE_MCUX_SDK_SUBDIR.
#
# A board with a USB console (ROSETTE_IMXRT_USB_CONSOLE) also gets
#
#   TINYUSB_PATH   TinyUSB (github.com/hathach/tinyusb): the USB device stack
#                  and its i.MX RT controller driver, src/ only
#
# fetched the same way and pinned by ROSETTE_TINYUSB_GIT_*.
#
# Board inputs (board.cmake):
#   ROSETTE_IMXRT_DEVICE       SDK device directory, e.g. MIMXRT1062
#   ROSETTE_IMXRT_SDK_BOARD    SDK board whose clock_config.c sets the clock
#                              tree, e.g. evkmimxrt1060
#   ROSETTE_IMXRT_USB_CONSOLE  ON for printf() over USB CDC (imxrt_usb_console.c)

include_guard(GLOBAL)
include(FetchContent)

if(NOT ROSETTE_IMXRT_DEVICE)
  message(FATAL_ERROR "ROSETTE_CHIP=imxrt needs ROSETTE_IMXRT_DEVICE from the board (e.g. MIMXRT1062)")
endif()

if(NOT ROSETTE_IMXRT_SDK_BOARD)
  message(FATAL_ERROR "ROSETTE_CHIP=imxrt needs ROSETTE_IMXRT_SDK_BOARD from the board (e.g. evkmimxrt1060)")
endif()

if(NOT CMAKE_TOOLCHAIN_FILE)
  set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_LIST_DIR}/arm_gcc_cortex_m7.cmake")
endif()

set(FETCHCONTENT_QUIET OFF)

set(ROSETTE_MCUX_SDK_GIT_REPOSITORY "https://github.com/nxp-mcuxpresso/mcux-sdk.git"
    CACHE STRING "MCUXpresso SDK repository fetched when MCUX_SDK_PATH is unset")
set(ROSETTE_MCUX_SDK_GIT_TAG "MCUX_2.16.100" CACHE STRING "MCUXpresso SDK release tag")
set(ROSETTE_MCUX_SDK_GIT_COMMIT "0420001d787c2c7bb062a2da4c46233d9daf2737"
    CACHE STRING "Commit ROSETTE_MCUX_SDK_GIT_TAG must resolve to")
set(ROSETTE_MCUX_SDK_SUBDIR "" CACHE STRING
    "Directory of the fetched repository laid out like mcux-sdk (empty: its root)")

set(ROSETTE_CMSIS_GIT_REPOSITORY "https://github.com/ARM-software/CMSIS_5.git"
    CACHE STRING "CMSIS repository fetched when CMSIS_PATH is unset")
set(ROSETTE_CMSIS_GIT_TAG "5.9.0" CACHE STRING "CMSIS release tag")
set(ROSETTE_CMSIS_GIT_COMMIT "2b7495b8535bdcb306dac29b9ded4cfb679d7e5c"
    CACHE STRING "Commit ROSETTE_CMSIS_GIT_TAG must resolve to")

set(ROSETTE_TINYUSB_GIT_REPOSITORY "https://github.com/hathach/tinyusb.git"
    CACHE STRING "TinyUSB repository fetched when TINYUSB_PATH is unset")
set(ROSETTE_TINYUSB_GIT_TAG "0.21.0" CACHE STRING "TinyUSB release tag")
set(ROSETTE_TINYUSB_GIT_COMMIT "dae3f9a366bfcddbf9dcf1b48d7500286a849539"
    CACHE STRING "Commit ROSETTE_TINYUSB_GIT_TAG must resolve to")

# Declares and populates <name> as a sparse checkout of <repo> at <tag>/<commit>.
function(_imxrt_fetch_sparse name repo tag commit)
  list(JOIN ARGN "|" _paths)
  FetchContent_Declare(${name}
    DOWNLOAD_COMMAND ${CMAKE_COMMAND}
      -DREPO=${repo} -DTAG=${tag} -DCOMMIT=${commit}
      "-DPATHS=${_paths}" -DDEST=<SOURCE_DIR>
      -P ${CMAKE_CURRENT_FUNCTION_LIST_DIR}/sparse_git_fetch.cmake
    # Neither tree is a CMake project of ours; port.cmake compiles the files.
    SOURCE_SUBDIR rosette-no-cmake)
  FetchContent_MakeAvailable(${name})
  set(${name}_SOURCE_DIR "${${name}_SOURCE_DIR}" PARENT_SCOPE)
endfunction()

foreach(_var MCUX_SDK_PATH CMSIS_PATH TINYUSB_PATH)
  if(NOT ${_var} AND DEFINED ENV{${_var}})
    set(${_var} "$ENV{${_var}}")
  endif()
endforeach()

if(MCUX_SDK_PATH AND EXISTS "${MCUX_SDK_PATH}/devices/${ROSETTE_IMXRT_DEVICE}")
  message(STATUS "Using local MCUXpresso SDK: ${MCUX_SDK_PATH}")
else()
  message(STATUS "Fetching MCUXpresso SDK ${ROSETTE_MCUX_SDK_GIT_TAG} "
                 "(sparse: ${ROSETTE_IMXRT_DEVICE} + drivers)")
  set(_sdk "${ROSETTE_MCUX_SDK_SUBDIR}")
  if(_sdk)
    set(_sdk "/${_sdk}")
  endif()

  _imxrt_fetch_sparse(rosette_mcux_sdk
    ${ROSETTE_MCUX_SDK_GIT_REPOSITORY} ${ROSETTE_MCUX_SDK_GIT_TAG} ${ROSETTE_MCUX_SDK_GIT_COMMIT}
    "${_sdk}/devices/${ROSETTE_IMXRT_DEVICE}/"
    "${_sdk}/drivers/common/"
    "${_sdk}/drivers/igpio/"
    "${_sdk}/drivers/lpspi/"
    "${_sdk}/drivers/lpuart/"
    "${_sdk}/drivers/pit/"
    "${_sdk}/drivers/trng/"
    "${_sdk}/boards/${ROSETTE_IMXRT_SDK_BOARD}/clock_config.*"
    "/COPYING-BSD-3"
    "/LICENSE*")
  set(MCUX_SDK_PATH "${rosette_mcux_sdk_SOURCE_DIR}${_sdk}")
endif()

set(MCUX_SDK_PATH "${MCUX_SDK_PATH}" CACHE PATH "MCUXpresso SDK root" FORCE)

if(CMSIS_PATH AND EXISTS "${CMSIS_PATH}/CMSIS/Core/Include/core_cm7.h")
  message(STATUS "Using local CMSIS: ${CMSIS_PATH}")
else()
  message(STATUS "Fetching CMSIS ${ROSETTE_CMSIS_GIT_TAG} (sparse: CMSIS/Core/Include)")
  _imxrt_fetch_sparse(rosette_cmsis
    ${ROSETTE_CMSIS_GIT_REPOSITORY} ${ROSETTE_CMSIS_GIT_TAG} ${ROSETTE_CMSIS_GIT_COMMIT}
    "/CMSIS/Core/Include/" "/LICENSE.txt")
  set(CMSIS_PATH "${rosette_cmsis_SOURCE_DIR}")
endif()

set(CMSIS_PATH "${CMSIS_PATH}" CACHE PATH "CMSIS 5 root (holds CMSIS/Core/Include)" FORCE)

if(ROSETTE_IMXRT_USB_CONSOLE)
  if(TINYUSB_PATH AND EXISTS "${TINYUSB_PATH}/src/tusb.h")
    message(STATUS "Using local TinyUSB: ${TINYUSB_PATH}")
  else()
    message(STATUS "Fetching TinyUSB ${ROSETTE_TINYUSB_GIT_TAG} (sparse: src)")
    _imxrt_fetch_sparse(rosette_tinyusb
      ${ROSETTE_TINYUSB_GIT_REPOSITORY} ${ROSETTE_TINYUSB_GIT_TAG} ${ROSETTE_TINYUSB_GIT_COMMIT}
      "/src/" "/LICENSE")
    set(TINYUSB_PATH "${rosette_tinyusb_SOURCE_DIR}")
  endif()

  set(TINYUSB_PATH "${TINYUSB_PATH}" CACHE PATH "TinyUSB root (holds src/tusb.h)" FORCE)
endif()
