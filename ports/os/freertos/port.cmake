# ports/os/freertos — the FreeRTOS OS layer: kernel mutex and binary semaphore.
#
# Included by cmake/rosette_platform.cmake with ROSETTE_PORT_DIR set to this
# directory, after the chip port. Every target that links `rosette_platform`
# finds this directory's rosette_os_port.hpp through platform.hpp, and links
# the kernel, `freertos_kernel`.
#
# The kernel comes from FREERTOS_KERNEL_PATH (environment or -D) when set, or
# is fetched. Its configuration is the board's: FreeRTOSConfig.h is found in
# ROSETTE_FREERTOS_CONFIG_DIR (default boards/common/freertos/) and the
# board's directory, and may include the chip SDK's headers that the chip port
# names in ROSETTE_CHIP_BASE_HEADERS. The kernel's own port (FREERTOS_PORT)
# follows ROSETTE_PROCESSOR unless the board sets it.

include(FetchContent)
set(FETCHCONTENT_QUIET OFF)

# FreeRTOS-Kernel V11 needs FREERTOS_PORT and a `freertos_config` INTERFACE
# target before the kernel is added.
if(NOT DEFINED FREERTOS_PORT)
  if(ROSETTE_PROCESSOR STREQUAL "RP2350")
    set(FREERTOS_PORT GCC_ARM_CM33_NTZ_NONSECURE CACHE STRING "" FORCE)
  else()
    set(FREERTOS_PORT GCC_ARM_CM0 CACHE STRING "" FORCE)
  endif()
endif()
set(FREERTOS_HEAP 4 CACHE STRING "" FORCE)

if(NOT DEFINED ROSETTE_FREERTOS_CONFIG_DIR)
  set(ROSETTE_FREERTOS_CONFIG_DIR "${ROSETTE_ROOT}/boards/common/freertos")
endif()

if(NOT TARGET freertos_config)
  add_library(freertos_config INTERFACE)
  target_include_directories(freertos_config SYSTEM INTERFACE
    "${ROSETTE_FREERTOS_CONFIG_DIR}"
    "${ROSETTE_BOARD_DIR}")
  target_link_libraries(freertos_config INTERFACE ${ROSETTE_CHIP_BASE_HEADERS})
endif()

if(DEFINED ENV{FREERTOS_KERNEL_PATH})
  set(FREERTOS_KERNEL_PATH "$ENV{FREERTOS_KERNEL_PATH}")
endif()

if(NOT TARGET freertos_kernel)
  if(DEFINED FREERTOS_KERNEL_PATH AND EXISTS "${FREERTOS_KERNEL_PATH}")
    message(STATUS "Using local FreeRTOS-Kernel: ${FREERTOS_KERNEL_PATH}")
    add_subdirectory(${FREERTOS_KERNEL_PATH} freertos_kernel EXCLUDE_FROM_ALL)
  else()
    message(STATUS "Fetching FreeRTOS-Kernel V11.1.0 — first run only")
    FetchContent_Declare(freertos_kernel
      GIT_REPOSITORY https://github.com/FreeRTOS/FreeRTOS-Kernel.git
      GIT_TAG        V11.1.0
      GIT_SHALLOW    TRUE
      GIT_PROGRESS   TRUE
      GIT_SUBMODULES "")
    FetchContent_MakeAvailable(freertos_kernel)
  endif()
endif()

target_include_directories(rosette_platform PUBLIC ${ROSETTE_PORT_DIR})
target_link_libraries(rosette_platform PUBLIC freertos_kernel)

# Examples that use FreeRTOS directly check this.
set(ROSETTE_BOARD_ENABLE_FREERTOS ON)
