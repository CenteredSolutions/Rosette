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

# The kernel's Pico SDK port (a board sets FREERTOS_PORT=GCC_RP2040). Its
# headers and port.c include SDK headers the kernel's generic CMake does not
# provide (the port's own library.cmake would), and the SDK sources compiled
# into each image take the port's adapter header, so an SDK mutex (stdio,
# malloc) or sleep_ms() taken in a task blocks that task in the kernel
# instead of spinning. The board's FreeRTOSConfig.h must then set
# configSUPPORT_PICO_SYNC_INTEROP and configSUPPORT_PICO_TIME_INTEROP to 1:
# the kernel library is not compiled with LIB_PICO_SYNC / LIB_PICO_TIME,
# from which rp2040_config.h would otherwise infer them.
#
# FREE_RTOS_KERNEL_SMP declares the kernel SMP-capable, as the port's own
# library.cmake does; it does not select SMP. The V11 kernel is one source for
# one or two cores, and the board's FreeRTOSConfig.h still picks the count.
# The Pico SDK's pico_flash is the only reader: it lets flash.c consult
# configNUMBER_OF_CORES. With 2, flash_safe_execute() parks the other core in
# a RAM-resident task with interrupts off while flash is erased or programmed,
# and pico_flash fails the build unless configUSE_CORE_AFFINITY is 1, since
# the handshake pins that task to the other core. With 1, flash.c compiles no
# FreeRTOS handshake and flash_safe_execute() is unaffected by this define.
if(FREERTOS_PORT STREQUAL "GCC_RP2040")
  get_target_property(_rf_kernel_dir freertos_kernel SOURCE_DIR)
  target_link_libraries(freertos_kernel_port_headers INTERFACE
    hardware_sync_headers hardware_irq_headers hardware_clocks_headers
    hardware_exception_headers pico_sync_headers pico_time_headers)
  # As the port's library.cmake does: port.c's SDK interop (the cross-core
  # FIFO, sio_hw) is compiled only with LIB_PICO_MULTICORE.
  target_link_libraries(freertos_kernel_port PRIVATE pico_multicore)
  target_compile_definitions(rosette_platform PUBLIC
    LIB_FREERTOS_KERNEL=1
    FREE_RTOS_KERNEL_SMP=1
    PICO_CONFIG_RTOS_ADAPTER_HEADER=${_rf_kernel_dir}/portable/ThirdParty/GCC/RP2040/include/freertos_sdk_config.h)
endif()

# Examples that use FreeRTOS directly check this.
set(ROSETTE_BOARD_ENABLE_FREERTOS ON)
