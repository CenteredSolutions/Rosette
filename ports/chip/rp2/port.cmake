# SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
# SPDX-License-Identifier: Apache-2.0

# ports/chip/rp2 — the RP2040 / RP2350 chip port, on the Pico SDK.
#
# Included by cmake/rosette_platform.cmake with ROSETTE_PORT_DIR set to this
# directory, after the `rosette_platform` target exists; prelude.cmake beside
# this file has already included the Pico SDK, before project(). Outputs:
#   chip_rp2.cpp                  clock, unique id, boot nonce, GPIO interrupts, bare-metal sync hooks
#   rp2_flash.c                   the flash hooks (flash.hpp), for every OS
#   rp2_newlib_time.c             newlib clock_gettime() / _gettimeofday()
#   nothrow_operator_new.cpp      `new (std::nothrow)` over malloc, beside the SDK's
#                                 new_delete.cpp, so no image links the C++ unwinder
#   rp2_spi_bus.hpp, rp2_qspi_bus.hpp
#                                 WIZnet register buses; this directory is on the
#                                 include path for the board's composition
#   ROSETTE_CHIP_STORAGE_SOURCES  the LittleFS volume in the board's flash region
#                                 (needs the board's board_config.h)
#   rosette_chip_add_image_outputs(<target>)
#                                 the .uf2 / .hex / .bin images, and stdio over USB
#   ROSETTE_CHIP_TEST_LIBRARIES   pico_multicore, for the platform contract test's
#                                 second core
#   ROSETTE_CHIP_BASE_HEADERS     pico_base_headers, for a third-party configuration
#                                 header that needs the SDK's (FreeRTOSConfig.h)

if(NOT COMMAND pico_sdk_init)
  message(FATAL_ERROR "ROSETTE_CHIP=rp2 needs the Pico SDK: include cmake/rosette_prelude.cmake "
                      "(or ports/chip/rp2/prelude.cmake) before project()")
endif()
get_property(_rp2_sdk_initialized GLOBAL PROPERTY ROSETTE_RP2_SDK_INITIALIZED)
if(NOT _rp2_sdk_initialized)
  pico_sdk_init()
  set_property(GLOBAL PROPERTY ROSETTE_RP2_SDK_INITIALIZED TRUE)
endif()

# Optimization level for the optimized configurations. Code executes in place
# from QSPI flash through the XIP cache, so the smaller instruction footprint
# of -Os raises the cache hit rate and usually outweighs the extra instructions
# -O3 would save; it also keeps the image small enough for the flash region.
# CMake's GNU default for Release and RelWithDebInfo is -O3 or -O2, so the
# level is rewritten here, once, for every target in this directory tree.
set(ROSETTE_MCU_OPT_LEVEL "s" CACHE STRING
  "Compiler -O level for Release and RelWithDebInfo builds on the rp2 chip (s, 2, 3)")
set_property(CACHE ROSETTE_MCU_OPT_LEVEL PROPERTY STRINGS s 2 3)
foreach(_rp2_lang C CXX)
  foreach(_rp2_config RELEASE RELWITHDEBINFO)
    string(REGEX REPLACE "-O[0-3sg]" "-O${ROSETTE_MCU_OPT_LEVEL}"
      CMAKE_${_rp2_lang}_FLAGS_${_rp2_config} "${CMAKE_${_rp2_lang}_FLAGS_${_rp2_config}}")
  endforeach()
endforeach()

target_sources(rosette_platform PRIVATE
  ${ROSETTE_PORT_DIR}/chip_rp2.cpp
  ${ROSETTE_PORT_DIR}/rp2_flash.c
  ${ROSETTE_PORT_DIR}/rp2_newlib_time.c)
target_include_directories(rosette_platform PUBLIC ${ROSETTE_PORT_DIR})
target_link_libraries(rosette_platform PUBLIC
  pico_stdlib pico_flash pico_rand pico_time pico_unique_id
  hardware_flash hardware_gpio hardware_irq hardware_spi hardware_sync hardware_timer)
# The SDK's pico_cxx_options compiles its allocator replacements into every
# target that links it, executables included; the nothrow form rides along so
# an executable's own object always wins over libstdc++'s, however its static
# libraries are ordered.
target_sources(pico_cxx_options INTERFACE ${ROSETTE_PORT_DIR}/nothrow_operator_new.cpp)

set(ROSETTE_CHIP_STORAGE_SOURCES ${ROSETTE_PORT_DIR}/rp2_littlefs_region.cpp)
set(ROSETTE_CHIP_TEST_LIBRARIES pico_multicore)
set(ROSETTE_CHIP_BASE_HEADERS pico_base_headers)

function(rosette_chip_add_image_outputs target)
  pico_enable_stdio_usb(${target} 1)
  pico_enable_stdio_uart(${target} 0)
  pico_add_extra_outputs(${target})
endfunction()
