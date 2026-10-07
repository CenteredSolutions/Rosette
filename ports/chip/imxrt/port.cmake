# ports/chip/imxrt — the NXP i.MX RT10xx chip port, on the MCUXpresso SDK.
#
# Included by cmake/rosette_platform.cmake with ROSETTE_PORT_DIR set to this
# directory, after the `rosette_platform` target exists; prelude.cmake beside
# this file has already installed the toolchain and set MCUX_SDK_PATH and
# CMSIS_PATH, before project(). Board inputs (board.cmake):
#   ROSETTE_IMXRT_DEVICE            SDK device directory, e.g. MIMXRT1062
#   ROSETTE_IMXRT_CPU               part number for the device header, e.g. MIMXRT1062DVL6A
#   ROSETTE_IMXRT_SDK_BOARD         SDK board whose clock_config.c this port applies
#   ROSETTE_IMXRT_FLASH_SIZE_BYTES  QSPI NOR size, for the boot block and the ROM
#   ROSETTE_IMXRT_USB_CONSOLE       ON: printf() also goes to a USB serial port
#   ROSETTE_IMXRT_USB_VID, _PID, _PRODUCT
#                                   optional USB identity (imxrt_usb_descriptors.c)
#
# Outputs:
#   rosette_mcux_sdk              the SDK's device, system, clock tree, and the
#                                 drivers this port and its boards use (gpio,
#                                 lpspi, lpuart, pit, trng)
#   chip_imxrt.cpp                clock tree and time base, unique id, boot
#                                 nonce, GPIO interrupts, bare-metal sync hooks
#   imxrt_newlib_syscalls.c       printf() to the board's consoles, clock_gettime()
#   rosette_imxrt_usb_console     with ROSETTE_IMXRT_USB_CONSOLE: TinyUSB and the
#                                 CDC ACM console over it (imxrt_usb_console.c)
#   imxrt_flash_region.c          the flash hooks (flash.hpp): no storage region
#   startup, IVT, boot block      the SDK's startup_<device>.S and
#                                 fsl_flexspi_nor_boot.c, and this port's
#                                 imxrt_flexspi_nor_config.c; every image links
#                                 them (-u below), since nothing calls them
#   imxrt_lpspi_bus.hpp, imxrt_gpio_pin.h, imxrt_console.h
#                                 the WIZnet bus, pin numbering, and console;
#                                 this directory is on the include path for the
#                                 board's composition
#   ROSETTE_CHIP_STORAGE_SOURCES  a LittleFS hook that never mounts
#   rosette_chip_add_image_outputs(<target>)
#                                 the .hex (teensy_loader_cli, NXP tools) and .bin images
#
# Memory: the SDK's flash linker script for the device. Code and constants
# execute in place from flash; .data, .bss, heap, and stack are in the
# 128 KiB DTCM; ITCM and OCRAM are unused.

if(NOT MCUX_SDK_PATH OR NOT CMSIS_PATH)
  message(FATAL_ERROR "ROSETTE_CHIP=imxrt needs the MCUXpresso SDK and CMSIS: include "
                      "cmake/rosette_prelude.cmake (or ports/chip/imxrt/prelude.cmake) before project()")
endif()

foreach(_var ROSETTE_IMXRT_CPU ROSETTE_IMXRT_FLASH_SIZE_BYTES)
  if(NOT ${_var})
    message(FATAL_ERROR "ROSETTE_CHIP=imxrt needs ${_var} from the board's board.cmake")
  endif()
endforeach()

set(_imxrt_device_dir "${MCUX_SDK_PATH}/devices/${ROSETTE_IMXRT_DEVICE}")
set(_imxrt_clock_dir "${MCUX_SDK_PATH}/boards/${ROSETTE_IMXRT_SDK_BOARD}")

set(ROSETTE_IMXRT_HEAP_BYTES "0x4000" CACHE STRING "newlib heap (malloc) in DTCM, bytes")
set(ROSETTE_IMXRT_STACK_BYTES "0x4000" CACHE STRING "Main stack in DTCM, bytes")

add_library(rosette_mcux_sdk STATIC
  ${_imxrt_device_dir}/system_${ROSETTE_IMXRT_DEVICE}.c
  ${_imxrt_device_dir}/drivers/fsl_clock.c
  ${_imxrt_clock_dir}/clock_config.c
  ${MCUX_SDK_PATH}/drivers/common/fsl_common.c
  ${MCUX_SDK_PATH}/drivers/common/fsl_common_arm.c
  ${MCUX_SDK_PATH}/drivers/igpio/fsl_gpio.c
  ${MCUX_SDK_PATH}/drivers/lpspi/fsl_lpspi.c
  ${MCUX_SDK_PATH}/drivers/lpuart/fsl_lpuart.c
  ${MCUX_SDK_PATH}/drivers/pit/fsl_pit.c
  ${MCUX_SDK_PATH}/drivers/trng/fsl_trng.c)
# SYSTEM: the vendor headers' warnings are not Rosette's to fix.
target_include_directories(rosette_mcux_sdk SYSTEM PUBLIC
  ${CMSIS_PATH}/CMSIS/Core/Include
  ${_imxrt_device_dir}
  ${_imxrt_device_dir}/drivers
  ${_imxrt_device_dir}/xip
  ${_imxrt_clock_dir}
  ${MCUX_SDK_PATH}/drivers/common
  ${MCUX_SDK_PATH}/drivers/igpio
  ${MCUX_SDK_PATH}/drivers/lpspi
  ${MCUX_SDK_PATH}/drivers/lpuart
  ${MCUX_SDK_PATH}/drivers/pit
  ${MCUX_SDK_PATH}/drivers/trng)
# XIP_EXTERNAL_FLASH: the clock tree leaves the FlexSPI clock as the boot ROM
# set it, since the image executes from that flash.
target_compile_definitions(rosette_mcux_sdk PUBLIC
  CPU_${ROSETTE_IMXRT_CPU}
  XIP_EXTERNAL_FLASH=1
  XIP_BOOT_HEADER_ENABLE=1
  XIP_BOOT_HEADER_DCD_ENABLE=0
  BOARD_FLASH_SIZE=${ROSETTE_IMXRT_FLASH_SIZE_BYTES}
  ROSETTE_IMXRT_FLASH_SIZE_BYTES=${ROSETTE_IMXRT_FLASH_SIZE_BYTES})
target_compile_options(rosette_mcux_sdk PRIVATE -w)

set(_imxrt_startup ${_imxrt_device_dir}/gcc/startup_${ROSETTE_IMXRT_DEVICE}.S)
set_source_files_properties(${_imxrt_startup} PROPERTIES
  LANGUAGE ASM COMPILE_DEFINITIONS "__STARTUP_CLEAR_BSS")

target_sources(rosette_platform PRIVATE
  ${ROSETTE_PORT_DIR}/chip_imxrt.cpp
  ${ROSETTE_PORT_DIR}/imxrt_flash_region.c
  ${ROSETTE_PORT_DIR}/imxrt_flexspi_nor_config.c
  ${ROSETTE_PORT_DIR}/imxrt_newlib_syscalls.c
  ${_imxrt_device_dir}/xip/fsl_flexspi_nor_boot.c
  ${_imxrt_startup})
target_include_directories(rosette_platform PUBLIC ${ROSETTE_PORT_DIR})
target_link_libraries(rosette_platform PUBLIC rosette_mcux_sdk)

# Nothing calls the vector table, the boot headers, or the chip's start-up
# constructor, so each is named undefined to pull its object out of the
# archive into every image.
target_link_options(rosette_platform INTERFACE
  -T${_imxrt_device_dir}/gcc/${ROSETTE_IMXRT_DEVICE}xxxxx_flexspi_nor.ld
  -Wl,--defsym=__heap_size__=${ROSETTE_IMXRT_HEAP_BYTES}
  -Wl,--defsym=__stack_size__=${ROSETTE_IMXRT_STACK_BYTES}
  -Wl,-u,__isr_vector
  -Wl,-u,image_vector_table
  -Wl,-u,g_boot_data
  -Wl,-u,rosette_imxrt_flexspi_nor_config
  -Wl,-u,rosette_imxrt_start_chip)

# The USB console is one archive with TinyUSB, so the descriptor callbacks
# TinyUSB calls and the console TinyUSB calls back into resolve within it.
if(ROSETTE_IMXRT_USB_CONSOLE)
  if(NOT TINYUSB_PATH)
    message(FATAL_ERROR "ROSETTE_IMXRT_USB_CONSOLE needs TinyUSB: ports/chip/imxrt/prelude.cmake sets TINYUSB_PATH")
  endif()

  set(_tusb "${TINYUSB_PATH}/src")
  add_library(rosette_imxrt_usb_console STATIC
    ${ROSETTE_PORT_DIR}/imxrt_usb_console.c
    ${ROSETTE_PORT_DIR}/imxrt_usb_descriptors.c
    ${_tusb}/tusb.c
    ${_tusb}/common/tusb_fifo.c
    ${_tusb}/device/usbd.c
    ${_tusb}/class/cdc/cdc_device.c
    ${_tusb}/portable/chipidea/ci_hs/dcd_ci_hs.c)
  target_include_directories(rosette_imxrt_usb_console SYSTEM PUBLIC ${_tusb})
  target_include_directories(rosette_imxrt_usb_console PUBLIC ${ROSETTE_PORT_DIR})
  target_compile_definitions(rosette_imxrt_usb_console PUBLIC CFG_TUSB_MCU=OPT_MCU_MIMXRT1XXX)
  foreach(_var ROSETTE_IMXRT_USB_VID ROSETTE_IMXRT_USB_PID)
    if(${_var})
      target_compile_definitions(rosette_imxrt_usb_console PRIVATE ${_var}=${${_var}})
    endif()
  endforeach()
  if(ROSETTE_IMXRT_USB_PRODUCT)
    target_compile_definitions(rosette_imxrt_usb_console PRIVATE
      "ROSETTE_IMXRT_USB_PRODUCT=\"${ROSETTE_IMXRT_USB_PRODUCT}\"")
  endif()
  target_link_libraries(rosette_imxrt_usb_console PUBLIC rosette_mcux_sdk)
  target_link_libraries(rosette_platform PUBLIC rosette_imxrt_usb_console)
  target_compile_definitions(rosette_platform PUBLIC ROSETTE_IMXRT_USB_CONSOLE=1)
endif()

set(ROSETTE_CHIP_STORAGE_SOURCES ${ROSETTE_PORT_DIR}/imxrt_littlefs_unmounted.cpp)
set(ROSETTE_CHIP_BASE_HEADERS rosette_mcux_sdk)

function(rosette_chip_add_image_outputs target)
  target_link_options(${target} PRIVATE "-Wl,-Map=$<TARGET_FILE:${target}>.map")
  add_custom_command(TARGET ${target} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${target}> $<TARGET_FILE_DIR:${target}>/${target}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${target}> $<TARGET_FILE_DIR:${target}>/${target}.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${target}>
    VERBATIM)
endfunction()
