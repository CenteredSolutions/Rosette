# NXP MIMXRT1020-EVK: an i.MX RT1021 with its own 10/100 ENET MAC, a
# Microchip KSZ8081RNB PHY over RMII, 8 MiB of ISSI IS25LP064A QSPI NOR, and
# 32 MiB of SDRAM (unused). The IP stack runs on the RT1021: FreeRTOS+TCP
# over the ENET, under FreeRTOS.
set(ROSETTE_CHIP      "imxrt"        PARENT_SCOPE)
set(ROSETTE_OS        "freertos"     PARENT_SCOPE)
set(ROSETTE_NET_STACK "freertos_tcp" PARENT_SCOPE)

# The ENET moves frames in FreeRTOS+TCP's own network buffers (ON), or copies
# them through buffers of its own (OFF), which costs about 9 KiB more and
# stays available to measure against (bsp.cmake picks the composition).
option(ROSETTE_IMXRT_ENET_ZERO_COPY "The EVK's ENET moves frames in FreeRTOS+TCP's buffers" ON)
set(ROSETTE_FREERTOS_TCP_ZERO_COPY ${ROSETTE_IMXRT_ENET_ZERO_COPY} PARENT_SCOPE)

# The chip port's inputs (ports/chip/imxrt/prelude.cmake, port.cmake). The
# clock tree is the SDK's evkmimxrt1020 one: 500 MHz core from the 24 MHz
# crystal.
set(ROSETTE_PROCESSOR              "IMXRT1021"       PARENT_SCOPE)
set(ROSETTE_IMXRT_DEVICE           "MIMXRT1021"      PARENT_SCOPE)
set(ROSETTE_IMXRT_CPU              "MIMXRT1021DAG5A" PARENT_SCOPE)
set(ROSETTE_IMXRT_SDK_BOARD        "evkmimxrt1020"   PARENT_SCOPE)
set(ROSETTE_IMXRT_FLASH_SIZE_BYTES "0x800000"        PARENT_SCOPE)

# The kernel's Cortex-M4F port: the RT1021's Cortex-M7 is r1p1, past the
# r0p1 erratum the ARM_CM7/r0p1 port works around. The kernel configuration
# is this board's (FreeRTOSConfig.h beside this file).
set(FREERTOS_PORT               "GCC_ARM_CM4F"           PARENT_SCOPE)
set(ROSETTE_FREERTOS_CONFIG_DIR "${CMAKE_CURRENT_LIST_DIR}" PARENT_SCOPE)

# Console: LPUART1, which the on-board DAPLink probe bridges to its USB
# virtual serial port. The RT1021's own USB port is not used.
set(ROSETTE_IMXRT_USB_CONSOLE OFF PARENT_SCOPE)
