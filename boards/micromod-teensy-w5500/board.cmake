# SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
# SPDX-License-Identifier: Apache-2.0

# SparkFun MicroMod Teensy (NXP i.MX RT1062) on a MicroMod Main Board, with
# the SparkFun MicroMod Ethernet Function Board (WIZnet W5500) in function
# slot 0. No RTOS: the W5500 runs the TCP/IP stack in hardware, and the
# firmware is a cooperative superloop.
set(ROSETTE_CHIP      "imxrt"     PARENT_SCOPE)
set(ROSETTE_OS        "baremetal" PARENT_SCOPE)
set(ROSETTE_NET_STACK "wiznet"    PARENT_SCOPE)

# The composition every WIZnet board shares (boards/common/wiznet/): the
# driver types over the chip port's bus, and the board network.
set(ROSETTE_BOARD_COMMON "wiznet" PARENT_SCOPE)

# The chip port's inputs (ports/chip/imxrt/prelude.cmake, port.cmake). The
# processor board carries the MIMXRT1062DVL6A and a 16 MiB Winbond
# W25Q128JV QSPI NOR (PJRC Teensyduino bootdata.c, ARDUINO_TEENSY_MICROMOD).
# Its clock tree is the SDK's evkmimxrt1060 one: both have a 24 MHz crystal.
set(ROSETTE_PROCESSOR              "IMXRT1062"       PARENT_SCOPE)
set(ROSETTE_IMXRT_DEVICE           "MIMXRT1062"      PARENT_SCOPE)
set(ROSETTE_IMXRT_CPU              "MIMXRT1062DVL6A" PARENT_SCOPE)
set(ROSETTE_IMXRT_SDK_BOARD        "evkmimxrt1060"   PARENT_SCOPE)
set(ROSETTE_IMXRT_FLASH_SIZE_BYTES "0x1000000"       PARENT_SCOPE)

# W5500 wiring through the MicroMod connector, by MicroMod signal and Teensy
# pin (SparkFun MicroMod Teensy hookup guide), for the configure summary.
set(ROSETTE_SPI_SCK    "SPI_SCK/13 (GPIO_B0_03)"  PARENT_SCOPE)
set(ROSETTE_SPI_TX     "SPI_COPI/11 (GPIO_B0_02)" PARENT_SCOPE)
set(ROSETTE_SPI_RX     "SPI_CIPO/12 (GPIO_B0_01)" PARENT_SCOPE)
set(ROSETTE_SPI_CS     "SPI_CS/10 (GPIO_B0_00)"   PARENT_SCOPE)
set(ROSETTE_WIZNET_RST "PWM0/3 (GPIO_EMC_05)"     PARENT_SCOPE)
set(ROSETTE_WIZNET_INT "D0/4 (GPIO_EMC_06)"       PARENT_SCOPE)

# Console: the processor's USB port as a CDC ACM serial port (TinyUSB), and
# the MicroMod UART1 (TX1/RX1) beside it. Setting the USB port to 134 baud
# enters the Teensy bootloader for teensy_loader_cli.
set(ROSETTE_IMXRT_USB_CONSOLE ON                        PARENT_SCOPE)
set(ROSETTE_IMXRT_USB_PRODUCT "Rosette MicroMod Teensy" PARENT_SCOPE)
