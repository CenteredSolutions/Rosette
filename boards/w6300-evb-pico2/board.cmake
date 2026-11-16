# SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
# SPDX-License-Identifier: Apache-2.0

# Ports (ports/<axis>/<name>/): the chip, the OS layer, and the IP stack. No
# RTOS: the W6300 runs the TCP/IP stack in hardware, and the firmware is a
# cooperative superloop.
set(ROSETTE_CHIP      "rp2"       PARENT_SCOPE)
set(ROSETTE_OS        "baremetal" PARENT_SCOPE)
set(ROSETTE_NET_STACK "wiznet"    PARENT_SCOPE)

# The composition the WIZnet EVB-Pico boards share (boards/common/wiznet/):
# the bus, the driver types, and the board network.
set(ROSETTE_BOARD_COMMON "wiznet" PARENT_SCOPE)

set(PICO_BOARD "wiznet_w6300_evb_pico2" CACHE STRING "" FORCE)
# The Pico SDK has no header for this board; <PICO_BOARD>.h lives beside this file.
set(PICO_BOARD_HEADER_DIRS "${CMAKE_CURRENT_LIST_DIR}" CACHE STRING "Directories holding <PICO_BOARD>.h" FORCE)
set(ROSETTE_PROCESSOR  "RP2350"   PARENT_SCOPE)

# W6300 QSPI wiring. The EVB routes the chip onto pins that are NOT a
# hardware-SPI-capable set, so the bus is bit-banged single-lane for now
# (ROSETTE_WIZNET_BUS_QSPI in board_config.h). 4-bit QSPI over PIO is phase 2.
#   SCK 17 · IO0/MOSI 18 · IO1/MISO 19 · CS 16 · RST 22
set(ROSETTE_SPI_SCK    17         PARENT_SCOPE)
set(ROSETTE_SPI_TX     18         PARENT_SCOPE)
set(ROSETTE_SPI_RX     19         PARENT_SCOPE)
set(ROSETTE_SPI_CS     16         PARENT_SCOPE)
set(ROSETTE_WIZNET_RST 22         PARENT_SCOPE)
set(ROSETTE_WIZNET_INT 15         PARENT_SCOPE)
set(ROSETTE_LED_PIN    25         PARENT_SCOPE)
