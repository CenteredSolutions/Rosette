# SPDX-FileCopyrightText: 2026 Centered Solutions, LLC
# SPDX-License-Identifier: Apache-2.0

# Ports (ports/<axis>/<name>/): the chip, the OS layer, and the IP stack. No
# RTOS: the W5100S runs the TCP/IP stack in hardware, and the firmware is a
# cooperative superloop.
set(ROSETTE_CHIP      "rp2"       PARENT_SCOPE)
set(ROSETTE_OS        "baremetal" PARENT_SCOPE)
set(ROSETTE_NET_STACK "wiznet"    PARENT_SCOPE)

# The composition the WIZnet EVB-Pico boards share (boards/common/rp2_wiznet/):
# the bus, the driver types, and the board network.
set(ROSETTE_BOARD_COMMON "rp2_wiznet" PARENT_SCOPE)

set(PICO_BOARD "wiznet_w5100s_evb_pico" CACHE STRING "" FORCE)
set(ROSETTE_PROCESSOR  "RP2040"   PARENT_SCOPE)

# W5100S wiring (identical to the RP2350 EVB).
set(ROSETTE_SPI_SCK    18         PARENT_SCOPE)
set(ROSETTE_SPI_TX     19         PARENT_SCOPE)   # MOSI
set(ROSETTE_SPI_RX     16         PARENT_SCOPE)   # MISO
set(ROSETTE_SPI_CS     17         PARENT_SCOPE)
set(ROSETTE_WIZNET_RST 20         PARENT_SCOPE)
set(ROSETTE_WIZNET_INT 21         PARENT_SCOPE)
set(ROSETTE_LED_PIN    25         PARENT_SCOPE)
