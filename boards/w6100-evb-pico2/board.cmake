# Ports (ports/<axis>/<name>/): the chip, the OS layer, and the IP stack. No
# RTOS: the W6100 runs the TCP/IP stack in hardware, and the firmware is a
# cooperative superloop.
set(ROSETTE_CHIP      "rp2"       PARENT_SCOPE)
set(ROSETTE_OS        "baremetal" PARENT_SCOPE)
set(ROSETTE_NET_STACK "wiznet"    PARENT_SCOPE)

# The composition the WIZnet EVB-Pico boards share (boards/common/rp2_wiznet/):
# the bus, the driver types, and the board network.
set(ROSETTE_BOARD_COMMON "rp2_wiznet" PARENT_SCOPE)

set(PICO_BOARD "wiznet_w6100_evb_pico2" CACHE STRING "" FORCE)
# The Pico SDK has no header for this board; <PICO_BOARD>.h lives beside this file.
set(PICO_BOARD_HEADER_DIRS "${CMAKE_CURRENT_LIST_DIR}" CACHE STRING "Directories holding <PICO_BOARD>.h" FORCE)
set(ROSETTE_PROCESSOR  "RP2350"   PARENT_SCOPE)

# W6100 wiring — same SPI0 pin map as the W5100S-EVB-Pico2.
set(ROSETTE_SPI_SCK    18         PARENT_SCOPE)
set(ROSETTE_SPI_TX     19         PARENT_SCOPE)
set(ROSETTE_SPI_RX     16         PARENT_SCOPE)
set(ROSETTE_SPI_CS     17         PARENT_SCOPE)
set(ROSETTE_WIZNET_RST 20         PARENT_SCOPE)
set(ROSETTE_WIZNET_INT 21         PARENT_SCOPE)
set(ROSETTE_LED_PIN    25         PARENT_SCOPE)
