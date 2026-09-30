# Platform port (include/rosette/platform/platform.hpp): OS layer and chip layer.
set(ROSETTE_OS   "baremetal" PARENT_SCOPE)
set(ROSETTE_CHIP "rp2" PARENT_SCOPE)
set(ROSETTE_BOARD_FAMILY "pico" PARENT_SCOPE)
set(ROSETTE_BOARD_ENABLE_PICO_SDK ON PARENT_SCOPE)

# No RTOS: the W5100S runs the TCP/IP stack in hardware and the firmware is a
# cooperative superloop — the same poll-mode path as the RP2350 EVB. The RP2040
# EVB is the debugger-friendly variant of that board.
set(ROSETTE_BOARD_ENABLE_FREERTOS OFF PARENT_SCOPE)
set(ROSETTE_BOARD_ENABLE_WIZNET ON PARENT_SCOPE)
set(ROSETTE_BOARD_NET_STACK "wiznet" PARENT_SCOPE)

set(ROSETTE_BOARD_BUILD_BSP ON PARENT_SCOPE)

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
