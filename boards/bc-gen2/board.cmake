# Platform port (include/rosette/platform/platform.hpp): OS layer and chip layer.
set(ROSETTE_OS   "freertos" PARENT_SCOPE)
set(ROSETTE_CHIP "rp2" PARENT_SCOPE)
set(ROSETTE_BOARD_FAMILY "pico" PARENT_SCOPE)
set(ROSETTE_BOARD_ENABLE_PICO_SDK ON PARENT_SCOPE)
set(ROSETTE_BOARD_ENABLE_FREERTOS ON PARENT_SCOPE)
set(ROSETTE_BOARD_ENABLE_WIZNET ON PARENT_SCOPE)
set(ROSETTE_BOARD_BUILD_BSP ON PARENT_SCOPE)
set(ROSETTE_BOARD_BUILD_EXAMPLE ON PARENT_SCOPE)
set(ROSETTE_BOARD_EXAMPLE_SOURCE "examples/main_rp2350.cpp" PARENT_SCOPE)

# Use pico as the base board definition; the BSP overrides anything
# that differs (e.g. wiring or board-specific hardware setup).
set(PICO_BOARD "wiznet_w5100s_evb_pico" CACHE STRING "" FORCE)
set(ROSETTE_PROCESSOR  "RP2040"   PARENT_SCOPE)

# Custom wiring — adjust to match your schematic.
set(ROSETTE_SPI_SCK    10         PARENT_SCOPE)
set(ROSETTE_SPI_TX     11         PARENT_SCOPE)
set(ROSETTE_SPI_RX      8         PARENT_SCOPE)
set(ROSETTE_SPI_CS      9         PARENT_SCOPE)
set(ROSETTE_WIZNET_RST 12         PARENT_SCOPE)
set(ROSETTE_WIZNET_INT 13         PARENT_SCOPE)
set(ROSETTE_LED_PIN    23         PARENT_SCOPE)
