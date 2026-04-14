# Use pico2 as the base board definition; the BSP overrides anything
# that differs (e.g. clock tree, flash size via linker script).
set(PICO_BOARD "wiznet_w5100s_evb_pico" CACHE STRING "" FORCE)
set(RTPS_PROCESSOR  "RP2040"   PARENT_SCOPE)
# Custom wiring — adjust to match your schematic.
set(RTPS_SPI_SCK    10         PARENT_SCOPE)
set(RTPS_SPI_TX     11         PARENT_SCOPE)
set(RTPS_SPI_RX      8         PARENT_SCOPE)
set(RTPS_SPI_CS      9         PARENT_SCOPE)
set(RTPS_W5100S_RST 12         PARENT_SCOPE)
set(RTPS_W5100S_INT 13         PARENT_SCOPE)
set(RTPS_LED_PIN    23         PARENT_SCOPE)
