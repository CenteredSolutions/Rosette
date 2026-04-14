set(PICO_BOARD "wiznet_w5100s_evb_pico" CACHE STRING "" FORCE)
set(RTPS_PROCESSOR  "RP2040"   PARENT_SCOPE)
# Standard WIZnet EVB wiring — same across pico and pico2 EVBs.
set(RTPS_SPI_SCK    18         PARENT_SCOPE)
set(RTPS_SPI_TX     19         PARENT_SCOPE)   # MOSI
set(RTPS_SPI_RX     16         PARENT_SCOPE)   # MISO
set(RTPS_SPI_CS     17         PARENT_SCOPE)
set(RTPS_W5100S_RST 20         PARENT_SCOPE)
set(RTPS_W5100S_INT 21         PARENT_SCOPE)
set(RTPS_LED_PIN    25         PARENT_SCOPE)