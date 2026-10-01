# ports/chip/rp2 — the RP2040 / RP2350 chip port, on the Pico SDK.
#
# Included by cmake/rosette_platform.cmake with ROSETTE_PORT_DIR set to this
# directory, after the `rosette_platform` target exists and pico_sdk_init()
# has run. Outputs:
#   chip_rp2.cpp                  clock, unique id, GPIO interrupts, bare-metal sync hooks
#   rp2_flash.c                   the flash hooks (flash.hpp), for every OS
#   rp2_newlib_time.c             newlib clock_gettime() / _gettimeofday()
#   rp2_spi_bus.hpp, rp2_qspi_bus.hpp
#                                 WIZnet register buses; this directory is on the
#                                 include path for the board's composition
#   ROSETTE_CHIP_STORAGE_SOURCES  the LittleFS volume in the board's flash region
#                                 (needs the board's board_config.h)
target_sources(rosette_platform PRIVATE
  ${ROSETTE_PORT_DIR}/chip_rp2.cpp
  ${ROSETTE_PORT_DIR}/rp2_flash.c
  ${ROSETTE_PORT_DIR}/rp2_newlib_time.c)
target_include_directories(rosette_platform PUBLIC ${ROSETTE_PORT_DIR})
target_link_libraries(rosette_platform PUBLIC
  pico_stdlib pico_flash pico_time pico_unique_id
  hardware_flash hardware_gpio hardware_irq hardware_spi hardware_sync hardware_timer)
set(ROSETTE_CHIP_STORAGE_SOURCES ${ROSETTE_PORT_DIR}/rp2_littlefs_region.cpp)
