# ports/chip/stub — the compile-only chip port.
#
# Included by cmake/rosette_platform.cmake with ROSETTE_PORT_DIR set to this
# directory, after the `rosette_platform` target exists. Outputs:
#   chip_stub.cpp                 the platform, flash, and runtime hooks, in rosette_platform
#   stub_atomic.c                 the atomic helpers a Cortex-M0+ calls, in rosette_platform
#   ROSETTE_CHIP_STORAGE_SOURCES  the LittleFS block device, for the storage library to compile
target_sources(rosette_platform PRIVATE
  ${ROSETTE_PORT_DIR}/chip_stub.cpp
  ${ROSETTE_PORT_DIR}/stub_atomic.c)
set(ROSETTE_CHIP_STORAGE_SOURCES ${ROSETTE_PORT_DIR}/stub_littlefs.cpp)
