# ports/chip/host — the host chip port (Linux, macOS), with ports/os/posix.
#
# Included by cmake/rosette_platform.cmake with ROSETTE_PORT_DIR set to this
# directory, after the `rosette_platform` target exists. Outputs:
#   chip_host.cpp                 CLOCK_MONOTONIC, a per-process unique id, OS wall clock
#   ROSETTE_CHIP_STORAGE_SOURCES  a file-backed LittleFS volume and flash hooks
# The library is position-independent, since rmw_rosette links it into a
# shared object.
target_sources(rosette_platform PRIVATE ${ROSETTE_PORT_DIR}/chip_host.cpp)
set_target_properties(rosette_platform PROPERTIES POSITION_INDEPENDENT_CODE ON)
set(ROSETTE_CHIP_STORAGE_SOURCES ${ROSETTE_PORT_DIR}/host_flash_file.cpp)
