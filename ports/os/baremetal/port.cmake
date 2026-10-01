# ports/os/baremetal — the OS layer for a superloop with no RTOS, on the chip's sync hooks.
#
# Included by cmake/rosette_platform.cmake with ROSETTE_PORT_DIR set to this
# directory. Every target that links `rosette_platform` finds this directory's
# rosette_os_port.hpp through platform.hpp. The chip port supplies the hooks
# it is built on (rosette/platform/chip_sync.hpp).
target_include_directories(rosette_platform PUBLIC ${ROSETTE_PORT_DIR})
