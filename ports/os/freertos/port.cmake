# ports/os/freertos — the FreeRTOS OS layer: kernel mutex and binary semaphore.
#
# Included by cmake/rosette_platform.cmake with ROSETTE_PORT_DIR set to this
# directory. Every target that links `rosette_platform` finds this directory's
# rosette_os_port.hpp through platform.hpp, and links the kernel target the
# board's build defines (`freertos_kernel`, with the board's FreeRTOSConfig.h).
target_include_directories(rosette_platform PUBLIC ${ROSETTE_PORT_DIR})
target_link_libraries(rosette_platform PUBLIC freertos_kernel)
