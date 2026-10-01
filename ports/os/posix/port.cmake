# ports/os/posix — the POSIX OS layer (Linux, macOS): pthread mutex, self-pipe event.
#
# Included by cmake/rosette_platform.cmake with ROSETTE_PORT_DIR set to this
# directory. Every target that links `rosette_platform` finds this directory's
# rosette_os_port.hpp through platform.hpp, and links the system thread library.
target_include_directories(rosette_platform PUBLIC ${ROSETTE_PORT_DIR})
find_package(Threads REQUIRED)
target_link_libraries(rosette_platform PUBLIC Threads::Threads)
