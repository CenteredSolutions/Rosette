# ports/net/posix — the host's IP stack through BSD sockets (Linux, macOS).
#
# Included by cmake/rosette_net.cmake with ROSETTE_PORT_DIR set to this
# directory, after the `rosette_net` target exists. Outputs, in rosette_net:
#   posix_socket.{hpp,cpp}        PosixSocketProvider; host test drivers may name it
#   posix_host_interface.{hpp,cpp} the one interface the process is on (ROSETTE_INTERFACE)
#   posix_board_network.cpp       rosette/net/board_network.hpp over that provider and
#                                 the operating system's own address
target_sources(rosette_net PRIVATE
  ${ROSETTE_PORT_DIR}/posix_socket.cpp
  ${ROSETTE_PORT_DIR}/posix_host_interface.cpp
  ${ROSETTE_PORT_DIR}/posix_board_network.cpp)
target_include_directories(rosette_net PUBLIC ${ROSETTE_PORT_DIR})
