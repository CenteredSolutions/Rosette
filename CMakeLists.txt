cmake_minimum_required(VERSION 3.20)

# Board selection + toolchain setup — must run before project(). See the file
# for the ROSETTE_BOARD values. A standalone example includes this same prelude.
include(${CMAKE_CURRENT_LIST_DIR}/cmake/rosette_prelude.cmake)

project(rosette C CXX ASM)
set(CMAKE_C_STANDARD   11)
set(CMAKE_CXX_STANDARD 17)

include(cmake/micro_ros.cmake)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ------------------------------------------------------------
# The ports the board selected (boards/<board>/board.cmake):
#
#   rosette_platform  ports/chip/${ROSETTE_CHIP} + ports/os/${ROSETTE_OS}
#   rosette_net       the portable network core + ports/net/${ROSETTE_NET_STACK}
#
# The sockets, address management, and SNTP that the RTPS core
# (cmake/rosette_rtps.cmake), rmw_rosette (cmake/rmw_rosette_firmware.cmake),
# and the board BSP open go through rosette_net. See docs/architecture.md.
# ------------------------------------------------------------
set(ROSETTE_ROOT ${CMAKE_CURRENT_SOURCE_DIR})
include(cmake/rosette_platform.cmake)
include(cmake/rosette_net.cmake)

set(ROSETTE_INTERNAL_INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/include/rosette/net
    ${CMAKE_CURRENT_SOURCE_DIR}/include/rosette/platform
    ${CMAKE_CURRENT_SOURCE_DIR}/include/rosette/ros)

target_include_directories(rosette_net PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${ROSETTE_BOARD_DIR}
    ${ROSETTE_INTERNAL_INCLUDE_DIRS}
)

# ------------------------------------------------------------
# rosette_fs — the LittleFS sources
#
# cmake/littlefs.cmake locates a local checkout (LITTLEFS_PATH) or fetches one.
# rosette_fs compiles it; rosette_storage layers the file API on top.
# ------------------------------------------------------------
include(cmake/littlefs.cmake)

add_library(rosette_fs STATIC
    ${ROSETTE_LITTLEFS_DIR}/lfs.c
    ${ROSETTE_LITTLEFS_DIR}/lfs_util.c
)
target_include_directories(rosette_fs PUBLIC
    ${ROSETTE_LITTLEFS_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# rosette_session — the ROS session supervisor (include/rosette/ros/session_supervisor.hpp),
# the micro-ROS agent's gate for it (agent_session_gate.hpp), and the ROS view of
# the wall clock (time_sync.hpp). Backend-neutral:
# it needs no rcl or rmw headers (the gate reaches the agent through an
# AgentLink), so rclc firmware on either backend links the same library.
add_library(rosette_session STATIC
    src/ros/session_supervisor.cpp
    src/ros/agent_session_gate.cpp
    src/ros/time_sync.cpp)
target_link_libraries(rosette_session PUBLIC rosette_net)

# Platform conformance test for the selected board's port. On a Pico it
# reports over USB serial; on the host it is also tests/platform's ctest.
add_executable(rosette_platform_contract tests/platform/platform_contract_test.cpp)
target_link_libraries(rosette_platform_contract PRIVATE rosette_platform ${ROSETTE_CHIP_TEST_LIBRARIES})
rosette_add_image_outputs(rosette_platform_contract)

if(ROSETTE_NET_STACK STREQUAL "posix")
    # The SNTP client against a real NTP server, checked against the host clock.
    add_executable(sntp_live_probe tests/net/sntp_live_probe.cpp)
    target_link_libraries(sntp_live_probe PRIVATE rosette_net)
endif()

# ------------------------------------------------------------
# rosette_bsp — a real board: its board_init.c, and the composition it shares
# with other boards (boards/common/${ROSETTE_BOARD_COMMON}/bsp.cmake), which
# for a WIZnet board instantiates the driver and implements the board network.
# A host board has neither; its board network is in rosette_net
# (ports/net/posix).
# ------------------------------------------------------------
if(EXISTS ${ROSETTE_BOARD_DIR}/board_init.c)
    add_library(rosette_bsp STATIC ${ROSETTE_BOARD_DIR}/board_init.c)
    target_include_directories(rosette_bsp PUBLIC
        ${ROSETTE_BOARD_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${ROSETTE_INTERNAL_INCLUDE_DIRS}
    )
    target_link_libraries(rosette_bsp PUBLIC rosette_net)
    if(ROSETTE_BOARD_COMMON)
        include(boards/common/${ROSETTE_BOARD_COMMON}/bsp.cmake)
    endif()
endif()

# ------------------------------------------------------------
# rosette_storage — LittleFS volume + generic file API
#
#   rosette::storage::Volume        mounts the board storage region
#   rosette::storage::FileStore     content-agnostic keyed blob store
#   rosette::storage::DeviceConfig  Rosette's own boot configuration record
#
# The block device comes from the chip port (ROSETTE_CHIP_STORAGE_SOURCES):
# the board's flash region on a microcontroller, a file-backed volume on a host.
# ------------------------------------------------------------
add_library(rosette_storage STATIC
    src/storage/volume.cpp
    src/storage/file_store.cpp
    src/storage/device_config.cpp
    ${ROSETTE_CHIP_STORAGE_SOURCES}
)
target_include_directories(rosette_storage PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
# The board's board_config.h places the volume in flash.
target_include_directories(rosette_storage PRIVATE ${ROSETTE_BOARD_DIR})
target_link_libraries(rosette_storage PUBLIC rosette_fs rosette_platform)

if(NOT CMAKE_CROSSCOMPILING)
    # DeviceConfig records on the host volume, including a stored version-1 record.
    add_executable(device_config_test tests/storage/device_config_test.cpp)
    target_link_libraries(device_config_test PRIVATE rosette_storage)
endif()

# ------------------------------------------------------------
# rosette_agent — the micro-ROS agent backend's session pieces
#
# rmw_agent_link() (the XRCE custom transport over Rosette sockets, agent
# ping, and the time exchange that sets platform::WallClock), on top of
# rosette_session's AgentSessionGate. The application itself is stock rclc
# and links micro_ros_vendor through this target. The agent locators and
# UdpSocketTransport are headers under include/rosette/ros/.
# ------------------------------------------------------------
if(ROSETTE_ENABLE_MICRO_ROS)
    add_library(rosette_agent STATIC
        src/ros/agent_rmw_link.cpp)
    target_link_libraries(rosette_agent PUBLIC
        rosette_session
        rosette_net
        micro_ros_vendor
    )
    # The XRCE socket's receive buffer holds a whole reliable-stream window
    # from the agent (ros/socket_transport.hpp).
    math(EXPR _rosette_xrce_rx_bytes
         "${ROSETTE_MICRO_ROS_STREAM_HISTORY} * ${ROSETTE_MICRO_ROS_MAX_TRANSPORT_MTU}")
    target_compile_definitions(rosette_agent PUBLIC
        ROSETTE_XRCE_SOCKET_RECEIVE_BUFFER=${_rosette_xrce_rx_bytes}u)

    # The agent path on a host, against a real micro_ros_agent (tests/agent/run_agent_live.sh).
    if(ROSETTE_NET_STACK STREQUAL "posix")
        add_executable(agent_live_probe tests/agent/agent_live_probe.cpp)
        target_link_libraries(agent_live_probe PRIVATE rosette_agent)
    endif()
endif()

# ------------------------------------------------------------
# Examples
#
# Each example is a self-contained directory (source, headers, its own
# interface package, its own CMakeLists) so it reads as one unit and can also
# be configured standalone. Interface packages under examples/ are compiled
# into libmicroros automatically (see cmake/micro_ros.cmake).
# ------------------------------------------------------------
add_subdirectory(examples/storage_demo)
add_subdirectory(examples/led_files)
add_subdirectory(examples/rtps_chatter)
add_subdirectory(examples/rclc_chatter)
add_subdirectory(examples/wiznet_udp_echo)

message(STATUS "")
message(STATUS "Rosette build configuration")
message(STATUS "  Board       : ${ROSETTE_BOARD}")
message(STATUS "  Board dir   : ${ROSETTE_BOARD_DIR}")
message(STATUS "  Ports       : chip ${ROSETTE_CHIP}, os ${ROSETTE_OS}, net ${ROSETTE_NET_STACK}")
if(PICO_BOARD)
    message(STATUS "  PICO_BOARD  : ${PICO_BOARD}")
endif()
message(STATUS "  Processor   : ${ROSETTE_PROCESSOR}")
if(ROSETTE_BOARD_COMMON STREQUAL "rp2_wiznet")
    message(STATUS "  SPI pins    : SCK=${ROSETTE_SPI_SCK} TX=${ROSETTE_SPI_TX} RX=${ROSETTE_SPI_RX} CS=${ROSETTE_SPI_CS}")
    message(STATUS "  WIZnet RST  : ${ROSETTE_WIZNET_RST}")
    message(STATUS "  WIZnet INT  : ${ROSETTE_WIZNET_INT}")
endif()
message(STATUS "")
