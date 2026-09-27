cmake_minimum_required(VERSION 3.20)

# Board selection + toolchain setup — must run before project(). See the file
# for the ROSETTE_BOARD values. A standalone example includes this same prelude.
include(${CMAKE_CURRENT_LIST_DIR}/cmake/rosette_prelude.cmake)

project(rosette C CXX ASM)
set(CMAKE_C_STANDARD   11)
set(CMAKE_CXX_STANDARD 17)

include(cmake/micro_ros.cmake)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(ROSETTE_BOARD_ENABLE_PICO_SDK)
    pico_sdk_init()
endif()

if(ROSETTE_BOARD_ENABLE_FREERTOS)
    include(cmake/freertos_tcp.cmake)
endif()

if(ROSETTE_BOARD_ENABLE_WIZNET)
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/cmake/wiznet.cmake")
        include(cmake/wiznet.cmake)
    else()
        message(FATAL_ERROR
            "Selected board '${ROSETTE_BOARD}' requires cmake/wiznet.cmake, but it is missing.")
    endif()
endif()

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

# ------------------------------------------------------------
# rosette_net — socket abstraction layer (rosette::net)
#
# The transport-neutral SocketProvider interface and its host backend. The
# RTPS core (cmake/rosette_rtps.cmake), rmw_rosette
# (cmake/rmw_rosette_firmware.cmake), and the board BSP all open their
# sockets through it. See docs/architecture.md.
# ------------------------------------------------------------
add_library(rosette_net STATIC
    src/net/socket.cpp
    src/net/address_candidate.cpp
    src/net/address_list.cpp
    src/net/address_manager.cpp
    src/net/network_config.cpp
    src/net/wiznet/dhcp_client.cpp
    src/net/wiznet/dhcp_provider.cpp
    src/net/wiznet/dhcp_wire.cpp
    src/support/fnv1a.cpp
)

set(ROSETTE_INTERNAL_INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/include/rosette/net
    ${CMAKE_CURRENT_SOURCE_DIR}/include/rosette/platform
    ${CMAKE_CURRENT_SOURCE_DIR}/include/rosette/ros)

target_include_directories(rosette_net PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${ROSETTE_BOARD_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/config
    ${ROSETTE_INTERNAL_INCLUDE_DIRS}
)

# The platform port the board selected (ROSETTE_OS / ROSETTE_CHIP in its
# board.cmake); carries ROSETTE_OS_* / ROSETTE_CHIP_* to every consumer.
set(ROSETTE_ROOT ${CMAKE_CURRENT_SOURCE_DIR})
include(cmake/rosette_platform.cmake)
target_link_libraries(rosette_net PUBLIC rosette_platform)

# Platform conformance test for the selected board's port. On a Pico it
# reports over USB serial; on the host it is also tests/platform's ctest.
add_executable(rosette_platform_contract tests/platform/platform_contract_test.cpp)
target_link_libraries(rosette_platform_contract PRIVATE rosette_platform)
if(ROSETTE_BOARD_ENABLE_PICO_SDK)
    target_link_libraries(rosette_platform_contract PRIVATE pico_stdlib pico_multicore)
    pico_enable_stdio_usb(rosette_platform_contract 1)
    pico_enable_stdio_uart(rosette_platform_contract 0)
    pico_add_extra_outputs(rosette_platform_contract)
endif()

if(NOT ROSETTE_BOARD_ENABLE_PICO_SDK)
    # Host: platform runtime + POSIX socket backend live here.
    target_sources(rosette_net PRIVATE src/platform/posix/host_runtime.cpp)
endif()

if(ROSETTE_BOARD_ENABLE_FREERTOS)
    target_link_libraries(rosette_net PUBLIC freertos_kernel freertos_tcp)
endif()

if(ROSETTE_BOARD_ENABLE_WIZNET)
    target_link_libraries(rosette_net PUBLIC wiznet_iolib)
    # The register-level WIZnet driver (WiznetDevice / WiznetSocketProvider /
    # WiznetLink / Dhcp*) is header-only and templated on <Chip, Bus>; the
    # firmware TU that builds the board's BoardLink instantiates it. No source
    # here picks a concrete chip/bus.
endif()

if(ROSETTE_BOARD_ENABLE_PICO_SDK)
    target_link_libraries(rosette_net PUBLIC
        pico_stdlib hardware_spi hardware_flash hardware_sync
    )
endif()

# ------------------------------------------------------------
# rosette_bsp — board bring-up + the platform runtime for a real board.
#
# The FreeRTOS variant carries the MACRAW driver and FreeRTOS+TCP glue; the
# no-RTOS variant is just board init + the bare pico platform runtime.
# ------------------------------------------------------------
if(ROSETTE_BOARD_BUILD_BSP)
    add_library(rosette_bsp STATIC
        ${ROSETTE_BOARD_DIR}/board_init.c
    )

    if(ROSETTE_BOARD_ENABLE_FREERTOS)
        target_sources(rosette_bsp PRIVATE
            src/net/freertos/igmp.cpp
            src/net/freertos/socket.cpp
            src/net/wiznet/macraw.cpp
            src/platform/pico_freertos.c
        )
    else()
        target_sources(rosette_bsp PRIVATE src/platform/pico_bare.c)
        if(ROSETTE_BOARD_ENABLE_WIZNET)
            # No RTOS: UDP/IP runs in the chip's hardware sockets, and the
            # board network module owns the chip, link, and addressing.
            target_sources(rosette_bsp PRIVATE src/net/wiznet/board_network.cpp)
        endif()
    endif()

    target_include_directories(rosette_bsp PUBLIC
        ${ROSETTE_BOARD_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${ROSETTE_INTERNAL_INCLUDE_DIRS}
    )

    target_link_libraries(rosette_bsp PUBLIC
        rosette_net pico_stdlib pico_unique_id
        hardware_spi hardware_flash hardware_sync hardware_gpio
    )
    if(ROSETTE_BOARD_ENABLE_WIZNET)
        target_link_libraries(rosette_bsp PUBLIC wiznet_iolib)
    endif()
    if(ROSETTE_BOARD_ENABLE_FREERTOS)
        target_link_libraries(rosette_bsp PUBLIC freertos_kernel freertos_tcp)
    endif()
endif()

# ------------------------------------------------------------
# rosette_storage — LittleFS volume + generic file API
#
#   rosette::storage::Volume        mounts the board storage region
#   rosette::storage::FileStore     content-agnostic keyed blob store
#   rosette::storage::DeviceConfig  Rosette's own boot configuration record
#
# The board block-device implementation is compiled in per target: the Pico
# flash binding for FreeRTOS boards, a file-backed volume for host builds.
# ------------------------------------------------------------
add_library(rosette_storage STATIC
    src/storage/volume.cpp
    src/storage/file_store.cpp
    src/storage/device_config.cpp
)
target_include_directories(rosette_storage PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(rosette_storage PUBLIC rosette_fs)

if(ROSETTE_BOARD_ENABLE_PICO_SDK)
    target_sources(rosette_storage PRIVATE src/platform/pico_littlefs.cpp)
    target_include_directories(rosette_storage PRIVATE ${ROSETTE_BOARD_DIR})
    target_link_libraries(rosette_storage PUBLIC rosette_bsp)
else()
    target_sources(rosette_storage PRIVATE src/platform/posix/host_littlefs.cpp)
endif()

# ------------------------------------------------------------
# rosette_ros — the header-only micro-ROS client wrapper
#
# All of include/rosette/ros/*.hpp (Context, Node, Publisher, Subscription,
# Timer, Service, Client, ActionServer/Client, transports, Agent locators).
# Consumers link this one target to pull the wrappers plus the generated
# micro-ROS library and its compile definitions.
# ------------------------------------------------------------
if(ROSETTE_ENABLE_MICRO_ROS)
    add_library(rosette_ros INTERFACE)
    target_include_directories(rosette_ros INTERFACE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${ROSETTE_INTERNAL_INCLUDE_DIRS}
    )
    target_link_libraries(rosette_ros INTERFACE
        rosette_net
        micro_ros_vendor
    )
endif()

# ------------------------------------------------------------
# Examples
#
# Each example is a self-contained directory (source, headers, its own
# interface package, its own CMakeLists) so it reads as one unit and can also
# be configured standalone. Interface packages under examples/ are compiled
# into libmicroros automatically (see cmake/micro_ros.cmake).
# ------------------------------------------------------------
add_subdirectory(examples/rosette_smoke)
add_subdirectory(examples/storage_demo)
add_subdirectory(examples/led_files)
add_subdirectory(examples/rtps_chatter)
add_subdirectory(examples/rclc_chatter)
add_subdirectory(examples/wiznet_udp_echo)

message(STATUS "")
message(STATUS "Rosette build configuration")
message(STATUS "  Board       : ${ROSETTE_BOARD}")
message(STATUS "  Family      : ${ROSETTE_BOARD_FAMILY}")
message(STATUS "  Board dir   : ${ROSETTE_BOARD_DIR}")
if(ROSETTE_BOARD_ENABLE_PICO_SDK)
    message(STATUS "  PICO_BOARD  : ${PICO_BOARD}")
endif()
message(STATUS "  Processor   : ${ROSETTE_PROCESSOR}")
if(ROSETTE_BOARD_ENABLE_WIZNET)
    message(STATUS "  SPI pins    : SCK=${ROSETTE_SPI_SCK} TX=${ROSETTE_SPI_TX} RX=${ROSETTE_SPI_RX} CS=${ROSETTE_SPI_CS}")
    message(STATUS "  WIZnet RST  : ${ROSETTE_WIZNET_RST}")
    message(STATUS "  WIZnet INT  : ${ROSETTE_WIZNET_INT}")
endif()
message(STATUS "")
