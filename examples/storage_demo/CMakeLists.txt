cmake_minimum_required(VERSION 3.20)

# storage_demo — exercises rosette::storage (Volume / FileStore / DeviceConfig)
# against the host file-backed volume. Host build only.
#
#   from the repo root:  cmake -S . -B build   (then --target storage_demo)
#   standalone:          cd examples/storage_demo && cmake -S . -B build

if(NOT TARGET rosette_net)
    include(${CMAKE_CURRENT_LIST_DIR}/../../cmake/rosette_prelude.cmake)
    project(rosette_example_storage_demo C CXX ASM)
    add_subdirectory("${ROSETTE_SOURCE_DIR}" "${CMAKE_BINARY_DIR}/rosette")
    return()
endif()

if(ROSETTE_BOARD_ENABLE_FREERTOS)
    return()
endif()

add_executable(storage_demo main.cpp)
target_link_libraries(storage_demo PRIVATE rosette_storage)
